// ============================================================================
// rectifier.cpp —— 正射矫正模块实现
// 逐行等价移植自 Python 版 build_rectify_maps / _plane_intersect /
// _project_board / RectifyMaps.rectify / board_mm_to_px。
// ============================================================================

#include "rectifier.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "common/image_io.h"
#include "common/logger.h"

namespace {

// 分行计算映射表的行块大小（控制内存），对齐 Python 版 chunk_rows
constexpr int kChunkRows = 256;

// 图像像素反投影射线与板面（z=0）求交点（板面毫米坐标）
// 流程：undistortPoints 得归一化坐标 → 射线 (x,y,1) →
//       板面系 P = λ·(Rᵀ·ray) + (-Rᵀ·t)，令 z=0 解 λ = -b[2]/a[2]
// @param calib 标定结果
// @param uv    图像像素坐标
// @param outXY 输出参数：板面坐标 (X, Y)（mm）
// @param errMsg 输出参数：失败时的中文错误描述
// @return 求交成功返回 true
bool PlaneIntersectBoard(const cam::CalibrationResult& calib, cv::Point2d uv,
                         cv::Point2d& outXY, std::string& errMsg) {
    std::vector<cv::Point2d> norm;
    try {
        cv::undistortPoints(std::vector<cv::Point2d>{uv}, norm,
                            calib.camera_matrix, calib.dist_coeffs);
    } catch (const cv::Exception& e) {
        errMsg = "去畸变归一化失败: " + std::string(e.what());
        return false;
    }
    cv::Mat rot;
    cv::Rodrigues(calib.rvec, rot);          // 3x3 CV_64F
    const cv::Mat rt = rot.t();
    const cv::Mat ray = (cv::Mat_<double>(3, 1) << norm[0].x, norm[0].y, 1.0);
    const cv::Mat a = rt * ray;              // P_board = λ·a + b
    const cv::Mat b = -(rt * calib.tvec);
    const double a2 = a.at<double>(2);
    if (std::abs(a2) < 1e-12) {
        errMsg = "视线方向与板面近似平行，无法求交（标定外参异常）";
        return false;
    }
    const double lam = -b.at<double>(2) / a2;
    const cv::Mat p = lam * a + b;
    outXY = cv::Point2d(p.at<double>(0), p.at<double>(1));
    return true;
}

// 板面毫米点集 (N,3) 正向投影回原图像素 (N,2)
// 调用方需自行捕获 cv::Exception
std::vector<cv::Point2d> ProjectBoard(const cam::CalibrationResult& calib,
                                      const std::vector<cv::Point3d>& pts) {
    std::vector<cv::Point2d> proj;
    cv::projectPoints(pts, calib.rvec, calib.tvec,
                      calib.camera_matrix, calib.dist_coeffs, proj);
    return proj;
}

}  // namespace

namespace cam {

bool Rectifier::Load(const std::string& calibXmlPath, double targetMmPerPx,
                     std::string& errMsg) {
    ready_ = false;
    errMsg.clear();
    if (targetMmPerPx <= 0.0) {
        errMsg = "目标像素当量必须为正数，实际: " + std::to_string(targetMmPerPx);
        common::LogMsg(common::LERROR, errMsg);
        return false;
    }
    if (!LoadCalibrationXml(calibXmlPath, calib_, errMsg)) {
        return false;  // errMsg 已由 LoadCalibrationXml 填写
    }
    mm_per_px_ = targetMmPerPx;

    const int w = calib_.image_size.width;
    const int h = calib_.image_size.height;
    const double cx = (w - 1) / 2.0;
    const double cy = (h - 1) / 2.0;

    // ---- 图像中心反投影到板面，作为正射图中心锚点 (Xc, Yc) ----
    cv::Point2d centerMm;
    if (!PlaneIntersectBoard(calib_, cv::Point2d(cx, cy), centerMm, errMsg)) {
        common::LogMsg(common::LERROR, "构建正射映射失败: " + errMsg);
        return false;
    }
    center_x_mm_ = centerMm.x;
    center_y_mm_ = centerMm.y;

    // ---- 方向符号：让输出图与原始图同向（板面 +X 在原始图向右则 sign_x=+1）----
    try {
        const std::vector<cv::Point2d> p = ProjectBoard(calib_, {
            {center_x_mm_, center_y_mm_, 0.0},
            {center_x_mm_ + 1.0, center_y_mm_, 0.0},
            {center_x_mm_, center_y_mm_ + 1.0, 0.0}});
        const cv::Point2d dx = p[1] - p[0];
        const cv::Point2d dy = p[2] - p[0];
        sign_x_ = dx.x >= 0 ? 1.0 : -1.0;
        sign_y_ = dy.y >= 0 ? 1.0 : -1.0;
        if (std::abs(dx.y) > std::abs(dx.x)) {
            common::LogMsg(common::LWARN,
                           "板面 X 轴在图像中更接近竖直方向，相机可能旋转了约 90°，"
                           "标定照常进行（测量不受影响），但请确认安装方向");
        }
    } catch (const cv::Exception& e) {
        errMsg = "方向符号投影计算失败: " + std::string(e.what());
        common::LogMsg(common::LERROR, errMsg);
        return false;
    }

    // ---- 逐输出像素合成 remap 表：输出像素 → 板面毫米 → 正投影回原图取色 ----
    map_x_.create(h, w, CV_32FC1);
    map_y_.create(h, w, CV_32FC1);
    try {
        for (int y0 = 0; y0 < h; y0 += kChunkRows) {
            const int rows = std::min(kChunkRows, h - y0);
            std::vector<cv::Point3d> pts;
            pts.reserve(static_cast<size_t>(rows) * w);
            for (int v = y0; v < y0 + rows; ++v) {
                const double yMm = center_y_mm_ + sign_y_ * (v - cy) * mm_per_px_;
                for (int u = 0; u < w; ++u) {
                    pts.emplace_back(center_x_mm_ + sign_x_ * (u - cx) * mm_per_px_,
                                     yMm, 0.0);
                }
            }
            const std::vector<cv::Point2d> proj = ProjectBoard(calib_, pts);
            for (int r = 0; r < rows; ++r) {
                float* mx = map_x_.ptr<float>(y0 + r);
                float* my = map_y_.ptr<float>(y0 + r);
                const size_t base = static_cast<size_t>(r) * w;
                for (int u = 0; u < w; ++u) {
                    mx[u] = static_cast<float>(proj[base + u].x);
                    my[u] = static_cast<float>(proj[base + u].y);
                }
            }
        }
    } catch (const cv::Exception& e) {
        errMsg = "正射映射表构建失败: " + std::string(e.what());
        common::LogMsg(common::LERROR, errMsg);
        map_x_.release();
        map_y_.release();
        return false;
    }

    char buf[160];
    std::snprintf(buf, sizeof(buf),
                  "正射映射表已生成: %dx%d, 1px=%.4fmm, 中心锚点 (%.1f, %.1f)mm",
                  w, h, mm_per_px_, center_x_mm_, center_y_mm_);
    common::LogMsg(common::LINFO, buf);
    ready_ = true;
    return true;
}

bool Rectifier::IsReady() const {
    return ready_;
}

cv::Mat Rectifier::Rectify(const cv::Mat& gray) const {
    if (!ready_) {
        common::LogMsg(common::LERROR, "矫正器未就绪（请先 Load 标定 XML）");
        return cv::Mat();
    }
    if (gray.empty()) {
        common::LogMsg(common::LERROR, "正射矫正输入为空图");
        return cv::Mat();
    }
    if (gray.size() != calib_.image_size) {
        common::LogMsg(common::LERROR,
                       "图像尺寸 (" + std::to_string(gray.cols) + ", " +
                       std::to_string(gray.rows) + ") 与标定尺寸 (" +
                       std::to_string(calib_.image_size.width) + ", " +
                       std::to_string(calib_.image_size.height) +
                       ") 不一致，请用同机位同分辨率采图标定");
        return cv::Mat();
    }
    // 约定输入 8UC1 灰度；8UC3/8UC4 先转灰度，其余深度不支持
    cv::Mat g = gray;
    if (gray.channels() != 1) {
        common::LogMsg(common::LWARN, "正射矫正输入非单通道，已自动转灰度");
        g = common::ToGray8(gray);
    }
    if (g.empty() || g.depth() != CV_8U) {
        common::LogMsg(common::LERROR, "正射矫正仅支持 8 位图像");
        return cv::Mat();
    }
    cv::Mat out;
    cv::remap(g, out, map_x_, map_y_, cv::INTER_LINEAR, cv::BORDER_CONSTANT,
              cv::Scalar(0));
    return out;
}

cv::Point2d Rectifier::BoardMmToPx(double xMm, double yMm) const {
    if (!ready_) {
        common::LogMsg(common::LERROR, "矫正器未就绪（请先 Load 标定 XML）");
        return cv::Point2d(0.0, 0.0);
    }
    const double cx = (calib_.image_size.width - 1) / 2.0;
    const double cy = (calib_.image_size.height - 1) / 2.0;
    return cv::Point2d(cx + sign_x_ * (xMm - center_x_mm_) / mm_per_px_,
                       cy + sign_y_ * (yMm - center_y_mm_) / mm_per_px_);
}

double Rectifier::MmPerPx() const {
    return mm_per_px_;
}

}  // namespace cam
