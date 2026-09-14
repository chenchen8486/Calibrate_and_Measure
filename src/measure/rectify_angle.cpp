// ============================================================================
// measure/rectify_angle.cpp —— 两级旋转校正实现
// 等价移植自 Python 工程 src/edge_measure/rectify.py
// ============================================================================

#include "measure/rectify_angle.h"

#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <cmath>

#include <opencv2/imgproc.hpp>

#include "common/logger.h"
#include "measure/edge_refine.h"
#include "measure/measure_core.h"

namespace cam {
namespace {

constexpr double kRad2Deg = 180.0 / CV_PI;

std::string Fmt(const char* fmt, ...) {
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    return buf;
}

enum class Side { Left, Right, Top, Bottom };

// 沿某一侧逐行/列采集亚像素边缘点（掩膜初值 + 窗口阶跃卡尺精化）。
// 以掩膜逐行/列极值为原点、沿外侧法线做 SnapProfile（step 检测器），
// 吸附失败时保持掩膜位置（RANSAC 拟合对少量此类点免疫）。
std::vector<cv::Point2d> SampleSideEdges(const cv::Mat& gray, const cv::Mat& mask,
                                         Side side, const common::RefineConfig& refineCfg,
                                         int step) {
    const int h = mask.rows, w = mask.cols;
    const cv::Point2d normal =
        (side == Side::Left)   ? cv::Point2d(-1.0, 0.0)
        : (side == Side::Right) ? cv::Point2d(1.0, 0.0)
        : (side == Side::Top)   ? cv::Point2d(0.0, -1.0)
                                : cv::Point2d(0.0, 1.0);
    std::vector<cv::Point2d> points;
    if (side == Side::Left || side == Side::Right) {
        for (int y = 0; y < h; y += step) {
            std::vector<cv::Point> nz;
            cv::findNonZero(mask.row(y), nz);
            if (nz.empty()) {
                continue;
            }
            const double x0 =
                (side == Side::Left) ? (double)nz.front().x : (double)nz.back().x;
            const cv::Point2d origin(x0, (double)y);
            const double delta = SnapProfile(gray, origin, normal, refineCfg);
            const cv::Point2d p = origin + normal * delta;
            points.emplace_back(p.x, (double)y);
        }
    } else {
        for (int x = 0; x < w; x += step) {
            std::vector<cv::Point> nz;
            cv::findNonZero(mask.col(x), nz);
            if (nz.empty()) {
                continue;
            }
            const double y0 =
                (side == Side::Top) ? (double)nz.front().y : (double)nz.back().y;
            const cv::Point2d origin((double)x, y0);
            const double delta = SnapProfile(gray, origin, normal, refineCfg);
            const cv::Point2d p = origin + normal * delta;
            points.emplace_back((double)x, p.y);
        }
    }
    return points;
}

}  // namespace

bool FindReferenceAngle(const std::vector<cv::Point>& contour,
                        const common::RotateConfig& cfg,
                        double& angleDeg, std::string& errMsg) {
    const double perimeter = cv::arcLength(contour, true);
    std::vector<cv::Point> approx;
    cv::approxPolyDP(contour, approx, cfg.approx_epsilon_ratio * perimeter, true);
    const int n = (int)approx.size();
    if (n < 3) {
        errMsg = "多边形逼近退化，无法确定基准边";
        common::LogMsg(common::LERROR, Fmt("多边形逼近退化：顶点数 %d", n));
        return false;
    }

    // 1) 候选边方向角归一化 [-45,45)（mod 90°）；物理约束：摆放倾角 ±2° 量级，
    //    只接受 ±8° 内的边参与投票（留出余量）
    struct Candidate {
        int idx;      // 逼近多边形边索引
        double len;   // 边长
        double ang;   // 归一化方向角
    };
    std::vector<Candidate> inRange;
    double totalLen = 0.0;
    for (int i = 0; i < n; ++i) {
        const cv::Point2d p1 = approx[i];
        const cv::Point2d p2 = approx[(i + 1) % n];
        const double segLen = std::hypot(p2.x - p1.x, p2.y - p1.y);
        const double ang =
            NormalizeAngle90(std::atan2(p2.y - p1.y, p2.x - p1.x) * kRad2Deg);
        if (std::fabs(ang) <= 8.0) {
            inRange.push_back({i, segLen, ang});
            totalLen += segLen;
        }
    }
    if (inRange.empty()) {
        errMsg = "未找到主方向基准边（产品可能倾斜超限或轮廓异常）";
        common::LogMsg(common::LERROR, errMsg);
        return false;
    }

    // 2) 边长加权直方图投票，bin = 0.1°；主峰 ±3 bin 加权质心作为粗角度
    const double binWidth = 0.1, halfRange = 8.0;
    const int numBins = (int)(2 * halfRange / binWidth);
    std::vector<double> hist(numBins, 0.0);
    for (const Candidate& c : inRange) {
        const int b = (int)((c.ang + halfRange) / binWidth) % numBins;
        hist[b] += c.len;
    }
    const int peak = (int)(std::max_element(hist.begin(), hist.end()) - hist.begin());
    const int lo = std::max(0, peak - 3);
    const int hi = std::min(numBins, peak + 4);
    double wSum = 0.0, cSum = 0.0;
    for (int b = lo; b < hi; ++b) {
        wSum += hist[b];
        cSum += ((b + 0.5) * binWidth - halfRange) * hist[b];
    }
    const double coarse = cSum / wSum;

    // 3) 精拟合：每条主峰候选边（±0.75° 内）分别收集带内轮廓点做最小二乘拟合，
    //    再按边长加权平均方向角。严禁合并多条平行边的点集统一拟合——
    //    分离点簇会让 fitLine 收敛到斜穿线。
    std::vector<cv::Point2d> pts;
    pts.reserve(contour.size());
    for (const cv::Point& p : contour) {
        pts.emplace_back((double)p.x, (double)p.y);
    }
    double fineSum = 0.0, fineW = 0.0, votedLen = 0.0;
    for (const Candidate& c : inRange) {
        if (std::fabs(c.ang - coarse) > 0.75) {
            continue;
        }
        votedLen += c.len;
        double fineAng = c.ang;  // 点不足时用逼近边角度
        if (c.len > 1e-9) {
            const cv::Point2d p1 = approx[c.idx];
            const cv::Point2d p2 = approx[(c.idx + 1) % n];
            const cv::Point2d segDir = (p2 - p1) * (1.0 / c.len);
            const cv::Point2d segNormal(-segDir.y, segDir.x);
            std::vector<cv::Point2d> segPts;
            for (const cv::Point2d& p : pts) {
                const cv::Point2d rel = p - p1;
                const double along = rel.dot(segDir);
                const double perp = std::fabs(rel.dot(segNormal));
                if (along >= -cfg.edge_band && along <= c.len + cfg.edge_band &&
                    perp <= cfg.edge_band) {
                    segPts.push_back(p);
                }
            }
            if (segPts.size() >= 20) {
                LineResult line;
                std::string fitErr;
                if (!FitLineLeastSquares(segPts, line, fitErr)) {
                    errMsg = "基准边直线拟合失败：" + fitErr;
                    common::LogMsg(common::LERROR, errMsg);
                    return false;
                }
                double a = NormalizeAngle90(
                    std::atan2(line.direction.y, line.direction.x) * kRad2Deg);
                // 与粗角对齐（防 90° 跳变）：差异超 45° 说明方向相反，翻回
                if (std::fabs(a - coarse) > 45.0) {
                    a = (a > coarse) ? a - 90.0 : a + 90.0;
                }
                fineAng = a;
            }
        }
        fineSum += fineAng * c.len;
        fineW += c.len;
    }
    angleDeg = fineSum / fineW;
    common::LogMsg(common::LINFO,
                   Fmt("主方向投票: %d 条候选边, 主峰角 %.3f°, 精拟合角 %.3f° "
                       "(投票边长占比 %.0f%%)",
                       (int)inRange.size(), coarse, angleDeg,
                       votedLen / totalLen * 100.0));
    return true;
}

void RotateImageAndMask(const cv::Mat& gray, const cv::Mat& mask, double angleDeg,
                        cv::Mat& rotGray, cv::Mat& rotMask) {
    const cv::Point2d center(gray.cols / 2.0, gray.rows / 2.0);
    const cv::Mat mat = cv::getRotationMatrix2D(center, -angleDeg, 1.0);
    // 测量分支用双线性插值（保持灰度连续），掩膜用最近邻（保持二值）
    cv::warpAffine(gray, rotGray, mat, gray.size(), cv::INTER_LINEAR,
                   cv::BORDER_CONSTANT, 0);
    cv::warpAffine(mask, rotMask, mat, mask.size(), cv::INTER_NEAREST,
                   cv::BORDER_CONSTANT, 0);
    common::LogMsg(common::LINFO, Fmt("旋转校正完成: %.3f°", -angleDeg));
}

double RefineAngleByEdges(const cv::Mat& gray, const cv::Mat& mask,
                          const common::MeasureConfig& measureCfg,
                          const common::RefineConfig& refineCfg, int minPoints) {
    const Side sides[4] = {Side::Left, Side::Right, Side::Top, Side::Bottom};
    const char* sideNames[4] = {"left", "right", "top", "bottom"};
    int bestIdx = -1;
    std::vector<cv::Point2d> bestPts;
    for (int i = 0; i < 4; ++i) {
        std::vector<cv::Point2d> pts = SampleSideEdges(gray, mask, sides[i], refineCfg, 8);
        if (pts.size() > bestPts.size()) {
            bestIdx = i;
            bestPts = std::move(pts);
        }
    }
    if ((int)bestPts.size() < minPoints) {
        common::LogMsg(common::LWARN,
                       Fmt("精校正采样点不足（%d），跳过", (int)bestPts.size()));
        return 0.0;
    }
    LineResult line;
    std::string err;
    if (!RansacLine(bestPts, measureCfg.ransac_threshold, line, err)) {
        common::LogMsg(common::LWARN, "精校正 RANSAC 拟合失败：" + err + "，跳过");
        return 0.0;
    }
    const double residual =
        NormalizeAngle90(std::atan2(line.direction.y, line.direction.x) * kRad2Deg);
    // 实测方向角的相反数才是"需补偿角"（实证标定：rotate(-a) 使该测量值 +a）
    const double compensation = -residual;
    common::LogMsg(common::LINFO,
                   Fmt("精校正: 最优侧 %s, 采样 %d 点, 内点率 %.0f%%, "
                       "实测方向角 %.3f°, 补偿角 %.3f°",
                       sideNames[bestIdx], (int)bestPts.size(),
                       line.inlierRatio * 100.0, residual, compensation));
    return compensation;
}

}  // namespace cam
