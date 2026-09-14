// ============================================================================
// measure/code_detect.cpp —— 码区检测实现（OpenCV 自带检测器）
// 设计约定与实测结论见 code_detect.h 头注释。
// ============================================================================

#include "measure/code_detect.h"

#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <cmath>
#include <string>

#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/objdetect/barcode.hpp>

#include "common/logger.h"

namespace cam {
namespace {

std::string Fmt(const char* fmt, ...) {
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    return buf;
}

// 两个矩形的交并比（去重用）
double RectIou(const cv::Rect2d& a, const cv::Rect2d& b) {
    const double x1 = std::max(a.x, b.x);
    const double y1 = std::max(a.y, b.y);
    const double x2 = std::min(a.x + a.width, b.x + b.width);
    const double y2 = std::min(a.y + a.height, b.y + b.height);
    const double inter = std::max(0.0, x2 - x1) * std::max(0.0, y2 - y1);
    const double uni = a.area() + b.area() - inter;
    return uni > 0.0 ? inter / uni : 0.0;
}

// 同类型候选按 IoU 去重（条码检测器偶发同一码出多个近重合框），保留高置信度
void DedupeRegions(std::vector<CodeRegion>& regions) {
    for (size_t i = 0; i < regions.size(); ++i) {
        for (size_t j = regions.size(); j-- > i + 1;) {
            if (regions[i].type != regions[j].type) {
                continue;
            }
            const cv::Rect2d a(regions[i].x, regions[i].y, regions[i].w, regions[i].h);
            const cv::Rect2d b(regions[j].x, regions[j].y, regions[j].w, regions[j].h);
            if (RectIou(a, b) > 0.5) {
                if (regions[j].confidence > regions[i].confidence) {
                    regions[i] = regions[j];
                }
                regions.erase(regions.begin() + (ptrdiff_t)j);
            }
        }
    }
}

}  // namespace

bool DetectCodeRegions(const cv::Mat& gray,
                       const std::vector<cv::Point>& rotContour,
                       double totalAngleDeg,
                       const common::CodeDetectConfig& cfg,
                       std::vector<CodeRegion>& regions) {
    regions.clear();
    if (!cfg.enabled || gray.empty()) {
        return true;
    }
    try {
        // 1) 检测尺度：max_side<=0 时全分辨率（条码条纹对降采样敏感，实测
        //    0.5 倍以下检出率明显下跌；高分辨率相机确认检出率后可调小加速）
        const double scale =
            cfg.max_side > 0
                ? std::min(1.0, (double)cfg.max_side /
                                    (double)std::max(gray.cols, gray.rows))
                : 1.0;
        cv::Mat detImg = gray;
        if (scale < 1.0) {
            cv::resize(gray, detImg, cv::Size(), scale, scale, cv::INTER_AREA);
        }
        const double invScale = 1.0 / scale;

        // 2) 校正映射矩阵（与 RotateImageAndMask 完全一致）：检出四角点经
        //    精确仿射变换进旋转校正坐标系，点变换不涉及像素插值
        const cv::Point2d imgCenter(gray.cols / 2.0, gray.rows / 2.0);
        const cv::Mat rotMat = cv::getRotationMatrix2D(imgCenter, -totalAngleDeg, 1.0);

        // 3) 候选四角点（detImg 坐标）→ 原图尺度 → 校正坐标系外接矩形 → 过滤入库
        auto acceptQuad = [&](const std::vector<cv::Point>& quad, CodeType type,
                              double conf, CodeRegion& out) -> bool {
            if (quad.size() != 4) {
                return false;
            }
            std::vector<cv::Point2f> pts;
            pts.reserve(4);
            for (const cv::Point& p : quad) {
                pts.emplace_back((float)(p.x * invScale), (float)(p.y * invScale));
            }
            cv::transform(pts, pts, rotMat);
            float x1 = pts[0].x, y1 = pts[0].y, x2 = pts[0].x, y2 = pts[0].y;
            for (const cv::Point2f& p : pts) {
                x1 = std::min(x1, p.x);
                y1 = std::min(y1, p.y);
                x2 = std::max(x2, p.x);
                y2 = std::max(y2, p.y);
            }
            cv::Rect2d r(x1, y1, x2 - x1, y2 - y1);
            if (r.area() < cfg.min_area_px) {
                return false;
            }
            const cv::Point2d center(r.x + r.width * 0.5, r.y + r.height * 0.5);
            if (!rotContour.empty() &&
                cv::pointPolygonTest(rotContour, center, false) < 0.0) {
                return false;  // 码一定印在产品上，中心落在产品外的候选判为误报
            }
            // 裁剪到图像范围内（检测器四角点可能略微越界）
            const double cx1 = std::max(0.0, r.x);
            const double cy1 = std::max(0.0, r.y);
            const double cx2 = std::min((double)gray.cols, r.x + r.width);
            const double cy2 = std::min((double)gray.rows, r.y + r.height);
            if (cx2 <= cx1 || cy2 <= cy1) {
                return false;
            }
            out.type = type;
            out.x = cx1;
            out.y = cy1;
            out.w = cx2 - cx1;
            out.h = cy2 - cy1;
            out.confidence = conf;
            return true;
        };

        std::vector<CodeRegion> found;

        // 4) 二维码：检测器旋转不变，单档检测
        try {
            cv::QRCodeDetector qr;
            std::vector<std::string> infos;
            std::vector<std::vector<cv::Point>> quads;
            qr.detectAndDecodeMulti(detImg, infos, quads);
            for (size_t i = 0; i < quads.size(); ++i) {
                CodeRegion r;
                const double conf = (i < infos.size() && !infos[i].empty()) ? 1.0 : 0.5;
                if (acceptQuad(quads[i], CodeType::QR, conf, r)) {
                    found.push_back(r);
                }
            }
        } catch (const cv::Exception& e) {
            common::LogMsg(common::LWARN,
                           std::string("二维码检测异常（跳过，不影响其他输出）：") + e.what());
        }

        // 5) 一维码：条码检测器对方向敏感（实测竖条码 0° 档命中、90° 反漏检），
        //    0°/90° 两档各检一次，任一检出即停
        bool barFound = false;
        for (int pass = 0; pass < 2 && !barFound; ++pass) {
            try {
                cv::Mat passImg;
                if (pass == 0) {
                    passImg = detImg;
                } else {
                    cv::rotate(detImg, passImg, cv::ROTATE_90_CLOCKWISE);
                }
                cv::barcode::BarcodeDetector bar;
                std::vector<std::string> infos, types;
                std::vector<cv::Point> pts;  // 每个码连续 4 个角点
                bar.detectAndDecodeWithType(passImg, infos, types, pts);
                if (pass == 1) {
                    // 90° 顺时针图坐标映射回 detImg 坐标：
                    // src(x,y) -> dst(H-1-y, x)，逆变换 dst(x',y') -> src(y', H-1-x')
                    const int H = detImg.rows;
                    for (cv::Point& p : pts) {
                        const int px = p.x;
                        p.x = p.y;
                        p.y = H - 1 - px;
                    }
                }
                for (size_t base = 0, i = 0; base + 4 <= pts.size(); base += 4, ++i) {
                    const std::vector<cv::Point> quad(pts.begin() + (ptrdiff_t)base,
                                                      pts.begin() + (ptrdiff_t)base + 4);
                    CodeRegion r;
                    const double conf =
                        (i < infos.size() && !infos[i].empty()) ? 1.0 : 0.5;
                    if (acceptQuad(quad, CodeType::BAR, conf, r)) {
                        found.push_back(r);
                        barFound = true;
                    }
                }
            } catch (const cv::Exception& e) {
                common::LogMsg(common::LWARN,
                               std::string("一维码检测异常（跳过，不影响其他输出）：") +
                                   e.what());
            }
        }

        DedupeRegions(found);
        regions = std::move(found);

        int qrCount = 0, barCount = 0;
        for (const CodeRegion& r : regions) {
            if (r.type == CodeType::QR) {
                ++qrCount;
            } else {
                ++barCount;
            }
            common::LogMsg(common::LINFO,
                           Fmt("  %s: (%.1f, %.1f) %.1fx%.1f, %s",
                               r.type == CodeType::QR ? "二维码" : "一维码",
                               r.x, r.y, r.w, r.h,
                               r.confidence >= 1.0 ? "已解码" : "仅定位"));
        }
        common::LogMsg(common::LINFO,
                       Fmt("码区检测: 二维码 %d 个, 一维码 %d 个（检测尺度 %.2f）",
                           qrCount, barCount, scale));
        return true;
    } catch (const cv::Exception& e) {
        regions.clear();
        common::LogMsg(common::LWARN,
                       std::string("码区检测失败（不影响宽高与水平边输出）：") + e.what());
        return false;
    }
}

}  // namespace cam
