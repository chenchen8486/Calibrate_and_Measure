// ============================================================================
// measure/code_detect.cpp —— 码区检测实现（OpenCV 自带检测器 + 条纹候选兜底）
// 设计约定与实测结论见 code_detect.h 头注释。
// ============================================================================

#include "measure/code_detect.h"

#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <cmath>
#include <climits>
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

// 同类型候选按 IoU 去重，保留高置信度（解码体系结果优先于条纹兜底候选）
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

// 条纹密度校验：候选区域沿条纹伸展方向取 5 条扫描线（10%~90%），统计 Otsu
// 二值跳变次数。真条码条纹贯通整个区域，每条扫描线跳变都多；文字块在字行/
// 字列间隙处跳变接近 0。再对跳变最少的那条线算条纹宽度变异系数（CV）：
// 条码条纹宽度按模块宽度量化（CV 小），文字笔画宽度杂乱（CV 大）。
// 判据：最小跳变数与密度达标，且 CV 不超限
bool StripeDensityOk(const cv::Mat& gray, const cv::Rect& r, bool verticalBars) {
    cv::Mat roi = gray(r).clone();
    cv::threshold(roi, roi, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    const double fracs[5] = {0.1, 0.3, 0.5, 0.7, 0.9};
    int minCnt = INT_MAX;
    std::vector<int> minRuns;
    for (double f : fracs) {
        cv::Mat line = verticalBars
                           ? roi.row(std::min(roi.rows - 1, (int)(roi.rows * f)))
                           : roi.col(std::min(roi.cols - 1, (int)(roi.cols * f)));
        const int n = line.cols * line.rows;
        int cnt = 0;
        std::vector<int> runs;
        for (int i = 1; i < n; ++i) {
            if (line.at<uchar>(i) != line.at<uchar>(i - 1)) {
                ++cnt;
            }
        }
        if (cnt < minCnt) {
            minCnt = cnt;
            // 记录该线的条纹宽度序列（同色游程长度）
            minRuns.clear();
            int run = 1;
            for (int i = 1; i < n; ++i) {
                if (line.at<uchar>(i) == line.at<uchar>(i - 1)) {
                    ++run;
                } else {
                    minRuns.push_back(run);
                    run = 1;
                }
            }
            minRuns.push_back(run);
        }
    }
    const double longSide = (double)std::max(r.width, r.height);
    if (minCnt < 12 || minCnt / longSide < 0.08 || minRuns.size() < 8) {
        return false;
    }
    // 去掉首尾两条端部游程（可能是留白区，长度不代表条纹）
    if (minRuns.size() > 4) {
        minRuns.erase(minRuns.begin());
        minRuns.pop_back();
    }
    double mean = 0.0;
    for (int v : minRuns) {
        mean += v;
    }
    mean /= (double)minRuns.size();
    double var = 0.0;
    for (int v : minRuns) {
        var += (v - mean) * (v - mean);
    }
    const double cv = mean > 0.0 ? std::sqrt(var / (double)minRuns.size()) / mean : 9.0;
    // CV 上限 1.3：盒 flap 弯曲处条码有透视畸变，实测真码 CV 可达 1.2；
    // 文字误报主要由短边下限与最小跳变数挡掉
    return cv <= 1.3;
}

// 条纹候选兜底（不依赖解码，覆盖 Code128/药品监管码等解码体系外条码）：
// 梯度各向异性（|gx|-|gy| / |gy|-|gx|）+ Otsu + 沿条纹方向形态学闭运算合并
// 成块，再按长宽比/短边/面积/条纹密度过滤。内部降采样到 2736 以内控制耗时，
// 返回原图尺度候选矩形。
void FindStripeCandidates(const cv::Mat& gray, std::vector<cv::Rect>& rects) {
    rects.clear();
    const double scale =
        std::min(1.0, 2736.0 / (double)std::max(gray.cols, gray.rows));
    cv::Mat simg = gray;
    if (scale < 1.0) {
        cv::resize(gray, simg, cv::Size(), scale, scale, cv::INTER_AREA);
    }
    const double invScale = 1.0 / scale;
    for (int orient = 0; orient < 2; ++orient) {
        const bool verticalBars = (orient == 0);  // 竖直条纹（区域横长）
        cv::Mat gx, gy, diff, bw;
        cv::Sobel(simg, gx, CV_32F, 1, 0);
        cv::Sobel(simg, gy, CV_32F, 0, 1);
        cv::subtract(verticalBars ? cv::abs(gx) : cv::abs(gy),
                     verticalBars ? cv::abs(gy) : cv::abs(gx), diff);
        diff.convertTo(bw, CV_8U);
        cv::threshold(bw, bw, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
        // 闭运算核：沿条纹伸展方向拉长，桥接条纹间隙把整块码区连成一团
        const int kLong = std::max(9, simg.cols / 100);
        const int kShort = std::max(3, simg.cols / 400);
        const cv::Mat kernel = cv::getStructuringElement(
            cv::MORPH_RECT,
            verticalBars ? cv::Size(kLong, kShort) : cv::Size(kShort, kLong));
        cv::morphologyEx(bw, bw, cv::MORPH_CLOSE, kernel);
        cv::morphologyEx(bw, bw, cv::MORPH_OPEN,
                         cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(bw, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        for (const std::vector<cv::Point>& c : contours) {
            const cv::Rect r = cv::boundingRect(c);
            // 区域长边方向须与条纹取向一致：竖条纹码区横长，横条纹码区纵长
            if (verticalBars != (r.width >= r.height)) {
                continue;
            }
            const double longSide = (double)std::max(r.width, r.height);
            const double shortSide = (double)std::min(r.width, r.height);
            const double aspect = longSide / shortSide;
            // 长宽比上限放到 6.0：药品监管码（20 位）条区长宽比可达 5+；
            // 短边下限 40（约 80px 原图）：说明书文字行/列达不到，条码条长足够
            if (aspect < 1.8 || aspect > 6.0 || shortSide < 40.0 ||
                r.area() < 2000) {
                continue;
            }
            if (!StripeDensityOk(simg, r, verticalBars)) {
                continue;
            }
            rects.emplace_back((int)(r.x * invScale), (int)(r.y * invScale),
                               (int)(r.width * invScale), (int)(r.height * invScale));
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

        // 2) 校正映射矩阵（与 RotateImageAndMask 完全一致）：检出四角点经
        //    精确仿射变换进旋转校正坐标系，点变换不涉及像素插值
        const cv::Point2d imgCenter(gray.cols / 2.0, gray.rows / 2.0);
        const cv::Mat rotMat = cv::getRotationMatrix2D(imgCenter, -totalAngleDeg, 1.0);

        // 3) 候选四角点（坐标所在尺度由 coordToOrig 给出）→ 原图尺度 →
        //    校正坐标系外接矩形 → 过滤入库
        auto acceptQuad = [&](const std::vector<cv::Point>& quad, double coordToOrig,
                              CodeType type, double conf, CodeRegion& out) -> bool {
            if (quad.size() != 4) {
                return false;
            }
            std::vector<cv::Point2f> pts;
            pts.reserve(4);
            for (const cv::Point& p : quad) {
                pts.emplace_back((float)(p.x * coordToOrig), (float)(p.y * coordToOrig));
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
        const double invScale = 1.0 / scale;

        // 4) 二维码：检测器旋转不变，单档检测
        try {
            cv::QRCodeDetector qr;
            std::vector<std::string> infos;
            std::vector<std::vector<cv::Point>> quads;
            qr.detectAndDecodeMulti(detImg, infos, quads);
            for (size_t i = 0; i < quads.size(); ++i) {
                CodeRegion r;
                const double conf = (i < infos.size() && !infos[i].empty()) ? 1.0 : 0.5;
                if (acceptQuad(quads[i], invScale, CodeType::QR, conf, r)) {
                    found.push_back(r);
                }
            }
        } catch (const cv::Exception& e) {
            common::LogMsg(common::LWARN,
                           std::string("二维码检测异常（跳过，不影响其他输出）：") + e.what());
        }

        // 5) 一维码（解码体系）：条码检测器对方向敏感（实测竖条码 0° 档命中、
        //    90° 反漏检），0°/90° 两档各检一次，任一检出即停
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
                    if (acceptQuad(quad, invScale, CodeType::BAR, conf, r)) {
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

        // 6) 条纹候选兜底：仅当解码体系未检出任何一维码时运行。不依赖解码，
        //    定位致密平行条纹区域，覆盖 Code128/药品监管码等体系外条码；
        //    候选置信度 0.3（未确认仅定位），前端可按阈值过滤
        if (!barFound) {
            try {
                std::vector<cv::Rect> stripes;
                FindStripeCandidates(gray, stripes);  // 返回原图尺度矩形
                for (const cv::Rect& r : stripes) {
                    const std::vector<cv::Point> quad = {
                        {r.x, r.y}, {r.x + r.width, r.y},
                        {r.x + r.width, r.y + r.height}, {r.x, r.y + r.height}};
                    CodeRegion cr;
                    if (acceptQuad(quad, 1.0, CodeType::BAR, 0.3, cr)) {
                        found.push_back(cr);
                    }
                }
            } catch (const cv::Exception& e) {
                common::LogMsg(common::LWARN,
                               std::string("条纹候选兜底异常（跳过，不影响其他输出）：") +
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
                               r.confidence >= 1.0
                                   ? "已解码"
                                   : (r.confidence >= 0.5 ? "仅定位" : "条纹候选")));
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
