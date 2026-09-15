// ============================================================================
// measure/code_detect_ai.cpp —— AI 码区检测实现（码区类实例掩膜 + 后处理）
// 设计约定与回退策略见 code_detect_ai.h 头注释。
// ============================================================================

#include "measure/code_detect_ai.h"

#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <string>

#include <opencv2/imgproc.hpp>

#include "common/logger.h"
#include "measure/code_detect.h"
#include "measure/segment_ai.h"

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

// 两个矩形的交并比（同码区多查询去重用）
double RectIou(const cv::Rect& a, const cv::Rect& b) {
    const int x1 = std::max(a.x, b.x);
    const int y1 = std::max(a.y, b.y);
    const int x2 = std::min(a.x + a.width, b.x + b.width);
    const int y2 = std::min(a.y + a.height, b.y + b.height);
    const double inter =
        (double)std::max(0, x2 - x1) * (double)std::max(0, y2 - y1);
    const double uni = (double)a.area() + (double)b.area() - inter;
    return uni > 0.0 ? inter / uni : 0.0;
}

// 过阈查询产出的一幅实例掩膜经连通域分解后的候选
struct CodeCand {
    cv::Rect rect;    // 原图尺度外接矩形
    double   area;    // 连通域面积（像素²）
    float    score;   // 所属查询的类别置信度
};

// 码区类别索引（训练约定：两类模型 类 0 = 盒子、类 1 = 码区）。
// 这是算法侧与模型之间的合同，不属于部署配置；重训时导出顺序不符应在
// 训练侧调整，不在 ini 里兜底
constexpr int kCodeClass = 1;

}  // namespace

bool DetectCodeRegionsAi(IAiSegmenter& seg, const cv::Mat& gray,
                         const std::vector<cv::Point>& rotContour,
                         double totalAngleDeg, const common::CodeDetectConfig& cfg,
                         const common::CodeDetectAiConfig& aiCfg,
                         std::vector<CodeRegion>& regions) {
    regions.clear();
    if (!cfg.enabled || gray.empty()) {
        return true;
    }
    try {
        // 1) 整图推理取码区类实例掩膜（不做 ROI crop，码区可能出现在产品
        //    任意面）；模型无码区类别通道（旧的单类模型）时返回 false，
        //    由调用方回退传统三层链
        std::vector<cv::Mat> masks;
        std::vector<float> scores;
        std::string desc;
        if (!seg.InferClassMasks(gray, kCodeClass, aiCfg.threshold,
                                 masks, scores, desc)) {
            return false;  // 原因已记 Warn
        }

        // 2) 逐掩膜连通域分解 + 过滤：面积双下限去琐碎噪声，矩形度保证
        //    "大且完整"，长宽比上限排除细长假区（QR 约 1，一维码 2~5）
        const double imgArea = (double)gray.cols * (double)gray.rows;
        const double minArea =
            std::max(cfg.min_area_px, aiCfg.min_area_ratio * imgArea);
        std::vector<CodeCand> cands;
        for (size_t i = 0; i < masks.size(); ++i) {
            cv::Mat labels, stats, centroids;
            const int n = cv::connectedComponentsWithStats(masks[i], labels, stats,
                                                           centroids, 8);
            for (int lb = 1; lb < n; ++lb) {
                const int x = stats.at<int>(lb, cv::CC_STAT_LEFT);
                const int y = stats.at<int>(lb, cv::CC_STAT_TOP);
                const int w = stats.at<int>(lb, cv::CC_STAT_WIDTH);
                const int h = stats.at<int>(lb, cv::CC_STAT_HEIGHT);
                const double area = stats.at<int>(lb, cv::CC_STAT_AREA);
                if (area < minArea) {
                    continue;
                }
                const double extent = area / ((double)w * (double)h);
                if (extent < aiCfg.extent_min) {
                    continue;
                }
                const double aspect =
                    (double)std::max(w, h) / (double)std::max(1, std::min(w, h));
                if (aspect > aiCfg.aspect_max) {
                    continue;
                }
                cands.push_back({cv::Rect(x, y, w, h), area, scores[i]});
            }
        }

        // 3) 同码区多查询候选按 IoU 去重，保留高分
        for (size_t i = 0; i < cands.size(); ++i) {
            for (size_t j = cands.size(); j-- > i + 1;) {
                if (RectIou(cands[i].rect, cands[j].rect) > 0.5) {
                    if (cands[j].score > cands[i].score) {
                        cands[i] = cands[j];
                    }
                    cands.erase(cands.begin() + (ptrdiff_t)j);
                }
            }
        }

        // 4) 按面积降序取前 max_count 个（一张图 1~2 个码），逐个过公共入口
        std::sort(cands.begin(), cands.end(),
                  [](const CodeCand& a, const CodeCand& b) { return a.area > b.area; });
        if ((int)cands.size() > aiCfg.max_count) {
            cands.resize((size_t)aiCfg.max_count);
        }
        for (const CodeCand& c : cands) {
            const cv::Rect& r = c.rect;
            const std::vector<cv::Point2f> quad = {
                {(float)r.x, (float)r.y},
                {(float)(r.x + r.width), (float)r.y},
                {(float)(r.x + r.width), (float)(r.y + r.height)},
                {(float)r.x, (float)(r.y + r.height)}};
            // 类型启发式：QR 码规范为方形，一维码必然细长
            const double aspect =
                (double)std::max(r.width, r.height) /
                (double)std::max(1, std::min(r.width, r.height));
            const CodeType type =
                aspect <= aiCfg.qr_aspect_tol ? CodeType::QR : CodeType::BAR;
            CodeRegion cr;
            if (AcceptCodeQuad(quad, totalAngleDeg, gray.size(), rotContour,
                               cfg.min_area_px, type, (double)c.score, cr)) {
                regions.push_back(cr);
            }
        }

        int qrCount = 0, barCount = 0;
        for (const CodeRegion& r : regions) {
            if (r.type == CodeType::QR) {
                ++qrCount;
            } else {
                ++barCount;
            }
            common::LogMsg(common::LINFO,
                           Fmt("  %s: (%.1f, %.1f) %.1fx%.1f, AI 置信度 %.2f",
                               r.type == CodeType::QR ? "二维码" : "一维码",
                               r.x, r.y, r.w, r.h, r.confidence));
        }
        common::LogMsg(common::LINFO,
                       Fmt("码区检测(AI): 二维码 %d 个, 一维码 %d 个"
                           "（过阈查询 %d 个%s）",
                           qrCount, barCount, (int)masks.size(),
                           desc.empty() ? "" : (", " + desc).c_str()));
        return true;
    } catch (const cv::Exception& e) {
        regions.clear();
        common::LogMsg(common::LWARN,
                       std::string("AI 码区检测异常（由调用方回退传统检测）：") + e.what());
        return false;
    } catch (const std::exception& e) {
        regions.clear();
        common::LogMsg(common::LWARN,
                       std::string("AI 码区检测异常（由调用方回退传统检测）：") + e.what());
        return false;
    }
}

}  // namespace cam
