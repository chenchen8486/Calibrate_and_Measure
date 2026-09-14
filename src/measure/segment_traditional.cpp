// ============================================================================
// measure/segment_traditional.cpp —— 传统背景差分分割实现
// 等价移植自 Python 工程 src/edge_measure/background.py:106-216
// （segment_product + _filter_components）
// ============================================================================

#include "measure/segment_traditional.h"

#include <algorithm>
#include <cstdarg>
#include <cstdio>

#include <opencv2/imgproc.hpp>

#include "common/logger.h"
#include "measure/edge_refine.h"

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

}  // namespace

bool SegmentProduct(const cv::Mat& gray, const cv::Mat& background,
                    const common::TradSegConfig& cfg, cv::Mat& mask,
                    cv::Mat* alignedOut, std::string& errMsg) {
    // 1) 与测量同链的增强图（双边滤波 + gamma + 亮度对齐）；
    //    eBg 为背景模型经同样双边+gamma 处理后的版本
    cv::Mat eBg;
    cv::Mat aligned = BuildEnhanced(gray, background, cfg, &eBg);

    // 2) absdiff -> Otsu 阈值仅在 [diff_floor, diff_ceiling] 区间采纳（clip）
    cv::Mat diff, otsuDst, maskThresh;
    cv::absdiff(aligned, eBg, diff);
    const double otsu = cv::threshold(diff, otsuDst, 0, 255,
                                      cv::THRESH_BINARY | cv::THRESH_OTSU);
    const double thresh = std::clamp(otsu, cfg.diff_floor, cfg.diff_ceiling);
    cv::threshold(diff, maskThresh, thresh, 255, cv::THRESH_BINARY);
    common::LogMsg(common::LDEBUG,
                   Fmt("差分阈值: otsu=%.1f, 采用=%.1f, 差分均值=%.2f", otsu, thresh,
                       cv::mean(diff)[0]));

    // 3) 形态学：开运算去细小噪点、闭运算填补前景内部空洞
    const cv::Mat kOpen = cv::getStructuringElement(
        cv::MORPH_ELLIPSE, cv::Size(cfg.morph_open_kernel, cfg.morph_open_kernel));
    const cv::Mat kClose = cv::getStructuringElement(
        cv::MORPH_ELLIPSE, cv::Size(cfg.morph_close_kernel, cfg.morph_close_kernel));
    cv::Mat morph;
    cv::morphologyEx(maskThresh, morph, cv::MORPH_OPEN, kOpen);
    cv::morphologyEx(morph, morph, cv::MORPH_CLOSE, kClose);

    // 4) 连通域过滤：保留面积占比 >= min_area_ratio 且不贴图边的连通域，
    //    各保留域 findContours + drawContours(FILLED) 填孔取并集
    const int h = morph.rows, w = morph.cols;
    cv::Mat labels, stats, centroids;
    const int num = cv::connectedComponentsWithStats(morph, labels, stats, centroids, 8,
                                                     CV_32S);
    if (num <= 1) {
        errMsg = "分割失败：掩膜中没有连通域";
        common::LogMsg(common::LERROR, errMsg);
        return false;
    }
    cv::Mat keep = cv::Mat::zeros(h, w, CV_8UC1);
    int kept = 0;
    for (int label = 1; label < num; ++label) {
        const double areaRatio =
            stats.at<int>(label, cv::CC_STAT_AREA) / ((double)h * (double)w);
        if (areaRatio < cfg.min_area_ratio) {
            continue;
        }
        const int x  = stats.at<int>(label, cv::CC_STAT_LEFT);
        const int y  = stats.at<int>(label, cv::CC_STAT_TOP);
        const int cw = stats.at<int>(label, cv::CC_STAT_WIDTH);
        const int ch = stats.at<int>(label, cv::CC_STAT_HEIGHT);
        if (x <= 1 || y <= 1 || x + cw >= w - 1 || y + ch >= h - 1) {
            common::LogMsg(common::LDEBUG,
                           Fmt("剔除贴边连通域 %d（面积占比 %.2f%%）", label,
                               areaRatio * 100.0));
            continue;
        }
        cv::Mat component;
        cv::compare(labels, label, component, cv::CMP_EQ);
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(component.clone(), contours, cv::RETR_EXTERNAL,
                         cv::CHAIN_APPROX_NONE);
        cv::drawContours(keep, contours, -1, cv::Scalar(255), cv::FILLED);
        ++kept;
    }
    if (kept == 0) {
        errMsg = "分割失败：连通域均过小或均为贴边杂物";
        common::LogMsg(common::LERROR,
                       Fmt("分割失败：所有连通域面积占比均低于 %.3f%% 或均为贴边杂物",
                           cfg.min_area_ratio * 100.0));
        return false;
    }
    common::LogMsg(common::LDEBUG, Fmt("连通域过滤：保留 %d / %d 个区域", kept, num - 1));

    const double coverage = cv::countNonZero(keep) / ((double)h * (double)w);
    common::LogMsg(common::LINFO, Fmt("分割完成：前景占比 %.1f%%", coverage * 100.0));
    if (coverage < 0.01) {
        errMsg = "分割失败：未找到有效产品区域（前景占比过低，疑似无产品或背板已变更）";
        common::LogMsg(common::LERROR,
                       Fmt("分割失败：前景占比过低 %.3f%%", coverage * 100.0));
        return false;
    }
    mask = keep;
    if (alignedOut != nullptr) {
        *alignedOut = aligned;
    }
    return true;
}

}  // namespace cam
