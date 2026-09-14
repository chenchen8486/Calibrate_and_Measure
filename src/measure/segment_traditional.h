#pragma once
// ============================================================================
// measure/segment_traditional.h —— 传统背景差分分割
// ----------------------------------------------------------------------------
// 移植自 Python 工程 src/edge_measure/background.py 的 segment_product：
//   双边滤波 -> gamma 暗部提升 -> 全局亮度对齐 -> absdiff ->
//   Otsu 阈值（仅在 [diff_floor, diff_ceiling] 区间采纳）-> 形态学 ->
//   多连通域并集（面积占比过滤 + 贴边剔除 + 逐域外轮廓填孔）。
// ============================================================================

#include <string>

#include <opencv2/core.hpp>

#include "common/ini_config.h"

namespace cam {

// 背景差分分割产品掩膜。
//
// Args:
//   gray       待测灰度图（H, W），8UC1。
//   background 背景模型（H, W），8UC1。
//   cfg        传统分割参数。
//   mask       输出参数：产品二值掩膜（H, W），前景 255 背景 0。
//   alignedOut 可选输出：与测量同链的增强图（双边+gamma+亮度对齐），
//              供后续两级旋转校正与亚像素测量复用；不需要时传 nullptr。
//   errMsg     输出参数：失败时的中文错误描述。
//
// Returns:
//   成功返回 true；无连通域、连通域均过小/贴边、前景占比 < 1% 时
//   返回 false 并填充 errMsg（对应 Python 抛 ValueError 的分支）。
bool SegmentProduct(const cv::Mat& gray, const cv::Mat& background,
                    const common::TradSegConfig& cfg, cv::Mat& mask,
                    cv::Mat* alignedOut, std::string& errMsg);

}  // namespace cam
