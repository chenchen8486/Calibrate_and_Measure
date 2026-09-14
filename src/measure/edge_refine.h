#pragma once
// ============================================================================
// measure/edge_refine.h —— 增强图构建与掩膜边缘法线卡尺精修
// ----------------------------------------------------------------------------
// 移植自 Python 工程 src/edge_measure/edge_refine.py：
//   吸附目标图 = 与分割/测量同链的增强图（双边滤波 + gamma + 亮度对齐），
//   保证"分割-精修-测量"三个环节看到的是同一份边缘定义。
// 注意：C++ 侧 RefineConfig 无 detector 字段，固定使用 step 窗口均值差检测器
//       （Python 默认且推荐的检测器，软边/离焦缓坡可见）。
// ============================================================================

#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "common/ini_config.h"

namespace cam {

// 双边滤波 + gamma LUT 基础增强（亮度对齐前的公共步骤）。
// LUT 按 (i/255)^gamma * 255 截断取整，与 Python np.astype(uint8) 一致。
//
// Args:
//   img 8UC1 灰度图。
//   cfg 传统分割参数（复用其中的双边/gamma 配置）。
//
// Returns:
//   增强后的 8UC1 灰度图。
cv::Mat ApplyBilateralGamma(const cv::Mat& img, const common::TradSegConfig& cfg);

// 生成测量基准增强图（与 segment_product 前处理链保持一致）：
// 双边滤波（保边降噪）-> gamma 暗部提升 -> 整图中位数亮度对齐（照明漂移补偿）。
//
// Args:
//   gray        原灰度图（H, W）。
//   background  背景模型（H, W）。
//   cfg         传统分割参数（复用其中的双边/gamma 配置）。
//   enhancedBg  可选输出：背景模型经同样双边+gamma 处理后的图（传统分割差分用）。
//
// Returns:
//   增强灰度图（H, W），8UC1。
cv::Mat BuildEnhanced(const cv::Mat& gray, const cv::Mat& background,
                      const common::TradSegConfig& cfg, cv::Mat* enhancedBg = nullptr);

// 沿法线剖面搜索边缘并返回亚像素吸附偏移（step 窗口均值差检测器）。
//
// 梯度符号约束：由剖面内/外段均值决定期望符号（内亮外暗期望负边缘），只接受
// 符号匹配的峰；合格区按连通段取各段峰顶，选离原点最近的一段（防阴影带拉走）；
// 抛物线插值得亚像素偏移，弱边缘（信号不足）保持原点不动。
//
// Args:
//   enhanced 增强灰度图（BuildEnhanced 产物）。
//   origin   剖面原点 (x, y)。
//   normal   指向掩膜外侧的单位法线。
//   cfg      精修参数。
//
// Returns:
//   沿法线的亚像素偏移（像素，正 = 向外）；无合格边缘时返回 0.0。
double SnapProfile(const cv::Mat& enhanced, const cv::Point2d& origin,
                   const cv::Point2d& normal, const common::RefineConfig& cfg);

// 边缘精修调试信息（对应 Python refine_mask 的 origins/refined/deltas 返回值）
struct RefineDebugInfo {
    std::vector<cv::Point2d> origins;  // 采样原点点集
    std::vector<cv::Point2d> refined;  // 精修后点集
    std::vector<double>      deltas;   // 各点法向带符号吸附偏移（正 = 向外）
};

// 掩膜边缘精修入口：外轮廓每 sample_step 采样 -> 定向法线 -> 逐点 SnapProfile
// -> 吸附偏移循环滑窗中值去野（窗口 median_window，偏差 > outlier_tol 回退中值）
// -> fillPoly 重建掩膜。
//
// Args:
//   mask        待精修二值掩膜（H, W），前景 255。
//   enhanced    增强灰度图（BuildEnhanced 产物）。
//   cfg         精修参数。
//   refinedMask 输出参数：精修后的二值掩膜。
//   debug       可选输出：采样原点/精修点/偏移，供调试用；不需要时传 nullptr。
//   errMsg      输出参数：失败时的中文错误描述。
//
// Returns:
//   成功返回 true；掩膜中无轮廓返回 false。
bool RefineMask(const cv::Mat& mask, const cv::Mat& enhanced,
                const common::RefineConfig& cfg, cv::Mat& refinedMask,
                RefineDebugInfo* debug, std::string& errMsg);

}  // namespace cam
