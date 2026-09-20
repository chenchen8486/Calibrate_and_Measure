#pragma once
// ============================================================================
// measure/background.h —— 背景建模与外轮廓提取
// ----------------------------------------------------------------------------
// 移植自 Python 工程 src/edge_measure/background.py：
//   现场前提：背板为常量（同一批次固定不动；换背板时重新建模即可）。
//   无空背板图时，用多帧产品图逐像素取中位数合成背景模型 —— 产品位置随机，
//   中位数天然落在背板上；再用时序 MAD 置信度修剪 + 双二次曲面外推产品覆盖区。
// ============================================================================

#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace cam {

// 时序置信度修剪 + 低频曲面重建的背景建模。
//
// 流程：全图堆叠逐像素中位数 -> MAD < madThreshold 判为高置信背板像素
//       （占比 < 20% 报错）-> 高置信点（cv::RNG(42) 抽稀至 fitSamples）拟合
//       双二次曲面 z = a·x² + b·y² + c·xy + d·x + e·y + f（坐标归一化，
//       cv::solve DECOMP_SVD）-> 高置信区保留实测中值、其余曲面外推
//       -> clip [0,255] -> 8UC1 -> 写缓存。
//
// Args:
//   imagePaths   参与建模的图像路径列表（建议 >= 5 张，产品位置各异）。
//   cachePath    背景模型缓存路径；为空字符串时不写缓存。
//   background   输出参数：背景模型灰度图（H, W），8UC1。
//   errMsg       输出参数：失败时的中文错误描述。
//   madThreshold 高置信背板像素的 MAD 上限（灰度级）。
//   fitSamples   参与曲面拟合的最大采样点数（随机抽稀加速）。
//
// Returns:
//   建模成功返回 true；图像列表为空、尺寸不一致、高置信像素过少或
//   曲面求解失败时返回 false 并填充 errMsg。
bool BuildBackgroundModel(const std::vector<std::string>& imagePaths,
                          const std::string& cachePath,
                          cv::Mat& background, std::string& errMsg,
                          double madThreshold = 3.0, int fitSamples = 50000);

// 优先加载缓存的背景模型；不存在或读取失败则从图集建模。
//
// Args:
//   imagePaths 输入图集路径列表。
//   cachePath  背景模型缓存路径。
//   background 输出参数：背景模型灰度图。
//   errMsg     输出参数：失败时的中文错误描述。
//
// Returns:
//   成功返回 true。
bool LoadOrBuildBackground(const std::vector<std::string>& imagePaths,
                           const std::string& cachePath,
                           cv::Mat& background, std::string& errMsg);

// 仅加载缓存背景模型，不现建（生产交付路径的唯一文件来源）。
//
// Args:
//   cachePath  背景模型缓存路径。
//   background 输出参数：背景模型灰度图（8UC1）。
//   errMsg     输出参数：失败时的中文错误描述。
//
// Returns:
//   缓存存在且读取成功返回 true；不存在或读取失败返回 false 并填充 errMsg。
bool LoadBackgroundCache(const std::string& cachePath,
                         cv::Mat& background, std::string& errMsg);

// 提取掩膜的最大面积外轮廓（findContours RETR_EXTERNAL + CHAIN_APPROX_NONE）。
//
// Args:
//   mask    二值掩膜（8UC1，前景 255）。
//   contour 输出参数：轮廓点集。
//   errMsg  输出参数：失败时的中文错误描述。
//
// Returns:
//   成功返回 true；掩膜中无轮廓返回 false。
bool ExtractOuterContour(const cv::Mat& mask, std::vector<cv::Point>& contour,
                         std::string& errMsg);

}  // namespace cam
