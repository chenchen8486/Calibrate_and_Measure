#pragma once
// ============================================================================
// measure/rectify_angle.h —— 两级旋转校正（主方向投票粗校 + 亚像素边缘精校）
// ----------------------------------------------------------------------------
// 移植自 Python 工程 src/edge_measure/rectify.py：
//   粗校：approxPolyDP 多边形逼近，候选边方向角 mod 90° 归一化后边长加权
//         直方图投票，主峰边收集带内轮廓点 fitLine 精拟合；
//   精校：四侧逐行/列取掩膜极值点为原点沿外侧法线 SnapProfile 精化，
//         点数最多一侧 RANSAC 拟合直线，残余角取反作为补偿角。
// ============================================================================

#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "common/ini_config.h"

namespace cam {

// 主方向投票法估计产品摆放倾角（粗校正）。
//
// 产品主体存在多条互相平行/垂直的长直边，按 mod 90° 归一化后它们在方向
// 直方图上形成主峰；边长按弧长加权投票，孤立斜边无法成峰（替代单一最长边）。
//
// Args:
//   contour  外轮廓点集。
//   cfg      角度校正参数（approx_epsilon_ratio / edge_band）。
//   angleDeg 输出参数：直边倾角（度，相对水平方向，归一化 [-45,45)）。
//   errMsg   输出参数：失败时的中文错误描述。
//
// Returns:
//   成功返回 true；多边形逼近退化或无 ±8° 内候选边返回 false。
bool FindReferenceAngle(const std::vector<cv::Point>& contour,
                        const common::RotateConfig& cfg,
                        double& angleDeg, std::string& errMsg);

// 绕图像中心反向旋转，使基准边水平。
//
// Args:
//   gray     灰度图（增强图，INTER_LINEAR 双线性插值保持灰度连续）。
//   mask     产品掩膜（INTER_NEAREST 最近邻保持二值）。
//   angleDeg 当前倾角（度）；正值为逆时针倾斜，将以 -angleDeg 旋转校正。
//   rotGray  输出参数：旋转后的灰度图。
//   rotMask  输出参数：旋转后的掩膜。
void RotateImageAndMask(const cv::Mat& gray, const cv::Mat& mask, double angleDeg,
                        cv::Mat& rotGray, cv::Mat& rotMask);

// 精校正：用增强图亚像素边缘实测残余补偿角（两级对准的第二级）。
//
// 掩膜轮廓可能沿阴影/深色印刷走样（≠ 真实产品边缘），仅够粗校；本函数在
// 粗校正后的增强图上，对四侧分别采集亚像素边缘点并 RANSAC 拟合直线，取采样
// 点最多一侧的拟合方向作为残余补偿角。
//
// Args:
//   gray       粗校正后的增强图。
//   mask       粗校正后的产品掩膜。
//   measureCfg 测量参数（复用 ransac_threshold）。
//   refineCfg  精修参数（卡尺复用窗口阶跃检测器）。
//   minPoints  最少采样点数，不足则放弃精校。
//
// Returns:
//   需补偿的校正角（度）：与 FindReferenceAngle 的符号约定相反——返回"把当前
//   状态转到水平还需施加的角度"，与粗校角直接相加即可。采样不足或拟合失败
//   返回 0.0 并记 Warn。
double RefineAngleByEdges(const cv::Mat& gray, const cv::Mat& mask,
                          const common::MeasureConfig& measureCfg,
                          const common::RefineConfig& refineCfg, int minPoints = 30);

}  // namespace cam
