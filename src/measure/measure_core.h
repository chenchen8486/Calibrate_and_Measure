#pragma once
// ============================================================================
// measure/measure_core.h —— 亚像素测量核心（宽高 + 水平边排名）
// ----------------------------------------------------------------------------
// 移植自 Python 工程 src/edge_measure/measure.py 与 measure_ranked_edges.py：
//   - 宽高：掩膜轮廓语义极值定界（含耳片），窗口阶跃卡尺亚像素精化；
//   - 水平边排名：切线角筛选 + 种子 run 合并 + 逐 run RANSAC 拟合 +
//     全轮廓投影延展 + 同边去重 + 上半部分区域过滤；
//   - RANSAC/最小二乘直线拟合与角度归一化放在本模块，供 rectify_angle 复用。
// ============================================================================

#include <cmath>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "common/ini_config.h"
#include "measure_types.h"

namespace cam {

// 直线拟合结果（对应 Python measure.py 的 LineResult）
struct LineResult {
    cv::Point2d direction{0.0, 0.0};  // 单位方向向量
    cv::Point2d point{0.0, 0.0};      // 直线上一点
    double      inlierRatio = 0.0;    // RANSAC 内点率（拟合质量指标）
    int         numPoints   = 0;      // 采样点数
};

// 角度归一化到 [-45, 45)（mod 90° 意义下最接近 0 的表示）。
// 与 Python (a + 45) % 90 - 45 等价（Python 取模结果非负）。
inline double NormalizeAngle90(double a) {
    a = std::fmod(a + 45.0, 90.0);
    if (a < 0.0) {
        a += 90.0;
    }
    return a - 45.0;
}

// 最小二乘直线拟合（cv::fitLine DIST_L2 封装）。
//
// Args:
//   points 点集 (x, y)。
//   out    输出参数：拟合结果（direction 为单位向量）。
//   errMsg 输出参数：失败时的中文错误描述。
//
// Returns:
//   成功返回 true；点数不足或方向向量退化返回 false。
bool FitLineLeastSquares(const std::vector<cv::Point2d>& points, LineResult& out,
                         std::string& errMsg);

// 对 2D 点集做 RANSAC 直线拟合（OpenCV 无直接接口，手动实现）。
// 迭代次数 min(200, max(50, 2N))，固定随机源 cv::RNG(42)，内点再 fitLine 精化。
//
// Args:
//   points    点集 (x, y)，至少 5 个。
//   threshold 内点距离阈值（像素）。
//   out       输出参数：拟合结果（含内点率）。
//   errMsg    输出参数：失败时的中文错误描述。
//
// Returns:
//   成功返回 true；点数不足返回 false。
bool RansacLine(const std::vector<cv::Point2d>& points, double threshold,
                LineResult& out, std::string& errMsg);

// 测量产品总宽（左右语义极值线间距）与总高（上下语义极值线间距）。
//
// 两步：1) 掩膜轮廓极值带（extreme_band）内的轮廓点给出"哪条边算边"的语义答案，
//        沿扫描轴 10%~90% 分位区间均匀取 refine_rows 行；
//       2) 每行以轮廓极值为原点、沿外侧法线发窗口阶跃卡尺（SnapProfile），
//        亚像素定位后取中位数。
// 输出四条边界线按 payload.py 约定围成闭合测量矩形（竖线纵向跨 [yTop,yBottom]，
// 横线横向跨 [xLeft,xRight]），坐标为角度校正后图坐标系，未做小数位截断
// （保留位数由 measure_pipeline 组装输出时统一处理）。
//
// Args:
//   rotAligned 校正后的增强灰度图。
//   rotMask    校正后的产品掩膜。
//   cfg        测量参数。
//   refineCfg  精修参数（卡尺复用窗口阶跃检测器）。
//   width      输出参数：宽度结果。
//   height     输出参数：高度结果。
//   errMsg     输出参数：失败时的中文错误描述。
//
// Returns:
//   成功返回 true；掩膜无轮廓返回 false。
bool MeasureWidthHeight(const cv::Mat& rotAligned, const cv::Mat& rotMask,
                        const common::MeasureConfig& cfg,
                        const common::RefineConfig& refineCfg,
                        WidthResult& width, HeightResult& height, std::string& errMsg);

// 水平边高度排名测量：返回上半部分全部候选边（先排名后判定，阈值不淘汰只判定）。
//
// 流程：轮廓 ±3 点循环平滑 -> ±tangent_span 差分切线角，|角|<=angle_tol_deg 或
//       >= 180-angle_tol_deg 入围 -> 连续入围种子 run（相邻间隙 <= merge_gap_pts
//       且高度差 <= 2·line_dist_tol 合并，含首尾环绕）-> 各 run RANSAC 拟合 ->
//       全部轮廓点投影到该直线 perp<=line_dist_tol 入选、沿直线排序、
//       断档 > span_gap_tol 切开、取与种子交叠连续段、端点=段端亚像素投影 ->
//       同边去重（y 中心差 <= 2·line_dist_tol 且 x 交叠 >= 30% 较短边）->
//       y_center > 上半部分分界（top + ranked_region_ratio·(bottom-top)）丢弃。
//
// Args:
//   rotContour 校正后的外轮廓点集。
//   cfg        测量参数。
//   candidates 输出参数：候选水平边列表，按 (y_center, x_left) 排序；
//              satisfied = width >= edge_width_threshold。
//   errMsg     输出参数：失败时的中文错误描述。
//
// Returns:
//   成功返回 true（无候选边也返回 true，candidates 为空）；轮廓为空返回 false。
bool MeasureRankedEdges(const std::vector<cv::Point>& rotContour,
                        const common::MeasureConfig& cfg,
                        std::vector<HorizontalEdge>& candidates, std::string& errMsg);

}  // namespace cam
