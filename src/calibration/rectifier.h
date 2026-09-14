#pragma once
// ============================================================================
// rectifier.h —— 正射矫正模块（功能 3 之几何矫正）
// 等价移植自 Python 版 build_rectify_maps / RectifyMaps：
//   加载标定 XML → 图像中心反投影求板面锚点 → 投影定方向符号（防镜像）
//   → 逐输出像素合成"去畸变 + 去倾斜 + 刻度对齐"的单张 remap 表
// 输出图为板面正射视图：输出像素 (u,v) 线性对应板面毫米
// （X = Xc ± (u-cx)·s，Y = Yc ± (v-cy)·s），再经 K/畸变/外参正投影回原图取色。
// ============================================================================

#include <string>

#include <opencv2/core.hpp>

#include "calibration/calibrator.h"

namespace cam {

// 正射矫正器：加载标定 XML 构建 remap 表（去畸变+去倾斜+刻度对齐一步完成）
class Rectifier {
public:
    // 加载标定 XML 并构建正射 remap 表
    // @param calibXmlPath  标定 XML 文件路径（功能 2 产物）
    // @param targetMmPerPx 正射输出图刻度（毫米/像素），须为正数
    // @param errMsg        输出参数：失败时的中文错误描述
    // @return 加载与映射表构建成功返回 true
    bool Load(const std::string& calibXmlPath, double targetMmPerPx, std::string& errMsg);

    // 查询矫正器是否就绪（Load 成功后才可调用 Rectify / BoardMmToPx）
    // @return 已就绪返回 true
    bool IsReady() const;

    // 把原始灰度图重映射为正射 metric 图
    // 输入 8UC1 灰度（尺寸须与标定 image_size 一致；8UC3/8UC4 会先转灰度并告警），
    // 输出正射灰度图，1px = MmPerPx()
    // @param gray 原始灰度图
    // @return 正射校正图；未就绪或尺寸不符时记错误日志并返回空 Mat
    cv::Mat Rectify(const cv::Mat& gray) const;

    // 板面毫米坐标 → 正射图像素坐标（下游把毫米结果画回图上用）
    // @param xMm 板面 X 坐标（毫米）
    // @param yMm 板面 Y 坐标（毫米）
    // @return 正射图像素坐标；未就绪时记错误日志并返回 (0,0)
    cv::Point2d BoardMmToPx(double xMm, double yMm) const;

    // 正射输出图刻度（毫米/像素）
    // @return 当前生效的 mm_per_px
    double MmPerPx() const;

private:
    CalibrationResult calib_;        // 标定参数（K/畸变/外参/尺寸/格距）
    cv::Mat map_x_, map_y_;          // cv::remap 映射表（CV_32FC1）
    double mm_per_px_ = 0.15;        // 输出正射图刻度
    double center_x_mm_ = 0.0;       // 输出图中心对应的板面毫米坐标 Xc
    double center_y_mm_ = 0.0;       // 输出图中心对应的板面毫米坐标 Yc
    double sign_x_ = 1.0;            // 板面 +X 在输出图 u 方向的符号（±1，防镜像）
    double sign_y_ = 1.0;            // 板面 +Y 在输出图 v 方向的符号（±1）
    bool ready_ = false;             // 映射表是否已就绪
};

}  // namespace cam
