#pragma once
// ============================================================================
// calibrator.h —— 功能 2：棋盘格标定求解
// 等价移植自 Python 版 src/edge_measure/calibration.py 与 scripts/calibrate.py：
//   SB 角点检测 + cornerSubPix 亚像素细化 → 多视图角点均值降噪
//   → 单视图 calibrateCamera（fx=fy、主点固定图像中心）→ XML 参数落盘
//   → 正射验证闭环（校正图重检角点回理想网格）→ QA 质检图
// 坐标链：原始图像素 --remap--> 正射 metric 图（1px = mm_per_px，与板面毫米整列对齐）
//
// 职责划分说明（与 Python 版 run_calibration 总行为一致）：
//   - SolveAndSaveCalibration 只持有角点、不持有源图，负责：均值 → 求解 →
//     XML 落盘 → RMS 质量门禁；
//   - 正射验证闭环与 4 张 QA 图均依赖原始采图，统一由对外唯一入口
//     BuildCalibrationFile 驱动完成，总质量门禁（rms>0.3 或 mean_px>0.5）
//     也在 BuildCalibrationFile 收口判定。
// ============================================================================

#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "common/ini_config.h"

namespace cam {

// ---------------------------------------------------------------------------
// 标定结果数据结构（与 XML 落盘字段一一对应；created 时间串仅写入 XML，
// 不回读进本结构体）
// ---------------------------------------------------------------------------
struct CalibrationResult {
    cv::Mat camera_matrix;   // 3x3 CV_64F（fx=fy，主点固定图像中心）
    cv::Mat dist_coeffs;     // 1x5 CV_64F：k1,k2,p1,p2,k3
    cv::Mat rvec, tvec;      // 3x1 CV_64F，板面外参（tvec 单位 mm）
    cv::Size image_size;
    double square_x_mm = 15.0, square_y_mm = 15.0;  // 实测格距
    cv::Size pattern_size{34, 27};                    // 内角点
    double rms = 0.0;
    double mm_per_px = 0.15;                          // 正射刻度
};

// 单图角点检测：SB 算法 + cornerSubPix 亚像素细化（窗口 21）
// 输入任意 8UC1/8UC3/8UC4（内部 ToGray8）；失败返回 false 并给中文原因
// @param image       棋盘格采图（8/24/32 通道均可）
// @param patternSize 内角点数（cols x rows）
// @param corners     输出参数：亚像素角点（行优先，与物点网格顺序一致）
// @param errMsg      输出参数：失败时的中文错误描述
// @return 检测成功且数量等于 patternSize 面积时返回 true
bool DetectBoardCorners(const cv::Mat& image, cv::Size patternSize,
                        std::vector<cv::Point2f>& corners, std::string& errMsg);

// 多视图（固定机位采图取均值等价降噪）求解标定并保存 XML
// cornersList 每张图一组角点（顺序须与 DetectBoardCorners 输出一致）；单图即自身
// 内部：输入校验 → 均值角点 → calibrateCamera(CALIB_FIX_ASPECT_RATIO|
//       CALIB_FIX_PRINCIPAL_POINT) → RMS 分级日志 → XML 落盘
// 质量门禁（本函数负责 rms 部分）：rms>0.3 时返回 false（文件仍写出，
// errMsg 说明质量偏低）；验证 mean_px>0.5 的门禁由 BuildCalibrationFile 完成
// @param cornersList 各标定图的角点集合（至少 1 张，各张数量须一致）
// @param imageSize   标定图尺寸（宽 x 高，像素）
// @param cfg         [calibrate] 段配置（内角点/实测格距/输出 XML/QA 目录/正射刻度）
// @param errMsg      输出参数：失败时的中文错误描述
// @return 求解成功且 rms 达标返回 true
bool SolveAndSaveCalibration(const std::vector<std::vector<cv::Point2f>>& cornersList,
                             cv::Size imageSize,
                             const common::CalibrateConfig& cfg,
                             std::string& errMsg);

// 功能 2 对外唯一入口：给定一张棋盘格采图（8/24/32 通道），求解并生成标定 XML
// 参数全部读取 iniPath 的 [calibrate] 段
// 内部：读配置 → ToGray8 → 角点检测 → SolveAndSaveCalibration → 回读 XML 自检
//       → Rectifier 构建正射映射 → 4 张 QA 图落 cfg.qa_dir → 验证闭环统计
//       → 总质量门禁（rms>0.3 或验证 mean_px>0.5 时返回 false，文件与 QA 图仍输出）
// @param boardImage 棋盘格标定板采图（吸风展平、生产同条件）
// @param iniPath    全工程 ini 配置文件路径
// @param errMsg     输出参数：失败或质量偏低时的中文描述
// @return 标定成功且质量达标返回 true
bool BuildCalibrationFile(const cv::Mat& boardImage, const std::string& iniPath, std::string& errMsg);

// 功能 2 多图入口：给定多张棋盘格采图（固定机位），角点取均值后求解并生成标定 XML
// 多张采图逐张 DetectBoardCorners，任一失败即整体失败并在 errMsg 中报告是哪一张；
// 各张角点取均值等价降噪；求解/落盘/QA/验证闭环与单图入口 BuildCalibrationFile
// 完全一致，QA 质检图与验证闭环使用第一张图作为代表图
// @param boardImages 棋盘格标定板采图列表（8/24/32 通道均可，尺寸须一致；空列表返回 false）
// @param iniPath     全工程 ini 配置文件路径
// @param errMsg      输出参数：失败或质量偏低时的中文描述
// @return 标定成功且质量达标返回 true
bool BuildCalibrationFileFromImages(const std::vector<cv::Mat>& boardImages,
                                    const std::string& iniPath, std::string& errMsg);

// 加载标定 XML（功能 3 矫正模块使用）
// 读取后做完整性校验与类型归一：camera_matrix 3x3 CV_64F、dist_coeffs 1x5 CV_64F、
// rvec/tvec 3x1 CV_64F
// @param xmlPath 标定 XML 文件路径
// @param out     输出参数：标定结果
// @param errMsg  输出参数：失败时的中文错误描述
// @return 加载且校验通过返回 true
bool LoadCalibrationXml(const std::string& xmlPath, CalibrationResult& out, std::string& errMsg);

}  // namespace cam
