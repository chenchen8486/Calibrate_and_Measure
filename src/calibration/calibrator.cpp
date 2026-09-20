// ============================================================================
// calibrator.cpp —— 功能 2：棋盘格标定求解实现
// 逐行等价移植自 Python 版：
//   src/edge_measure/calibration.py（detect_corners / solve_calibration /
//   reproj_residuals / validate_calibration）
//   scripts/calibrate.py（4 张 QA 质检图与退出码质量门禁）
// ============================================================================

#include "calibrator.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <string>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "common/image_io.h"
#include "common/logger.h"
#include "rectifier.h"

namespace {

// ---------------------------------------------------------------------------
// 常量（与 Python 版 _RMS_GOOD/_RMS_WARN 及 scripts 层门禁一致）
// ---------------------------------------------------------------------------
constexpr double kRmsGood = 0.1;          // RMS 重投影误差达标线（px）
constexpr double kRmsWarn = 0.3;          // 超过该值告警并触发质量门禁（px）
constexpr double kValidateMeanWarn = 0.5; // 正射验证 mean 残差门禁（px）
constexpr int kSubPixWindow = 21;         // cornerSubPix 窗口半径（像素）
constexpr double kResidualScale = 50.0;   // QA 残差矢量放大倍数
constexpr double kArrowTipLength = 8.0;   // QA 残差箭头尖端长度（对齐 Python 版入参）

// 格式化浮点数（%.nf），供日志与 QA 图标注使用
std::string FmtDouble(double v, int precision) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.*f", precision, v);
    return buf;
}

// 构造物理角点网格（z=0 平面，行优先 grid[r*cols+c]=(c*sx, r*sy, 0)，单位 mm）
// 与 Python 版 _object_grid 一致；用实测格距吸收打印机全局缩放与走纸各向异性
std::vector<cv::Point3f> BuildObjectGrid(double sx, double sy, cv::Size pattern) {
    std::vector<cv::Point3f> grid;
    grid.reserve(static_cast<size_t>(pattern.width) * pattern.height);
    for (int r = 0; r < pattern.height; ++r) {
        for (int c = 0; c < pattern.width; ++c) {
            grid.emplace_back(static_cast<float>(c * sx), static_cast<float>(r * sy), 0.0f);
        }
    }
    return grid;
}

// 当前时间串（ISO 格式，写入 XML created 字段）
std::string NowTimeString() {
    const std::time_t t = std::time(nullptr);
    std::tm tm{};
    localtime_s(&tm, &t);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S", &tm);
    return buf;
}

// 标定结果写 XML（cv::FileStorage；字段与 CalibrationResult 一一对应 + created 时间串）
// @param xmlPath 输出 XML 绝对路径（父目录不存在时自动创建；工程约定路径均为 ASCII）
// @param calib   标定结果
// @param errMsg  输出参数：失败时的中文错误描述
// @return 写入成功返回 true
bool SaveCalibrationXml(const std::string& xmlPath, const cam::CalibrationResult& calib,
                        std::string& errMsg) {
    std::error_code ec;
    const std::filesystem::path parent = std::filesystem::path(xmlPath).parent_path();
    if (!parent.empty()) {
        std::filesystem::create_directories(parent, ec);
    }
    if (ec) {
        errMsg = "创建标定输出目录失败: " + parent.string() + "，" + ec.message();
        common::LogMsg(common::LERROR, errMsg);
        return false;
    }
    cv::FileStorage fs;
    try {
        fs.open(xmlPath, cv::FileStorage::WRITE);
    } catch (const cv::Exception& e) {
        errMsg = "标定文件写入异常: " + xmlPath + "，" + e.what();
        common::LogMsg(common::LERROR, errMsg);
        return false;
    }
    if (!fs.isOpened()) {
        errMsg = "标定文件无法写入: " + xmlPath;
        common::LogMsg(common::LERROR, errMsg);
        return false;
    }
    fs << "camera_matrix" << calib.camera_matrix;
    fs << "dist_coeffs" << calib.dist_coeffs;
    fs << "rvec" << calib.rvec;
    fs << "tvec" << calib.tvec;
    fs << "image_width" << calib.image_size.width;
    fs << "image_height" << calib.image_size.height;
    fs << "square_x_mm" << calib.square_x_mm;
    fs << "square_y_mm" << calib.square_y_mm;
    fs << "pattern_cols" << calib.pattern_size.width;
    fs << "pattern_rows" << calib.pattern_size.height;
    fs << "rms" << calib.rms;
    fs << "mm_per_px" << calib.mm_per_px;
    fs << "created" << NowTimeString();
    fs.release();
    common::LogMsg(common::LINFO, "标定参数已保存: " + xmlPath);
    return true;
}

// QA 图上绘制标注文字（cv::putText 不支持中文，QA 图统一用 ASCII 标注；
// 位置/颜色对齐 Python 版 _put_text：org(40,60)、BGR(0,220,220)、字号约 40px）
void PutQaText(cv::Mat& canvas, const std::string& text) {
    cv::putText(canvas, text, cv::Point(40, 60), cv::FONT_HERSHEY_SIMPLEX,
                1.3, cv::Scalar(0, 220, 220), 2, cv::LINE_AA);
}

// numpy.percentile(err, 95) 等价实现（线性插值）
double Percentile95(std::vector<double> vals) {
    if (vals.empty()) {
        return 0.0;
    }
    std::sort(vals.begin(), vals.end());
    const double pos = 0.95 * static_cast<double>(vals.size() - 1);
    const size_t lo = static_cast<size_t>(pos);
    const size_t hi = std::min(lo + 1, vals.size() - 1);
    const double frac = pos - static_cast<double>(lo);
    return vals[lo] + frac * (vals[hi] - vals[lo]);
}

// 均值（空数组返回 0）
double MeanOf(const std::vector<double>& vals) {
    if (vals.empty()) {
        return 0.0;
    }
    double sum = 0.0;
    for (double v : vals) {
        sum += v;
    }
    return sum / static_cast<double>(vals.size());
}

// 最大值（空数组返回 0）
double MaxOf(const std::vector<double>& vals) {
    if (vals.empty()) {
        return 0.0;
    }
    return *std::max_element(vals.begin(), vals.end());
}

// 每行/列角点拟合直线后的最大垂直偏差（px），对应 Python 版 _straightness
// @param corners 正射图上重检出的角点（浮点像素坐标）
// @param lines   若干组角点下标，每组为同一行或同一列
// @return 所有行/列中的最大垂直偏差（px）
double Straightness(const std::vector<cv::Point2d>& corners,
                    const std::vector<std::vector<int>>& lines) {
    double worst = 0.0;
    for (const auto& idx : lines) {
        std::vector<cv::Point2f> pts;
        pts.reserve(idx.size());
        for (int i : idx) {
            pts.emplace_back(static_cast<float>(corners[i].x), static_cast<float>(corners[i].y));
        }
        cv::Vec4f line;
        cv::fitLine(pts, line, cv::DIST_L2, 0, 0.01, 0.01);
        const double vx = line[0], vy = line[1], x0 = line[2], y0 = line[3];
        for (int i : idx) {
            const double d = std::fabs((corners[i].x - x0) * vy - (corners[i].y - y0) * vx);
            worst = std::max(worst, d);
        }
    }
    return worst;
}

// 正射验证闭环统计量（对应 Python 版 validate_calibration 的 stats）
struct ValidateStats {
    double mean_px = 0.0;
    double p95_px = 0.0;
    double max_px = 0.0;
    double mean_mm = 0.0;
    double max_mm = 0.0;
    double row_straightness_max_px = 0.0;
    double col_straightness_max_px = 0.0;
};

// 多视图角点取均值（固定机位下多张静态图是噪声重复观测，均值等价降噪）
// 调用方须保证 cornersList 非空且各张角点数量一致
std::vector<cv::Point2f> MeanCorners(
    const std::vector<std::vector<cv::Point2f>>& cornersList) {
    const size_t n = cornersList.front().size();
    std::vector<cv::Point2f> mean(n, cv::Point2f(0.f, 0.f));
    for (const auto& c : cornersList) {
        for (size_t i = 0; i < n; ++i) {
            mean[i] += c[i];
        }
    }
    const float inv = 1.0f / static_cast<float>(cornersList.size());
    for (auto& p : mean) {
        p *= inv;
    }
    return mean;
}

// 读取 [calibrate] 段配置（路径字段已被 AppConfig::LoadFromIni 解析为绝对路径）；
// 未配置实测格距时按名义 15.0mm 计算并告警
// @param iniPath 全工程 ini 配置文件路径
// @param cfg     输出参数：[calibrate] 段配置
// @param errMsg  输出参数：失败时的中文错误描述
// @return 加载成功返回 true
bool LoadCalibrateConfig(const std::string& iniPath, common::CalibrateConfig& cfg,
                         bool& saveQa, std::string& errMsg) {
    common::AppConfig appCfg;
    std::string cfgErr;
    if (!appCfg.LoadFromIni(iniPath, cfgErr)) {
        errMsg = "读取标定配置失败: " + cfgErr;
        common::LogMsg(common::LERROR, errMsg);
        return false;
    }
    cfg = appCfg.calibrate;
    saveQa = appCfg.debug.save_intermediate;
    if (cfg.square_x_mm == 15.0 && cfg.square_y_mm == 15.0) {
        common::LogMsg(common::LWARN,
                       "未配置实测格距，按名义 15.0mm 计算；"
                       "打印缩放误差将直接进入测量结果，务必实测后重标");
    }
    return true;
}

}  // namespace

namespace cam {

bool DetectBoardCorners(const cv::Mat& image, cv::Size patternSize,
                        std::vector<cv::Point2f>& corners, std::string& errMsg) {
    corners.clear();
    errMsg.clear();
    if (patternSize.width <= 0 || patternSize.height <= 0) {
        errMsg = "内角点配置非法: " + std::to_string(patternSize.width) + "x" +
                 std::to_string(patternSize.height);
        common::LogMsg(common::LERROR, errMsg);
        return false;
    }
    cv::Mat gray = common::ToGray8(image);
    if (gray.empty()) {
        errMsg = "棋盘格图像格式不支持（须为 8UC1/8UC3/8UC4）";
        common::LogMsg(common::LERROR, "角点检测失败: " + errMsg);
        return false;
    }
    std::vector<cv::Point2f> found;
    bool ok = false;
    try {
        ok = cv::findChessboardCornersSB(gray, patternSize, found, cv::CALIB_CB_NORMALIZE_IMAGE);
    } catch (const cv::Exception& e) {
        errMsg = "棋盘格检测异常: " + std::string(e.what());
        common::LogMsg(common::LERROR, errMsg);
        return false;
    }
    const int expect = patternSize.width * patternSize.height;
    if (!ok || static_cast<int>(found.size()) != expect) {
        errMsg = "未检测到完整棋盘格（目标 " + std::to_string(patternSize.width) + "x" +
                 std::to_string(patternSize.height) + "=" + std::to_string(expect) +
                 " 个内角点），请检查标定板摆放与成像";
        common::LogMsg(common::LERROR, errMsg);
        return false;
    }
    try {
        cv::cornerSubPix(gray, found, cv::Size(kSubPixWindow, kSubPixWindow), cv::Size(-1, -1),
                         cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER,
                                          30, 0.01));
    } catch (const cv::Exception& e) {
        errMsg = "角点亚像素细化异常: " + std::string(e.what());
        common::LogMsg(common::LERROR, errMsg);
        return false;
    }
    corners = std::move(found);
    common::LogMsg(common::LINFO, "棋盘格角点检测成功: " + std::to_string(corners.size()) + " 个");
    return true;
}

bool SolveAndSaveCalibration(const std::vector<std::vector<cv::Point2f>>& cornersList,
                             cv::Size imageSize,
                             const common::CalibrateConfig& cfg,
                             std::string& errMsg) {
    errMsg.clear();
    const cv::Size pattern(cfg.pattern_cols, cfg.pattern_rows);
    const size_t expect = static_cast<size_t>(pattern.width) * pattern.height;

    // ---- 输入校验 ----
    if (cornersList.empty()) {
        errMsg = "无角点集合，无法标定";
        common::LogMsg(common::LERROR, errMsg);
        return false;
    }
    const size_t n = cornersList.front().size();
    for (const auto& c : cornersList) {
        if (c.size() != n) {
            errMsg = "各图角点数量不一致";
            common::LogMsg(common::LERROR, errMsg);
            return false;
        }
    }
    if (n != expect) {
        errMsg = "角点数量 " + std::to_string(n) + " 与内角点配置 " +
                 std::to_string(pattern.width) + "x" + std::to_string(pattern.height) + "=" +
                 std::to_string(expect) + " 不符";
        common::LogMsg(common::LERROR, errMsg);
        return false;
    }
    if (imageSize.width <= 0 || imageSize.height <= 0) {
        errMsg = "标定图尺寸非法";
        common::LogMsg(common::LERROR, errMsg);
        return false;
    }

    // ---- 多图角点取均值（固定机位下多张静态图是噪声重复观测，均值等价降噪）----
    const std::vector<cv::Point2f> meanCorners = MeanCorners(cornersList);
    common::LogMsg(common::LINFO, "角点集合 " + std::to_string(cornersList.size()) +
                   " 张取均值，每张 " + std::to_string(n) + " 点");

    // ---- 单视图标定求解：固定主点为图像中心、fx=fy，仅估焦距/畸变/板面外参 ----
    CalibrationResult result;
    result.image_size = imageSize;
    result.square_x_mm = cfg.square_x_mm;
    result.square_y_mm = cfg.square_y_mm;
    result.pattern_size = pattern;
    result.mm_per_px = cfg.target_mm_per_px;

    const std::vector<std::vector<cv::Point3f>> objPoints{
        BuildObjectGrid(cfg.square_x_mm, cfg.square_y_mm, pattern)};
    const std::vector<std::vector<cv::Point2f>> imgPoints{meanCorners};
    std::vector<cv::Mat> rvecs, tvecs;
    const int flags = cv::CALIB_FIX_ASPECT_RATIO | cv::CALIB_FIX_PRINCIPAL_POINT;
    double rms = 0.0;
    try {
        rms = cv::calibrateCamera(objPoints, imgPoints, imageSize,
                                  result.camera_matrix, result.dist_coeffs,
                                  rvecs, tvecs, flags);
    } catch (const cv::Exception& e) {
        errMsg = "标定求解失败: " + std::string(e.what());
        common::LogMsg(common::LERROR, errMsg);
        return false;
    }
    if (rvecs.empty() || tvecs.empty() || !std::isfinite(rms)) {
        errMsg = "标定求解结果非法（外参缺失或 RMS 非有限值）";
        common::LogMsg(common::LERROR, errMsg);
        return false;
    }
    result.rms = rms;
    result.rvec = rvecs[0];
    result.tvec = tvecs[0];
    result.dist_coeffs = result.dist_coeffs.reshape(1, 1);  // 统一 1x5

    // ---- RMS 分级日志 ----
    if (rms <= kRmsGood) {
        common::LogMsg(common::LINFO, "标定 RMS 重投影误差 " + FmtDouble(rms, 4) +
                       " px（达标 ≤0.1）");
    } else if (rms <= kRmsWarn) {
        common::LogMsg(common::LWARN, "标定 RMS 重投影误差 " + FmtDouble(rms, 4) +
                       " px（偏高，建议检查印刷与采图）");
    } else {
        common::LogMsg(common::LWARN, "标定 RMS 重投影误差 " + FmtDouble(rms, 4) +
                       " px（超 0.3 告警线，残差矢量图排查印刷缺陷）");
    }
    std::string distStr;
    for (int i = 0; i < result.dist_coeffs.cols; ++i) {
        distStr += (i == 0 ? "[" : " ") + FmtDouble(result.dist_coeffs.at<double>(0, i), 4);
    }
    distStr += "]";
    common::LogMsg(common::LINFO, "焦距 " +
                   FmtDouble(result.camera_matrix.at<double>(0, 0), 1) +
                   " px，畸变系数 " + distStr);

    // ---- XML 落盘 ----
    if (!SaveCalibrationXml(cfg.out_xml, result, errMsg)) {
        return false;  // errMsg 已由 SaveCalibrationXml 填写
    }

    // ---- 质量门禁（rms 部分；验证 mean_px 部分由 BuildCalibrationFile 收口）----
    if (rms > kRmsWarn) {
        errMsg = "标定质量偏低：RMS " + FmtDouble(rms, 4) +
                 " px 超过 0.3 px 门禁（标定文件仍已写出: " + cfg.out_xml +
                 "），请检查印刷精度与采图条件后重标";
        common::LogMsg(common::LWARN, errMsg);
        return false;
    }
    return true;
}

namespace {

// 求解收尾共享段（单图/多图两个对外入口共用）：回读 XML 自检 → 创建 QA 目录 →
// QA01 角点叠加 → QA02 残差矢量 → Rectifier 建表 → QA03 正射校正图 →
// 验证闭环（校正图重检角点回理想网格 + 行列直线度）→ QA04 → 汇总日志与总质量门禁
// @param gray        代表灰度图（8UC1；多图时取第一张）
// @param cornersList 各图角点集合（QA01/02 使用其均值角点，与求解输入一致）
// @param cfg         [calibrate] 段配置
// @param saveQa      是否落盘 QA 质检图（[debug] save_intermediate 开关）
// @param solveOk     SolveAndSaveCalibration 的返回值（false 多为 rms 门禁触发，文件已写出）
// @param solveErr    SolveAndSaveCalibration 的错误描述
// @param report      可选输出参数：标定摘要（rms/验证残差），传 nullptr 忽略；
//                    失败时也尽量填写已知字段（验证未完成时 verify* 保持 0）
// @param errMsg      输出参数：失败或质量偏低时的中文描述
// @return 标定成功且质量达标返回 true
bool FinishCalibrationAndQa(const cv::Mat& gray,
                            const std::vector<std::vector<cv::Point2f>>& cornersList,
                            const common::CalibrateConfig& cfg, bool saveQa,
                            bool solveOk, const std::string& solveErr,
                            CalibReport* report, std::string& errMsg) {
    errMsg.clear();
    const cv::Size pattern(cfg.pattern_cols, cfg.pattern_rows);
    const std::vector<cv::Point2f> corners = MeanCorners(cornersList);

    // ---- 1. 回读标定文件（落盘自检；同时供矫正器与质量门禁使用）----
    CalibrationResult calib;
    std::string loadErr;
    if (!LoadCalibrationXml(cfg.out_xml, calib, loadErr)) {
        // 求解阶段硬失败（未写出文件）时透传求解错误；否则为回读失败
        errMsg = solveOk ? ("标定文件回读失败: " + loadErr) : solveErr;
        common::LogMsg(common::LERROR, errMsg);
        return false;
    }
    if (report) {
        report->rms = calib.rms;
    }

    // ---- 2. 创建 QA 目录（工程约定路径均为 ASCII；[debug] 关闭时跳过全部 QA 图）----
    if (saveQa) {
        std::error_code ec;
        std::filesystem::create_directories(cfg.qa_dir, ec);
        if (ec) {
            errMsg = "创建 QA 目录失败: " + cfg.qa_dir + "，" + ec.message();
            common::LogMsg(common::LERROR, errMsg);
            return false;
        }
    }
    const std::string qaDir = cfg.qa_dir;

    // ---- 3. QA01：角点检测叠加图（叠加均值角点）----
    if (saveQa) {
        cv::Mat overlay;
        cv::cvtColor(gray, overlay, cv::COLOR_GRAY2BGR);
        cv::drawChessboardCorners(overlay, pattern, corners, true);
        PutQaText(overlay, "corners " + std::to_string(pattern.width) + "x" +
                  std::to_string(pattern.height) + "=" + std::to_string(corners.size()));
        common::SaveImage(qaDir + "/01_corners_overlay.bmp", overlay);
    }

    // ---- 4. QA02：残差矢量图（红箭头 = 均值角点 → 模型投影点，放大 50 倍）----
    if (saveQa) {
        const std::vector<cv::Point3f> obj =
            BuildObjectGrid(calib.square_x_mm, calib.square_y_mm, pattern);
        std::vector<cv::Point2f> projPts;  // 与 obj 深度一致（32F），混用 64F 会触发 OpenCV 断言
        try {
            cv::projectPoints(obj, calib.rvec, calib.tvec,
                              calib.camera_matrix, calib.dist_coeffs, projPts);
        } catch (const cv::Exception& e) {
            errMsg = "重投影计算失败: " + std::string(e.what());
            common::LogMsg(common::LERROR, errMsg);
            return false;
        }
        std::vector<double> errs;
        errs.reserve(projPts.size());
        cv::Mat canvas;
        cv::cvtColor(gray, canvas, cv::COLOR_GRAY2BGR);
        for (size_t i = 0; i < projPts.size(); ++i) {
            const cv::Point2f det = corners[i];
            errs.push_back(cv::norm(projPts[i] - det));
            const cv::Point p0(cvRound(det.x), cvRound(det.y));
            const cv::Point tip(cvRound(det.x + (projPts[i].x - det.x) * kResidualScale),
                                cvRound(det.y + (projPts[i].y - det.y) * kResidualScale));
            cv::arrowedLine(canvas, p0, tip, cv::Scalar(0, 0, 255), 2, cv::LINE_AA,
                            0, kArrowTipLength);
            cv::circle(canvas, p0, 3, cv::Scalar(0, 220, 0), -1, cv::LINE_AA);
        }
        PutQaText(canvas, "residual x" + FmtDouble(kResidualScale, 0) +
                  "  mean " + FmtDouble(MeanOf(errs), 3) + "px" +
                  "  p95 " + FmtDouble(Percentile95(errs), 3) + "px" +
                  "  max " + FmtDouble(MaxOf(errs), 3) + "px");
        common::SaveImage(qaDir + "/02_residual_vectors.bmp", canvas);
    }

    // ---- 5. 正射矫正器（XML 回读 + 映射表构建）----
    Rectifier rect;
    std::string rectErr;
    if (!rect.Load(cfg.out_xml, cfg.target_mm_per_px, rectErr)) {
        errMsg = "构建正射矫正映射失败: " + rectErr;
        common::LogMsg(common::LERROR, errMsg);
        return false;
    }
    cv::Mat rectified = rect.Rectify(gray);
    if (rectified.empty()) {
        errMsg = "正射矫正输出为空";
        common::LogMsg(common::LERROR, errMsg);
        return false;
    }
    if (saveQa) {
        common::SaveImage(qaDir + "/03_rectified.bmp", rectified);
    }

    // ---- 6. 验证闭环：校正图上重检角点，回理想网格比对 + 行列直线度 ----
    ValidateStats stats;
    bool validateOk = false;
    std::vector<cv::Point2f> corners2;
    std::string detErr;
    if (!DetectBoardCorners(rectified, pattern, corners2, detErr)) {
        common::LogMsg(common::LERROR, "正射校正图角点重检失败，验证闭环中断: " + detErr);
    } else {
        validateOk = true;
        const std::vector<cv::Point3f> obj =
            BuildObjectGrid(calib.square_x_mm, calib.square_y_mm, pattern);
        std::vector<cv::Point2d> ideal;
        ideal.reserve(obj.size());
        for (const auto& p : obj) {
            ideal.push_back(rect.BoardMmToPx(p.x, p.y));
        }
        std::vector<cv::Point2d> det2(corners2.begin(), corners2.end());
        std::vector<double> err2(corners2.size());
        for (size_t i = 0; i < corners2.size(); ++i) {
            err2[i] = cv::norm(det2[i] - ideal[i]);
        }
        stats.mean_px = MeanOf(err2);
        stats.p95_px = Percentile95(err2);
        stats.max_px = MaxOf(err2);
        stats.mean_mm = stats.mean_px * rect.MmPerPx();
        stats.max_mm = stats.max_px * rect.MmPerPx();
        if (report) {
            report->verifyMeanPx = stats.mean_px;
            report->verifyP95Px = stats.p95_px;
        }

        // 行/列直线度：每行/列角点拟合直线后的最大垂直偏差
        std::vector<std::vector<int>> rowIdx(pattern.height), colIdx(pattern.width);
        for (int r = 0; r < pattern.height; ++r) {
            for (int c = 0; c < pattern.width; ++c) {
                rowIdx[r].push_back(r * pattern.width + c);
                colIdx[c].push_back(r * pattern.width + c);
            }
        }
        stats.row_straightness_max_px = Straightness(det2, rowIdx);
        stats.col_straightness_max_px = Straightness(det2, colIdx);

        common::LogMsg(common::LINFO,
                       "校正验证: 残差 mean " + FmtDouble(stats.mean_px, 3) +
                       "px / p95 " + FmtDouble(stats.p95_px, 3) +
                       "px / max " + FmtDouble(stats.max_px, 3) +
                       "px（" + FmtDouble(stats.max_mm, 4) + "mm），行直线度 " +
                       FmtDouble(stats.row_straightness_max_px, 3) + "px，列直线度 " +
                       FmtDouble(stats.col_straightness_max_px, 3) + "px");

        // ---- QA04：校正图 + 角点叠加（绿圈=理想网格空心 r5，红点=实测实心 r3）----
        if (saveQa) {
            cv::Mat qa;
            cv::cvtColor(rectified, qa, cv::COLOR_GRAY2BGR);
            for (size_t i = 0; i < ideal.size(); ++i) {
                cv::circle(qa, cv::Point(cvRound(ideal[i].x), cvRound(ideal[i].y)),
                           5, cv::Scalar(0, 220, 0), 1, cv::LINE_AA);
                cv::circle(qa, cv::Point(cvRound(det2[i].x), cvRound(det2[i].y)),
                           3, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);
            }
            PutQaText(qa, "rectified residual  mean " + FmtDouble(stats.mean_px, 2) +
                      "px  max " + FmtDouble(stats.max_px, 2) + "px" +
                      " (" + FmtDouble(stats.max_mm, 3) + "mm)");
            common::SaveImage(qaDir + "/04_verify_grid.bmp", qa);
        }
    }

    // ---- 7. 汇总日志 + 总质量门禁 ----
    common::LogMsg(common::LINFO, "标定完成: RMS " + FmtDouble(calib.rms, 4) +
                   "px, 焦距 " + FmtDouble(calib.camera_matrix.at<double>(0, 0), 1) +
                   "px, 参数文件 " + cfg.out_xml);
    if (saveQa) {
        common::LogMsg(common::LINFO, "质检图见 " + qaDir);
    }
    if (!solveOk || !validateOk || stats.mean_px > kValidateMeanWarn) {
        errMsg = "标定质量偏低：RMS " + FmtDouble(calib.rms, 4) + "px，校正验证 mean " +
                 (validateOk ? FmtDouble(stats.mean_px, 3) + "px" : std::string("未通过")) +
                 "（标定文件已输出: " + cfg.out_xml +
                 (saveQa ? ("，QA 图: " + qaDir) : std::string()) +
                 "），请检查印刷精度与采图条件后重标";
        common::LogMsg(common::LWARN, errMsg);
        return false;
    }
    return true;
}

}  // namespace

bool BuildCalibrationFile(const cv::Mat& boardImage, const std::string& iniPath,
                          CalibReport* report, std::string& errMsg) {
    errMsg.clear();
    if (report) {
        *report = CalibReport{};
    }

    // ---- 1. 读取 [calibrate] 段配置（路径字段已被 LoadFromIni 解析为绝对路径）----
    common::CalibrateConfig cfg;
    bool saveQa = true;
    if (!LoadCalibrateConfig(iniPath, cfg, saveQa, errMsg)) {
        return false;  // errMsg 已由 LoadCalibrateConfig 填写
    }
    if (report) {
        report->xmlPath = cfg.out_xml;
    }
    const cv::Size pattern(cfg.pattern_cols, cfg.pattern_rows);

    // ---- 2. 转灰度 ----
    cv::Mat gray = common::ToGray8(boardImage);
    if (gray.empty()) {
        errMsg = "标定图像格式不支持（须为 8UC1/8UC3/8UC4）";
        common::LogMsg(common::LERROR, errMsg);
        return false;
    }

    // ---- 3. 角点检测 ----
    std::vector<cv::Point2f> corners;
    if (!DetectBoardCorners(gray, pattern, corners, errMsg)) {
        return false;  // errMsg 已由 DetectBoardCorners 填写
    }

    // ---- 4. 求解 + XML 落盘（单图即自身）----
    std::string solveErr;
    const bool solveOk =
        SolveAndSaveCalibration({corners}, gray.size(), cfg, solveErr);

    // ---- 5. 收尾：回读自检 → QA 图 → 验证闭环 → 总质量门禁（与多图入口共用）----
    return FinishCalibrationAndQa(gray, {corners}, cfg, saveQa, solveOk, solveErr,
                                  report, errMsg);
}

bool BuildCalibrationFileFromImages(const std::vector<cv::Mat>& boardImages,
                                    const std::string& iniPath,
                                    CalibReport* report, std::string& errMsg) {
    errMsg.clear();
    if (report) {
        *report = CalibReport{};
    }

    // ---- 0. 空列表早退 ----
    if (boardImages.empty()) {
        errMsg = "无标定采图（图像列表为空），无法标定";
        common::LogMsg(common::LERROR, errMsg);
        return false;
    }

    // ---- 1. 读取 [calibrate] 段配置（路径字段已被 LoadFromIni 解析为绝对路径）----
    common::CalibrateConfig cfg;
    bool saveQa = true;
    if (!LoadCalibrateConfig(iniPath, cfg, saveQa, errMsg)) {
        return false;  // errMsg 已由 LoadCalibrateConfig 填写
    }
    if (report) {
        report->xmlPath = cfg.out_xml;
    }
    const cv::Size pattern(cfg.pattern_cols, cfg.pattern_rows);

    // ---- 2. 逐张转灰度 + 尺寸一致性检查 + 角点检测（任一失败即整体失败）----
    cv::Mat grayFirst;
    std::vector<std::vector<cv::Point2f>> cornersList;
    cornersList.reserve(boardImages.size());
    for (size_t i = 0; i < boardImages.size(); ++i) {
        const std::string tag = "第 " + std::to_string(i + 1) + " 张标定采图";
        cv::Mat gray = common::ToGray8(boardImages[i]);
        if (gray.empty()) {
            errMsg = tag + "格式不支持（须为 8UC1/8UC3/8UC4）";
            common::LogMsg(common::LERROR, errMsg);
            return false;
        }
        if (grayFirst.empty()) {
            grayFirst = gray;
        } else if (gray.size() != grayFirst.size()) {
            errMsg = tag + "尺寸 (" + std::to_string(gray.cols) + ", " +
                     std::to_string(gray.rows) + ") 与首张 (" +
                     std::to_string(grayFirst.cols) + ", " +
                     std::to_string(grayFirst.rows) + ") 不一致";
            common::LogMsg(common::LERROR, errMsg);
            return false;
        }
        std::vector<cv::Point2f> corners;
        std::string detErr;
        if (!DetectBoardCorners(gray, pattern, corners, detErr)) {
            errMsg = tag + "角点检测失败: " + detErr;
            common::LogMsg(common::LERROR, errMsg);
            return false;
        }
        cornersList.push_back(std::move(corners));
    }
    common::LogMsg(common::LINFO, "多图标定采图 " + std::to_string(boardImages.size()) +
                   " 张全部检测成功，角点取均值降噪");

    // ---- 3. 角点均值求解 + XML 落盘 ----
    std::string solveErr;
    const bool solveOk =
        SolveAndSaveCalibration(cornersList, grayFirst.size(), cfg, solveErr);

    // ---- 4. 收尾：QA 与验证闭环以第一张图为代表图（与单图入口共用）----
    return FinishCalibrationAndQa(grayFirst, cornersList, cfg, saveQa, solveOk, solveErr,
                                  report, errMsg);
}

bool LoadCalibrationXml(const std::string& xmlPath, CalibrationResult& out,
                        std::string& errMsg) {
    errMsg.clear();
    cv::FileStorage fs;
    try {
        fs.open(xmlPath, cv::FileStorage::READ);
    } catch (const cv::Exception& e) {
        errMsg = "标定文件解析失败: " + xmlPath + "，" + e.what();
        common::LogMsg(common::LERROR, errMsg);
        return false;
    }
    if (!fs.isOpened()) {
        errMsg = "标定文件不存在或无法打开: " + xmlPath + "（请先完成功能 2 棋盘格标定）";
        common::LogMsg(common::LERROR, errMsg);
        return false;
    }
    int w = 0, h = 0, pcols = 0, prows = 0;
    std::string created;
    fs["camera_matrix"] >> out.camera_matrix;
    fs["dist_coeffs"] >> out.dist_coeffs;
    fs["rvec"] >> out.rvec;
    fs["tvec"] >> out.tvec;
    fs["image_width"] >> w;
    fs["image_height"] >> h;
    fs["square_x_mm"] >> out.square_x_mm;
    fs["square_y_mm"] >> out.square_y_mm;
    fs["pattern_cols"] >> pcols;
    fs["pattern_rows"] >> prows;
    fs["rms"] >> out.rms;
    fs["mm_per_px"] >> out.mm_per_px;
    fs["created"] >> created;
    fs.release();
    out.image_size = cv::Size(w, h);
    out.pattern_size = cv::Size(pcols, prows);

    // ---- 完整性校验 ----
    if (out.camera_matrix.empty() || out.camera_matrix.rows != 3 ||
        out.camera_matrix.cols != 3 ||
        out.dist_coeffs.empty() || out.dist_coeffs.total() != 5 ||
        out.rvec.empty() || out.rvec.total() != 3 ||
        out.tvec.empty() || out.tvec.total() != 3 ||
        w <= 0 || h <= 0 || pcols <= 0 || prows <= 0) {
        errMsg = "标定文件字段缺失或损坏: " + xmlPath;
        common::LogMsg(common::LERROR, errMsg);
        return false;
    }

    // ---- 类型与形状归一（全部 CV_64F；dist 1x5，rvec/tvec 3x1）----
    out.camera_matrix.convertTo(out.camera_matrix, CV_64F);
    out.dist_coeffs.convertTo(out.dist_coeffs, CV_64F);
    out.dist_coeffs = out.dist_coeffs.reshape(1, 1);
    out.rvec.convertTo(out.rvec, CV_64F);
    out.rvec = out.rvec.reshape(1, 3);
    out.tvec.convertTo(out.tvec, CV_64F);
    out.tvec = out.tvec.reshape(1, 3);

    common::LogMsg(common::LINFO, "标定参数已加载: " + xmlPath + "（RMS " +
                   FmtDouble(out.rms, 4) + "px, 标定时间 " + created + "）");
    return true;
}

}  // namespace cam
