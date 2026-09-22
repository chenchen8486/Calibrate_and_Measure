// ============================================================================
// cam_api.cpp —— 对外统一门面实现（只做转发与状态组装，不含算法逻辑）
// ============================================================================

#include "cam_api.h"

#include "checkerboard/board_generator.h"
#include "common/image_io.h"
#include "common/logger.h"

namespace cam {

const char* Version() {
    return "2.1.2";
}

// ---------------------------------------------------------------------------
// 功能 1：制板（直接转发）
// ---------------------------------------------------------------------------
bool MakeBoard(const std::string& iniPath, std::string& errMsg) {
    return GenerateCheckerboardBoard(iniPath, errMsg);
}

// ---------------------------------------------------------------------------
// 功能 2：标定（单图/多图入口统一，report 透传质量摘要）
// ---------------------------------------------------------------------------
bool Calibrate(const std::vector<cv::Mat>& boardImages, const std::string& iniPath,
               CalibReport* report, std::string& errMsg) {
    if (boardImages.size() == 1) {
        return BuildCalibrationFile(boardImages[0], iniPath, report, errMsg);
    }
    return BuildCalibrationFileFromImages(boardImages, iniPath, report, errMsg);
}

// ---------------------------------------------------------------------------
// 功能 3：测量会话
// ---------------------------------------------------------------------------
Measurer::Measurer() = default;
Measurer::~Measurer() = default;

bool Measurer::Init(const std::string& iniPath, std::string& errMsg) {
    ready_ = false;

    // 1)~3)：配置 + 背景建模 + 分割器（含 ONNX 会话预热与回退）
    if (!pipe_.Init(iniPath, errMsg)) {
        return false;
    }

    // 4) 几何矫正（可选）：与流水线共用同一标定文件，一致性由门面保证。
    //    标定文件缺失/加载失败不阻断初始化：记 Warn 降级为未矫正运行，
    //    测量照常（结果仅像素值），调用方可用 RectifyEnabled() 确认。
    const common::AppConfig& cfg = pipe_.Config();
    if (cfg.rectify.enabled) {
        std::string rectErr;
        if (!rectifier_.Load(cfg.rectify.calib_xml, cfg.rectify.target_mm_per_px,
                             rectErr)) {
            common::LogMsg(common::LWARN, "相机未标定或标定文件不可用（" + rectErr +
                           "），已跳过几何矫正，测量结果仅像素值");
        }
    }

    ready_ = true;
    return true;
}

MeasureOutput Measurer::Measure(const cv::Mat& image, const std::string& debugTag,
                                cv::Mat* basisImage) {
    if (basisImage) {
        basisImage->release();  // 保证任何失败分支都不留上一次的旧图
    }
    if (!ready_) {
        MeasureOutput out;
        out.code = RetCode::INTERNAL;
        out.message = "Measurer 未初始化（请先成功调用 Init）";
        return out;
    }

    // 矫正开启时：原图 → 灰度 → 正射矫正（尺寸须与标定采图一致）；
    // 矫正关闭时原图直进测量（MeasurePipeline 内部自行转灰度）
    cv::Mat input = image;
    if (rectifier_.IsReady()) {
        const cv::Mat gray = common::ToGray8(image);
        input = rectifier_.Rectify(gray);
        if (input.empty()) {
            MeasureOutput out;
            out.code = RetCode::INTERNAL;
            out.message = "正射校正失败（图像尺寸须与标定采图一致）";
            return out;
        }
    }
    return pipe_.Measure(input, debugTag, basisImage);
}

bool Measurer::SetBackground(const cv::Mat& image, std::string& errMsg) {
    if (!ready_) {
        errMsg = "Measurer 未初始化（请先成功调用 Init）";
        return false;
    }
    const cv::Mat gray = common::ToGray8(image);
    if (gray.empty()) {
        errMsg = "图像格式不支持（须 8UC1/8UC3/8UC4）";
        return false;
    }
    return pipe_.SetBackground(gray, errMsg);
}

bool Measurer::RectifyEnabled() const {
    return rectifier_.IsReady();
}

double Measurer::MmPerPx() const {
    return rectifier_.IsReady() ? rectifier_.MmPerPx() : 0.0;
}

bool Measurer::UsingAi() const {
    return pipe_.UsingAi();
}

const common::AppConfig& Measurer::Config() const {
    return pipe_.Config();
}

}  // namespace cam
