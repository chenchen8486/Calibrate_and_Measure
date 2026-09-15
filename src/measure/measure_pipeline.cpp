// ============================================================================
// measure/measure_pipeline.cpp —— 功能 3（图像测量）流水线门面实现
// 等价移植自 Python 工程 src/edge_measure/pipeline.py 的 process_single，
// 输出组装遵循 payload.py 的接口约定。
// 步骤调试图落盘由 ini [debug] save_intermediate 开关控制（见 Measure 的
// debugTag 参数）：开启时每张图向 paths.output_dir/debug/<debugTag>/ 落
// 灰度图/掩膜/增强图/旋转图/测量叠加图，关闭时零中间文件。
// ============================================================================

#include "measure/measure_pipeline.h"

#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <cctype>
#include <cmath>
#include <filesystem>

#include <opencv2/imgproc.hpp>

#include "calibration/rectifier.h"
#include "common/image_io.h"
#include "common/logger.h"
#include "measure/background.h"
#include "measure/code_detect.h"
#include "measure/code_detect_ai.h"
#include "measure/edge_refine.h"
#include "measure/measure_core.h"
#include "measure/rectify_angle.h"
#include "measure/segment_ai.h"
#include "measure/segment_traditional.h"

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

// 保留 digits 位小数（payload.py：坐标/长度 2 位，角度 3 位）
double RoundTo(double v, int digits) {
    const double s = std::pow(10.0, digits);
    return std::round(v * s) / s;
}

Point2d RoundPoint(const Point2d& p) {
    return Point2d{RoundTo(p.x, 2), RoundTo(p.y, 2)};
}

LineSegment RoundSegment(const LineSegment& seg) {
    return LineSegment{RoundPoint(seg.start), RoundPoint(seg.end)};
}

// 组装失败输出并记 Error 日志
MeasureOutput FailOutput(RetCode code, const std::string& msg) {
    MeasureOutput out;
    out.code = code;
    out.message = msg;
    common::LogMsg(common::LERROR, msg);
    return out;
}

// 落一张调试图（仅 [debug] save_intermediate=true 且 debugTag 非空时被调用）
void SaveDebugImg(const std::string& dir, const char* name, const cv::Mat& img) {
    if (img.empty()) {
        return;
    }
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    if (ec) {
        common::LogMsg(common::LWARN, "创建调试图目录失败: " + dir + "，" + ec.message());
        return;
    }
    common::SaveImage(dir + "\\" + name, img);
}

cv::Point ToCvPoint(const Point2d& p) {
    return cv::Point(cvRound(p.x), cvRound(p.y));
}

}  // namespace

MeasurePipeline::MeasurePipeline() = default;
MeasurePipeline::~MeasurePipeline() = default;

bool MeasurePipeline::Init(const std::string& iniPath, std::string& errMsg) {
    ready_ = false;
    useAi_ = false;
    codeAi_ = false;
    aiSeg_.reset();

    // 1) 配置加载（相对路径已在 LoadFromIni 内解析为绝对路径）
    if (!cfg_.LoadFromIni(iniPath, errMsg)) {
        common::LogMsg(common::LERROR, "配置加载失败：" + errMsg);
        return false;
    }

    // 2) 背景建模：paths.input_dir 全量图 + background_file 缓存
    //    （产品位置各异，中位数才落在背板上；始终使用全量图集）
    const std::vector<std::string> imagePaths = common::ListImages(cfg_.paths.input_dir);
    if (imagePaths.empty()) {
        errMsg = "输入目录无图像: " + cfg_.paths.input_dir;
        common::LogMsg(common::LERROR, errMsg);
        return false;
    }
    if (!LoadOrBuildBackground(imagePaths, cfg_.paths.background_file, background_,
                               errMsg)) {
        common::LogMsg(common::LERROR, "背景建模失败：" + errMsg);
        return false;
    }

    // 2.5) 几何矫正开启时：背景模型与每张输入图过同一张 remap 表（对齐 Python
    //      run_batch：rectifier.rectify(background)，输入图由调用方自行矫正）。
    //      矫正器仅此处使用一次，无需长期持有，用局部对象即可。
    //      标定文件缺失/不可用或背景校正失败时不阻断流程：记 Warn 降级为
    //      未矫正运行，测量结果仅像素值（调用方可用 RectifyEnabled 查询）。
    if (cfg_.rectify.enabled) {
        Rectifier rectifier;
        std::string rectErr;
        if (!rectifier.Load(cfg_.rectify.calib_xml, cfg_.rectify.target_mm_per_px,
                            rectErr)) {
            common::LogMsg(common::LWARN, "相机未标定或标定文件不可用（" + rectErr +
                           "），本次按未矫正运行，测量结果仅像素值");
        } else {
            cv::Mat rectBg = rectifier.Rectify(background_);
            if (rectBg.empty()) {
                common::LogMsg(common::LWARN,
                               "背景模型正射校正失败（尺寸与标定 image_size 不符），"
                               "本次按未矫正运行，测量结果仅像素值");
            } else {
                background_ = rectBg;
                common::LogMsg(common::LINFO, "几何矫正已启用，背景模型已同步正射校正");
            }
        }
    }

    // 3) 分割器初始化：segmentation.method=="ai" 或 code_detect.method=="ai"
    //    任一启用即创建 ONNX 会话（同一个两类模型一次加载，盒子分割取类 0、
    //    码区检测取 code_detect_ai.code_class，两路共用）；失败时两路各自回退
    std::string method = cfg_.segmentation.method;
    std::transform(method.begin(), method.end(), method.begin(),
                   [](unsigned char ch) { return (char)std::tolower(ch); });
    std::string codeMethod = cfg_.code_detect.method;
    std::transform(codeMethod.begin(), codeMethod.end(), codeMethod.begin(),
                   [](unsigned char ch) { return (char)std::tolower(ch); });
    codeAi_ = cfg_.code_detect.enabled && codeMethod == "ai";
    if (cfg_.code_detect.enabled && codeMethod != "ai" && codeMethod != "traditional") {
        common::LogMsg(common::LWARN,
                       "无法识别的码区检测方法 [" + cfg_.code_detect.method +
                           "]，按 traditional 处理（可选 traditional/ai）");
    }
    if (method == "ai" || codeAi_) {
        auto seg = std::make_unique<OnnxSegmenter>(cfg_.ai_seg, cfg_.trad_seg);
        if (seg->IsReady()) {
            aiSeg_ = std::move(seg);
            useAi_ = (method == "ai");
            common::LogMsg(common::LINFO, "AI 分割器就绪");
        } else {
            std::string fb = "AI 分割器初始化失败（" + seg->LastError() + "），";
            if (method == "ai" && codeAi_) {
                fb += "分割回退传统背景差分，码区检测回退传统三层链";
            } else if (method == "ai") {
                fb += "分割回退传统背景差分";
            } else {
                fb += "码区检测回退传统三层链";
            }
            common::LogMsg(common::LWARN, fb);
        }
    }
    ready_ = true;
    common::LogMsg(common::LINFO,
                   Fmt("测量流水线初始化完成，分割方式: %s，码区检测: %s",
                       useAi_ ? "ai" : "traditional",
                       codeAi_ ? (aiSeg_ ? "ai" : "traditional（AI 未就绪回退）")
                               : "traditional"));
    return true;
}

bool MeasurePipeline::IsReady() const {
    return ready_;
}

MeasureOutput MeasurePipeline::Measure(const cv::Mat& image, const std::string& debugTag) {
    if (!ready_) {
        return FailOutput(RetCode::INTERNAL, "测量流水线未初始化，请先调用 Init");
    }
    if (image.empty()) {
        return FailOutput(RetCode::EMPTY_IMAGE, "输入图像为空");
    }
    // 调试图开关：ini [debug] save_intermediate=true 且调用方给了标识才落盘
    const bool dbg = cfg_.debug.save_intermediate && !debugTag.empty();
    const std::string dbgDir = cfg_.paths.output_dir + "\\debug\\" + debugTag;
    try {
        cv::Mat gray = common::ToGray8(image);
        if (gray.empty()) {
            return FailOutput(RetCode::BAD_FORMAT,
                              "图像格式不支持（须 8UC1/8UC3/8UC4）");
        }
        std::string err;

        // ---- 阶段 1：分割（AI 或传统背景差分），产出掩膜与测量基准增强图 ----
        // 关键架构决策：分割与测量共用同一基准——"双边滤波+gamma+亮度对齐"后的
        // 增强图。双边滤波保边、gamma 单调映射，均不移动边缘位置。
        cv::Mat mask, aligned;
        if (useAi_) {
            std::string desc;
            if (!aiSeg_->Segment(gray, background_, mask, desc)) {
                return FailOutput(RetCode::NO_PRODUCT, "AI 分割失败：" + desc);
            }
            common::LogMsg(common::LINFO, "AI 分割来源: " + desc);
            aligned = BuildEnhanced(gray, background_, cfg_.trad_seg);
            if (cfg_.ai_seg.refine) {
                cv::Mat refined;
                if (!RefineMask(mask, aligned, cfg_.refine, refined, nullptr, err)) {
                    return FailOutput(RetCode::NO_PRODUCT, "掩膜边缘精修失败：" + err);
                }
                mask = refined;
            }
        } else {
            if (!SegmentProduct(gray, background_, cfg_.trad_seg, mask, &aligned, err)) {
                return FailOutput(RetCode::NO_PRODUCT, err);
            }
        }
        if (dbg) {
            SaveDebugImg(dbgDir, "01_input_gray.bmp", gray);
            SaveDebugImg(dbgDir, "02_mask.bmp", mask);
            SaveDebugImg(dbgDir, "03_enhanced_aligned.bmp", aligned);
        }

        // ---- 阶段 2：两级旋转校正（旋转对象为增强图 aligned）----
        std::vector<cv::Point> contour;
        if (!ExtractOuterContour(mask, contour, err)) {
            return FailOutput(RetCode::NO_PRODUCT, "产品掩膜异常：" + err);
        }
        double coarse = 0.0;
        if (!FindReferenceAngle(contour, cfg_.rotate, coarse, err)) {
            return FailOutput(RetCode::INTERNAL, "角度粗校正失败：" + err);
        }
        cv::Mat preAligned, preMask;
        RotateImageAndMask(aligned, mask, coarse, preAligned, preMask);
        const double residual =
            RefineAngleByEdges(preAligned, preMask, cfg_.measure, cfg_.refine);
        const double totalAngle = coarse + residual;
        common::LogMsg(common::LINFO,
                       Fmt("总校正角: %.3f° (粗 %.3f° + 精 %+.3f°)", totalAngle, coarse,
                           residual));
        cv::Mat rotAligned, rotMask;
        RotateImageAndMask(aligned, mask, totalAngle, rotAligned, rotMask);
        if (dbg) {
            SaveDebugImg(dbgDir, "04_rotated_aligned.bmp", rotAligned);
            SaveDebugImg(dbgDir, "05_rotated_mask.bmp", rotMask);
        }

        // ---- 阶段 3：测量输出（语义宽高 + 水平边排名，共 5 个输出值）----
        std::vector<cv::Point> rotContour;
        if (!ExtractOuterContour(rotMask, rotContour, err)) {
            return FailOutput(RetCode::NO_PRODUCT, "校正后产品掩膜异常：" + err);
        }
        MeasureOutput out;
        if (!MeasureWidthHeight(rotAligned, rotMask, cfg_.measure, cfg_.refine,
                                out.width, out.height, err)) {
            return FailOutput(RetCode::INTERNAL, "宽高测量失败：" + err);
        }
        // 水平边缺失不视为错误（"有多少条返回多少条"），失败仅告警
        if (!MeasureRankedEdges(rotContour, cfg_.measure, out.horizontalEdges, err)) {
            common::LogMsg(common::LWARN,
                           "水平边排名测量失败（不影响宽高输出）：" + err);
            out.horizontalEdges.clear();
        }
        // 码区缺失不视为错误（未检出返回空 vector），失败仅告警。
        // 检测在未旋转的原始灰度图上进行（旋转插值会平滑条码细条纹导致漏检，
        // 实测结论见 code_detect.h），检出框按校正角解析映射回校正坐标系。
        // code_detect.method=ai 时走深度学习支路（同一个两类模型取码区类掩膜
        // + 后处理），AI 支路失败/模型无码区类别通道时本帧自动回退传统三层链
        bool codeDone = false;
        if (codeAi_ && aiSeg_) {
            codeDone =
                DetectCodeRegionsAi(*aiSeg_, gray, rotContour, totalAngle,
                                    cfg_.code_detect, cfg_.code_detect_ai,
                                    out.codeRegions);
            if (!codeDone) {
                common::LogMsg(common::LWARN,
                               "AI 码区检测未跑通，本帧回退传统三层链");
            }
        }
        if (!codeDone) {
            if (!DetectCodeRegions(gray, rotContour, totalAngle, cfg_.code_detect,
                                   out.codeRegions)) {
                out.codeRegions.clear();
            }
        }

        // ---- 阶段 4：按 payload.py 约定组装输出（坐标/长度 2 位，角度 3 位）----
        out.rotationDeg = RoundTo(totalAngle, 3);
        out.width.widthPx = RoundTo(out.width.widthPx, 2);
        out.width.leftLine = RoundSegment(out.width.leftLine);
        out.width.rightLine = RoundSegment(out.width.rightLine);
        out.height.heightPx = RoundTo(out.height.heightPx, 2);
        out.height.topLine = RoundSegment(out.height.topLine);
        out.height.bottomLine = RoundSegment(out.height.bottomLine);
        for (HorizontalEdge& e : out.horizontalEdges) {
            e.start = RoundPoint(e.start);
            e.end = RoundPoint(e.end);
            e.lengthPx = RoundTo(e.lengthPx, 2);
        }
        for (CodeRegion& c : out.codeRegions) {
            c.x = RoundTo(c.x, 2);
            c.y = RoundTo(c.y, 2);
            c.w = RoundTo(c.w, 2);
            c.h = RoundTo(c.h, 2);
        }
        if (dbg) {
            // 测量叠加图：蓝=宽度左右竖线，绿=高度上下横线，红=候选水平边（附序号），
            // 品红矩形=码区（附 QR/BAR 类型标）
            cv::Mat overlay;
            cv::cvtColor(rotAligned, overlay, cv::COLOR_GRAY2BGR);
            cv::line(overlay, ToCvPoint(out.width.leftLine.start),
                     ToCvPoint(out.width.leftLine.end), cv::Scalar(255, 0, 0), 2, cv::LINE_AA);
            cv::line(overlay, ToCvPoint(out.width.rightLine.start),
                     ToCvPoint(out.width.rightLine.end), cv::Scalar(255, 0, 0), 2, cv::LINE_AA);
            cv::line(overlay, ToCvPoint(out.height.topLine.start),
                     ToCvPoint(out.height.topLine.end), cv::Scalar(0, 200, 0), 2, cv::LINE_AA);
            cv::line(overlay, ToCvPoint(out.height.bottomLine.start),
                     ToCvPoint(out.height.bottomLine.end), cv::Scalar(0, 200, 0), 2, cv::LINE_AA);
            for (size_t i = 0; i < out.horizontalEdges.size(); ++i) {
                const HorizontalEdge& e = out.horizontalEdges[i];
                cv::line(overlay, ToCvPoint(e.start), ToCvPoint(e.end),
                         cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
                cv::putText(overlay, std::to_string(i + 1), ToCvPoint(e.start),
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);
            }
            for (const CodeRegion& c : out.codeRegions) {
                const cv::Rect rc(cvRound(c.x), cvRound(c.y), cvRound(c.w), cvRound(c.h));
                cv::rectangle(overlay, rc, cv::Scalar(255, 0, 255), 2, cv::LINE_AA);
                cv::putText(overlay, c.type == CodeType::QR ? "QR" : "BAR",
                            rc.tl() + cv::Point(0, -6), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                            cv::Scalar(255, 0, 255), 2);
            }
            SaveDebugImg(dbgDir, "06_measure_overlay.bmp", overlay);
        }
        out.code = RetCode::OK;
        common::LogMsg(common::LINFO,
                       Fmt("测量完成: 宽 %.2f px, 高 %.2f px, 校正角 %.3f°, "
                           "候选水平边 %d 条, 码区 %d 个",
                           out.width.widthPx, out.height.heightPx, out.rotationDeg,
                           (int)out.horizontalEdges.size(), (int)out.codeRegions.size()));
        return out;
    } catch (const cv::Exception& e) {
        return FailOutput(RetCode::INTERNAL,
                          std::string("测量流程 OpenCV 异常：") + e.what());
    } catch (const std::exception& e) {
        return FailOutput(RetCode::INTERNAL,
                          std::string("测量流程内部异常：") + e.what());
    }
}

}  // namespace cam
