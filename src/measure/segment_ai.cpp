// ============================================================================
// measure/segment_ai.cpp —— AI 分割（ONNX Runtime 推理）实现
// 对照 Python 工程 scripts/predict_seg.py:34-105（predict_mask / predict_mask_with_roi）
// ============================================================================

#include "measure/segment_ai.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <cctype>
#include <cmath>
#include <filesystem>

#include <onnxruntime_cxx_api.h>
#include <opencv2/imgproc.hpp>
#include <windows.h>  // MultiByteToWideChar（ONNX Runtime 宽字符模型路径）

#include "common/logger.h"
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

// UTF-8 -> UTF-16（Windows 下 Ort::Session 需要宽字符路径）
std::wstring Utf8ToWide(const std::string& s) {
    if (s.empty()) {
        return L"";
    }
    const int len = MultiByteToWideChar(CP_UTF8, 0, s.c_str(), -1, nullptr, 0);
    std::wstring w((size_t)len - 1, L'\0');
    MultiByteToWideChar(CP_UTF8, 0, s.c_str(), -1, w.data(), len);
    return w;
}

inline float Sigmoid(float x) {
    return 1.0f / (1.0f + std::exp(-x));
}

// ============================================================================
// ★★★ RF-DETR ONNX 预处理/后处理布局约定（联调对齐点）★★★
// 已与 Python 侧导出产物 assets/weights/small.onnx 实机核对一致
// （输入 [1,3,384,384]，输出 dets/labels/masks）；更换模型时需重新核对
// 本区域（张量名/尺寸/归一化参数）：
//   输入 : float32 [1, 3, H, W]，RGB 通道（灰度复制 3 通道），
//          像素 /255 后做 ImageNet mean/std 标准化；
//          H/W 取模型输入形状，动态维度（<=0）时兜底 560。
//   输出 : 分类 logits  float32 [1, Q, C]   —— sigmoid 后取最大类置信度；
//                    （检测框输出 [1, Q, 4] 同为 3 维，按"末维 != 4"区分）
//          掩膜 logits  float32 [1, Q, mh, mw] —— sigmoid 为前景概率，
//                    resize 到 crop 尺寸后以 0.5 二值化。
//   类别 : 当前交付模型 C=2（labels [1,100,2]），但类 1 未训练，仅类 0
//          盒子可用；码区支路需重训两类模型（约定 类 0 = 盒子、类 1 = 码区，
//          固定约定见 code_detect_ai.cpp kCodeClass，训练导出顺序不符时在
//          训练侧调整，不走配置）。盒子分割只取 argmax 类为 0 的查询，码区
//          推理只取 argmax 类为码区类的查询，两路互不抢占；当前模型 argmax
//          恒为 0，盒子行为与旧版一致，码区推理零检出（属预期）。
// ============================================================================
constexpr float kImageNetMean[3] = {0.485f, 0.456f, 0.406f};
constexpr float kImageNetStd[3]  = {0.229f, 0.224f, 0.225f};
constexpr int   kFallbackInputSize = 560;  // 动态输入尺寸兜底（RF-DETR 常用 560）

// 预处理：灰度 -> RGB 3 通道 -> resize 到模型输入尺寸 -> NCHW float32 归一化
void PreprocessCrop(const cv::Mat& cropGray, int inW, int inH, std::vector<float>& blob) {
    cv::Mat rgb, resized;
    cv::cvtColor(cropGray, rgb, cv::COLOR_GRAY2RGB);
    cv::resize(rgb, resized, cv::Size(inW, inH), 0.0, 0.0, cv::INTER_LINEAR);
    blob.assign((size_t)3 * inH * inW, 0.0f);
    const size_t planeSize = (size_t)inH * inW;
    for (int y = 0; y < inH; ++y) {
        const cv::Vec3b* row = resized.ptr<cv::Vec3b>(y);
        for (int x = 0; x < inW; ++x) {
            for (int c = 0; c < 3; ++c) {
                blob[(size_t)c * planeSize + (size_t)y * inW + x] =
                    (row[x][c] / 255.0f - kImageNetMean[c]) / kImageNetStd[c];
            }
        }
    }
}

// 一次推理的产出：持有输出张量（读取数据期间必须存活）与识别出的角色索引
struct ModelOutputs {
    std::vector<Ort::Value> tensors;
    int scoresIdx = -1;             // 分类 logits [1,Q,C] 的下标
    int masksIdx = -1;              // 掩膜 logits [1,Q,mh,mw] 的下标
    std::vector<int64_t> scoresShape;
    std::vector<int64_t> masksShape;
};

// 预处理 + 推理 + 输出角色识别（InferCrop / InferClassMasks 共用）。
// 成功返回 true 且 scoresIdx/masksIdx 有效；布局无法识别返回 false（已记 Warn）。
bool RunSession(Ort::Session& session, const std::string& inputName,
                const std::vector<std::string>& outputNames, int inW, int inH,
                const cv::Mat& imgGray, ModelOutputs& mo) {
    std::vector<float> blob;
    PreprocessCrop(imgGray, inW, inH, blob);

    const std::array<int64_t, 4> inShape = {1, 3, inH, inW};
    const Ort::MemoryInfo memInfo = Ort::MemoryInfo::CreateCpu(
        OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);
    Ort::Value inputTensor = Ort::Value::CreateTensor<float>(
        memInfo, blob.data(), blob.size(), inShape.data(), inShape.size());

    const char* inNames[] = {inputName.c_str()};
    std::vector<const char*> outNames;
    outNames.reserve(outputNames.size());
    for (const std::string& s : outputNames) {
        outNames.push_back(s.c_str());
    }
    mo.tensors = session.Run(Ort::RunOptions{nullptr}, inNames, &inputTensor, 1,
                             outNames.data(), outNames.size());

    // 识别输出角色：分类 logits [1,Q,C]（末维!=4）、检测框 [1,Q,4]、掩膜 [1,Q,mh,mw]
    for (size_t i = 0; i < mo.tensors.size(); ++i) {
        auto info = mo.tensors[i].GetTensorTypeAndShapeInfo();
        if (info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
            continue;
        }
        std::vector<int64_t> shape = info.GetShape();
        if (shape.size() == 3) {
            if (shape[2] != 4 && mo.scoresIdx < 0) {
                mo.scoresIdx = (int)i;
                mo.scoresShape = shape;
            }
        } else if (shape.size() == 4 && mo.masksIdx < 0) {
            mo.masksIdx = (int)i;
            mo.masksShape = shape;
        }
    }
    if (mo.scoresIdx < 0 || mo.masksIdx < 0) {
        common::LogMsg(common::LWARN,
                       "ONNX 输出布局无法识别（缺少 3 维分类或 4 维掩膜输出），"
                       "请按文件顶部注释核对模型导出格式");
        return false;
    }
    if (mo.masksShape[1] != mo.scoresShape[1] || mo.masksShape[2] <= 0 ||
        mo.masksShape[3] <= 0) {
        common::LogMsg(common::LWARN, "ONNX 输出查询数不一致或掩膜尺寸非法");
        return false;
    }
    return true;
}

}  // namespace

struct OnnxSegmenter::Impl {
    Ort::Env env;                          // 必须早于 session 构造
    std::unique_ptr<Ort::Session> session;
    std::string inputName;
    std::vector<std::string> outputNames;
    int inputH = kFallbackInputSize;
    int inputW = kFallbackInputSize;

    Impl() : env(ORT_LOGGING_LEVEL_WARNING, "cam_ai_seg") {}
};

OnnxSegmenter::OnnxSegmenter(const common::AiSegConfig& aiCfg,
                             const common::TradSegConfig& tradCfg)
    : impl_(new Impl()), aiCfg_(aiCfg), tradCfg_(tradCfg) {
    namespace fs = std::filesystem;
    std::error_code ec;
    if (aiCfg_.onnx_model.empty() || !fs::exists(aiCfg_.onnx_model, ec)) {
        lastError_ = "ONNX 模型文件不存在: " + aiCfg_.onnx_model;
        common::LogMsg(common::LERROR, lastError_);
        return;
    }
    try {
        auto makeOptions = []() {
            Ort::SessionOptions so;
            so.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
            return so;
        };
        std::string dev = aiCfg_.device;
        std::transform(dev.begin(), dev.end(), dev.begin(),
                       [](unsigned char ch) { return (char)std::tolower(ch); });
        if (dev != "auto" && dev != "cuda" && dev != "cpu") {
            common::LogMsg(common::LWARN,
                           "无法识别的推理设备 [" + aiCfg_.device +
                               "]，按 auto 处理（可选 auto/cuda/cpu）");
            dev = "auto";
        }
        const std::wstring wpath = Utf8ToWide(aiCfg_.onnx_model);
        bool useCuda = false;
        // auto/cuda 都直接试注册 CUDA EP 并建 CUDA 会话，失败一律回退 CPU 重建
        // （AI 分割能力不丢）。早年版本用 LoadLibraryEx 预检 provider DLL 的
        // 依赖，实测会误报（依赖齐全时仍报 1114 初始化失败，而 ORT 自身的加载
        // 路径完全正常），故废弃预检，直接以真实注册/建会话结果为判据。
        if (dev == "auto" || dev == "cuda") {
            Ort::SessionOptions soCuda = makeOptions();
            OrtStatus* st = OrtSessionOptionsAppendExecutionProvider_CUDA(soCuda, 0);
            if (st == nullptr) {
                try {
                    impl_->session.reset(
                        new Ort::Session(impl_->env, wpath.c_str(), soCuda));
                    useCuda = true;
                } catch (const Ort::Exception& e) {
                    common::LogMsg(common::LWARN,
                                   std::string("CUDA 会话创建失败（") + e.what() +
                                       "），回退 CPU 推理");
                }
            } else {
                const std::string ortErr = Ort::GetApi().GetErrorMessage(st);
                Ort::GetApi().ReleaseStatus(st);
                if (dev == "cuda") {
                    common::LogMsg(common::LWARN,
                                   "CUDA EP 注册失败（" + ortErr + "），回退 CPU 推理");
                } else {
                    common::LogMsg(common::LINFO,
                                   "未检测到可用的 CUDA 运行环境，AI 分割使用 CPU 推理"
                                   "（如需 GPU 请安装 CUDA 12.x + cuDNN 9.x 运行时）");
                }
            }
        }
        if (!useCuda) {
            Ort::SessionOptions so = makeOptions();
            impl_->session.reset(new Ort::Session(impl_->env, wpath.c_str(), so));
        }

        // 读取输入/输出张量信息（输入尺寸动态时按兜底值处理）
        Ort::AllocatorWithDefaultOptions alloc;
        {
            auto inName = impl_->session->GetInputNameAllocated(0, alloc);
            impl_->inputName = inName.get();
        }
        const std::vector<int64_t> shape =
            impl_->session->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
        if (shape.size() == 4 && shape[2] > 0 && shape[3] > 0) {
            impl_->inputH = (int)shape[2];
            impl_->inputW = (int)shape[3];
        } else {
            common::LogMsg(common::LWARN,
                           Fmt("模型输入 H/W 为动态维度，暂按 %d 处理（联调时确认）",
                               kFallbackInputSize));
        }
        const size_t nOut = impl_->session->GetOutputCount();
        for (size_t i = 0; i < nOut; ++i) {
            auto nm = impl_->session->GetOutputNameAllocated(i, alloc);
            impl_->outputNames.push_back(nm.get());
        }
        ready_ = true;
        common::LogMsg(common::LINFO,
                       std::string("ONNX 分割会话已创建（") + (useCuda ? "CUDA" : "CPU") +
                           "）: " + aiCfg_.onnx_model +
                           Fmt("，输入尺寸 %dx%d", impl_->inputW, impl_->inputH));

        // 模型预热：首次推理要一次性完成 cuDNN 卷积策略搜索、显存分配器扩张与
        // CUDA 模块加载，不做预热会让现场第一帧测量出现秒级卡顿。此处用一张
        // 合成灰图先跑一次完整推理（计算图与输入内容无关，结果丢弃即可；
        // 纯灰图无产品检出属预期，InferCrop 返回 false 不代表预热失败）。
        // CPU 模式收益小但无害，预热抛异常只记 Warn，不影响就绪状态。
        {
            const auto t0 = std::chrono::steady_clock::now();
            cv::Mat dummy(impl_->inputH, impl_->inputW, CV_8UC1, cv::Scalar(128));
            cv::Mat dummyMask;
            std::string dummyDesc;
            try {
                InferCrop(dummy, dummyMask, dummyDesc);
                const long long ms =
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - t0).count();
                common::LogMsg(common::LINFO,
                               Fmt("模型预热完成，首次推理耗时 %lld ms"
                                   "（后续逐帧为稳定耗时）", ms));
            } catch (const std::exception& e) {
                common::LogMsg(common::LWARN,
                               std::string("模型预热失败（不影响后续推理）: ") + e.what());
            }
        }
    } catch (const Ort::Exception& e) {
        lastError_ = std::string("ONNX 会话创建失败: ") + e.what();
        common::LogMsg(common::LERROR, lastError_);
    } catch (const std::exception& e) {
        lastError_ = std::string("ONNX 会话创建异常: ") + e.what();
        common::LogMsg(common::LERROR, lastError_);
    }
}

OnnxSegmenter::~OnnxSegmenter() = default;

bool OnnxSegmenter::InferCrop(const cv::Mat& cropGray, cv::Mat& maskCrop,
                              std::string& detDesc) {
    ModelOutputs mo;
    if (!RunSession(*impl_->session, impl_->inputName, impl_->outputNames,
                    impl_->inputW, impl_->inputH, cropGray, mo)) {
        return false;
    }
    const float* scores = mo.tensors[mo.scoresIdx].GetTensorData<float>();
    const float* masks  = mo.tensors[mo.masksIdx].GetTensorData<float>();
    const int64_t Q  = mo.scoresShape[1];
    const int64_t C  = mo.scoresShape[2];
    const int64_t mh = mo.masksShape[2];
    const int64_t mw = mo.masksShape[3];

    // 逐查询：argmax 类须为盒子类（kBoxClass）才参与面积竞争。两类模型
    // （0=盒子，1=码区）下防止码区查询抢占盒子实例；旧单类模型 argmax 恒为 0，
    // 行为与旧版完全一致。置信度过阈即记入 descs（不论类别），便于漏检时排查
    constexpr int kBoxClass = 0;
    int bestQ = -1;
    long long bestArea = -1;
    std::string descs;
    for (int64_t q = 0; q < Q; ++q) {
        const float* qs = scores + q * C;
        float bestScore = 0.0f;
        int bestCls = 0;
        for (int64_t c = 0; c < C; ++c) {
            const float s = Sigmoid(qs[c]);
            if (s > bestScore) {
                bestScore = s;
                bestCls = (int)c;
            }
        }
        if (bestScore < aiCfg_.threshold) {
            continue;
        }
        if (!descs.empty()) {
            descs += " | ";
        }
        descs += Fmt("类%d:%.2f", bestCls, bestScore);
        if (bestCls != kBoxClass) {
            continue;
        }
        // 模型分辨率下统计前景面积（sigmoid > 0.5），用于选最大实例
        const float* qm = masks + q * mh * mw;
        long long area = 0;
        for (int64_t i = 0; i < mh * mw; ++i) {
            if (Sigmoid(qm[i]) > 0.5f) {
                ++area;
            }
        }
        if (area > bestArea) {
            bestArea = area;
            bestQ = (int)q;
        }
    }
    if (bestQ < 0) {
        detDesc = descs;
        return false;  // 无过阈检测
    }
    // 最大面积实例的掩膜：sigmoid -> resize 到 crop 尺寸 -> 0.5 二值化
    cv::Mat maskF((int)mh, (int)mw, CV_32FC1);
    const float* qm = masks + (int64_t)bestQ * mh * mw;
    for (int64_t i = 0; i < mh * mw; ++i) {
        maskF.ptr<float>()[i] = Sigmoid(qm[i]);
    }
    cv::Mat maskUp;
    cv::resize(maskF, maskUp, cropGray.size(), 0.0, 0.0, cv::INTER_LINEAR);
    cv::threshold(maskUp, maskUp, 0.5, 255.0, cv::THRESH_BINARY);
    maskUp.convertTo(maskCrop, CV_8UC1);
    detDesc = descs;
    return true;
}

bool OnnxSegmenter::InferClassMasks(const cv::Mat& gray, int targetClass, double threshold,
                                    std::vector<cv::Mat>& classMasks,
                                    std::vector<float>& classScores, std::string& desc) {
    classMasks.clear();
    classScores.clear();
    desc.clear();
    if (!ready_) {
        desc = "AI 分割器未就绪：" + lastError_;
        return false;
    }
    ModelOutputs mo;
    if (!RunSession(*impl_->session, impl_->inputName, impl_->outputNames,
                    impl_->inputW, impl_->inputH, gray, mo)) {
        return false;
    }
    const float* scores = mo.tensors[mo.scoresIdx].GetTensorData<float>();
    const float* masks  = mo.tensors[mo.masksIdx].GetTensorData<float>();
    const int64_t Q  = mo.scoresShape[1];
    const int64_t C  = mo.scoresShape[2];
    const int64_t mh = mo.masksShape[2];
    const int64_t mw = mo.masksShape[3];
    if (targetClass < 0 || targetClass >= (int)C) {
        desc = Fmt("模型输出仅 %d 类，无类别索引 %d 的通道"
                   "（AI 码区检测需换两类重训模型）", (int)C, targetClass);
        common::LogMsg(common::LWARN, desc);
        return false;
    }
    // 逐查询：argmax 类为目标类且该类置信度过阈才取（防止盒子查询凭借
    // 次强码区分数混入）；掩膜 sigmoid -> resize 到原图尺度 -> 0.5 二值化
    std::string descs;
    for (int64_t q = 0; q < Q; ++q) {
        const float* qs = scores + q * C;
        float bestScore = 0.0f;
        int bestCls = 0;
        for (int64_t c = 0; c < C; ++c) {
            const float s = Sigmoid(qs[c]);
            if (s > bestScore) {
                bestScore = s;
                bestCls = (int)c;
            }
        }
        const float targetScore = Sigmoid(qs[targetClass]);
        if (bestCls != targetClass || targetScore < (float)threshold) {
            continue;
        }
        cv::Mat maskF((int)mh, (int)mw, CV_32FC1);
        const float* qm = masks + q * mh * mw;
        for (int64_t i = 0; i < mh * mw; ++i) {
            maskF.ptr<float>()[i] = Sigmoid(qm[i]);
        }
        cv::Mat maskUp;
        cv::resize(maskF, maskUp, gray.size(), 0.0, 0.0, cv::INTER_LINEAR);
        cv::threshold(maskUp, maskUp, 0.5, 255.0, cv::THRESH_BINARY);
        cv::Mat maskBin;
        maskUp.convertTo(maskBin, CV_8UC1);
        if (cv::countNonZero(maskBin) == 0) {
            continue;
        }
        if (!descs.empty()) {
            descs += " | ";
        }
        descs += Fmt("类%d:%.2f", targetClass, targetScore);
        classMasks.push_back(maskBin);
        classScores.push_back(targetScore);
    }
    desc = descs;
    return true;
}

bool OnnxSegmenter::Segment(const cv::Mat& gray, const cv::Mat& background,
                            cv::Mat& mask, std::string& desc, RetCode* failCode) {
    auto fail = [&](RetCode code, const std::string& d) {
        if (failCode) {
            *failCode = code;
        }
        desc = d;
        return false;
    };
    if (!ready_) {
        return fail(RetCode::INTERNAL, "AI 分割器未就绪：" + lastError_);
    }
    // 1) 传统背景差分粗定位（ROI 级 100% 召回，局部边缘缺陷不影响）
    cv::Mat coarse;
    std::string err;
    if (!SegmentProduct(gray, background, tradCfg_, coarse, nullptr, err)) {
        return fail(RetCode::NO_PRODUCT, "传统粗定位失败：" + err);
    }
    // 2) 粗掩膜包围盒外扩 expand_ratio 后 crop（放大后目标占比大，召回质变）
    std::vector<cv::Point> nz;
    cv::findNonZero(coarse, nz);
    if (nz.empty()) {
        return fail(RetCode::NO_PRODUCT, "传统粗定位失败：粗掩膜为空");
    }
    const cv::Rect bbox = cv::boundingRect(nz);
    const int h = gray.rows, w = gray.cols;
    const int margin = (int)(std::max(bbox.width, bbox.height) * aiCfg_.expand_ratio);
    const int cx0 = std::max(0, bbox.x - margin);
    const int cy0 = std::max(0, bbox.y - margin);
    const int cx1 = std::min(w, bbox.x + bbox.width + margin);
    const int cy1 = std::min(h, bbox.y + bbox.height + margin);
    const cv::Rect roi(cx0, cy0, cx1 - cx0, cy1 - cy0);
    if (roi.width <= 0 || roi.height <= 0) {
        return fail(RetCode::NO_PRODUCT, "传统粗定位失败：ROI 非法");
    }
    const cv::Mat crop = gray(roi).clone();
    common::LogMsg(common::LDEBUG,
                   Fmt("ROI crop: [%d:%d, %d:%d] 尺寸 %dx%d", cy0, cy1, cx0, cx1,
                       roi.width, roi.height));

    // 3) ONNX 推理；漏检/推理失败不再退回粗掩膜兜底出数（避免非目标杂物被
    //    当成产品测量），按原因码上报由调用方复核
    cv::Mat maskCrop;
    std::string detDesc;
    bool ok = false;
    try {
        ok = InferCrop(crop, maskCrop, detDesc);
    } catch (const Ort::Exception& e) {
        return fail(RetCode::INTERNAL,
                    std::string("ONNX 推理异常: ") + e.what());
    }
    if (!ok || maskCrop.empty()) {
        common::LogMsg(common::LWARN, "ROI 内 AI 漏检，返回 AI_MISS 由调用方复核");
        return fail(RetCode::AI_MISS,
                    "ROI 内有物体但模型未识别为目标产品（板上无目标产品或模型失效），"
                    "需人工复核" +
                        (detDesc.empty() ? "" : "（检测明细: " + detDesc + "）"));
    }

    // 4) 掩膜映射回原图坐标，并与粗 ROI 取交集（防 AI 掩膜溢出到 ROI 外的杂物）
    mask = cv::Mat::zeros(h, w, CV_8UC1);
    maskCrop.copyTo(mask(roi));
    cv::Mat roiMask = cv::Mat::zeros(h, w, CV_8UC1);
    roiMask(roi).setTo(255);
    cv::bitwise_and(mask, roiMask, mask);
    desc = detDesc.empty() ? "ai" : ("ai | " + detDesc);
    return true;
}

}  // namespace cam
