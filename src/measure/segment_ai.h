#pragma once
// ============================================================================
// measure/segment_ai.h —— AI 分割（ONNX Runtime 推理）
// ----------------------------------------------------------------------------
// 对照 Python 工程 scripts/predict_seg.py 的 predict_mask_with_roi 流程：
//   传统背景差分粗定位（ROI 级 100% 召回）-> bbox 外扩 -> crop 放大推理 ->
//   最大面积实例掩膜映射回原图 -> 与 ROI 取交集；漏检时回退传统粗掩膜。
// ============================================================================

#include <memory>
#include <string>

#include <opencv2/core.hpp>

#include "common/ini_config.h"

namespace cam {

// AI 分割器抽象接口
class IAiSegmenter {
public:
    virtual ~IAiSegmenter() = default;

    // 对灰度图分割产品掩膜。
    //
    // Args:
    //   gray       8UC1 灰度图。
    //   background 背景模型（用于传统粗定位）。
    //   mask       输出参数：产品二值掩膜（H, W），前景 255。
    //   desc       输出参数：来源描述（"ai" / "fallback" + 检测置信度信息）。
    //
    // Returns:
    //   成功返回 true（含 fallback 兜底成功）；粗定位失败或分割器未就绪返回 false。
    virtual bool Segment(const cv::Mat& gray, const cv::Mat& background,
                         cv::Mat& mask, std::string& desc) = 0;
};

// ONNX Runtime 实现的 RF-DETR-seg 分割器。
// 构造即加载模型：CUDA EP 注册失败自动回退 CPU 并记 Warn；
// 模型缺失/加载失败时 IsReady() 为 false，由调用方回退传统分割。
class OnnxSegmenter : public IAiSegmenter {
public:
    // Args:
    //   aiCfg   AI 分割参数（模型路径/阈值/外扩比例/设备）。
    //   tradCfg 传统分割参数（ROI 粗定位复用 SegmentProduct）。
    OnnxSegmenter(const common::AiSegConfig& aiCfg, const common::TradSegConfig& tradCfg);
    ~OnnxSegmenter() override;

    // 模型会话是否就绪
    bool IsReady() const { return ready_; }
    // 最近一次初始化失败的中文原因
    const std::string& LastError() const { return lastError_; }

    bool Segment(const cv::Mat& gray, const cv::Mat& background,
                 cv::Mat& mask, std::string& desc) override;

private:
    // ROI crop 内推理 + 后处理，产出 crop 尺寸的二值掩膜
    // （预处理/后处理的张量布局约定集中在 segment_ai.cpp 的注释块中，联调时对齐）
    bool InferCrop(const cv::Mat& cropGray, cv::Mat& maskCrop, std::string& detDesc);

    struct Impl;  // pImpl：隔离 onnxruntime 头文件
    std::unique_ptr<Impl> impl_;

    common::AiSegConfig   aiCfg_;
    common::TradSegConfig tradCfg_;
    bool        ready_ = false;
    std::string lastError_;
};

}  // namespace cam
