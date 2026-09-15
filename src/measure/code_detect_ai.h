#pragma once
// ============================================================================
// measure/code_detect_ai.h —— AI 码区检测（两类分割模型的码区类 + 后处理）
// ----------------------------------------------------------------------------
// 路线：IAiSegmenter::InferClassMasks 整图推理取码区类实例掩膜 ->
//   连通域 -> 面积双下限 / 矩形度 / 长宽比过滤 -> 同码区多查询按 IoU 去重 ->
//   按面积降序取前 max_count 个 -> 公共入口 AcceptCodeQuad 映射进校正坐标系
//   （与传统三层链同一坐标约定：检测在未旋转原图上做，检出框按校正角仿射
//   映射回去，中心须落在产品轮廓内）。
// 类型启发式：长短边比 <= qr_aspect_tol 报 QR，否则报 BAR；
// confidence 填该连通域所属查询的类别置信度（0~1，越高越可信）。
// 未检出返回空 vector，不算错误；模型未就绪 / 无码区类别通道（旧的单类
// 模型）/ 推理异常返回 false 并记 Warn，由调用方回退传统三层链。
// ============================================================================

#include <vector>

#include <opencv2/core.hpp>

#include "common/ini_config.h"
#include "measure_types.h"

namespace cam {

class IAiSegmenter;  // 前向声明（实现在 measure/segment_ai.h）

// AI 支路检测产品表面的码区。
//
// Args:
//   seg           已就绪的 AI 分割器（与盒子分割共享同一会话）；
//   gray          原始输入灰度图（8UC1，未经旋转/增强）；
//   rotContour    旋转校正后的产品外轮廓（过滤用，与输出同坐标系）；
//   totalAngleDeg 总校正角（度），检出框按此角绕图心解析映射进校正坐标系；
//   cfg           [code_detect] 配置（enabled / min_area_px）；
//   aiCfg         [code_detect_ai] 后处理配置；
//   regions       输出参数：检出的码区列表（校正坐标系，未做小数修约）。
//
// Returns:
//   正常完成返回 true（含 enabled=false 与零检出）；
//   分割器未就绪 / 模型无码区类别通道 / 推理异常返回 false，
//   regions 已清空，失败原因已记 Warn。
bool DetectCodeRegionsAi(IAiSegmenter& seg,
                         const cv::Mat& gray,
                         const std::vector<cv::Point>& rotContour,
                         double totalAngleDeg,
                         const common::CodeDetectConfig& cfg,
                         const common::CodeDetectAiConfig& aiCfg,
                         std::vector<CodeRegion>& regions);

}  // namespace cam
