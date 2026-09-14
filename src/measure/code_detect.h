#pragma once
// ============================================================================
// measure/code_detect.h —— 码区检测（产品表面二维码/一维码外接矩形定位）
// ----------------------------------------------------------------------------
// 实现路线（三层，置信度递减）：
//   二维码：cv::QRCodeDetector（detectAndDecodeMulti），解码成功 1.0 / 仅定位 0.5；
//   一维码（解码体系）：cv::barcode::BarcodeDetector（detectAndDecodeWithType，
//           EAN/UPC 系可解码 1.0 / 仅定位 0.5），0°/90° 两档命中即停；
//   一维码（条纹兜底）：仅当解码体系零检出时运行，形态学定位致密平行条纹区域，
//           覆盖 Code128/药品监管码等解码体系外条码，候选置信度 0.3（未确认）。
// 本模块只取外接矩形，解码成功与否只影响置信度。
// 关键工程结论（实测，5.5K 图像 + EAN-13 竖条码）：
//   检测必须在【未做任何旋转/增强的原始灰度图】上进行。旋转插值（含 1° 的
//   INTER_LINEAR warp）会把条码细条纹抗锯齿平滑掉，检测器全尺度 0 命中；
//   原图同尺度正常检出并解码。检出四角点再按校正角解析映射进旋转校正坐标系
//   （点是精确变换，不涉及像素插值），最终输出与宽高/水平边同一坐标系。
// 误报过滤（码一定印在产品上）：
//   1. 外接矩形中心必须落在产品轮廓内；
//   2. 面积下限过滤噪点候选。
// 条码检测对方向与尺度敏感（实测竖条码在 0° 档命中、同一图转 90° 反而漏检），
// 因此条码按 0°/90° 两档各检一次，命中即停；二维码检测器旋转不变，单档。
// 未检出返回空 vector，不算错误；检测器内部异常一律吞掉并记 Warn，
// 绝不拖累宽高与水平边主输出。
// ============================================================================

#include <vector>

#include <opencv2/core.hpp>

#include "common/ini_config.h"
#include "measure_types.h"

namespace cam {

// 检测产品表面的码区。
//
// Args:
//   gray          原始输入灰度图（8UC1，未经旋转/增强）；
//   rotContour    旋转校正后的产品外轮廓（过滤用，与输出同坐标系）；
//   totalAngleDeg 总校正角（度），检出框按此角绕图心解析映射进校正坐标系；
//   cfg           [code_detect] 配置；enabled=false 时直接返回空；
//   regions       输出参数：检出的码区列表（校正坐标系，未做小数修约）。
//
// Returns:
//   正常完成返回 true（含 enabled=false 与零检出）；
//   检测器异常时返回 false，regions 已清空，失败原因已记 Warn。
bool DetectCodeRegions(const cv::Mat& gray,
                       const std::vector<cv::Point>& rotContour,
                       double totalAngleDeg,
                       const common::CodeDetectConfig& cfg,
                       std::vector<CodeRegion>& regions);

}  // namespace cam
