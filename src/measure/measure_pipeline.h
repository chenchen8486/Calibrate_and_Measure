#pragma once
// ============================================================================
// measure/measure_pipeline.h —— 功能 3（图像测量）流水线门面
// ----------------------------------------------------------------------------
// 调用方唯一需要看的头文件。对齐 Python 工程 pipeline.py 的 process_single：
//   分割（AI/传统）-> 两级旋转校正 -> 语义宽高测量 + 水平边排名。
// 本类只做"纯测量流程"：不关心图像是否经过几何矫正（棋盘格去畸变/去倾斜），
// 需要几何矫正时由调用方先调 Rectifier 再传入矫正后图像。
// 注意：rectify.enabled=true 时 Init 会把背景模型用同一标定文件同步正射校正，
// 此时调用方必须先用同一标定文件 Rectify 输入图再调 Measure（与 Python 版
// pipeline.py run_batch 中 gray/background 同过一张 remap 表的行为对齐）。
// ============================================================================

#include <memory>
#include <string>

#include <opencv2/core.hpp>

#include "common/ini_config.h"
#include "measure_types.h"

namespace cam {

class IAiSegmenter;  // 前向声明（实现在 measure/segment_ai.h）

// 图像测量流水线（功能 3 门面）
class MeasurePipeline {
public:
    MeasurePipeline();
    ~MeasurePipeline();

    // 加载 iniPath 全部测量配置并初始化：
    //   1) LoadFromIni 解析配置（相对路径自动转绝对路径）；
    //   2) 背景建模：paths.input_dir 全量图 + paths.background_file 缓存；
    //   3) 分割器初始化：segmentation.method=="ai" 或 code_detect.method=="ai"
    //      任一启用即创建 ONNX 会话（同一个两类模型一次加载，盒子分割取类 0、
    //      码区检测取 code_detect_ai.code_class）；模型缺失/加载失败时两路
    //      各自自动回退传统实现并记 Warn。
    //
    // Args:
    //   iniPath ini 配置文件路径。
    //   errMsg  输出参数：失败时的中文错误描述。
    //
    // Returns:
    //   初始化成功返回 true。
    bool Init(const std::string& iniPath, std::string& errMsg);

    // 流水线是否就绪（Init 成功后为 true）
    bool IsReady() const;

    // 实际生效的分割方式是否为 AI 链（segmentation.method=="ai" 但模型
    // 缺失/加载失败时会自动回退传统分割，此时返回 false；供门面层上报）
    bool UsingAi() const { return useAi_; }

    // 纯测量流程：输入一帧图像，输出 5 个测量值
    // （widthPx / heightPx / 上半部分候选水平边列表）在 MeasureOutput 中。
    //
    // Args:
    //   image      输入图像，8UC1/8UC3/8UC4。
    //   debugTag   调试图标识（一般用图像文件名去扩展名）。
    //              仅当 ini [debug] save_intermediate=true 且本参数非空时，
    //              向 paths.output_dir/debug/<debugTag>/ 落分割掩膜、增强图、
    //              旋转图与测量叠加图；否则不产生任何中间文件。
    //   basisImage 可选输出参数：两级旋转校正后的测量基准图（灰度 8UC1，
    //              未画任何标注），即宽高与水平边实际测量的那张图。
    //              本层只做旋转校正，是否已含正射矫正由调用方传入的 image
    //              决定；传 nullptr 不产生额外开销，旋转前失败（无产品、
    //              背景未就绪等）时输出为空 Mat。
    //
    // Returns:
    //   MeasureOutput：code==OK 时 width/height/horizontalEdges/codeRegions 有效；
    //   失败时 code 取 EMPTY_IMAGE / BAD_FORMAT / NO_PRODUCT / INTERNAL，
    //   message 附中文原因。
    MeasureOutput Measure(const cv::Mat& image, const std::string& debugTag = "",
                          cv::Mat* basisImage = nullptr);

    // 现场学习背景：用一帧空背板灰度图（8UC1）替换背景模型，并尝试写回
    // paths.background_file 缓存（写失败仅 Warn，本次会话仍生效）。
    // 矫正开启时用同一标定文件同步正射校正（语义与 Init 背景建模一致）。
    // 换机/开班时调一次即可，免放 input_dir 图、免手工维护缓存文件。
    // 须在 Init 成功后调用。
    //
    // Args:
    //   gray   空背板灰度图（8UC1，板上无任何产品；矫正开启时尺寸须与标定采图一致）。
    //   errMsg 输出参数：失败时的中文原因。
    //
    // Returns:
    //   背景设置成功返回 true。
    bool SetBackground(const cv::Mat& gray, std::string& errMsg);

    // 全量配置（调试/可视化用）
    const common::AppConfig& Config() const { return cfg_; }

private:
    common::AppConfig cfg_;              // 全工程配置
    cv::Mat           background_;       // 背景模型（8UC1）
    std::unique_ptr<IAiSegmenter> aiSeg_;    // AI 分割器（任一支路启用 ai 且就绪时非空）
    bool              useAi_  = false;   // 是否实际使用 AI 分割（盒子掩膜链路）
    bool              codeAi_ = false;   // 码区检测是否配置为 AI 支路（code_detect.method=="ai"）
    bool              ready_  = false;   // 初始化完成标记
};

}  // namespace cam
