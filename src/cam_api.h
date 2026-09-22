#pragma once
// ============================================================================
// cam_api.h —— Calibrate_and_Measure 对外统一门面（集成交付唯一入口）
// ----------------------------------------------------------------------------
// 软件同事接入本工程只需要 include 本头文件。三大功能：
//   功能 1 制板：MakeBoard        —— 生成标定板打印三件套（PDF/预览图/打印说明）
//   功能 2 标定：Calibrate        —— 采图求解标定 XML，返回机器可读质量摘要
//   功能 3 测量：Measurer 类      —— Init 一次（重资源）后逐帧 Measure
//
// 设计约定：
//   1. 所有函数以 ini 配置文件路径为配置入口，显式传参，无任何隐式全局状态；
//      ini 内的相对路径一律相对 ini 文件所在目录解析，ini 可放任意位置。
//   2. 错误模型统一：初始化类调用返回 bool + errMsg（中文原因）；
//      逐帧测量返回 MeasureOutput（code 四档 + message），见 measure_types.h。
//   3. 几何矫正（rectify.enabled=true 时）由 Measurer 内部完成，调用方只传
//      相机原图，不需要也不应该自己先调 Rectifier（标定文件一致性由门面保证）。
//   4. Measurer 非线程安全，多相机场景请每相机一个实例并各自串行调用。
// ============================================================================

#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "calibration/calibrator.h"    // CalibReport
#include "calibration/rectifier.h"     // Rectifier
#include "common/ini_config.h"         // common::AppConfig
#include "measure/measure_pipeline.h"  // MeasurePipeline
#include "measure_types.h"             // MeasureOutput

namespace cam {

// 门面版本号（随接口变更递增）
// @return 版本串，当前 "2.1.3"
const char* Version();

// ---------------------------------------------------------------------------
// 功能 1：生成棋盘格标定板打印文件
// 读取 iniPath 的 [checkerboard] 段，在 out_dir 下生成：
//   checkerboard.pdf（矢量打印）、checkerboard_preview.png（人工核对）、
//   打印说明.txt（交打印店）
// @param iniPath 配置文件路径
// @param errMsg  输出参数：失败时的中文原因
// @return 成功返回 true
// ---------------------------------------------------------------------------
bool MakeBoard(const std::string& iniPath, std::string& errMsg);

// ---------------------------------------------------------------------------
// 功能 2：棋盘格标定求解
// 传入 1~n 张棋盘格采图（多图固定机位，角点自动取均值降噪），求解并写出
// 标定 XML（[calibrate] out_xml），同时输出 QA 质检图与验证闭环统计。
// 质量门禁：RMS ≤ 0.3px 且正射验证 mean ≤ 0.5px；越限返回 false 但
// 标定文件与 QA 图仍保留，report 内数值可用于上位机展示偏差。
// @param boardImages 棋盘格采图列表（8UC1/8UC3/8UC4，尺寸须一致）
// @param iniPath     配置文件路径
// @param report      可选输出参数：标定摘要（rms/验证残差/输出路径），
//                    传 nullptr 忽略；失败时已知字段也会尽量填写
// @param errMsg      输出参数：失败或质量偏低时的中文描述
// @return 标定成功且质量达标返回 true
// ---------------------------------------------------------------------------
bool Calibrate(const std::vector<cv::Mat>& boardImages, const std::string& iniPath,
               CalibReport* report, std::string& errMsg);

// ---------------------------------------------------------------------------
// 功能 3：测量会话（重资源持有者）
// 生命周期：构造 → Init（一次性，秒级）→ 逐帧 Measure → 析构自动释放。
// ---------------------------------------------------------------------------
class Measurer {
public:
    Measurer();
    ~Measurer();
    Measurer(const Measurer&) = delete;
    Measurer& operator=(const Measurer&) = delete;

    // 一次性初始化（首次调用耗时秒级，之后 Measure 为正常单帧耗时）：
    //   1) 加载 iniPath 全部配置（相对路径按 ini 所在目录解析）；
    //   2) 背景加载：仅加载 paths.background_file 缓存；缓存缺失/不可读
    //      不阻断，记 Warn 进入背景未就绪态（Measure 返回 NO_BACKGROUND），
    //      由 SetBackground 现场学习补学——生产唯一建模入口，Init 不再
    //      使用 input_dir 现建背景（中位数现建仅 demo 保留）；
    //   3) 分割器：segmentation.method=="ai" 时创建 ONNX 会话并完成
    //      预热（消除首帧卡顿）；模型缺失/加载失败自动回退传统分割
    //      并记 Warn，Init 仍成功（可用 UsingAi() 确认实际生效链路）；
    //   4) rectify.enabled=true 时加载标定 XML 构建正射 remap 表；
    //      相机未标定/标定文件缺失时记 Warn 降级为未矫正运行，Init 仍成功，
    //      测量照常（结果仅像素值），用 RectifyEnabled() 确认实际状态。
    // @param iniPath 配置文件路径
    // @param errMsg  输出参数：失败时的中文原因
    // @return 初始化成功返回 true
    bool Init(const std::string& iniPath, std::string& errMsg);

    // 是否就绪（Init 成功后为 true）
    bool IsReady() const { return ready_; }

    // 单帧测量：内部按需完成 正射矫正 → 分割 → 两级旋转校正 →
    // 宽高测量 + 上半部分水平边排名 + 码区（二维码/一维码）定位。
    // 调用方只传相机原图。
    // @param image      输入图像，8UC1/8UC3/8UC4；矫正开启时尺寸须与标定采图一致
    // @param debugTag   调试图标识（一般用图像名去扩展名）；仅当 ini [debug]
    //                   save_intermediate=true 且本参数非空时落过程图，
    //                   部署置 false 即零中间文件
    // @param basisImage 可选输出参数：旋转校正后的测量基准图（灰度 8UC1，
    //                   未画任何标注，即宽高与水平边测量实际使用的图）。
    //                   [rectify] enabled=true 且标定可用时为正射矫正 + 旋转
    //                   校正后的图，关闭或标定不可用时为原始灰度 + 旋转校正
    //                   后的图（同一条调用路径）。传 nullptr 无额外开销；
    //                   旋转前失败（背景未就绪/无产品等）时输出为空 Mat
    // @return MeasureOutput：code==OK 时 width/height/horizontalEdges/codeRegions 有效
    MeasureOutput Measure(const cv::Mat& image, const std::string& debugTag = "",
                          cv::Mat* basisImage = nullptr);

    // 现场学习背景：用一帧空背板图设置背景模型（换机/开班时调一次，
    // 生产环境唯一建模入口，免放 input_dir 图、免手工维护缓存文件）。
    // 内部：转灰度 → 矫正开启时同步正射校正 → 写入内存背景 → 尝试落盘
    // paths.background_file 缓存（写失败仅 Warn，本次会话仍生效；
    // 写成功则下次 Init 直接加载复用）。
    // 须在 Init 成功后调用；Init 处于背景未就绪态时亦可调用，调用后即就绪。
    // @param image  空背板图（8UC1/8UC3/8UC4，板上无任何产品）
    // @param errMsg 输出参数：失败时的中文原因
    // @return 背景设置成功返回 true
    bool SetBackground(const cv::Mat& image, std::string& errMsg);

    // 几何矫正是否生效
    bool RectifyEnabled() const;

    // 毫米换算系数：毫米 = 像素 × MmPerPx()；矫正关闭时返回 0（仅像素结果）
    double MmPerPx() const;

    // 实际生效的分割是否 AI 链（AI 加载失败回退传统后为 false）
    bool UsingAi() const;

    // 全量配置（只读；调试/上位机展示用）
    const common::AppConfig& Config() const;

private:
    MeasurePipeline pipe_;       // 纯测量流水线
    Rectifier       rectifier_;  // 正射矫正器（rectify.enabled=true 时就绪）
    bool            ready_ = false;
};

}  // namespace cam
