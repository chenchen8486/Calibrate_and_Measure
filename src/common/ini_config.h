#pragma once
// ============================================================================
// ini_config.h —— 零依赖 ini 解析器与全工程配置结构体
// 约定：
//   1. ini 文本按 utf-8 读取，可容忍 utf-8-sig BOM（EF BB BF）；
//   2. 支持 [section]、key = value、# 与 ; 行注释及行尾注释、键值两侧空白修剪；
//   3. ini 中所有相对路径一律相对"ini 文件所在目录"解析（LoadFromIni 行为）；
//      旧行为"相对工程根目录（exe 上两级）"仅保留在 FindProjectRoot 与
//      单参 ResolvePath 中，供 demo 入口定位默认 config.ini 使用。
// ============================================================================

#include <map>
#include <string>

namespace common {

// ---------------------------------------------------------------------------
// IniFile：轻量 ini 文件解析器（零第三方依赖）
// ---------------------------------------------------------------------------
class IniFile {
public:
    // 加载并解析 ini 文件
    // @param path   ini 文件路径（utf-8，可带 BOM）
    // @param errMsg 输出参数：失败时的中文错误描述
    // @return 解析成功返回 true；文件打不开或格式错误返回 false
    bool Load(const std::string& path, std::string& errMsg);

    // 读取字符串值
    // @param section  段名（不含方括号）
    // @param key      键名
    // @param fallback 键缺失时返回的默认值
    // @return 修剪后的值字符串；不存在时返回 fallback
    std::string GetString(const std::string& section, const std::string& key,
                          const std::string& fallback) const;

    // 读取浮点值；解析失败返回 fallback
    double GetDouble(const std::string& section, const std::string& key,
                     double fallback) const;

    // 读取整数值；解析失败返回 fallback
    int GetInt(const std::string& section, const std::string& key,
               int fallback) const;

    // 读取布尔值；"1/true/yes/on"（大小写不敏感）为真，其余为假；缺失返回 fallback
    bool GetBool(const std::string& section, const std::string& key,
                 bool fallback) const;

    // 判断指定段中的指定键是否存在
    // @return 存在返回 true
    bool Has(const std::string& section, const std::string& key) const;

private:
    // 数据表：段名 -> (键名 -> 值)
    std::map<std::string, std::map<std::string, std::string>> data_;
};

// ---------------------------------------------------------------------------
// 路径定位工具
// ---------------------------------------------------------------------------

// 定位工程根目录
// 规则：GetModuleFileNameA 取 exe 路径 → 向上两级；
//       若该目录下不存在 config.ini，则退回 exe 所在目录。
// @return 工程根目录绝对路径（以反斜杠结尾）
std::string FindProjectRoot();

// 将 ini 中的路径解析为绝对路径
// 规则：p 为绝对路径（含 ':' 或以 '/'、'\\' 开头）时原样返回；
//       否则拼接到 FindProjectRoot() 得到的工程根之后。
// @param p ini 中配置的路径（相对或绝对）
// @return 解析后的绝对路径
std::string ResolvePath(const std::string& p);

// 将路径 p 相对指定基准目录解析为绝对路径（集成交付用：
// AppConfig::LoadFromIni 以 ini 文件所在目录为基准调用本重载，
// 调用方把 ini 放任意位置都能正确解析其中的相对路径）
// @param p       待解析路径（相对或绝对）
// @param baseDir 相对路径的基准目录（结尾有无分隔符均可）
// @return 解析后的绝对路径
std::string ResolvePath(const std::string& p, const std::string& baseDir);

// ---------------------------------------------------------------------------
// 全工程配置结构体（按功能分组，默认值即出厂参数）
// ---------------------------------------------------------------------------

// [checkerboard] 功能 1：棋盘格标定板生成
struct CheckerboardConfig {
    double square_mm  = 15.0;   // 单格边长（毫米），决定打印后的物理尺寸
    int    cols       = 35;     // 横向格数（列数）
    int    rows       = 28;     // 纵向格数（行数）
    double border_mm  = 15.0;   // 棋盘四周留白边距（毫米），保证角点提取不受纸边干扰
    double paper_w_mm = 555.0;  // 纸张宽度（毫米）
    double paper_h_mm = 450.0;  // 纸张高度（毫米）
    std::string out_dir = "assets/checkerboard";  // 标定板图像输出目录（相对工程根）
};

// [calibrate] 功能 2：相机标定求解
struct CalibrateConfig {
    std::string input_dir = "assets/test_data/calibration";  // 标定采图目录（相对 ini 目录，菜单功能 2 用）
    int    pattern_cols = 34;   // 内角点列数；缺省跟随 [checkerboard] cols - 1（显式配置优先）
    int    pattern_rows = 27;   // 内角点行数；缺省跟随 [checkerboard] rows - 1（显式配置优先）
    double square_x_mm  = 15.0; // 实测横向格距（毫米）；缺省跟随 [checkerboard] square_mm，
                                // 打印后用卡尺实测回填实际值
    double square_y_mm  = 15.0; // 实测纵向格距（毫米），同上
    std::string out_xml = "assets/calibration/checkerboard_calib.xml";  // 标定结果输出文件（相对 ini 目录）
    double target_mm_per_px = 0.15;  // 标定后期望的像素当量（毫米/像素），用于 QA 评估
    std::string qa_dir = "temp/calibration_qa";  // 标定质量检查图像输出目录（相对 ini 目录）
};

// [rectify] 功能 3 之 0)：图像几何矫正开关
struct RectifyConfig {
    bool enabled = false;  // 是否启用畸变矫正（标定未完成时可关闭）
    std::string calib_xml = "assets/calibration/checkerboard_calib.xml";  // 标定参数文件；
                                // 缺省跟随 [calibrate] out_xml（相对 ini 目录，显式配置优先）
    double target_mm_per_px = 0.15;  // 矫正后期望的像素当量（毫米/像素）；
                                // 缺省跟随 [calibrate] target_mm_per_px
};

// [paths] 通用路径
struct PathsConfig {
    std::string input_dir       = "assets/test_data/input";    // 待测图像输入目录（相对工程根）
    std::string output_dir      = "assets/test_data/output";   // 测量结果输出目录（相对工程根）
    std::string background_file = "temp/background_model.bmp"; // 背景建模参考图缓存（相对工程根）
};

// [segmentation] 分割方式选择
struct SegmentationConfig {
    std::string method = "ai";  // 分割方法："ai"=ONNX 模型分割，"traditional"=传统背景差分
};

// [ai_seg] AI 分割（ONNX Runtime 推理）
struct AiSegConfig {
    std::string onnx_model = "assets/weights/small.onnx";  // ONNX 模型文件（相对 ini 目录）
    double threshold    = 0.3;   // 前景概率阈值（0~1），越低召回越高
    double expand_ratio = 0.08;  // 掩码外扩比例（相对目标尺寸），保证边缘完整保留
    bool   refine       = true;  // 是否在 AI 掩码基础上做形态学精修
    std::string device  = "auto"; // 推理设备："auto"=检测到可用 CUDA 12 环境才用 GPU，
                                  // 否则静默回退 CPU；"cuda"=强制 GPU；"cpu"=强制 CPU
};

// [trad_seg] 传统背景差分分割
struct TradSegConfig {
    int    bilateral_d       = 21;    // 双边滤波邻域直径（像素）
    double sigma_color       = 15.0;  // 双边滤波颜色域标准差
    double sigma_space       = 15.0;  // 双边滤波空间域标准差
    double gamma             = 0.5;   // 伽马校正指数（<1 提亮暗部差异）
    double diff_floor        = 7.0;   // 差分下限（灰度级），低于该值视为背景噪声
    double diff_ceiling      = 25.0;  // 差分上限（灰度级），达到该值视为确定前景
    int    morph_open_kernel = 5;     // 开运算核尺寸（像素），去除细小噪点
    int    morph_close_kernel = 61;   // 闭运算核尺寸（像素），填补前景内部空洞
    double min_area_ratio    = 0.005; // 有效前景最小面积占比（相对整幅图像）
};

// [refine] 边缘精修（亚像素轮廓提取）
struct RefineConfig {
    double step_window        = 10.0; // 阶跃边缘拟合窗口半宽（像素）
    int    sample_step        = 6;    // 轮廓采样间隔（像素）
    double search_radius      = 25.0; // 法向搜索半径（像素）
    double profile_step       = 0.5;  // 灰度剖面采样步长（像素）
    double gradient_threshold = 6.0;  // 有效边缘最小梯度幅值（灰度级/像素）
    double peak_ratio         = 0.35; // 次峰相对主峰的最小比值，用于剔除伪边缘
    double outlier_tol        = 3.0;  // 离群点剔除容差（像素）
    int    median_window      = 9;    // 中值平滑窗口长度（采样点个数，取奇数）
};

// [rotate] 角度校正
struct RotateConfig {
    double approx_epsilon_ratio = 0.004; // 多边形逼近系数（相对轮廓周长）
    int    edge_band            = 8;     // 边缘带宽度（像素），用于提取直线段附近的轮廓点
};

// [measure] 边缘测量
struct MeasureConfig {
    double extreme_band         = 5.0;   // 极值判定带宽（像素）
    int    refine_rows          = 15;    // 精修参与行数
    double ransac_threshold     = 2.0;   // RANSAC 直线拟合内点阈值（像素）
    double edge_width_threshold = 300.0; // 边缘条最大宽度（像素），超过则判为异常
    double angle_tol_deg        = 3.0;   // 共线判定角度容差（度）
    int    tangent_span         = 5;     // 切线方向估计的邻域点数
    double line_dist_tol        = 7.0;   // 点到直线距离容差（像素）
    double span_gap_tol         = 15.0;  // 跨度间隙容差（像素）
    int    merge_gap_pts        = 10;    // 边缘段合并允许的最大断点间隔（点数）
    double ranked_region_ratio  = 0.5;   // 有效排名区域比例（相对边缘总长）
};

// [debug] 中间结果落盘开关（功能 2 的 QA 质检图 + 功能 3 的调试图）
struct DebugConfig {
    // true  = 保存中间结果：标定 QA 质检图（calibrate.qa_dir）、
    //         每张测量图的分割掩膜/增强图/旋转图/测量叠加图（paths.output_dir/debug/）；
    // false = 部署模式，只输出最终结果（标定 XML、测量 CSV），不落任何过程图。
    // 注意：背景模型缓存 paths.background_file 属于功能性缓存，不受本开关控制。
    bool save_intermediate = true;
};

// 全工程总配置
struct AppConfig {
    CheckerboardConfig  checkerboard;  // 功能 1：制板
    CalibrateConfig     calibrate;     // 功能 2：标定
    RectifyConfig       rectify;       // 功能 3：几何矫正
    PathsConfig         paths;         // 通用路径
    SegmentationConfig  segmentation;  // 分割方式
    AiSegConfig         ai_seg;        // AI 分割参数
    TradSegConfig       trad_seg;      // 传统分割参数
    RefineConfig        refine;        // 边缘精修参数
    RotateConfig        rotate;        // 角度校正参数
    MeasureConfig       measure;       // 测量参数
    DebugConfig         debug;         // 中间结果落盘开关

    // 从 ini 文件加载配置；缺失的段/键保留默认值
    // 路径类字段读取后自动经 ResolvePath 转为绝对路径，
    // 相对路径以 ini 文件所在目录为基准
    // @param iniPath ini 文件路径
    // @param errMsg  输出参数：失败时的中文错误描述
    // @return 加载成功返回 true
    bool LoadFromIni(const std::string& iniPath, std::string& errMsg);
};

}  // namespace common
