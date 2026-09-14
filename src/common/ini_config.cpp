// ============================================================================
// ini_config.cpp —— ini 解析器与全工程配置的实现
// ============================================================================

#include "ini_config.h"

#include <windows.h>  // GetModuleFileNameA

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <fstream>
#include <sstream>

#include "logger.h"

namespace common {
namespace {

// 去除字符串两侧空白字符
// @param s 原始字符串
// @return 修剪后的字符串
std::string Trim(const std::string& s) {
    size_t b = s.find_first_not_of(" \t\r\n");
    if (b == std::string::npos) {
        return "";
    }
    size_t e = s.find_last_not_of(" \t\r\n");
    return s.substr(b, e - b + 1);
}

// 判断路径是否为绝对路径：含 ':'（如 D:/...）或以 '/'、'\\' 开头
// @param p 待判断路径
// @return 绝对路径返回 true
bool IsAbsolutePath(const std::string& p) {
    if (p.empty()) {
        return false;
    }
    if (p.find(':') != std::string::npos) {
        return true;
    }
    return p.front() == '/' || p.front() == '\\';
}

// 返回路径的上一级目录（兼容 '/' 与 '\\'）
// @param dir 目录路径（可带或不带结尾分隔符）
// @return 上一级目录；已到根时原样返回
std::string ParentDir(const std::string& dir) {
    std::string d = dir;
    while (!d.empty() && (d.back() == '/' || d.back() == '\\')) {
        d.pop_back();
    }
    size_t pos = d.find_last_of("/\\");
    if (pos == std::string::npos) {
        return d;
    }
    return d.substr(0, pos);
}

// 判断文件是否存在
// @param path 文件路径
// @return 存在返回 true
bool FileExists(const std::string& path) {
    DWORD attr = GetFileAttributesA(path.c_str());
    return attr != INVALID_FILE_ATTRIBUTES && !(attr & FILE_ATTRIBUTE_DIRECTORY);
}

}  // namespace

// ---------------------------------------------------------------------------
// IniFile 实现
// ---------------------------------------------------------------------------

bool IniFile::Load(const std::string& path, std::string& errMsg) {
    std::ifstream ifs(path, std::ios::binary);
    if (!ifs.is_open()) {
        errMsg = "无法打开配置文件: " + path;
        LogMsg(LERROR, errMsg);
        return false;
    }
    try {
        std::stringstream ss;
        ss << ifs.rdbuf();
        std::string content = ss.str();
        // 跳过 utf-8-sig BOM（EF BB BF）
        if (content.size() >= 3 &&
            static_cast<unsigned char>(content[0]) == 0xEF &&
            static_cast<unsigned char>(content[1]) == 0xBB &&
            static_cast<unsigned char>(content[2]) == 0xBF) {
            content.erase(0, 3);
        }

        data_.clear();
        std::string section;  // 当前段名，段外键直接忽略
        std::istringstream lines(content);
        std::string line;
        int lineNo = 0;
        while (std::getline(lines, line)) {
            ++lineNo;
            // 去掉行尾注释：';' 与 '#' 起注释
            size_t c1 = line.find('#');
            size_t c2 = line.find(';');
            size_t cut = std::min(c1 == std::string::npos ? line.size() : c1,
                                  c2 == std::string::npos ? line.size() : c2);
            std::string s = Trim(line.substr(0, cut));
            if (s.empty()) {
                continue;
            }
            // 段头 [section]
            if (s.front() == '[') {
                size_t rb = s.find(']');
                if (rb == std::string::npos || rb <= 1) {
                    errMsg = "配置文件第 " + std::to_string(lineNo) + " 行段头格式错误: " + s;
                    LogMsg(LERROR, errMsg);
                    return false;
                }
                section = Trim(s.substr(1, rb - 1));
                continue;
            }
            // 键值对 key = value
            size_t eq = s.find('=');
            if (eq == std::string::npos) {
                LogMsg(LWARN, "配置文件第 " + std::to_string(lineNo) +
                              " 行缺少 '='，已忽略: " + s);
                continue;
            }
            if (section.empty()) {
                LogMsg(LWARN, "配置文件第 " + std::to_string(lineNo) +
                              " 行位于任何段之外，已忽略: " + s);
                continue;
            }
            std::string key = Trim(s.substr(0, eq));
            std::string val = Trim(s.substr(eq + 1));
            if (key.empty()) {
                LogMsg(LWARN, "配置文件第 " + std::to_string(lineNo) +
                              " 行键名为空，已忽略");
                continue;
            }
            data_[section][key] = val;
        }
    } catch (const std::exception& e) {
        errMsg = "解析配置文件异常: " + path + "，原因: " + e.what();
        LogMsg(LERROR, errMsg);
        return false;
    }
    LogMsg(LINFO, "配置文件加载成功: " + path);
    return true;
}

std::string IniFile::GetString(const std::string& section, const std::string& key,
                               const std::string& fallback) const {
    auto si = data_.find(section);
    if (si == data_.end()) {
        return fallback;
    }
    auto ki = si->second.find(key);
    if (ki == si->second.end()) {
        return fallback;
    }
    return ki->second;
}

double IniFile::GetDouble(const std::string& section, const std::string& key,
                          double fallback) const {
    std::string v = GetString(section, key, "");
    if (v.empty()) {
        return fallback;
    }
    try {
        size_t used = 0;
        double d = std::stod(v, &used);
        if (used != v.size()) {
            LogMsg(LWARN, "配置项 [" + section + "] " + key + " 数值含多余字符: " + v);
        }
        return d;
    } catch (const std::exception&) {
        LogMsg(LWARN, "配置项 [" + section + "] " + key +
                      " 无法解析为浮点数: " + v + "，使用默认值");
        return fallback;
    }
}

int IniFile::GetInt(const std::string& section, const std::string& key,
                    int fallback) const {
    std::string v = GetString(section, key, "");
    if (v.empty()) {
        return fallback;
    }
    try {
        size_t used = 0;
        int d = std::stoi(v, &used);
        if (used != v.size()) {
            LogMsg(LWARN, "配置项 [" + section + "] " + key + " 数值含多余字符: " + v);
        }
        return d;
    } catch (const std::exception&) {
        LogMsg(LWARN, "配置项 [" + section + "] " + key +
                      " 无法解析为整数: " + v + "，使用默认值");
        return fallback;
    }
}

bool IniFile::GetBool(const std::string& section, const std::string& key,
                      bool fallback) const {
    std::string v = GetString(section, key, "");
    if (v.empty()) {
        return fallback;
    }
    std::string low = v;
    std::transform(low.begin(), low.end(), low.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (low == "1" || low == "true" || low == "yes" || low == "on") {
        return true;
    }
    if (low == "0" || low == "false" || low == "no" || low == "off") {
        return false;
    }
    LogMsg(LWARN, "配置项 [" + section + "] " + key +
                  " 无法解析为布尔值: " + v + "，使用默认值");
    return fallback;
}

bool IniFile::Has(const std::string& section, const std::string& key) const {
    auto si = data_.find(section);
    if (si == data_.end()) {
        return false;
    }
    return si->second.find(key) != si->second.end();
}

// ---------------------------------------------------------------------------
// 路径定位工具实现
// ---------------------------------------------------------------------------

std::string FindProjectRoot() {
    char buf[MAX_PATH] = {0};
    DWORD len = GetModuleFileNameA(nullptr, buf, MAX_PATH);
    if (len == 0 || len >= MAX_PATH) {
        // 取不到 exe 路径时退回当前工作目录
        LogMsg(LWARN, "GetModuleFileNameA 失败，工程根退回当前工作目录");
        char cwd[MAX_PATH] = {0};
        GetCurrentDirectoryA(MAX_PATH, cwd);
        return std::string(cwd) + "\\";
    }
    std::string exeDir = ParentDir(buf);          // exe 所在目录（如 工程根/x64/Release）
    std::string root = ParentDir(ParentDir(exeDir));  // 上两级 = 工程根
    if (!FileExists(root + "\\config.ini")) {
        // 上两级没有 config.ini（例如 exe 直接放在工程根），退回 exe 同目录
        root = exeDir;
    }
    return root + "\\";
}

std::string ResolvePath(const std::string& p, const std::string& baseDir) {
    if (p.empty() || IsAbsolutePath(p)) {
        return p;
    }
    // 拼接时去掉相对路径开头的分隔符与基准目录结尾的分隔符，避免双分隔符
    std::string rel = p;
    while (!rel.empty() && (rel.front() == '/' || rel.front() == '\\')) {
        rel.erase(rel.begin());
    }
    std::string base = baseDir;
    while (!base.empty() && (base.back() == '/' || base.back() == '\\')) {
        base.pop_back();
    }
    return base + "\\" + rel;
}

std::string ResolvePath(const std::string& p) {
    if (p.empty() || IsAbsolutePath(p)) {
        return p;
    }
    std::string root = FindProjectRoot();
    return ResolvePath(p, root);
}

// ---------------------------------------------------------------------------
// AppConfig 实现
// ---------------------------------------------------------------------------

bool AppConfig::LoadFromIni(const std::string& iniPath, std::string& errMsg) {
    IniFile ini;
    if (!ini.Load(iniPath, errMsg)) {
        return false;  // errMsg 已由 IniFile::Load 填好
    }

    // [checkerboard] 功能 1：制板
    checkerboard.square_mm  = ini.GetDouble("checkerboard", "square_mm", checkerboard.square_mm);
    checkerboard.cols       = ini.GetInt("checkerboard", "cols", checkerboard.cols);
    checkerboard.rows       = ini.GetInt("checkerboard", "rows", checkerboard.rows);
    checkerboard.border_mm  = ini.GetDouble("checkerboard", "border_mm", checkerboard.border_mm);
    checkerboard.paper_w_mm = ini.GetDouble("checkerboard", "paper_w_mm", checkerboard.paper_w_mm);
    checkerboard.paper_h_mm = ini.GetDouble("checkerboard", "paper_h_mm", checkerboard.paper_h_mm);
    checkerboard.out_dir    = ini.GetString("checkerboard", "out_dir", checkerboard.out_dir);

    // [calibrate] 功能 2：标定
    // 联动规则：内角点/标称格距缺省跟随 [checkerboard] 制板参数（pattern =
    // cols-1 x rows-1，标称格距 = square_mm），段内显式配置的键优先。
    // 实测格距（卡尺量打印实物）与标称值不同才需显式回填 square_x_mm/square_y_mm。
    calibrate.input_dir     = ini.GetString("calibrate", "input_dir", calibrate.input_dir);
    calibrate.pattern_cols  = ini.GetInt("calibrate", "pattern_cols", checkerboard.cols - 1);
    calibrate.pattern_rows  = ini.GetInt("calibrate", "pattern_rows", checkerboard.rows - 1);
    calibrate.square_x_mm   = ini.GetDouble("calibrate", "square_x_mm", checkerboard.square_mm);
    calibrate.square_y_mm   = ini.GetDouble("calibrate", "square_y_mm", checkerboard.square_mm);
    calibrate.out_xml       = ini.GetString("calibrate", "out_xml", calibrate.out_xml);
    calibrate.target_mm_per_px = ini.GetDouble("calibrate", "target_mm_per_px", calibrate.target_mm_per_px);
    calibrate.qa_dir        = ini.GetString("calibrate", "qa_dir", calibrate.qa_dir);
    if (!ini.Has("calibrate", "pattern_cols") || !ini.Has("calibrate", "pattern_rows") ||
        !ini.Has("calibrate", "square_x_mm") || !ini.Has("calibrate", "square_y_mm")) {
        char sqBuf[64];
        std::snprintf(sqBuf, sizeof(sqBuf), "%g", calibrate.square_x_mm);
        LogMsg(LINFO, "标定参数缺省项跟随 [checkerboard]: 内角点 " +
               std::to_string(calibrate.pattern_cols) + "x" +
               std::to_string(calibrate.pattern_rows) + "，标称格距 " +
               sqBuf + " mm");
    }

    // [rectify] 功能 3 之 0)：几何矫正
    // 联动规则：标定文件/正射刻度缺省跟随 [calibrate] 的产物配置（out_xml /
    // target_mm_per_px），段内显式配置的键优先。
    rectify.enabled    = ini.GetBool("rectify", "enabled", rectify.enabled);
    rectify.calib_xml  = ini.GetString("rectify", "calib_xml", calibrate.out_xml);
    rectify.target_mm_per_px = ini.GetDouble("rectify", "target_mm_per_px", calibrate.target_mm_per_px);

    // [paths] 通用路径
    paths.input_dir       = ini.GetString("paths", "input_dir", paths.input_dir);
    paths.output_dir      = ini.GetString("paths", "output_dir", paths.output_dir);
    paths.background_file = ini.GetString("paths", "background_file", paths.background_file);

    // [segmentation] 分割方式
    segmentation.method = ini.GetString("segmentation", "method", segmentation.method);

    // [ai_seg] AI 分割
    ai_seg.onnx_model   = ini.GetString("ai_seg", "onnx_model", ai_seg.onnx_model);
    ai_seg.threshold    = ini.GetDouble("ai_seg", "threshold", ai_seg.threshold);
    ai_seg.expand_ratio = ini.GetDouble("ai_seg", "expand_ratio", ai_seg.expand_ratio);
    ai_seg.refine       = ini.GetBool("ai_seg", "refine", ai_seg.refine);
    ai_seg.device       = ini.GetString("ai_seg", "device", ai_seg.device);

    // [trad_seg] 传统背景差分分割
    trad_seg.bilateral_d       = ini.GetInt("trad_seg", "bilateral_d", trad_seg.bilateral_d);
    trad_seg.sigma_color       = ini.GetDouble("trad_seg", "sigma_color", trad_seg.sigma_color);
    trad_seg.sigma_space       = ini.GetDouble("trad_seg", "sigma_space", trad_seg.sigma_space);
    trad_seg.gamma             = ini.GetDouble("trad_seg", "gamma", trad_seg.gamma);
    trad_seg.diff_floor        = ini.GetDouble("trad_seg", "diff_floor", trad_seg.diff_floor);
    trad_seg.diff_ceiling      = ini.GetDouble("trad_seg", "diff_ceiling", trad_seg.diff_ceiling);
    trad_seg.morph_open_kernel = ini.GetInt("trad_seg", "morph_open_kernel", trad_seg.morph_open_kernel);
    trad_seg.morph_close_kernel = ini.GetInt("trad_seg", "morph_close_kernel", trad_seg.morph_close_kernel);
    trad_seg.min_area_ratio    = ini.GetDouble("trad_seg", "min_area_ratio", trad_seg.min_area_ratio);

    // [refine] 边缘精修
    refine.step_window        = ini.GetDouble("refine", "step_window", refine.step_window);
    refine.sample_step        = ini.GetInt("refine", "sample_step", refine.sample_step);
    refine.search_radius      = ini.GetDouble("refine", "search_radius", refine.search_radius);
    refine.profile_step       = ini.GetDouble("refine", "profile_step", refine.profile_step);
    refine.gradient_threshold = ini.GetDouble("refine", "gradient_threshold", refine.gradient_threshold);
    refine.peak_ratio         = ini.GetDouble("refine", "peak_ratio", refine.peak_ratio);
    refine.outlier_tol        = ini.GetDouble("refine", "outlier_tol", refine.outlier_tol);
    refine.median_window      = ini.GetInt("refine", "median_window", refine.median_window);

    // [rotate] 角度校正
    rotate.approx_epsilon_ratio = ini.GetDouble("rotate", "approx_epsilon_ratio", rotate.approx_epsilon_ratio);
    rotate.edge_band            = ini.GetInt("rotate", "edge_band", rotate.edge_band);

    // [measure] 测量
    measure.extreme_band         = ini.GetDouble("measure", "extreme_band", measure.extreme_band);
    measure.refine_rows          = ini.GetInt("measure", "refine_rows", measure.refine_rows);
    measure.ransac_threshold     = ini.GetDouble("measure", "ransac_threshold", measure.ransac_threshold);
    measure.edge_width_threshold = ini.GetDouble("measure", "edge_width_threshold", measure.edge_width_threshold);
    measure.angle_tol_deg        = ini.GetDouble("measure", "angle_tol_deg", measure.angle_tol_deg);
    measure.tangent_span         = ini.GetInt("measure", "tangent_span", measure.tangent_span);
    measure.line_dist_tol        = ini.GetDouble("measure", "line_dist_tol", measure.line_dist_tol);
    measure.span_gap_tol         = ini.GetDouble("measure", "span_gap_tol", measure.span_gap_tol);
    measure.merge_gap_pts        = ini.GetInt("measure", "merge_gap_pts", measure.merge_gap_pts);
    measure.ranked_region_ratio  = ini.GetDouble("measure", "ranked_region_ratio", measure.ranked_region_ratio);

    // [code_detect] 码区检测
    code_detect.enabled      = ini.GetBool("code_detect", "enabled", code_detect.enabled);
    code_detect.max_side     = ini.GetInt("code_detect", "max_side", code_detect.max_side);
    code_detect.min_area_px  = ini.GetDouble("code_detect", "min_area_px", code_detect.min_area_px);

    // [debug] 中间结果落盘开关
    debug.save_intermediate = ini.GetBool("debug", "save_intermediate", debug.save_intermediate);

    // 路径类字段统一解析为绝对路径（相对 ini 文件所在目录；
    // iniPath 本身为相对路径时先按旧规则相对工程根解析再取目录）
    const std::string iniDir = ParentDir(ResolvePath(iniPath));
    checkerboard.out_dir  = ResolvePath(checkerboard.out_dir, iniDir);
    calibrate.input_dir   = ResolvePath(calibrate.input_dir, iniDir);
    calibrate.out_xml     = ResolvePath(calibrate.out_xml, iniDir);
    calibrate.qa_dir      = ResolvePath(calibrate.qa_dir, iniDir);
    rectify.calib_xml     = ResolvePath(rectify.calib_xml, iniDir);
    paths.input_dir       = ResolvePath(paths.input_dir, iniDir);
    paths.output_dir      = ResolvePath(paths.output_dir, iniDir);
    paths.background_file = ResolvePath(paths.background_file, iniDir);
    ai_seg.onnx_model     = ResolvePath(ai_seg.onnx_model, iniDir);

    LogMsg(LINFO, "全工程配置加载完成: " + iniPath);
    return true;
}

}  // namespace common
