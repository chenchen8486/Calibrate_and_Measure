#pragma once
// ============================================================================
// board_generator.h —— 功能 1：棋盘格标定板打印文件生成
//
// 依据 ini 配置 [checkerboard] 段生成三件套产物（等价移植自
// scripts/make_checkerboard.py，几何与 Python 版逐像素/逐字节一致）：
//   1. checkerboard.pdf         矢量打印文件（手写 PDF 1.4，任意 DPI 边缘锐利）
//   2. checkerboard_preview.png 人工核对预览图（仅供核对，勿用于打印）
//   3. 打印说明.txt              随版面参数自动生成，直接交打印店
// ============================================================================

#include <string>

namespace cam {

// 功能 1 对外唯一入口：生成棋盘格标定板打印文件
//
// 用法：
//   std::string err;
//   if (!cam::GenerateCheckerboardBoard("config.ini", err)) {
//       // err 为中文失败原因（参数非法或写文件失败）
//   }
//
// 读取 iniPath 的 [checkerboard] 段（square_mm/cols/rows/border_mm/
// paper_w_mm/paper_h_mm/out_dir，缺失键用默认值，相对 out_dir 自动解析为
// 绝对路径），在 out_dir 下生成上述三个产物。
//
// @param iniPath 配置文件路径
// @param errMsg  输出参数：失败时的中文原因
// @return 成功返回 true；参数非法（边距不足、格数奇偶相同等）或写文件失败返回 false
bool GenerateCheckerboardBoard(const std::string& iniPath, std::string& errMsg);

}  // namespace cam
