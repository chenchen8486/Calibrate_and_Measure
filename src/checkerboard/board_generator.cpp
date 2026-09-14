// ============================================================================
// board_generator.cpp —— 功能 1：棋盘格标定板打印文件生成（实现）
//
// 算法等价移植自 scripts/make_checkerboard.py，逐行对应：
//   LayoutBoard      <- layout_board     （版面计算，纯函数）
//   BuildPdf         <- build_pdf        （零依赖手写 PDF 1.4）
//   RenderPreview    <- render_preview   （PNG 预览，px_per_mm = 4.0）
//   BuildPrintNote   <- build_print_note （打印说明文本，措辞照抄 Python 版）
// ============================================================================

#include "checkerboard/board_generator.h"

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/imgproc.hpp>

#include "common/image_io.h"
#include "common/ini_config.h"
#include "common/logger.h"

namespace cam {
namespace {

// PDF 用户空间单位换算：1mm = 72/25.4 pt
constexpr double kPtPerMm = 72.0 / 25.4;

// 毫米矩形（x, y 为左下角坐标，原点纸张左下角，单位 mm）
struct MmRect {
    double x;
    double y;
    double w;
    double h;
};

// ---------------------------------------------------------------------------
// 数值格式化工具（与 Python 的格式化输出保持一致）
// ---------------------------------------------------------------------------

// 等价 Python 的 %.3f
std::string Fmt3(double v) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.3f", v);
    return buf;
}

// 等价 Python 的 %g（默认 6 位有效数字）
std::string FmtG(double v) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.6g", v);
    return buf;
}

// 等价 Python 的 %.1f（用于错误信息中的边距数值）
std::string Fmt1(double v) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.1f", v);
    return buf;
}

// ---------------------------------------------------------------------------
// 版面计算（对应 Python layout_board）
// ---------------------------------------------------------------------------

// 计算棋盘格版面：黑格矩形列表 + 左上角方向标记条
// 图案在纸张内居中；坐标原点在纸张左下角（PDF 用户空间约定），单位 mm。
// @param cfg    [checkerboard] 段配置
// @param rects  输出参数：毫米矩形列表（黑格 + 末尾一根方向标记条）
// @param errMsg 输出参数：参数非法时的中文原因
// @return 参数合法返回 true；边距不足或格数奇偶相同返回 false
bool LayoutBoard(const common::CheckerboardConfig& cfg,
                 std::vector<MmRect>& rects, std::string& errMsg) {
    if (cfg.square_mm <= 0.0 || cfg.cols < 2 || cfg.rows < 2 ||
        cfg.paper_w_mm <= 0.0 || cfg.paper_h_mm <= 0.0 || cfg.border_mm < 0.0) {
        errMsg = "参数非法：格子边长/纸张尺寸必须为正，横纵格数必须不小于 2";
        return false;
    }

    const double pattern_w = cfg.square_mm * cfg.cols;
    const double pattern_h = cfg.square_mm * cfg.rows;
    const double ox = (cfg.paper_w_mm - pattern_w) / 2.0;
    const double oy = (cfg.paper_h_mm - pattern_h) / 2.0;
    if (ox < cfg.border_mm) {
        errMsg = "横向留白不足：需要至少 " + Fmt1(cfg.border_mm) +
                 "mm，实际仅 " + Fmt1(ox) + "mm";
        return false;
    }
    if (oy < cfg.border_mm) {
        errMsg = "纵向留白不足：需要至少 " + Fmt1(cfg.border_mm) +
                 "mm，实际仅 " + Fmt1(oy) + "mm";
        return false;
    }
    if ((cfg.cols % 2) == (cfg.rows % 2)) {
        errMsg = "横纵格数必须一奇一偶（当前 " + std::to_string(cfg.cols) + "x" +
                 std::to_string(cfg.rows) + " 存在 180° 旋转歧义）";
        return false;
    }

    rects.clear();
    for (int r = 0; r < cfg.rows; ++r) {
        for (int c = 0; c < cfg.cols; ++c) {
            if ((r + c) % 2 == 0) {  // 黑格
                rects.push_back({ox + c * cfg.square_mm, oy + r * cfg.square_mm,
                                 cfg.square_mm, cfg.square_mm});
            }
        }
    }

    // 方向标记：左上角静区内一根细长条（极端长宽比不会被四边形检测误当方格），
    // 供人工辨识纸张摆放方向；不侵入图案静区主体
    const double mark_w = cfg.square_mm * 0.6;
    const double mark_h = cfg.square_mm * 0.15;
    rects.push_back({ox * 0.4, cfg.paper_h_mm - oy * 0.4 - mark_h, mark_w, mark_h});
    return true;
}

// ---------------------------------------------------------------------------
// 矢量 PDF 拼装（对应 Python build_pdf，内容流仅矩形填充）
// ---------------------------------------------------------------------------

// 把毫米矩形列表拼装为单页矢量 PDF 字节流
// @param rects      毫米矩形列表（全部填充为黑色）
// @param paper_w_mm 纸张宽（MediaBox 宽）
// @param paper_h_mm 纸张高（MediaBox 高）
// @return 完整 PDF 文件字节
std::string BuildPdf(const std::vector<MmRect>& rects,
                     double paper_w_mm, double paper_h_mm) {
    // 内容流：首行设定填充色为黑，随后每矩形一行 "x y w h re f"（毫米转 pt）
    std::ostringstream content;
    content << "0 0 0 rg\n";
    for (const MmRect& rc : rects) {
        content << Fmt3(rc.x * kPtPerMm) << " " << Fmt3(rc.y * kPtPerMm) << " "
                << Fmt3(rc.w * kPtPerMm) << " " << Fmt3(rc.h * kPtPerMm) << " re f\n";
    }
    const std::string content_str = content.str();

    // 4 个对象：1 Catalog、2 Pages、3 Page、4 内容流
    std::vector<std::string> objects;
    objects.emplace_back("<< /Type /Catalog /Pages 2 0 R >>");
    objects.emplace_back("<< /Type /Pages /Kids [3 0 R] /Count 1 >>");
    objects.emplace_back("<< /Type /Page /Parent 2 0 R /MediaBox [0 0 " +
                         Fmt3(paper_w_mm * kPtPerMm) + " " +
                         Fmt3(paper_h_mm * kPtPerMm) +
                         "] /Contents 4 0 R /Resources << >> >>");
    objects.emplace_back("<< /Length " + std::to_string(content_str.size()) +
                         " >>\nstream\n" + content_str + "endstream");

    // 按字节偏移精确累加拼装（xref 表依赖每个对象的起始字节位置）
    std::string pdf = "%PDF-1.4\n%\xe2\xe3\xcf\xd3\n";
    std::vector<size_t> offsets;
    for (size_t i = 0; i < objects.size(); ++i) {
        offsets.push_back(pdf.size());
        pdf += std::to_string(i + 1) + " 0 obj\n" + objects[i] + "\nendobj\n";
    }
    const size_t xref_pos = pdf.size();
    const size_t n = objects.size() + 1;

    std::ostringstream tail;
    tail << "xref\n0 " << n << "\n";
    tail << "0000000000 65535 f \n";
    char off_buf[32];
    for (size_t off : offsets) {
        std::snprintf(off_buf, sizeof(off_buf), "%010zu 00000 n \n", off);
        tail << off_buf;
    }
    tail << "trailer\n<< /Size " << n << " /Root 1 0 R >>\nstartxref\n"
         << xref_pos << "\n%%EOF\n";
    pdf += tail.str();
    return pdf;
}

// ---------------------------------------------------------------------------
// PNG 预览（对应 Python render_preview，仅供人工核对版面，不作打印用途）
// ---------------------------------------------------------------------------

// 等价 Python 的 round()：四舍六入五取偶（half-even）。
// 用 llrint（默认 FE_TONEAREST 舍入模式）而非 lround（half-up），
// 保证任意参数组合下预览图与 Python 版逐像素一致。
int PyRound(double v) {
    return static_cast<int>(std::llrint(v));
}

// 渲染版面预览图（白底黑格 + 灰色纸边线），比例尺 4 px/mm
// @param rects      毫米矩形列表
// @param paper_w_mm 纸张宽
// @param paper_h_mm 纸张高
// @return BGR 预览图
cv::Mat RenderPreview(const std::vector<MmRect>& rects,
                      double paper_w_mm, double paper_h_mm) {
    constexpr double kPxPerMm = 4.0;
    const int w = PyRound(paper_w_mm * kPxPerMm);
    const int h = PyRound(paper_h_mm * kPxPerMm);
    cv::Mat img(h, w, CV_8UC3, cv::Scalar(255, 255, 255));
    for (const MmRect& rc : rects) {
        // 毫米（左下原点）转像素（左上原点）
        const int x0 = PyRound(rc.x * kPxPerMm);
        const int y0 = PyRound((paper_h_mm - rc.y - rc.h) * kPxPerMm);
        const int x1 = PyRound((rc.x + rc.w) * kPxPerMm);
        const int y1 = PyRound((paper_h_mm - rc.y) * kPxPerMm);
        cv::rectangle(img, cv::Point(x0, y0), cv::Point(x1, y1),
                      cv::Scalar(0, 0, 0), cv::FILLED);
    }
    cv::rectangle(img, cv::Point(0, 0), cv::Point(w - 1, h - 1),
                  cv::Scalar(180, 180, 180), 1);
    return img;
}

// ---------------------------------------------------------------------------
// 打印说明文档（对应 Python build_print_note，措辞与 Python 版一致）
// ---------------------------------------------------------------------------

// 生成打印说明文本（给打印店客服，数值全部从版面参数带出）
// @param cfg [checkerboard] 段配置
// @return 说明文本（中文，纯文本，不含 BOM）
std::string BuildPrintNote(const common::CheckerboardConfig& cfg) {
    const double patt_w = cfg.square_mm * cfg.cols;
    const double patt_h = cfg.square_mm * cfg.rows;
    const double span10 = 10.0 * cfg.square_mm;
    const std::string pw = FmtG(cfg.paper_w_mm);
    const std::string ph = FmtG(cfg.paper_h_mm);
    const std::string sq = FmtG(cfg.square_mm);

    std::ostringstream os;
    os << "棋盘格标定板打印说明\n"
       << "============================================================\n"
       << "\n"
       << "一、交付文件\n"
       << "- checkerboard.pdf：矢量打印文件（只需打印这一个文件）\n"
       << "\n"
       << "二、成品规格\n"
       << "- 纸张成品尺寸：" << pw << " × " << ph << " mm（宽 × 高），幅面需 A1 及以上，印后裁切\n"
       << "- 棋盘格图案：" << cfg.cols << " 列 × " << cfg.rows << " 行，格子边长 " << sq << " mm（黑白相间）\n"
       << "- 图案区域：" << FmtG(patt_w) << " × " << FmtG(patt_h) << " mm，四边各留 "
       << FmtG(cfg.border_mm) << " mm 白色边距\n"
       << "- 方向标记：左上角白边内有一根黑色细长条，用于辨识摆放方向，属正常设计，\n"
       << "  请勿当作脏点修掉\n"
       << "\n"
       << "三、打印要求（重要，请逐条落实）\n"
       << "1. 请用 PDF 原文件直接输出，不要转成 JPG/PNG 图片后再打印；\n"
       << "2. 必须 100% 实际尺寸打印，关闭\"适应页面 / 适合纸张 / 自动缩放\"等一切缩放选项；\n"
       << "3. 打印分辨率不低于 600 DPI，建议 1200 DPI；\n"
       << "4. 纸张用哑光厚卡纸或铜版纸（克重 ≥200g），黑白打印、纯黑实底；\n"
       << "   不要覆亮膜（反光会影响后续拍照）；\n"
       << "5. 印后按 " << pw << " × " << ph << " mm 裁切成品（裁切偏差 ±1mm 内可接受，\n"
       << "   图案本身的尺寸精度才是关键）。\n"
       << "\n"
       << "四、印后验收指标（请配合实测后告诉我们数值）\n"
       << "1. 横向连续 10 个格子的总宽度：应为 " << FmtG(span10) << " mm，偏差不超过 ±0.5 mm；\n"
       << "2. 纵向连续 10 个格子的总高度：应为 " << FmtG(span10) << " mm，偏差不超过 ±0.5 mm；\n"
       << "3. 横向与纵向实测值之差不超过 0.3 mm（横竖比例必须一致）；\n"
       << "4. 格子边缘平直无锯齿，黑白分明无灰边。\n"
       << "如任何一项不达标，请先不要裁切交付，联系我们调整后再印。\n"
       << "\n";
    return os.str();
}

// 写出 PDF 二进制文件；失败记日志并填 errMsg
bool WritePdfFile(const std::string& path, const std::string& bytes,
                  std::string& errMsg) {
    std::ofstream ofs(path, std::ios::binary | std::ios::trunc);
    if (!ofs.is_open()) {
        errMsg = "无法创建文件：" + path;
        common::LogMsg(common::LERROR, "打开 PDF 输出文件失败: " + path);
        return false;
    }
    ofs.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
    ofs.close();
    if (!ofs) {
        errMsg = "写入文件失败：" + path;
        common::LogMsg(common::LERROR, "写入 PDF 文件失败: " + path);
        return false;
    }
    return true;
}

// 写出打印说明文本文件（utf-8-sig 编码，与 Python 版产物一致）；失败记日志并填 errMsg
bool WriteTextFile(const std::string& path, const std::string& text,
                   std::string& errMsg) {
    std::ofstream ofs(path, std::ios::binary | std::ios::trunc);
    if (!ofs.is_open()) {
        errMsg = "无法创建文件：" + path;
        common::LogMsg(common::LERROR, "打开文本输出文件失败: " + path);
        return false;
    }
    const std::string bom = "\xef\xbb\xbf";  // utf-8-sig BOM
    ofs.write(bom.data(), static_cast<std::streamsize>(bom.size()));
    ofs.write(text.data(), static_cast<std::streamsize>(text.size()));
    ofs.close();
    if (!ofs) {
        errMsg = "写入文件失败：" + path;
        common::LogMsg(common::LERROR, "写入文本文件失败: " + path);
        return false;
    }
    return true;
}

}  // namespace

// ---------------------------------------------------------------------------
// 对外入口（对应 Python generate_checkerboard）
// ---------------------------------------------------------------------------

bool GenerateCheckerboardBoard(const std::string& iniPath, std::string& errMsg) {
    // 1. 读取配置（相对路径自动解析为绝对路径）
    common::AppConfig cfg;
    if (!cfg.LoadFromIni(iniPath, errMsg)) {
        common::LogMsg(common::LERROR, "读取配置失败: " + errMsg);
        return false;
    }
    const common::CheckerboardConfig& cb = cfg.checkerboard;

    // 2. 版面计算与参数合法性检查
    std::vector<MmRect> rects;
    if (!LayoutBoard(cb, rects, errMsg)) {
        common::LogMsg(common::LERROR, "棋盘格版面参数非法: " + errMsg);
        return false;
    }

    // 3. 创建输出目录
    std::error_code ec;
    std::filesystem::create_directories(cb.out_dir, ec);
    if (ec) {
        errMsg = "创建输出目录失败：" + cb.out_dir + "（" + ec.message() + "）";
        common::LogMsg(common::LERROR, errMsg);
        return false;
    }
    const std::string sep =
        (!cb.out_dir.empty() && cb.out_dir.back() != '/' && cb.out_dir.back() != '\\')
            ? "\\" : "";
    const std::string pdf_path = cb.out_dir + sep + "checkerboard.pdf";
    const std::string preview_path = cb.out_dir + sep + "checkerboard_preview.png";
    const std::string note_path = cb.out_dir + sep + "Readme.txt";

    // 4. 生成矢量 PDF
    if (!WritePdfFile(pdf_path, BuildPdf(rects, cb.paper_w_mm, cb.paper_h_mm), errMsg)) {
        return false;
    }
    common::LogMsg(common::LINFO, "矢量 PDF 已生成: " + pdf_path + "（" +
                   std::to_string(rects.size() - 1) + " 个黑格）");

    // 5. 生成 PNG 预览图（仅供核对，勿用于打印）
    if (!common::SaveImage(preview_path,
                           RenderPreview(rects, cb.paper_w_mm, cb.paper_h_mm))) {
        errMsg = "保存预览图失败：" + preview_path;
        return false;  // SaveImage 内部已记 Error 日志
    }
    common::LogMsg(common::LINFO, "预览图已生成: " + preview_path + "（仅供核对，勿用于打印）");

    // 6. 生成打印说明（随版面参数同步，可直接发打印店）
    if (!WriteTextFile(note_path, BuildPrintNote(cb), errMsg)) {
        return false;
    }
    common::LogMsg(common::LINFO, "打印说明已生成: " + note_path + "（随版面参数同步，可直接发打印店）");

    // 7. 输出版面摘要日志
    const int inner_c = cb.cols - 1;
    const int inner_r = cb.rows - 1;
    std::ostringstream summary;
    summary << "纸张 " << FmtG(cb.paper_w_mm) << "x" << FmtG(cb.paper_h_mm)
            << "mm，图案 " << cb.cols << "x" << cb.rows << " 格（"
            << FmtG(cb.square_mm) << "mm/格），内角点 " << inner_c << "x" << inner_r
            << "=" << (inner_c * inner_r) << " 个，角点覆盖跨度 "
            << FmtG((cb.cols - 2) * cb.square_mm) << "x"
            << FmtG((cb.rows - 2) * cb.square_mm) << "mm";
    common::LogMsg(common::LINFO, summary.str());
    return true;
}

}  // namespace cam
