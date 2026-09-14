// ============================================================================
// main.cpp —— Calibrate_and_Measure 菜单驱动入口
//
// 本工程三大功能（全部配置集中在工程根目录 config.ini，注释见文件内说明）：
//   功能 1：生成棋盘格标定板打印文件（checkerboard.pdf / 预览图 / 打印说明.txt）
//   功能 2：棋盘格标定求解（采图 → 标定 XML + QA 质检图）
//   功能 3：批量图像测量（可选几何矫正 → AI/传统分割 → 宽高与水平边测量）
//
// 用法：
//   1. 无参数运行进入交互菜单；
//   2. 把一张棋盘格采图拖到 exe 上（命令行第一个参数为图片路径），
//      等价于菜单功能 2 的单图标定。
//
// 菜单每次执行功能都会重新读取 config.ini，改完配置直接再选功能即可生效。
// ============================================================================

#include <windows.h>  // SetConsoleOutputCP

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "common/image_io.h"
#include "common/ini_config.h"
#include "common/logger.h"
#include "checkerboard/board_generator.h"
#include "calibration/calibrator.h"
#include "calibration/rectifier.h"
#include "measure/measure_pipeline.h"
#include "measure_types.h"

namespace {

// ---------------------------------------------------------------------------
// 小工具
// ---------------------------------------------------------------------------

// 控制台切到 UTF-8，保证中文提示不乱码（源码按 /utf-8 编译）
void SetupConsole() {
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);
}

// 全工程 ini 配置文件路径（工程根目录下的 config.ini）
std::string IniPath() {
    return common::FindProjectRoot() + "config.ini";
}

void PrintMenu() {
    std::cout << "\n"
                 "============================================================\n"
                 "  Calibrate_and_Measure  标定与测量系统\n"
                 "============================================================\n"
                 "  配置文件: " << IniPath() << "\n"
                 "------------------------------------------------------------\n"
                 "  1) 生成棋盘格标定板（PDF + 预览图 + 打印说明）\n"
                 "  2) 棋盘格标定（读取 [calibrate] input_dir 采图 → 标定 XML）\n"
                 "  3) 批量图像测量（读取 [paths] input_dir，输出宽高与水平边）\n"
                 "  q) 退出\n"
                 "------------------------------------------------------------\n"
                 "请选择: ";
}

// ---------------------------------------------------------------------------
// 功能 1：生成棋盘格标定板
// ---------------------------------------------------------------------------
void RunMakeBoard() {
    std::string err;
    if (cam::GenerateCheckerboardBoard(IniPath(), err)) {
        std::cout << "[成功] 标定板文件已生成（详见上方日志中的输出目录）。" << std::endl;
    } else {
        std::cout << "[失败] " << err << std::endl;
    }
}

// ---------------------------------------------------------------------------
// 功能 2：棋盘格标定（单图或多图）
// ---------------------------------------------------------------------------

// 对给定图像列表执行标定（内部区分单图/多图入口）
void CalibrateWithImages(const std::vector<cv::Mat>& images, const char* srcDesc) {
    std::string err;
    bool ok = false;
    if (images.size() == 1) {
        std::cout << "单张采图标定: " << srcDesc << std::endl;
        ok = cam::BuildCalibrationFile(images[0], IniPath(), err);
    } else {
        std::cout << "共 " << images.size() << " 张采图，角点取均值降噪后标定。" << std::endl;
        ok = cam::BuildCalibrationFileFromImages(images, IniPath(), err);
    }
    if (ok) {
        std::cout << "[成功] 标定完成，标定文件与 QA 质检图已输出（见 config.ini 的 "
                     "[calibrate] out_xml / qa_dir）。" << std::endl;
    } else {
        std::cout << "[未通过] " << err << std::endl;
    }
}

// 菜单功能 2：扫描 [calibrate] input_dir 下的全部采图
void RunCalibrateFromDir() {
    common::AppConfig cfg;
    std::string err;
    if (!cfg.LoadFromIni(IniPath(), err)) {
        std::cout << "[失败] " << err << std::endl;
        return;
    }
    const std::vector<std::string> files = common::ListImages(cfg.calibrate.input_dir);
    if (files.empty()) {
        std::cout << "[提示] 标定采图目录为空，请先把棋盘格采图放入: "
                  << cfg.calibrate.input_dir << std::endl;
        return;
    }
    std::vector<cv::Mat> images;
    images.reserve(files.size());
    for (const std::string& f : files) {
        cv::Mat img = common::LoadImageAny(f);
        if (img.empty()) {
            std::cout << "[失败] 图像读取失败: " << f << std::endl;
            return;
        }
        images.push_back(img);
    }
    CalibrateWithImages(images, files.front().c_str());
}

// 命令行拖入单张图片的快捷标定入口
void RunCalibrateSingleFile(const std::string& imagePath) {
    cv::Mat img = common::LoadImageAny(imagePath);
    if (img.empty()) {
        std::cout << "[失败] 图像读取失败: " << imagePath << std::endl;
        return;
    }
    CalibrateWithImages({img}, imagePath.c_str());
}

// ---------------------------------------------------------------------------
// 功能 3：批量图像测量
// ---------------------------------------------------------------------------

// 把一条线段的 4 个坐标追加到 CSV 当前行（",x1,y1,x2,y2"）
void AppendSeg(std::ofstream& csv, const cam::LineSegment& s) {
    csv << "," << s.start.x << "," << s.start.y << "," << s.end.x << "," << s.end.y;
}

// 把一张图的测量结果打印到控制台，并写：
//   汇总表 measure_results.csv 一行（含宽高四线的端点坐标）；
//   明细表 measure_edges.csv 每条候选水平边一行（端点坐标 + 长度 + 判定）。
void ReportOne(const std::string& imageName, const cam::MeasureOutput& out,
               std::ofstream& csv, std::ofstream& edgesCsv) {
    if (out.code != cam::RetCode::OK) {
        std::cout << "  [失败] " << out.message << std::endl;
        csv << imageName;
        for (int i = 0; i < 21; ++i) {
            csv << ",";  // 20 个数值列留空 + message 列前的分隔
        }
        csv << out.message << "\n";
        return;
    }
    // printf 格式串保持纯 ASCII（MSVC 格式检查器对 UTF-8 中文格式串会误报
    // C4819/C4477），中文文案一律经 %s 参数传入
    std::printf("  %s %.3f deg\n", "旋转角:", out.rotationDeg);
    std::printf("  %s %.2f px  (x_left=%.2f, x_right=%.2f)\n", "宽度:",
                out.width.widthPx, out.width.leftLine.start.x, out.width.rightLine.start.x);
    std::printf("  %s %.2f px  (y_top=%.2f, y_bottom=%.2f)\n", "高度:",
                out.height.heightPx, out.height.topLine.start.y, out.height.bottomLine.start.y);
    std::printf("  %s %llu\n", "上半部分水平边数:",
                static_cast<unsigned long long>(out.horizontalEdges.size()));
    for (size_t i = 0; i < out.horizontalEdges.size(); ++i) {
        const cam::HorizontalEdge& e = out.horizontalEdges[i];
        std::printf("    edge_%llu: (%.2f,%.2f)-(%.2f,%.2f) len=%.2f px %s\n",
                    static_cast<unsigned long long>(i + 1), e.start.x, e.start.y,
                    e.end.x, e.end.y, e.lengthPx, e.satisfied ? "[OK]" : "[SHORT]");
        // 边明细表：一行一条边，端点坐标 + 长度 + 是否满足宽度判定
        edgesCsv << imageName << "," << (i + 1) << "," << e.start.x << "," << e.start.y
                 << "," << e.end.x << "," << e.end.y << "," << e.lengthPx << ","
                 << (e.satisfied ? 1 : 0) << "\n";
    }
    // 汇总表：宽高四线端点坐标全部落列，message 留空
    csv << imageName << "," << out.rotationDeg << "," << out.width.widthPx << ","
        << out.height.heightPx << "," << out.horizontalEdges.size();
    AppendSeg(csv, out.width.leftLine);
    AppendSeg(csv, out.width.rightLine);
    AppendSeg(csv, out.height.topLine);
    AppendSeg(csv, out.height.bottomLine);
    csv << ",\n";
}

void RunMeasureBatch() {
    const std::string ini = IniPath();

    // 1) 测量流水线初始化（背景建模 + 分割器；rectify 开启时背景已同步正射校正）
    cam::MeasurePipeline pipe;
    std::string err;
    if (!pipe.Init(ini, err)) {
        std::cout << "[失败] 测量流水线初始化失败: " << err << std::endl;
        return;
    }
    const common::AppConfig& cfg = pipe.Config();

    // 2) 几何矫正模块（可选）：输入图先矫正再测量，测量函数本身不关心是否矫正
    cam::Rectifier rectifier;
    if (cfg.rectify.enabled) {
        if (!rectifier.Load(cfg.rectify.calib_xml, cfg.rectify.target_mm_per_px, err)) {
            std::cout << "[失败] 矫正模块加载失败: " << err << std::endl;
            return;
        }
        std::cout << "几何矫正已启用，标定文件: " << cfg.rectify.calib_xml
                  << "，正射刻度: " << rectifier.MmPerPx() << " mm/px" << std::endl;
    }

    // 3) 批量处理
    const std::vector<std::string> files = common::ListImages(cfg.paths.input_dir);
    if (files.empty()) {
        std::cout << "[提示] 待测图像目录为空: " << cfg.paths.input_dir << std::endl;
        return;
    }

    // 汇总表：一行一张图；明细表：一行一条候选水平边（坐标/长度见明细）
    {
        std::error_code ec;
        std::filesystem::create_directories(cfg.paths.output_dir, ec);
        if (ec) {
            std::cout << "[警告] 创建输出目录失败: " << cfg.paths.output_dir << std::endl;
        }
    }
    std::ofstream csv(cfg.paths.output_dir + "\\measure_results.csv",
                      std::ios::out | std::ios::trunc);
    std::ofstream edgesCsv(cfg.paths.output_dir + "\\measure_edges.csv",
                           std::ios::out | std::ios::trunc);
    if (csv.is_open()) {
        csv << "\xEF\xBB\xBF";  // utf-8-sig，Excel 打开不乱码
        csv << "image,rotation_deg,width_px,height_px,edge_count,"
               "width_left_x1,width_left_y1,width_left_x2,width_left_y2,"
               "width_right_x1,width_right_y1,width_right_x2,width_right_y2,"
               "height_top_x1,height_top_y1,height_top_x2,height_top_y2,"
               "height_bottom_x1,height_bottom_y1,height_bottom_x2,height_bottom_y2,"
               "message\n";
    }
    if (edgesCsv.is_open()) {
        edgesCsv << "\xEF\xBB\xBF";
        edgesCsv << "image,edge_index,x1,y1,x2,y2,length_px,satisfied\n";
    }

    int okCount = 0, failCount = 0;
    for (const std::string& f : files) {
        const std::string name = f.substr(f.find_last_of("/\\") + 1);
        std::cout << "\n=== 测量: " << name << " ===" << std::endl;
        cv::Mat img = common::LoadImageAny(f);
        if (img.empty()) {
            std::cout << "  [失败] 图像读取失败" << std::endl;
            ++failCount;
            continue;
        }
        // 软件收到相机原图 → 先矫正 → 再测量（矫正关闭时原图直进测量）
        if (rectifier.IsReady()) {
            cv::Mat gray = common::ToGray8(img);
            img = rectifier.Rectify(gray);
            if (img.empty()) {
                std::cout << "  [失败] 正射校正失败（图像尺寸须与标定采图一致）" << std::endl;
                ++failCount;
                continue;
            }
        }
        // debugTag = 去扩展名的文件名，[debug] 开关打开时用于落调试图
        const std::string tag = name.substr(0, name.find_last_of('.'));
        const cam::MeasureOutput out = pipe.Measure(img, tag);
        ReportOne(name, out, csv, edgesCsv);
        if (out.code == cam::RetCode::OK) {
            ++okCount;
        } else {
            ++failCount;
        }
    }
    std::cout << "\n批量测量完成: 成功 " << okCount << " 张，失败 " << failCount
              << " 张。\n结果汇总: " << cfg.paths.output_dir << "\\measure_results.csv"
              << "\n边明细:   " << cfg.paths.output_dir << "\\measure_edges.csv"
              << std::endl;
}

}  // namespace

int main(int argc, char* argv[]) {
    SetupConsole();

    // 命令行拖入一张图片 = 快捷执行功能 2 单图标定
    if (argc > 1) {
        RunCalibrateSingleFile(argv[1]);
        system("pause");
        return 0;
    }

    for (;;) {
        PrintMenu();
        std::string choice;
        if (!std::getline(std::cin, choice)) {
            break;
        }
        // 每个功能分支独立兜底，任何异常都只给中文提示，不闪退
        try {
            if (choice == "1") {
                RunMakeBoard();
            } else if (choice == "2") {
                RunCalibrateFromDir();
            } else if (choice == "3") {
                RunMeasureBatch();
            } else if (choice == "q" || choice == "Q") {
                break;
            } else {
                std::cout << "无效输入，请重新选择。" << std::endl;
            }
        } catch (const std::exception& e) {
            std::cout << "[异常] 功能执行出错: " << e.what() << std::endl;
        }
    }
    std::cout << "已退出。" << std::endl;
    return 0;
}
