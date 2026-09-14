#pragma once
// ============================================================================
// measure_types.h —— 功能 3（图像测量）对外数据类型定义
// ----------------------------------------------------------------------------
// 对齐 Python 工程《INTERFACE.md》v1.4 协议：
//   - 所有长度、坐标均为像素（px），浮点（double），亚像素保留 2 位小数；
//   - 坐标系为角度校正后的图像坐标系：原点左上，x 向右，y 向下；
//   - 宽高四线端点约定：宽度两条竖线的纵向范围 = 高度上下两条横线的位置，
//     高度两条横线的横向范围 = 宽度左右两条竖线的位置，四线围成闭合测量矩形。
// ============================================================================

#include <string>
#include <vector>

namespace cam {

// 返回码
enum class RetCode : int {
    OK          = 0,    // 成功
    EMPTY_IMAGE = 1,    // 输入图像为空
    BAD_FORMAT  = 2,    // 图像格式不支持（须 8UC1/8UC3/8UC4）
    NO_PRODUCT  = 3,    // 未检出产品
    INTERNAL    = 100   // 算法内部异常（message 附中文说明）
};

// 像素坐标点（亚像素）
struct Point2d {
    double x = 0.0;
    double y = 0.0;
};

// 有限线段：起点/终点
struct LineSegment {
    Point2d start;
    Point2d end;
};

// 结构体一：产品宽度
struct WidthResult {
    LineSegment leftLine;      // 左侧竖线起点/终点
    LineSegment rightLine;     // 右侧竖线起点/终点
    double      widthPx = 0.0; // 宽度数值（左右竖线间距）
    bool        ok      = false;
};

// 结构体二：产品高度
struct HeightResult {
    LineSegment topLine;        // 上边横线起点/终点
    LineSegment bottomLine;     // 下边横线起点/终点
    double      heightPx = 0.0; // 高度数值（上下横线间距）
    bool        ok       = false;
};

// 结构体三：上半部分一条水平边线段
struct HorizontalEdge {
    Point2d start;                 // 起点（亚像素，拟合直线上的投影点）
    Point2d end;                   // 终点（亚像素）
    double  lengthPx  = 0.0;       // 线段长度（px，沿拟合直线的跨度）
    bool    satisfied = false;     // 长度是否满足 edge_width_threshold（判定不淘汰）
};

// 单帧完整测量结果
struct MeasureOutput {
    RetCode     code = RetCode::OK;  // 返回码
    std::string message;             // 失败时的中文原因，成功为空
    double      rotationDeg = 0.0;   // 总校正角（度）

    WidthResult  width;              // 结构体一：宽度（OK 时有效）
    HeightResult height;             // 结构体二：高度（OK 时有效）

    // 结构体三：上半部分候选水平边，有多少条返回多少条，按 (y, x_left) 排序
    std::vector<HorizontalEdge> horizontalEdges;
};

}  // namespace cam
