#pragma once
// ============================================================================
// image_io.h —— 图像读写工具
// 支持 8/24/32 位 bmp/jpg/png/tif 的读取与保存；
// OpenCV 解码失败时使用手工 BMP 解码兜底，保证产线 24/32 位 BMP 可读。
// 所有相对路径在传入前应由调用方经 ResolvePath 解析。
// ============================================================================

#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace common {

// 读取图像（任意通道原样读入，不做颜色转换）
// 流程：先 cv::imread(IMREAD_UNCHANGED)；失败时走手工 BMP 解码兜底
//       （支持 24/32bit、BI_RGB/BI_BITFIELDS、底向上/顶向下存储）。
// @param path 图像文件绝对路径
// @return 解码后的图像；读不到返回空 Mat 并记 Error 日志
cv::Mat LoadImageAny(const std::string& path);

// 将图像转为 8 位单通道灰度图
// 8UC1 原样返回；8UC3 按 BGR2GRAY 转换；8UC4 按 BGRA2GRAY 转换；其余返回空 Mat。
// @param img 输入图像
// @return 8UC1 灰度图；不支持的类型返回空 Mat 并记 Error 日志
cv::Mat ToGray8(const cv::Mat& img);

// 保存图像
// 流程：先 cv::imwrite；32 位图保存失败时用手工 32bit BI_RGB BMP 写出兜底。
// @param path 输出文件路径（格式由扩展名决定）
// @param img  待保存图像
// @return 保存成功返回 true；失败返回 false 并记 Error 日志
bool SaveImage(const std::string& path, const cv::Mat& img);

// 列出目录下全部图像文件
// 接受扩展名：.bmp/.png/.jpg/.jpeg/.tif/.tiff（大小写不敏感）；
// 按文件名排序；剔除文件名（不含扩展名）含 "demo" 或 "副本" 的条目。
// @param dir 目录路径
// @return 排序后的图像文件完整路径列表；目录打不开返回空列表并记 Error 日志
std::vector<std::string> ListImages(const std::string& dir);

}  // namespace common
