// ============================================================================
// image_io.cpp —— 图像读写工具实现（含手工 BMP 编解码兜底）
// ============================================================================

#include "image_io.h"

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "logger.h"

namespace common {
namespace {

// ---------------------------------------------------------------------------
// 手工 BMP 编解码（内部实现，不对外暴露）
// ---------------------------------------------------------------------------

// 从字节流按小端序读取 32 位无符号整数
// @param p 指向至少 4 字节数据的指针
// @return 解析出的 uint32 值
uint32_t ReadU32LE(const unsigned char* p) {
    return static_cast<uint32_t>(p[0]) |
           (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) |
           (static_cast<uint32_t>(p[3]) << 24);
}

// 从字节流按小端序读取 32 位有符号整数
// @param p 指向至少 4 字节数据的指针
// @return 解析出的 int32 值
int32_t ReadI32LE(const unsigned char* p) {
    return static_cast<int32_t>(ReadU32LE(p));
}

// 向字节流按小端序写入 32 位无符号整数
// @param p 指向至少 4 字节缓冲的指针
// @param v 待写入的值
void WriteU32LE(unsigned char* p, uint32_t v) {
    p[0] = static_cast<unsigned char>(v & 0xFF);
    p[1] = static_cast<unsigned char>((v >> 8) & 0xFF);
    p[2] = static_cast<unsigned char>((v >> 16) & 0xFF);
    p[3] = static_cast<unsigned char>((v >> 24) & 0xFF);
}

// 手工 BMP 解码：支持 24/32bit、BI_RGB/BI_BITFIELDS、底向上/顶向下
// @param path BMP 文件路径
// @return 解码出的图像（24bit -> 8UC3 BGR；32bit -> 8UC4 BGRA）；失败返回空 Mat
cv::Mat DecodeBmpManual(const std::string& path) {
    std::ifstream ifs(path, std::ios::binary);
    if (!ifs.is_open()) {
        LogMsg(LERROR, "手工 BMP 解码：无法打开文件: " + path);
        return cv::Mat();
    }
    try {
        // 读取 54 字节文件头 + 信息头（BITMAPFILEHEADER 14 + BITMAPINFOHEADER 40）
        unsigned char header[54] = {0};
        ifs.read(reinterpret_cast<char*>(header), sizeof(header));
        if (ifs.gcount() != static_cast<std::streamsize>(sizeof(header))) {
            LogMsg(LERROR, "手工 BMP 解码：文件头不足 54 字节: " + path);
            return cv::Mat();
        }
        // 校验 "BM" 魔数
        if (header[0] != 'B' || header[1] != 'M') {
            LogMsg(LERROR, "手工 BMP 解码：不是合法 BMP 文件（缺少 BM 魔数）: " + path);
            return cv::Mat();
        }
        uint32_t dataOffset = ReadU32LE(header + 10);  // 像素数据起始偏移
        int32_t  width      = ReadI32LE(header + 18);  // 图像宽度
        int32_t  heightRaw  = ReadI32LE(header + 22);  // 高度；正值=底向上，负值=顶向下
        uint32_t planes     = header[26] | (header[27] << 8);
        uint32_t bitCount   = header[28] | (header[29] << 8);
        uint32_t compression = ReadU32LE(header + 30); // 0=BI_RGB, 3=BI_BITFIELDS

        if (planes != 1 || (bitCount != 24 && bitCount != 32)) {
            LogMsg(LERROR, "手工 BMP 解码：仅支持 24/32 位 BMP，当前位深 " +
                           std::to_string(bitCount) + ": " + path);
            return cv::Mat();
        }
        // BI_RGB(0) 与 BI_BITFIELDS(3) 均按默认 BGR(A) 通道顺序处理
        if (compression != 0 && compression != 3) {
            LogMsg(LERROR, "手工 BMP 解码：仅支持 BI_RGB/BI_BITFIELDS，压缩方式 " +
                           std::to_string(compression) + ": " + path);
            return cv::Mat();
        }
        if (width <= 0 || heightRaw == 0) {
            LogMsg(LERROR, "手工 BMP 解码：非法图像尺寸: " + path);
            return cv::Mat();
        }
        bool topDown = heightRaw < 0;              // 负高度表示顶向下存储
        int height = topDown ? -heightRaw : heightRaw;

        // 读取整个文件剩余部分到缓冲，按 dataOffset 定位像素区
        ifs.seekg(0, std::ios::end);
        std::streamoff fileSize = ifs.tellg();
        ifs.seekg(0, std::ios::beg);
        std::vector<unsigned char> buf(static_cast<size_t>(fileSize));
        ifs.read(reinterpret_cast<char*>(buf.data()), fileSize);
        if (ifs.gcount() != fileSize) {
            LogMsg(LERROR, "手工 BMP 解码：读取文件内容不完整: " + path);
            return cv::Mat();
        }
        if (dataOffset >= buf.size()) {
            LogMsg(LERROR, "手工 BMP 解码：像素数据偏移越界: " + path);
            return cv::Mat();
        }

        int channels = (bitCount == 24) ? 3 : 4;
        cv::Mat img(height, width, CV_8UC(channels));
        // BMP 每行按 4 字节对齐
        size_t rowStride = (static_cast<size_t>(width) * channels + 3) & ~static_cast<size_t>(3);
        const unsigned char* pixels = buf.data() + dataOffset;
        size_t avail = buf.size() - dataOffset;
        if (rowStride * static_cast<size_t>(height) > avail) {
            LogMsg(LERROR, "手工 BMP 解码：像素数据长度不足: " + path);
            return cv::Mat();
        }
        // 逐行拷贝；底向上存储时上下翻转
        for (int y = 0; y < height; ++y) {
            int srcRow = topDown ? y : (height - 1 - y);
            const unsigned char* src = pixels + rowStride * static_cast<size_t>(srcRow);
            std::memcpy(img.ptr<unsigned char>(y), src, static_cast<size_t>(width) * channels);
        }
        LogMsg(LDEBUG, "手工 BMP 解码成功: " + path + " (" + std::to_string(width) +
                       "x" + std::to_string(height) + ", " + std::to_string(bitCount) + "bit)");
        return img;
    } catch (const std::exception& e) {
        LogMsg(LERROR, "手工 BMP 解码异常: " + path + "，原因: " + e.what());
        return cv::Mat();
    }
}

// 手工写出 32bit BI_RGB BMP（8UC4 图像兜底保存）
// @param path 输出文件路径
// @param img  8UC4 BGRA 图像
// @return 写出成功返回 true
bool EncodeBmp32Manual(const std::string& path, const cv::Mat& img) {
    if (img.empty() || img.type() != CV_8UC4) {
        LogMsg(LERROR, "手工 BMP 写出：仅接受 8UC4 图像: " + path);
        return false;
    }
    std::ofstream ofs(path, std::ios::binary | std::ios::trunc);
    if (!ofs.is_open()) {
        LogMsg(LERROR, "手工 BMP 写出：无法创建文件: " + path);
        return false;
    }
    try {
        int width = img.cols;
        int height = img.rows;
        uint32_t rowStride = static_cast<uint32_t>(width) * 4;  // 32bit 行天然 4 字节对齐
        uint32_t imageSize = rowStride * static_cast<uint32_t>(height);
        uint32_t fileSize = 54 + imageSize;

        unsigned char header[54] = {0};
        header[0] = 'B';
        header[1] = 'M';
        WriteU32LE(header + 2, fileSize);      // 文件总大小
        WriteU32LE(header + 10, 54);           // 像素数据偏移
        WriteU32LE(header + 14, 40);           // BITMAPINFOHEADER 大小
        WriteU32LE(header + 18, static_cast<uint32_t>(width));
        WriteU32LE(header + 22, static_cast<uint32_t>(height));  // 正高度 = 底向上存储
        header[26] = 1;                        // planes = 1
        header[28] = 32;                       // 32 位色
        WriteU32LE(header + 30, 0);            // BI_RGB 无压缩
        WriteU32LE(header + 34, imageSize);    // 像素数据大小
        ofs.write(reinterpret_cast<const char*>(header), sizeof(header));

        // 逐行写出：底向上存储，最后一行写在最前
        std::vector<unsigned char> rowBuf(rowStride);
        for (int y = height - 1; y >= 0; --y) {
            const unsigned char* src = img.ptr<unsigned char>(y);
            std::memcpy(rowBuf.data(), src, rowStride);
            ofs.write(reinterpret_cast<const char*>(rowBuf.data()), rowStride);
        }
        ofs.flush();
        if (!ofs.good()) {
            LogMsg(LERROR, "手工 BMP 写出：写入过程发生错误: " + path);
            return false;
        }
        LogMsg(LDEBUG, "手工 32bit BMP 写出成功: " + path);
        return true;
    } catch (const std::exception& e) {
        LogMsg(LERROR, "手工 BMP 写出异常: " + path + "，原因: " + e.what());
        return false;
    }
}

// 将字符串转小写（用于扩展名比较）
// @param s 原始字符串
// @return 全小写副本
std::string ToLower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}

}  // namespace

// ---------------------------------------------------------------------------
// 对外接口实现
// ---------------------------------------------------------------------------

cv::Mat LoadImageAny(const std::string& path) {
    cv::Mat img;
    try {
        img = cv::imread(path, cv::IMREAD_UNCHANGED);
    } catch (const cv::Exception& e) {
        LogMsg(LWARN, "cv::imread 异常，尝试手工 BMP 解码: " + path + "，原因: " + e.what());
    }
    if (!img.empty()) {
        return img;
    }
    // OpenCV 解码失败，尝试手工 BMP 兜底
    img = DecodeBmpManual(path);
    if (img.empty()) {
        LogMsg(LERROR, "图像读取失败（OpenCV 与手工解码均失败）: " + path);
    }
    return img;
}

cv::Mat ToGray8(const cv::Mat& img) {
    if (img.empty()) {
        LogMsg(LERROR, "ToGray8：输入图像为空");
        return cv::Mat();
    }
    if (img.type() == CV_8UC1) {
        return img;
    }
    cv::Mat gray;
    try {
        if (img.type() == CV_8UC3) {
            cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
            return gray;
        }
        if (img.type() == CV_8UC4) {
            cv::cvtColor(img, gray, cv::COLOR_BGRA2GRAY);
            return gray;
        }
    } catch (const cv::Exception& e) {
        LogMsg(LERROR, "ToGray8：颜色转换异常，原因: " + std::string(e.what()));
        return cv::Mat();
    }
    LogMsg(LERROR, "ToGray8：不支持的图像类型，type = " + std::to_string(img.type()));
    return cv::Mat();
}

bool SaveImage(const std::string& path, const cv::Mat& img) {
    if (img.empty()) {
        LogMsg(LERROR, "SaveImage：输入图像为空: " + path);
        return false;
    }
    try {
        if (cv::imwrite(path, img)) {
            return true;
        }
        LogMsg(LWARN, "cv::imwrite 返回失败: " + path);
    } catch (const cv::Exception& e) {
        LogMsg(LWARN, "cv::imwrite 异常: " + path + "，原因: " + e.what());
    }
    // 32 位图保存失败时用手工 32bit BI_RGB BMP 写出兜底
    if (img.type() == CV_8UC4) {
        return EncodeBmp32Manual(path, img);
    }
    LogMsg(LERROR, "图像保存失败: " + path);
    return false;
}

std::vector<std::string> ListImages(const std::string& dir) {
    static const char* kExts[] = {".bmp", ".png", ".jpg", ".jpeg", ".tif", ".tiff"};
    std::vector<std::string> result;
    std::error_code ec;
    if (!std::filesystem::is_directory(dir, ec)) {
        LogMsg(LERROR, "ListImages：目录不存在或不可访问: " + dir);
        return result;
    }
    try {
        for (const auto& entry : std::filesystem::directory_iterator(dir)) {
            if (!entry.is_regular_file()) {
                continue;
            }
            std::string ext = ToLower(entry.path().extension().string());
            bool ok = false;
            for (const char* e : kExts) {
                if (ext == e) {
                    ok = true;
                    break;
                }
            }
            if (!ok) {
                continue;
            }
            // 剔除文件名（不含扩展名）含 "demo" 或 "副本" 的条目
            std::string stem = entry.path().stem().string();
            std::string stemLow = ToLower(stem);
            if (stemLow.find("demo") != std::string::npos ||
                stem.find("副本") != std::string::npos) {
                continue;
            }
            result.push_back(entry.path().string());
        }
    } catch (const std::filesystem::filesystem_error& e) {
        LogMsg(LERROR, "ListImages：遍历目录异常: " + dir + "，原因: " + e.what());
        return result;
    }
    std::sort(result.begin(), result.end());
    LogMsg(LINFO, "输入目录 " + dir + " 共发现 " + std::to_string(result.size()) + " 张图像");
    return result;
}

}  // namespace common
