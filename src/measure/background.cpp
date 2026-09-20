// ============================================================================
// measure/background.cpp —— 背景建模与外轮廓提取实现
// 等价移植自 Python 工程 src/edge_measure/background.py
// ============================================================================

#include "measure/background.h"

#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <cmath>
#include <filesystem>
#include <numeric>

#include <opencv2/imgproc.hpp>

#include "common/image_io.h"
#include "common/logger.h"

namespace cam {
namespace {

// snprintf 风格日志格式化小工具
std::string Fmt(const char* fmt, ...) {
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    return buf;
}

// 序列中位数（偶数个取两中值平均，与 np.median 语义一致）。
// 注意：内部做 nth_element，会打乱 buf 元素顺序（调用方不依赖顺序）。
double MedianVec(std::vector<float>& buf) {
    const size_t n = buf.size();
    const size_t mid = n / 2;
    std::nth_element(buf.begin(), buf.begin() + mid, buf.end());
    const float hi = buf[mid];
    if (n % 2 != 0) {
        return hi;
    }
    const float lo = *std::max_element(buf.begin(), buf.begin() + mid);
    return (double)(lo + hi) * 0.5;
}

}  // namespace

bool BuildBackgroundModel(const std::vector<std::string>& imagePaths,
                          const std::string& cachePath,
                          cv::Mat& background, std::string& errMsg,
                          double madThreshold, int fitSamples) {
    if (imagePaths.empty()) {
        errMsg = "背景建模需要至少一张图像";
        common::LogMsg(common::LERROR, "背景建模失败：图像列表为空");
        return false;
    }

    // 1) 读取全部帧并校验尺寸一致
    std::vector<cv::Mat> frames;
    frames.reserve(imagePaths.size());
    for (const std::string& p : imagePaths) {
        cv::Mat g = common::ToGray8(common::LoadImageAny(p));
        if (g.empty()) {
            errMsg = "背景建模图像读取失败: " + p;
            common::LogMsg(common::LERROR, errMsg);
            return false;
        }
        if (!frames.empty() && frames[0].size() != g.size()) {
            errMsg = "背景建模失败：图像尺寸不一致";
            common::LogMsg(common::LERROR, errMsg);
            return false;
        }
        frames.push_back(g);
    }
    const int h = frames[0].rows;
    const int w = frames[0].cols;
    const int nFrames = (int)frames.size();

    // 2) 逐像素时序中位数 + MAD（中位绝对偏差），MAD < madThreshold 判为高置信背板像素
    cv::Mat medianImg(h, w, CV_32FC1);
    cv::Mat confident(h, w, CV_8UC1);
    long long confCount = 0;
    {
        std::vector<float> buf(nFrames), dev(nFrames);
        std::vector<const unsigned char*> rows(nFrames);
        for (int y = 0; y < h; ++y) {
            for (int f = 0; f < nFrames; ++f) {
                rows[f] = frames[f].ptr<unsigned char>(y);
            }
            float* medRow = medianImg.ptr<float>(y);
            unsigned char* confRow = confident.ptr<unsigned char>(y);
            for (int x = 0; x < w; ++x) {
                for (int f = 0; f < nFrames; ++f) {
                    buf[f] = (float)rows[f][x];
                }
                const double med = MedianVec(buf);
                for (int f = 0; f < nFrames; ++f) {
                    dev[f] = std::fabs(buf[f] - (float)med);
                }
                const double mad = MedianVec(dev);
                medRow[x] = (float)med;
                if (mad < madThreshold) {
                    confRow[x] = 255;
                    ++confCount;
                } else {
                    confRow[x] = 0;
                }
            }
        }
    }
    const double confRatio = (double)confCount / ((double)h * (double)w);
    common::LogMsg(common::LINFO,
                   Fmt("高置信背板像素占比 %.1f%%（MAD < %.1f）", confRatio * 100.0, madThreshold));
    if (confRatio < 0.2) {
        errMsg = "背景建模失败：高置信像素过少，无法可靠建模，请拍摄空背板图";
        common::LogMsg(common::LERROR,
                       Fmt("高置信背板像素过少（%.1f%%），无法可靠建模", confRatio * 100.0));
        return false;
    }

    // 3) 收集全部高置信点（供第 5 步回填实测中值），拟合子集用 cv::RNG(42) 抽稀
    std::vector<int> xs, ys;
    std::vector<float> vals;
    xs.reserve((size_t)confCount);
    ys.reserve((size_t)confCount);
    vals.reserve((size_t)confCount);
    for (int y = 0; y < h; ++y) {
        const unsigned char* confRow = confident.ptr<unsigned char>(y);
        const float* medRow = medianImg.ptr<float>(y);
        for (int x = 0; x < w; ++x) {
            if (confRow[x] != 0) {
                xs.push_back(x);
                ys.push_back(y);
                vals.push_back(medRow[x]);
            }
        }
    }
    const int total = (int)xs.size();
    std::vector<int> sel(total);
    std::iota(sel.begin(), sel.end(), 0);
    int m = total;
    if (m > fitSamples) {
        // Fisher-Yates 部分洗牌抽稀（无放回，统计语义同 np.random.choice）
        cv::RNG rng(42);
        for (int i = 0; i < fitSamples; ++i) {
            const int j = i + (int)rng.uniform(0, m - i);
            std::swap(sel[i], sel[j]);
        }
        sel.resize(fitSamples);
        m = fitSamples;
    }

    // 4) 双二次照度曲面最小二乘：z = a·x² + b·y² + c·xy + d·x + e·y + f（坐标归一化）
    cv::Mat design(m, 6, CV_64FC1);
    cv::Mat target(m, 1, CV_64FC1);
    for (int i = 0; i < m; ++i) {
        const double xn = (double)xs[sel[i]] / w;
        const double yn = (double)ys[sel[i]] / h;
        double* drow = design.ptr<double>(i);
        drow[0] = xn * xn;
        drow[1] = yn * yn;
        drow[2] = xn * yn;
        drow[3] = xn;
        drow[4] = yn;
        drow[5] = 1.0;
        target.at<double>(i) = vals[sel[i]];
    }
    cv::Mat coef;
    if (!cv::solve(design, target, coef, cv::DECOMP_SVD)) {
        errMsg = "背景建模失败：双二次曲面最小二乘求解失败";
        common::LogMsg(common::LERROR, errMsg);
        return false;
    }
    const double* c = coef.ptr<double>();

    // 5) 全图曲面外推；高置信区保留实测中位数（比曲面更贴局部纹理）
    cv::Mat modelF(h, w, CV_32FC1);
    for (int y = 0; y < h; ++y) {
        const double gy = (double)y / h;
        float* mrow = modelF.ptr<float>(y);
        for (int x = 0; x < w; ++x) {
            const double gx = (double)x / w;
            mrow[x] = (float)(c[0] * gx * gx + c[1] * gy * gy + c[2] * gx * gy
                              + c[3] * gx + c[4] * gy + c[5]);
        }
    }
    for (size_t i = 0; i < xs.size(); ++i) {
        modelF.at<float>(ys[i], xs[i]) = vals[i];
    }

    // 6) clip [0,255] 后截断转 8UC1（对齐 np.clip + astype(uint8) 的截断语义）
    background.create(h, w, CV_8UC1);
    for (int y = 0; y < h; ++y) {
        const float* mrow = modelF.ptr<float>(y);
        unsigned char* brow = background.ptr<unsigned char>(y);
        for (int x = 0; x < w; ++x) {
            const int iv = (int)mrow[x];  // 向零截断，同 np.astype
            brow[x] = (unsigned char)std::clamp(iv, 0, 255);
        }
    }
    common::LogMsg(common::LINFO,
                   Fmt("背景模型重建完成：%d 帧, 尺寸 %dx%d", nFrames, w, h));

    if (!cachePath.empty()) {
        // 确保缓存父目录存在后落盘；写失败仅告警（模型本体已在内存中可用）
        namespace fs = std::filesystem;
        std::error_code ec;
        const fs::path parent = fs::path(cachePath).parent_path();
        if (!parent.empty()) {
            fs::create_directories(parent, ec);
        }
        if (common::SaveImage(cachePath, background)) {
            common::LogMsg(common::LINFO, "背景模型已缓存: " + cachePath);
        } else {
            common::LogMsg(common::LWARN, "背景模型缓存写入失败: " + cachePath);
        }
    }
    return true;
}

bool LoadOrBuildBackground(const std::vector<std::string>& imagePaths,
                           const std::string& cachePath,
                           cv::Mat& background, std::string& errMsg) {
    namespace fs = std::filesystem;
    std::error_code ec;
    if (!cachePath.empty() && fs::exists(cachePath, ec)) {
        common::LogMsg(common::LINFO, "加载缓存背景模型: " + cachePath);
        cv::Mat g = common::ToGray8(common::LoadImageAny(cachePath));
        if (!g.empty()) {
            background = g;
            return true;
        }
        common::LogMsg(common::LWARN, "缓存背景模型读取失败，改为重新建模: " + cachePath);
    }
    return BuildBackgroundModel(imagePaths, cachePath, background, errMsg);
}

bool LoadBackgroundCache(const std::string& cachePath,
                         cv::Mat& background, std::string& errMsg) {
    namespace fs = std::filesystem;
    std::error_code ec;
    if (cachePath.empty() || !fs::exists(cachePath, ec)) {
        errMsg = "缓存文件不存在: " + cachePath;
        return false;
    }
    cv::Mat g = common::ToGray8(common::LoadImageAny(cachePath));
    if (g.empty()) {
        errMsg = "缓存文件读取失败: " + cachePath;
        return false;
    }
    common::LogMsg(common::LINFO, "加载缓存背景模型: " + cachePath);
    background = g;
    return true;
}

bool ExtractOuterContour(const cv::Mat& mask, std::vector<cv::Point>& contour,
                         std::string& errMsg) {
    std::vector<std::vector<cv::Point>> contours;
    // clone 兜底：避免任何 OpenCV 版本下 findContours 改动源掩膜的歧义
    cv::findContours(mask.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    if (contours.empty()) {
        errMsg = "掩膜中未找到轮廓";
        common::LogMsg(common::LERROR, errMsg);
        return false;
    }
    size_t best = 0;
    double bestArea = -1.0;
    for (size_t i = 0; i < contours.size(); ++i) {
        const double a = cv::contourArea(contours[i]);
        if (a > bestArea) {
            bestArea = a;
            best = i;
        }
    }
    contour = contours[best];
    common::LogMsg(common::LDEBUG,
                   Fmt("外轮廓点数: %d, 周长: %.1f px", (int)contour.size(),
                       cv::arcLength(contour, true)));
    return true;
}

}  // namespace cam
