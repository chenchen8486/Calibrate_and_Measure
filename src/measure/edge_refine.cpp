// ============================================================================
// measure/edge_refine.cpp —— 增强图构建与掩膜边缘法线卡尺精修实现
// 等价移植自 Python 工程 src/edge_measure/edge_refine.py
// ============================================================================

#include "measure/edge_refine.h"

#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <cmath>

#include <opencv2/imgproc.hpp>

#include "common/logger.h"
#include "measure/background.h"

namespace cam {
namespace {

std::string Fmt(const char* fmt, ...) {
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    return buf;
}

// 8UC1 灰度图的中位数（直方图实现；偶数个取两中值平均，与 np.median 一致）
double MedianOfGray8(const cv::Mat& img) {
    long long hist[256] = {0};
    for (int y = 0; y < img.rows; ++y) {
        const unsigned char* row = img.ptr<unsigned char>(y);
        for (int x = 0; x < img.cols; ++x) {
            ++hist[row[x]];
        }
    }
    const long long n = (long long)img.rows * img.cols;
    if (n == 0) {
        return 0.0;
    }
    // 排序后下标 k1=(n-1)/2 与 k2=n/2（奇数时相同，偶数时取两者平均）
    const long long k1 = (n - 1) / 2, k2 = n / 2;
    long long acc = 0;
    int v1 = -1, v2 = -1;
    for (int i = 0; i < 256 && (v1 < 0 || v2 < 0); ++i) {
        acc += hist[i];
        if (v1 < 0 && acc > k1) v1 = i;
        if (v2 < 0 && acc > k2) v2 = i;
    }
    return (v1 + v2) * 0.5;
}

// 循环序列（闭合轮廓）滑动中值滤波
std::vector<double> MedianFilterCircular(const std::vector<double>& values, int window) {
    const int n = (int)values.size();
    if (window < 1) {
        window = 1;
    }
    const int halfW = window / 2;
    std::vector<double> out(n);
    std::vector<double> buf(window);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < window; ++j) {
            int idx = (i - halfW + j) % n;
            if (idx < 0) idx += n;
            buf[j] = values[idx];
        }
        std::sort(buf.begin(), buf.end());
        out[i] = (window % 2 != 0) ? buf[window / 2]
                                   : (buf[window / 2 - 1] + buf[window / 2]) * 0.5;
    }
    return out;
}

// 计算轮廓第 i 点指向掩膜外侧的单位法线。
// 前后 ±2 点差分得切线，法线取切线垂直方向；以"沿负法线走 3 px 应落在掩膜内"
// 判定朝向，不符则翻转。
bool OrientedNormal(const std::vector<cv::Point2d>& pts, int i, const cv::Mat& mask,
                    cv::Point2d& normal) {
    const int n = (int)pts.size();
    const cv::Point2d tangent = pts[(i + 2) % n] - pts[(i - 2 + n) % n];
    const double norm = std::hypot(tangent.x, tangent.y);
    if (norm < 1e-9) {
        return false;
    }
    const cv::Point2d t = tangent * (1.0 / norm);
    normal = cv::Point2d(-t.y, t.x);
    const cv::Point2d pIn = pts[i] - normal * 3.0;
    const int x = cvRound(pIn.x), y = cvRound(pIn.y);
    if (x >= 0 && x < mask.cols && y >= 0 && y < mask.rows &&
        mask.at<unsigned char>(y, x) == 0) {
        normal = -normal;  // 负法线侧不在掩膜内，说明当前方向反了
    }
    return true;
}

}  // namespace

cv::Mat ApplyBilateralGamma(const cv::Mat& img, const common::TradSegConfig& cfg) {
    cv::Mat filtered;
    cv::bilateralFilter(img, filtered, cfg.bilateral_d, cfg.sigma_color, cfg.sigma_space);
    cv::Mat lut(1, 256, CV_8UC1);
    for (int i = 0; i < 256; ++i) {
        // 截断取整，与 np.astype(uint8) 一致
        lut.at<unsigned char>(i) = (unsigned char)(std::pow(i / 255.0, cfg.gamma) * 255.0);
    }
    cv::Mat out;
    cv::LUT(filtered, lut, out);
    return out;
}

cv::Mat BuildEnhanced(const cv::Mat& gray, const cv::Mat& background,
                      const common::TradSegConfig& cfg, cv::Mat* enhancedBg) {
    cv::Mat eGray = ApplyBilateralGamma(gray, cfg);
    cv::Mat eBg = ApplyBilateralGamma(background, cfg);
    // 全局亮度对齐：整图中位数差作为照明漂移补偿（产品占比 <50%，中位数必落在背板）
    const double offset = MedianOfGray8(eBg) - MedianOfGray8(eGray);
    common::LogMsg(common::LDEBUG, Fmt("亮度对齐偏移: %.2f", offset));
    cv::Mat aligned;
    cv::add(eGray, cv::Scalar(offset), aligned);  // 饱和加，同 cv2.add
    if (enhancedBg != nullptr) {
        *enhancedBg = eBg;
    }
    return aligned;
}

double SnapProfile(const cv::Mat& enhanced, const cv::Point2d& origin,
                   const cv::Point2d& normal, const common::RefineConfig& cfg) {
    const double r = cfg.search_radius;
    // step 检测器窗口有 half 半径的盲区，剖面向外延拓 step_window 保证 |t|<=r 全程有效
    const double ext = cfg.step_window;
    const double start = -(r + ext);
    const double stop = (r + ext) + cfg.profile_step * 0.5;
    const int n = (int)std::ceil((stop - start) / cfg.profile_step);
    if (n < 5) {
        return 0.0;
    }
    // 1) 法线剖面亚像素采样（双线性 + 边缘复制）
    cv::Mat mapX(1, n, CV_32FC1), mapY(1, n, CV_32FC1);
    std::vector<float> ts(n);
    for (int i = 0; i < n; ++i) {
        const float t = (float)(start + i * cfg.profile_step);
        ts[i] = t;
        mapX.at<float>(0, i) = (float)(origin.x + normal.x * t);
        mapY.at<float>(0, i) = (float)(origin.y + normal.y * t);
    }
    cv::Mat profile8, profile;
    cv::remap(enhanced, profile8, mapX, mapY, cv::INTER_LINEAR, cv::BORDER_REPLICATE);
    profile8.convertTo(profile, CV_32FC1);
    const float* prof = profile.ptr<float>(0);

    // 2) 期望边缘符号：内段(t<-5px)与外段(t>+5px)均值对比，内亮外暗期望负边缘
    double innerSum = 0.0, outerSum = 0.0;
    int innerCnt = 0, outerCnt = 0;
    for (int i = 0; i < n; ++i) {
        if (ts[i] < -5.0f) {
            innerSum += prof[i];
            ++innerCnt;
        } else if (ts[i] > 5.0f) {
            outerSum += prof[i];
            ++outerCnt;
        }
    }
    if (innerCnt == 0 || outerCnt == 0) {
        return 0.0;
    }
    const double expectSign = (innerSum / innerCnt) > (outerSum / outerCnt) ? -1.0 : 1.0;

    // 3) 高斯平滑
    cv::Mat smoothM;
    cv::GaussianBlur(profile, smoothM, cv::Size(5, 1), 1.0);
    const float* smooth = smoothM.ptr<float>(0);

    // 4) 窗口均值差阶跃信号（前缀和实现）：
    //    sig[i] = mean(smooth[i+1 .. i+half]) - mean(smooth[i-half .. i-1])
    //    逐像素差分对 15~20px 离焦缓坡失明，窗口均值差把缓坡累积成清晰峰。
    const int half = std::max(1, (int)std::lround(cfg.step_window / cfg.profile_step));
    std::vector<double> cs(n + 1, 0.0);
    for (int i = 0; i < n; ++i) {
        cs[i + 1] = cs[i] + smooth[i];
    }
    std::vector<double> signedSig(n, 0.0);  // 符号匹配强度（正值 = 符号匹配）
    std::vector<char> validMask(n, 0);
    for (int i = half; i < n - half; ++i) {
        const double outer = (cs[i + 1 + half] - cs[i + 1]) / half;
        const double inner = (cs[i] - cs[i - half]) / half;
        signedSig[i] = (outer - inner) * expectSign;
        if (std::fabs(ts[i]) <= r) {
            validMask[i] = 1;  // 吸附目标限定在真实搜索半径内
        }
    }

    // 5) 最强符号匹配峰检查；弱边缘保持原点，不乱吸
    double strongest = 0.0;
    for (int i = 0; i < n; ++i) {
        if (validMask[i] && signedSig[i] > strongest) {
            strongest = signedSig[i];
        }
    }
    if (strongest < cfg.gradient_threshold) {
        return 0.0;
    }

    // 6) 合格区可能展宽成平台/宽三角，按连通段取各段峰顶，再选峰顶离原点最近的一段
    std::vector<int> apexes;
    for (int i = 0; i < n;) {
        if (validMask[i] && signedSig[i] >= cfg.peak_ratio * strongest) {
            int j = i, apex = i;
            while (j + 1 < n && validMask[j + 1] &&
                   signedSig[j + 1] >= cfg.peak_ratio * strongest) {
                ++j;
                if (signedSig[j] > signedSig[apex]) {
                    apex = j;
                }
            }
            apexes.push_back(apex);
            i = j + 1;
        } else {
            ++i;
        }
    }
    if (apexes.empty()) {
        return 0.0;
    }
    int k = apexes[0];
    for (const int a : apexes) {
        if (std::fabs(ts[a]) < std::fabs(ts[k])) {
            k = a;
        }
    }

    // 7) 抛物线亚像素插值（在符号匹配强度序列上）
    double offset = 0.0;
    if (k > 0 && k < n - 1) {
        const double y0 = signedSig[k - 1], y1 = signedSig[k], y2 = signedSig[k + 1];
        const double denom = y0 - 2.0 * y1 + y2;
        if (std::fabs(denom) > 1e-9) {
            offset = std::clamp(0.5 * (y0 - y2) / denom, -1.0, 1.0);
        }
    }
    const double tEdge = ts[k] + offset * cfg.profile_step;
    // 吸附幅度硬约束：不得超出搜索半径（抛物线插值外推保护）
    return std::clamp(tEdge, -r, r);
}

bool RefineMask(const cv::Mat& mask, const cv::Mat& enhanced,
                const common::RefineConfig& cfg, cv::Mat& refinedMask,
                RefineDebugInfo* debug, std::string& errMsg) {
    std::vector<cv::Point> contour;
    if (!ExtractOuterContour(mask, contour, errMsg)) {
        return false;
    }
    std::vector<cv::Point2d> pts;
    pts.reserve(contour.size());
    for (const cv::Point& p : contour) {
        pts.emplace_back((double)p.x, (double)p.y);
    }
    const int n = (int)pts.size();

    // 1) 轮廓每 sample_step 采样，定向法线 + 逐点剖面卡尺
    std::vector<cv::Point2d> origins, normals;
    std::vector<double> deltas;
    for (int i = 0; i < n; i += cfg.sample_step) {
        cv::Point2d normal;
        if (!OrientedNormal(pts, i, mask, normal)) {
            continue;
        }
        const double delta = SnapProfile(enhanced, pts[i], normal, cfg);
        origins.push_back(pts[i]);
        normals.push_back(normal);
        deltas.push_back(delta);
    }

    std::vector<cv::Point2d> refined;
    if (origins.empty()) {
        common::LogMsg(common::LWARN, "轮廓精修：无有效采样点，返回原轮廓重建掩膜");
        origins = pts;
        refined = pts;
        deltas.assign(n, 0.0);
    } else {
        // 2) 滑动中值去野点：吸附偏移偏离局部中值超阈值的点回退为中值偏移
        if ((int)deltas.size() >= cfg.median_window) {
            const std::vector<double> med = MedianFilterCircular(deltas, cfg.median_window);
            int outliers = 0;
            for (size_t i = 0; i < deltas.size(); ++i) {
                if (std::fabs(deltas[i] - med[i]) > cfg.outlier_tol) {
                    deltas[i] = med[i];
                    ++outliers;
                }
            }
            if (outliers > 0) {
                common::LogMsg(common::LDEBUG,
                               Fmt("吸附去野: %d/%d 点回退为中值偏移", outliers,
                                   (int)deltas.size()));
            }
        }
        double sumAbs = 0.0, maxAbs = 0.0;
        for (size_t i = 0; i < origins.size(); ++i) {
            refined.push_back(origins[i] + normals[i] * deltas[i]);
            sumAbs += std::fabs(deltas[i]);
            maxAbs = std::max(maxAbs, std::fabs(deltas[i]));
        }
        common::LogMsg(common::LINFO,
                       Fmt("轮廓精修: %d 采样点, 平均吸附 %.2f px, 最大 %.2f px",
                           (int)deltas.size(), sumAbs / deltas.size(), maxAbs));
    }

    // 3) fillPoly 重建掩膜
    refinedMask = cv::Mat::zeros(mask.size(), CV_8UC1);
    std::vector<cv::Point> poly;
    poly.reserve(refined.size());
    for (const cv::Point2d& p : refined) {
        poly.emplace_back(cvRound(p.x), cvRound(p.y));
    }
    const std::vector<std::vector<cv::Point>> polys = {poly};
    cv::fillPoly(refinedMask, polys, cv::Scalar(255));

    if (debug != nullptr) {
        debug->origins = origins;
        debug->refined = refined;
        debug->deltas = deltas;
    }
    return true;
}

}  // namespace cam
