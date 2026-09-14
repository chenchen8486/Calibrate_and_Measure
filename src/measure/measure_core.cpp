// ============================================================================
// measure/measure_core.cpp —— 亚像素测量核心实现
// 等价移植自 Python 工程 src/edge_measure/measure.py 与 measure_ranked_edges.py
// ============================================================================

#include "measure/measure_core.h"

#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <limits>

#include <opencv2/imgproc.hpp>

#include "common/logger.h"
#include "measure/background.h"
#include "measure/edge_refine.h"

namespace cam {
namespace {

constexpr double kRad2Deg = 180.0 / CV_PI;

std::string Fmt(const char* fmt, ...) {
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    return buf;
}

// 线性插值分位数（与 np.percentile 默认 linear 方法一致）
double PercentileLinear(std::vector<double> v, double p) {
    if (v.empty()) {
        return 0.0;
    }
    std::sort(v.begin(), v.end());
    if (v.size() == 1) {
        return v[0];
    }
    const double rank = p / 100.0 * ((double)v.size() - 1.0);
    const size_t lo = (size_t)std::floor(rank);
    const size_t hi = (size_t)std::ceil(rank);
    const double frac = rank - (double)lo;
    return v[lo] * (1.0 - frac) + v[hi] * frac;
}

// 中位数（np.median = 50% 线性分位数）
double MedianOf(std::vector<double> v) {
    return PercentileLinear(std::move(v), 50.0);
}

// 测量侧别
enum class Side { Left, Right, Top, Bottom };

cv::Point2d SideNormal(Side side) {
    switch (side) {
        case Side::Left:   return cv::Point2d(-1.0, 0.0);
        case Side::Right:  return cv::Point2d(1.0, 0.0);
        case Side::Top:    return cv::Point2d(0.0, -1.0);
        case Side::Bottom: return cv::Point2d(0.0, 1.0);
    }
    return cv::Point2d(0.0, 0.0);
}

const char* SideName(Side side) {
    switch (side) {
        case Side::Left:   return "left";
        case Side::Right:  return "right";
        case Side::Top:    return "top";
        case Side::Bottom: return "bottom";
    }
    return "?";
}

// 求某一侧的语义极值线位置（measure.py _semantic_extreme 等价移植）。
// 1) 掩膜轮廓极值带（extreme_band）内的轮廓点给出"哪条边算边"的语义答案，
//    沿扫描轴 10%~90% 分位区间均匀取 refine_rows 行；
// 2) 每行以轮廓极值为原点、沿外侧法线发窗口阶跃卡尺，亚像素定位后取中位数。
// 极值带无点时回退整像素极值。
double SemanticExtreme(const cv::Mat& gray, const std::vector<cv::Point2d>& contourPts,
                       Side side, const common::MeasureConfig& cfg,
                       const common::RefineConfig& refineCfg,
                       std::vector<cv::Point2d>& refinedOut) {
    refinedOut.clear();
    const int axis = (side == Side::Left || side == Side::Right) ? 0 : 1;  // 极值取值轴
    const int scanAxis = 1 - axis;  // 扫描轴
    const bool pickMin = (side == Side::Left || side == Side::Top);
    auto coord = [](const cv::Point2d& p, int a) { return a == 0 ? p.x : p.y; };

    double extreme = pickMin ? std::numeric_limits<double>::max()
                             : std::numeric_limits<double>::lowest();
    for (const cv::Point2d& p : contourPts) {
        const double v = coord(p, axis);
        extreme = pickMin ? std::min(extreme, v) : std::max(extreme, v);
    }
    std::vector<cv::Point2d> bandPts;
    for (const cv::Point2d& p : contourPts) {
        const double v = coord(p, axis);
        if ((pickMin && v <= extreme + cfg.extreme_band) ||
            (!pickMin && v >= extreme - cfg.extreme_band)) {
            bandPts.push_back(p);
        }
    }
    if (bandPts.empty()) {
        common::LogMsg(common::LWARN,
                       Fmt("%s 侧极值带无轮廓点，回退整像素极值 %.1f",
                           SideName(side), extreme));
        return extreme;
    }

    // 扫描轴取 10%~90% 分位的中段（避开角落过渡），均匀采样
    std::vector<double> scanVals;
    scanVals.reserve(bandPts.size());
    for (const cv::Point2d& p : bandPts) {
        scanVals.push_back(coord(p, scanAxis));
    }
    const double sLo = PercentileLinear(scanVals, 10.0);
    const double sHi = PercentileLinear(scanVals, 90.0);
    const int rows = std::max(3, cfg.refine_rows);
    const cv::Point2d normal = SideNormal(side);
    std::vector<double> posVals;
    posVals.reserve(rows);
    for (int k = 0; k < rows; ++k) {
        const double s = sLo + (sHi - sLo) * k / (rows - 1);
        const cv::Point2d origin =
            (axis == 0) ? cv::Point2d(extreme, s) : cv::Point2d(s, extreme);
        const double delta = SnapProfile(gray, origin, normal, refineCfg);
        const cv::Point2d pt = origin + normal * delta;
        refinedOut.push_back(pt);
        posVals.push_back(coord(pt, axis));
    }
    const double pos = MedianOf(posVals);
    common::LogMsg(common::LDEBUG,
                   Fmt("极值线[%s]: 整像素极值 %.1f → 亚像素 %.2f（%d 行精化）",
                       SideName(side), extreme, pos, rows));
    return pos;
}

// ---------------------------------------------------------------------------
// 水平边排名（measure_ranked_edges.py 等价移植）
// ---------------------------------------------------------------------------

// 循环索引（负值取模对齐 Python np.roll 语义）
int CircIdx(int i, int n) {
    return ((i % n) + n) % n;
}

// 闭合轮廓点的循环滑动平均（仅用于切线角估计，不污染宽度计算）。
// ±3 点滑动平均抹平旋转/栅格化后的像素级锯齿，又不影响真实圆角的分类。
std::vector<cv::Point2d> SmoothCircular(const std::vector<cv::Point2d>& pts, int radius) {
    const int n = (int)pts.size();
    std::vector<cv::Point2d> out(n);
    const double inv = 1.0 / (2 * radius + 1);
    for (int k = 0; k < n; ++k) {
        double sx = 0.0, sy = 0.0;
        for (int i = -radius; i <= radius; ++i) {
            const cv::Point2d& p = pts[CircIdx(k + i, n)];
            sx += p.x;
            sy += p.y;
        }
        out[k] = cv::Point2d(sx * inv, sy * inv);
    }
    return out;
}

// 找出闭合轮廓上全部连续近水平点段（种子 run），按轮廓行进顺序返回。
// 先旋转索引使起点落在非水平点上，自然处理首尾环绕。
std::vector<std::vector<int>> HorizontalRuns(const std::vector<cv::Point2d>& pts,
                                             double angleTolDeg, int span) {
    const int n = (int)pts.size();
    std::vector<std::vector<int>> runs;
    // 逐轮廓点切线角：先 ±3 点循环平滑，再 ±span 点差分，归一到 [0, 180)
    const std::vector<cv::Point2d> smooth = SmoothCircular(pts, 3);
    std::vector<char> nearH(n, 0);
    bool any = false, all = true;
    for (int i = 0; i < n; ++i) {
        const cv::Point2d& pNext = smooth[CircIdx(i + span, n)];
        const cv::Point2d& pPrev = smooth[CircIdx(i - span, n)];
        const double ang = std::fmod(
            std::fabs(std::atan2(pNext.y - pPrev.y, pNext.x - pPrev.x) * kRad2Deg), 180.0);
        const bool h = (ang <= angleTolDeg) || (ang >= 180.0 - angleTolDeg);
        nearH[i] = h ? 1 : 0;
        any = any || h;
        all = all && h;
    }
    if (!any || all) {
        return runs;  // 全水平（退化形状）或无水平边
    }
    int start = 0;
    while (start < n && nearH[start]) {
        ++start;  // 一个非水平点作为切开位置
    }
    int i = 0;
    while (i < n) {
        if (nearH[(i + start) % n]) {
            std::vector<int> run;
            int j = i;
            while (j < n && nearH[(j + start) % n]) {
                run.push_back((j + start) % n);
                ++j;
            }
            runs.push_back(std::move(run));
            i = j;
        } else {
            ++i;
        }
    }
    return runs;
}

// 合并被小间隙打断的相邻 run（间隙 <= merge_gap_pts 轮廓点且高度差
// <= 2·line_dist_tol）；线性合并后再做一次首尾环绕合并（间隙用模 N 计算）。
std::vector<std::vector<int>> MergeRuns(const std::vector<std::vector<int>>& runs,
                                        const std::vector<cv::Point2d>& pts,
                                        const common::MeasureConfig& cfg) {
    if (runs.size() < 2) {
        return runs;
    }
    auto medianY = [&pts](const std::vector<int>& run) {
        std::vector<double> ys;
        ys.reserve(run.size());
        for (const int idx : run) {
            ys.push_back(pts[idx].y);
        }
        return MedianOf(std::move(ys));
    };
    std::vector<std::vector<int>> merged;
    merged.push_back(runs[0]);
    for (size_t r = 1; r < runs.size(); ++r) {
        const std::vector<int>& prev = merged.back();
        const int gap = runs[r].front() - prev.back();  // 轮廓索引间距（含间隙点）
        const double yPrev = medianY(prev);
        const double yCur = medianY(runs[r]);
        if (gap <= cfg.merge_gap_pts + 1 &&
            std::fabs(yCur - yPrev) <= 2.0 * cfg.line_dist_tol) {
            merged.back().insert(merged.back().end(), runs[r].begin(), runs[r].end());
        } else {
            merged.push_back(runs[r]);
        }
    }
    if (merged.size() >= 2) {
        const int n = (int)pts.size();
        const std::vector<int>& first = merged.front();
        const std::vector<int>& last = merged.back();
        // 首尾环绕合并（Python 负数取模语义一致化）
        const int wrapGap = ((first.front() - last.back() - 1) % n + n) % n;
        const double yFirst = medianY(first);
        const double yLast = medianY(last);
        if (wrapGap <= cfg.merge_gap_pts + 1 &&
            std::fabs(yFirst - yLast) <= 2.0 * cfg.line_dist_tol) {
            std::vector<int> mergedRun = last;
            mergedRun.insert(mergedRun.end(), first.begin(), first.end());
            merged[0] = std::move(mergedRun);
            merged.pop_back();
        }
    }
    return merged;
}

// 用户规则的线段选取：全部轮廓点投影到拟合直线，垂直距离 <= line_dist_tol
// 且沿直线方向连续（断档 <= span_gap_tol）的、与种子交叠的段落入选；
// 端点 = 段端在直线上的亚像素投影（左右按 x 排序），不做角点外推。
bool SpanAlongLine(const std::vector<cv::Point2d>& pts,
                   const std::vector<cv::Point2d>& seed, const LineResult& line,
                   const common::MeasureConfig& cfg,
                   cv::Point2d& p0, cv::Point2d& p1, double& width) {
    const cv::Point2d dir = line.direction;
    const cv::Point2d normal(-dir.y, dir.x);
    std::vector<double> tOn;
    for (const cv::Point2d& p : pts) {
        const cv::Point2d rel = p - line.point;
        if (std::fabs(rel.dot(normal)) <= cfg.line_dist_tol) {
            tOn.push_back(rel.dot(dir));
        }
    }
    if (tOn.size() < 3) {
        return false;
    }
    std::sort(tOn.begin(), tOn.end());
    double sLo = std::numeric_limits<double>::max();
    double sHi = std::numeric_limits<double>::lowest();
    for (const cv::Point2d& p : seed) {
        const double t = (p - line.point).dot(dir);
        sLo = std::min(sLo, t);
        sHi = std::max(sHi, t);
    }
    // 连续段切分，取与种子区间有交叠的一段
    double segLo = tOn.front(), prev = tOn.front();
    auto tryPick = [&](double lo, double hi) -> bool {
        if (lo <= sHi && hi >= sLo) {
            p0 = line.point + dir * lo;
            p1 = line.point + dir * hi;
            if (p0.x > p1.x) {
                std::swap(p0, p1);
            }
            width = hi - lo;
            return true;
        }
        return false;
    };
    for (size_t i = 1; i < tOn.size(); ++i) {
        if (tOn[i] - prev > cfg.span_gap_tol) {
            if (tryPick(segLo, prev)) {
                return true;
            }
            segLo = tOn[i];
        }
        prev = tOn[i];
    }
    return tryPick(segLo, prev);
}

// 候选边记录（对应 Python records 字典项）
struct EdgeRecord {
    std::vector<cv::Point2d> seed;
    cv::Point2d p0, p1;      // 左/右端点（按 x 排序）
    double width   = 0.0;
    double yCenter = 0.0;
};

// 同边去重：中心高度差 <= 2·line_dist_tol 且 x 区间交叠 >= 30% 较短边时，
// 种子点并集重拟合、重新延展（斜率自适应），迭代至无变化。
void DedupeSameEdge(std::vector<EdgeRecord>& records,
                    const std::vector<cv::Point2d>& pts,
                    const common::MeasureConfig& cfg) {
    bool changed = true;
    while (changed) {
        changed = false;
        for (size_t i = 0; i < records.size() && !changed; ++i) {
            for (size_t j = i + 1; j < records.size(); ++j) {
                const EdgeRecord& a = records[i];
                const EdgeRecord& b = records[j];
                if (std::fabs(a.yCenter - b.yCenter) > 2.0 * cfg.line_dist_tol) {
                    continue;
                }
                const double lo = std::max(a.p0.x, b.p0.x);
                const double hi = std::min(a.p1.x, b.p1.x);
                const double shorter = std::min(a.width, b.width);
                if (hi <= lo || (hi - lo) < 0.3 * shorter) {
                    continue;
                }
                std::vector<cv::Point2d> seed = a.seed;
                seed.insert(seed.end(), b.seed.begin(), b.seed.end());
                LineResult line;
                std::string err;
                if (!RansacLine(seed, cfg.ransac_threshold, line, err)) {
                    continue;
                }
                cv::Point2d p0, p1;
                double width = 0.0;
                if (!SpanAlongLine(pts, seed, line, cfg, p0, p1, width)) {
                    continue;
                }
                EdgeRecord mergedRec;
                mergedRec.seed = std::move(seed);
                mergedRec.p0 = p0;
                mergedRec.p1 = p1;
                mergedRec.width = width;
                mergedRec.yCenter = (p0.y + p1.y) / 2.0;
                records[i] = std::move(mergedRec);
                records.erase(records.begin() + j);
                changed = true;
                break;
            }
        }
    }
}

}  // namespace

bool FitLineLeastSquares(const std::vector<cv::Point2d>& points, LineResult& out,
                         std::string& errMsg) {
    if (points.size() < 2) {
        errMsg = "直线拟合点数不足";
        common::LogMsg(common::LERROR, errMsg);
        return false;
    }
    std::vector<cv::Point2f> pts32;
    pts32.reserve(points.size());
    for (const cv::Point2d& p : points) {
        pts32.emplace_back((float)p.x, (float)p.y);
    }
    cv::Vec4f line;
    cv::fitLine(pts32, line, cv::DIST_L2, 0, 0.01, 0.01);
    const double norm = std::hypot(line[0], line[1]);
    if (norm < 1e-12) {
        errMsg = "直线拟合失败：方向向量退化";
        common::LogMsg(common::LERROR, errMsg);
        return false;
    }
    out.direction = cv::Point2d(line[0] / norm, line[1] / norm);
    out.point = cv::Point2d(line[2], line[3]);
    out.inlierRatio = 1.0;
    out.numPoints = (int)points.size();
    return true;
}

bool RansacLine(const std::vector<cv::Point2d>& points, double threshold,
                LineResult& out, std::string& errMsg) {
    const int n = (int)points.size();
    if (n < 5) {
        errMsg = Fmt("直线拟合点数不足: %d", n);
        common::LogMsg(common::LERROR, errMsg);
        return false;
    }
    cv::RNG rng(42);  // 固定随机源，保证可复现
    const int nIter = std::min(200, std::max(50, n * 2));
    std::vector<char> bestInliers;
    int bestCount = -1;
    std::vector<char> inliers(n);
    for (int iter = 0; iter < nIter; ++iter) {
        const int i = rng.uniform(0, n);
        const int j = rng.uniform(0, n);
        if (i == j) {
            continue;
        }
        cv::Point2d d = points[j] - points[i];
        const double norm = std::hypot(d.x, d.y);
        if (norm < 1e-9) {
            continue;
        }
        d = d * (1.0 / norm);
        const cv::Point2d normal(-d.y, d.x);
        int count = 0;
        for (int k = 0; k < n; ++k) {
            const double dist = std::fabs((points[k] - points[i]).dot(normal));
            inliers[k] = (dist <= threshold) ? 1 : 0;
            count += inliers[k];
        }
        if (count > bestCount) {
            bestCount = count;
            bestInliers = inliers;
        }
    }
    if (bestInliers.empty()) {
        errMsg = "RANSAC 直线拟合失败：无有效采样";
        common::LogMsg(common::LERROR, errMsg);
        return false;
    }
    std::vector<cv::Point2d> inlierPts;
    inlierPts.reserve((size_t)bestCount);
    for (int k = 0; k < n; ++k) {
        if (bestInliers[k]) {
            inlierPts.push_back(points[k]);
        }
    }
    std::string fitErr;
    if (!FitLineLeastSquares(inlierPts, out, fitErr)) {
        errMsg = fitErr;
        return false;
    }
    out.inlierRatio = (double)bestCount / n;
    out.numPoints = n;
    common::LogMsg(common::LDEBUG,
                   Fmt("RANSAC 拟合: %d/%d 内点 (%.1f%%)", bestCount, n,
                       out.inlierRatio * 100.0));
    return true;
}

bool MeasureWidthHeight(const cv::Mat& rotAligned, const cv::Mat& rotMask,
                        const common::MeasureConfig& cfg,
                        const common::RefineConfig& refineCfg,
                        WidthResult& width, HeightResult& height, std::string& errMsg) {
    std::vector<cv::Point> contour;
    if (!ExtractOuterContour(rotMask, contour, errMsg)) {
        return false;
    }
    std::vector<cv::Point2d> pts;
    pts.reserve(contour.size());
    for (const cv::Point& p : contour) {
        pts.emplace_back((double)p.x, (double)p.y);
    }

    std::vector<cv::Point2d> ptsL, ptsR, ptsT, ptsB;
    const double xLeft =
        SemanticExtreme(rotAligned, pts, Side::Left, cfg, refineCfg, ptsL);
    const double xRight =
        SemanticExtreme(rotAligned, pts, Side::Right, cfg, refineCfg, ptsR);
    const double yTop =
        SemanticExtreme(rotAligned, pts, Side::Top, cfg, refineCfg, ptsT);
    const double yBottom =
        SemanticExtreme(rotAligned, pts, Side::Bottom, cfg, refineCfg, ptsB);

    width.widthPx = xRight - xLeft;
    height.heightPx = yBottom - yTop;
    common::LogMsg(common::LINFO,
                   Fmt("宽度 %.2f px (极值线 %.1f~%.1f), 高度 %.2f px (极值线 %.1f~%.1f)",
                       width.widthPx, xLeft, xRight, height.heightPx, yTop, yBottom));
    // 四线互为端点边界（payload.py 约定）：竖线纵向跨 [yTop, yBottom]，
    // 横线横向跨 [xLeft, xRight]，围成闭合测量矩形
    width.leftLine.start  = {xLeft, yTop};
    width.leftLine.end    = {xLeft, yBottom};
    width.rightLine.start = {xRight, yTop};
    width.rightLine.end   = {xRight, yBottom};
    width.ok = true;
    height.topLine.start    = {xLeft, yTop};
    height.topLine.end      = {xRight, yTop};
    height.bottomLine.start = {xLeft, yBottom};
    height.bottomLine.end   = {xRight, yBottom};
    height.ok = true;
    return true;
}

bool MeasureRankedEdges(const std::vector<cv::Point>& rotContour,
                        const common::MeasureConfig& cfg,
                        std::vector<HorizontalEdge>& candidates, std::string& errMsg) {
    candidates.clear();
    std::vector<cv::Point2d> pts;
    pts.reserve(rotContour.size());
    for (const cv::Point& p : rotContour) {
        pts.emplace_back((double)p.x, (double)p.y);
    }
    if (pts.empty()) {
        errMsg = "水平边排名失败：轮廓为空";
        common::LogMsg(common::LERROR, errMsg);
        return false;
    }
    // 上半部分硬分界：产品自身高度的 ranked_region_ratio 处（与摆放位置解耦）
    double topY = pts[0].y, bottomY = pts[0].y;
    for (const cv::Point2d& p : pts) {
        topY = std::min(topY, p.y);
        bottomY = std::max(bottomY, p.y);
    }
    const double regionYMax = topY + cfg.ranked_region_ratio * (bottomY - topY);

    // 1) 种子 run -> 各自 RANSAC 拟合 -> 按用户规则（垂直距离阈值 + 连续跨度）延展
    const std::vector<std::vector<int>> runs =
        MergeRuns(HorizontalRuns(pts, cfg.angle_tol_deg, cfg.tangent_span), pts, cfg);
    std::vector<EdgeRecord> records;
    for (const std::vector<int>& run : runs) {
        if (run.size() < 5) {
            continue;
        }
        std::vector<cv::Point2d> seed;
        seed.reserve(run.size());
        for (const int idx : run) {
            seed.push_back(pts[idx]);
        }
        LineResult line;
        std::string err;
        if (!RansacLine(seed, cfg.ransac_threshold, line, err)) {
            continue;
        }
        cv::Point2d p0, p1;
        double width = 0.0;
        if (!SpanAlongLine(pts, seed, line, cfg, p0, p1, width)) {
            continue;
        }
        EdgeRecord rec;
        rec.seed = std::move(seed);
        rec.p0 = p0;
        rec.p1 = p1;
        rec.width = width;
        rec.yCenter = (p0.y + p1.y) / 2.0;
        records.push_back(std::move(rec));
    }

    // 2) 同边去重（长边起伏分成多段种子时，延展会生成近重合的多条候选）
    DedupeSameEdge(records, pts, cfg);

    // 3) 区域过滤（硬性范围）+ 生成候选（宽度阈值只判定、不淘汰）
    for (const EdgeRecord& rec : records) {
        if (rec.yCenter > regionYMax) {
            continue;
        }
        HorizontalEdge e;
        e.start = {rec.p0.x, rec.p0.y};
        e.end = {rec.p1.x, rec.p1.y};
        e.lengthPx = rec.width;
        e.satisfied = (rec.width >= cfg.edge_width_threshold);
        candidates.push_back(e);
    }
    std::sort(candidates.begin(), candidates.end(),
              [](const HorizontalEdge& a, const HorizontalEdge& b) {
                  const double ya = (a.start.y + a.end.y) * 0.5;
                  const double yb = (b.start.y + b.end.y) * 0.5;
                  if (ya != yb) {
                      return ya < yb;
                  }
                  return a.start.x < b.start.x;
              });
    common::LogMsg(common::LINFO,
                   Fmt("水平边排名: 上半部分候选 %d 条（分界 y=%.0f, 阈值 %.0f px）",
                       (int)candidates.size(), regionYMax, cfg.edge_width_threshold));

    // 4) 排名槽位（最高/次高/最低）仅作日志，不进入对外接口（与 payload.py 一致）：
    //    1 条 → 最高=最低；2 条 → 最高/最低；>=3 条 → 最高/次高/最低
    auto logSlot = [](const char* rank, const HorizontalEdge& e) {
        common::LogMsg(common::LINFO,
                       Fmt("  %s边: y=%.1f, x %.1f~%.1f, %s", rank,
                           (e.start.y + e.end.y) * 0.5, e.start.x, e.end.x,
                           e.satisfied
                               ? Fmt("%.1fpx", e.lengthPx).c_str()
                               : Fmt("不满足（实测 %.1fpx）", e.lengthPx).c_str()));
    };
    if (candidates.size() == 1) {
        logSlot("最高", candidates[0]);
        logSlot("最低", candidates[0]);
    } else if (candidates.size() == 2) {
        logSlot("最高", candidates[0]);
        logSlot("最低", candidates[1]);
    } else if (candidates.size() >= 3) {
        logSlot("最高", candidates[0]);
        logSlot("次高", candidates[1]);
        logSlot("最低", candidates.back());
    }
    return true;
}

}  // namespace cam
