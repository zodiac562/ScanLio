#include "scanlio/include/backend/loop_closure.h"

#include <algorithm>
#include <cmath>

namespace sad {

namespace {

// 角度折回 (-pi, pi]
double WrapToPi(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a <= -M_PI) a += 2.0 * M_PI;
    return a;
}

}  // namespace

std::vector<double> LoopClosure::Describe(const std::vector<Vec2d>& pts) const {
    const int R = options_.rings;
    const int S = options_.sectors;
    std::vector<double> desc(static_cast<size_t>(R) * S, 0.0);
    // 记录该格是否已有值，避免「最小距离」与「无点记 0」混淆。
    std::vector<bool> filled(static_cast<size_t>(R) * S, false);

    const double ring_width = options_.max_range / static_cast<double>(R);
    const double sector_width = 2.0 * M_PI / static_cast<double>(S);

    for (const auto& p : pts) {
        const double r = std::sqrt(p.x() * p.x() + p.y() * p.y());
        if (!(r < options_.max_range)) {
            continue;   // 超出量程或为 nan 的点不进入描述子
        }
        double phi = std::atan2(p.y(), p.x());
        if (phi < 0.0) {
            phi += 2.0 * M_PI;
        }

        int a = static_cast<int>(std::floor(r / ring_width));
        int b = static_cast<int>(std::floor(phi / sector_width));
        if (a >= R) a = R - 1;
        if (b >= S) b = S - 1;

        const size_t k = static_cast<size_t>(a) * S + static_cast<size_t>(b);
        if (!filled[k] || r < desc[k]) {
            desc[k] = r;
            filled[k] = true;
        }
    }
    return desc;
}

double LoopClosure::BestSimilarity(const std::vector<double>& d_cur,
                                   const std::vector<double>& d_hist,
                                   int* best_shift) const {
    const int R = options_.rings;
    const int S = options_.sectors;
    const size_t M = static_cast<size_t>(R) * S;
    if (d_cur.size() != M || d_hist.size() != M) {
        if (best_shift) *best_shift = 0;
        return 0.0;
    }
    if (best_shift) *best_shift = 0;

    // 两个范数的平方。描述子元素非负，故范数之积恒为正时可安全相除。
    double norm_cur_sq = 0.0;
    double norm_hist_sq = 0.0;
    for (size_t k = 0; k < M; ++k) {
        norm_cur_sq += d_cur[k] * d_cur[k];
        norm_hist_sq += d_hist[k] * d_hist[k];
    }
    if (norm_cur_sq <= 0.0 || norm_hist_sq <= 0.0) {
        return 0.0;
    }
    const double denom = std::sqrt(norm_cur_sq) * std::sqrt(norm_hist_sq);

    // 列循环移位：把历史描述子的每一行循环左移 shift 个扇区，
    // 再与当前描述子逐项相乘求和。取 S 次移位里的最大值。
    double best = 0.0;
    int best_s = 0;
    for (int s = 0; s < S; ++s) {
        double inner = 0.0;
        for (int a = 0; a < R; ++a) {
            const size_t row = static_cast<size_t>(a) * S;
            for (int b = 0; b < S; ++b) {
                const int shifted = (b + s) % S;
                inner += d_cur[row + b] * d_hist[row + shifted];
            }
        }
        const double c = inner / denom;
        if (c > best) {
            best = c;
            best_s = s;
        }
    }
    if (best_shift) *best_shift = best_s;
    return best;
}

std::vector<LoopClosure::Candidate> LoopClosure::GateAndRank(
    const std::vector<std::shared_ptr<Frame>>& history,
    const std::shared_ptr<Frame>& current,
    const SE2& predicted_pose) const {
    std::vector<Candidate> out;
    if (current == nullptr || history.empty()) {
        return out;
    }
    const std::vector<double> d_cur = Describe(current->pts_);

    for (size_t k = 0; k < history.size(); ++k) {
        const auto& f = history[k];
        if (f == nullptr) continue;

        // 条件二：索引间隔。只找足够早的历史帧。
        if (current->keyframe_id_ <= f->keyframe_id_) continue;
        const size_t gap = current->keyframe_id_ - f->keyframe_id_;
        if (gap < static_cast<size_t>(options_.min_index_gap)) continue;

        // 条件一：位置接近
        const double dx = predicted_pose.translation().x() - f->pose_.translation().x();
        const double dy = predicted_pose.translation().y() - f->pose_.translation().y();
        const double d2 = dx * dx + dy * dy;
        if (d2 > options_.max_distance * options_.max_distance) continue;

        // 条件三：朝向接近
        const double dh = WrapToPi(predicted_pose.so2().log() - f->pose_.so2().log());
        if (std::fabs(dh) > options_.max_heading_diff) continue;

        Candidate c;
        c.frame_index = k;
        c.keyframe_id = f->keyframe_id_;
        c.distance = std::sqrt(d2);
        c.heading_diff = dh;

        const std::vector<double> d_hist = Describe(f->pts_);
        c.similarity = BestSimilarity(d_cur, d_hist, &c.shift);
        out.push_back(c);
    }

    // 按相似度降序；同值时按索引升序，保证输出可复现。
    std::sort(out.begin(), out.end(),
              [](const Candidate& a, const Candidate& b) {
                  if (a.similarity != b.similarity) return a.similarity > b.similarity;
                  return a.frame_index < b.frame_index;
              });
    return out;
}

std::vector<LoopClosure::Edge> LoopClosure::Detect(
    const std::vector<std::shared_ptr<Frame>>& history,
    const std::shared_ptr<Frame>& current,
    const SE2& predicted_pose,
    Diagnostics* diag,
    std::string* err) const {
    if (diag) *diag = Diagnostics();
    if (err) err->clear();

    if (options_.rings < 1 || options_.sectors < 1 || options_.max_range <= 0.0 ||
        options_.max_distance <= 0.0 || options_.max_candidates < 1 ||
        options_.submap_half_window < 0) {
        if (err) *err = "LoopClosure::Options 参数非法";
        return {};
    }
    if (current == nullptr) {
        if (err) *err = "current 为空";
        return {};
    }

    // 单次调用：门控 + 相似度排序。诊断的三个计数都从同一次结果导出，
    // 避免重复计算导致口径不一致。
    std::vector<Candidate> gated = GateAndRank(history, current, predicted_pose);
    if (diag) {
        diag->num_history = static_cast<int>(history.size());
        diag->num_gated = static_cast<int>(gated.size());
    }

    std::vector<Candidate> candidates;
    for (const Candidate& c : gated) {
        if (c.similarity >= options_.min_similarity) {
            candidates.push_back(c);
        }
    }
    if (diag) diag->num_described = static_cast<int>(candidates.size());
    if (static_cast<int>(candidates.size()) > options_.max_candidates) {
        candidates.resize(static_cast<size_t>(options_.max_candidates));
    }

    std::vector<Edge> edges;
    for (const Candidate& c : candidates) {
        // 局部子图：候选帧前后各 h 帧，按其写入时的世界系位姿投点。
        NDT2D submap(options_.ndt_opts);
        const int h = options_.submap_half_window;
        const long lo = std::max<long>(0, static_cast<long>(c.frame_index) - h);
        const long hi = std::min<long>(static_cast<long>(history.size()) - 1,
                                       static_cast<long>(c.frame_index) + h);
        for (long q = lo; q <= hi; ++q) {
            if (history[static_cast<size_t>(q)] == nullptr) continue;
            submap.add_scan(history[static_cast<size_t>(q)]->pose_,
                            history[static_cast<size_t>(q)]->pts_);
        }

        // 配准初值取当前帧预测位姿；收敛值 T 是当前帧的世界系位姿。
        SE2 T = predicted_pose;
        const int effective_pts = submap.match(T, current->pts_);
        if (effective_pts < options_.ndt_opts.min_pts_num_) {
            continue;   // 有效点数不足，本帧不产出回环边
        }

        // 边值是「j 到当前帧」的相对运动：Z_ji = X_j⁻¹ ∘ T。
        // 用 SE2 复合取 translation() 与 so2().log()，
        // 不对整个位姿取 log()——其平移分量是加权平移 V⁻¹t，不是矩阵中的 t。
        Edge e;
        e.keyframe_id = c.keyframe_id;
        e.z = history[c.frame_index]->pose_.inverse() * T;
        e.effective_pts = effective_pts;
        e.similarity = c.similarity;
        e.shift = c.shift;
        edges.push_back(e);
    }

    if (diag) diag->num_verified = static_cast<int>(edges.size());
    return edges;
}

}  // namespace sad
