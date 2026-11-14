#include "scanlio/include/backend/backend_pipeline.h"

#include "scanlio/include/backend/loop_closure.h"
#include "scanlio/include/backend/pose_graph.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <utility>

namespace sad {

namespace {

double WrapToPi(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a <= -M_PI) a += 2.0 * M_PI;
    return a;
}

// 两个修正量之间按 α 插值：位置线性，角度折短弧后加权。
SE2 InterpCorrection(const SE2& a, const SE2& b, double alpha) {
    const Vec2d t = a.translation() * (1.0 - alpha) + b.translation() * alpha;
    const double dth = WrapToPi(b.so2().log() - a.so2().log());
    return SE2(SO2(WrapToPi(a.so2().log() + dth * alpha)), t);
}

}  // namespace

BackendOptions MakeBackendOptions(const LoopConfig& cfg, const NDT2D::Options& ndt) {
    BackendOptions out;
    out.loop.max_distance = cfg.max_distance;
    out.loop.min_index_gap = cfg.min_index_gap;
    out.loop.max_heading_diff = cfg.max_heading_diff_deg * M_PI / 180.0;
    out.loop.rings = cfg.rings;
    out.loop.sectors = cfg.sectors;
    out.loop.max_range = cfg.max_range;
    out.loop.min_similarity = cfg.min_similarity;
    out.loop.max_candidates = cfg.max_candidates;
    out.loop.submap_half_window = cfg.submap_half_window;
    out.loop.ndt_opts = ndt;

    out.graph.max_iterations = cfg.max_iterations;
    out.graph.function_tolerance = cfg.function_tolerance;
    out.graph.parameter_tolerance = cfg.parameter_tolerance;
    out.graph.enable_robust = cfg.enable_robust;
    out.graph.huber_delta = cfg.huber_delta;
    return out;
}

size_t ApplyFrameCorrections(std::vector<std::shared_ptr<Frame>>& frames,
                             const std::map<size_t, SE2>& kf_old,
                             const std::map<size_t, SE2>& kf_new) {
    if (frames.empty() || kf_old.empty() || kf_old.size() != kf_new.size()) {
        return 0;
    }

    // 按 keyframe_id 升序列出 (id, correction)。
    struct Item {
        size_t id;
        SE2 corr;
    };
    std::vector<Item> items;
    items.reserve(kf_old.size());
    for (const auto& kv : kf_old) {
        const auto it = kf_new.find(kv.first);
        if (it == kf_new.end()) continue;
        items.push_back({kv.first, it->second * kv.second.inverse()});
    }
    if (items.empty()) return 0;

    size_t moved = 0;
    for (const auto& f : frames) {
        if (f == nullptr) continue;
        // 关键帧的 pose_ 已由 Frontend::ApplyOptimizedPoses 改写，
        // 且 all_frames 与 Map::keyframes() 可能持有同一个 shared_ptr。
        // 跳过关键帧，避免对同一对象重复施加修正。
        if (kf_old.find(f->keyframe_id_) != kf_old.end()) continue;

        // 找到 f 所在区间：最后一个 id <= f->id_ 的关键帧，以及其后一个。
        size_t lo = 0;
        bool has_lo = false;
        for (size_t k = 0; k < items.size(); ++k) {
            if (items[k].id <= f->id_) {
                lo = k;
                has_lo = true;
            } else {
                break;
            }
        }

        SE2 corr;
        if (!has_lo) {
            corr = items.front().corr;              // 首个关键帧之前
        } else if (lo + 1 >= items.size()) {
            corr = items.back().corr;               // 末个关键帧之后
        } else {
            const size_t a = items[lo].id;
            const size_t b = items[lo + 1].id;
            double alpha = 0.0;
            if (b > a && f->id_ >= a) {
                alpha = static_cast<double>(f->id_ - a) / static_cast<double>(b - a);
            }
            corr = InterpCorrection(items[lo].corr, items[lo + 1].corr,
                                    std::max(0.0, std::min(1.0, alpha)));
        }
        f->pose_ = corr * f->pose_;
        ++moved;
    }
    return moved;
}

bool RunBackend(Frontend& frontend,
                std::vector<std::shared_ptr<Frame>>& all_frames,
                const std::shared_ptr<Frame>& current,
                const LoopConfig& cfg,
                const NDT2D::Options& ndt,
                BackendResult* result) {
    if (result) *result = BackendResult();
    if (result) result->attempted = true;

    if (!cfg.enable) {
        if (result) result->reason = "loop.enable 为 false";
        return false;
    }
    if (current == nullptr || all_frames.empty()) {
        if (result) result->reason = "current 为空或全部帧序列为空";
        return false;
    }

    const auto& kf = frontend.GetMap().keyframes();
    if (kf.size() < 2) {
        if (result) result->reason = "关键帧少于 2 个";
        return false;
    }

    // ---- 1. 建位姿图：节点按 keyframe_id 升序，里程计边连接相邻关键帧 ----
    const BackendOptions opts = MakeBackendOptions(cfg, ndt);
    PoseGraph graph(opts.graph);

    std::vector<size_t> ids;
    std::vector<std::shared_ptr<Frame>> history;
    std::map<size_t, SE2> kf_old;
    ids.reserve(kf.size());
    history.reserve(kf.size());
    for (const auto& kv : kf) {
        ids.push_back(kv.first);
        history.push_back(kv.second);
        kf_old[kv.first] = kv.second->pose_;
        graph.AddNode(kv.second->pose_);
    }
    for (size_t k = 0; k + 1 < ids.size(); ++k) {
        const SE2 rel = history[k]->pose_.inverse() * history[k + 1]->pose_;
        graph.AddEdge(static_cast<int>(k), static_cast<int>(k + 1), rel,
                      cfg.odom_omega);
    }

    // 当前帧若已是最后一个关键帧，直接复用该节点，避免同一帧出现两个节点
    // 而得到两个不同的优化结果。
    const bool current_is_keyframe = (ids.back() == current->keyframe_id_);
    int current_index = 0;
    if (current_is_keyframe) {
        current_index = static_cast<int>(ids.size() - 1);
    } else {
        current_index = graph.AddNode(current->pose_);
        const SE2 rel = history.back()->pose_.inverse() * current->pose_;
        graph.AddEdge(static_cast<int>(ids.size() - 1), current_index, rel,
                      cfg.odom_omega);
    }

    if (result) {
        result->num_nodes = static_cast<int>(graph.num_nodes());
        result->num_odom_edges = static_cast<int>(graph.num_edges());
    }

    // ---- 2. 回环检测，产出回环边 ----
    LoopClosure detector(opts.loop);
    LoopClosure::Diagnostics diag;
    std::string err;
    const std::vector<LoopClosure::Edge> loop_edges =
        detector.Detect(history, current, current->pose_, &diag, &err);
    if (!err.empty()) {
        if (result) result->reason = "回环检测失败：" + err;
        return false;
    }
    int accepted = 0;
    for (const auto& e : loop_edges) {
        const auto it = std::find(ids.begin(), ids.end(), e.keyframe_id);
        if (it == ids.end()) continue;
        const int j = static_cast<int>(it - ids.begin());
        if (graph.AddEdge(j, current_index, e.z, cfg.loop_omega)) ++accepted;
    }
    if (result) result->num_loop_edges = accepted;
    if (accepted == 0) {
        if (result) {
            result->reason = loop_edges.empty() ? "未检出回环边"
                                               : "检出的回环边端点不在关键帧库中";
        }
        return false;
    }

    // ---- 3. 求解 ----
    if (!graph.Optimize()) {
        if (result) result->reason = "位姿图求解未收敛";
        return false;
    }

    // ---- 4. 组装优化后关键帧位姿，记录改动量 ----
    std::map<size_t, SE2> kf_new;
    double max_shift = 0.0;
    for (size_t k = 0; k < ids.size(); ++k) {
        kf_new[ids[k]] = graph.pose(k);
        max_shift = std::max(max_shift,
                             (graph.pose(k).translation() -
                              kf_old[ids[k]].translation()).norm());
    }
    if (result) result->max_pose_shift = max_shift;

    // ---- 5. 落地：P2 已交付的接口 ----
    const SE2 optimized_current = graph.pose(current_index);
    frontend.ApplyOptimizedPoses(kf_new, optimized_current);

    // ---- 6. 搬移非关键帧并重建栅格 ----
    // ApplyFrameCorrections 内部跳过关键帧，避免对与 Map 共享的 shared_ptr
    // 重复施加修正。RebuildOccupancy 需要全部帧，故传 all_frames 全量。
    ApplyFrameCorrections(all_frames, kf_old, kf_new);
    // current 若不是关键帧，上一步会按插值搬移它；这里覆盖为图优化的精确结果，
    // 保证栅格重建、当前帧 TF 与 Path 用的是同一个位姿。
    current->pose_ = optimized_current;
    frontend.GetMap().RebuildOccupancy(all_frames);

    if (result) {
        result->optimized = true;
        result->reason = "完成";
    }
    return true;
}

}  // namespace sad
