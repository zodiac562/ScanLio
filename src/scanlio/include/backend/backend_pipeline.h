#ifndef SCANLIO_BACKEND_BACKEND_PIPELINE_H_
#define SCANLIO_BACKEND_BACKEND_PIPELINE_H_

// P4：把「回环检测 → 位姿图求解 → 位姿回写 → 地图与轨迹重建」串成一次可测的动作。
//
// 为什么单独成模块：
//   节点层只应做「话题 ↔ 结构体」的转换与调用，不应含算法逻辑。把接线逻辑
//   放进本模块后，可在离线单模块验证里核对整条链路，整链路实机验证留给 P5。
//
// 依赖方向：本模块属于 scanlio_backend，可同时使用 scanlio_core 的 Frontend/Map
// 与 backend 的 PoseGraph/LoopClosure。Config 不依赖本模块（见 config.h 注释）。

#include "scanlio/include/backend/loop_closure.h"
#include "scanlio/include/backend/pose_graph.h"
#include "scanlio/include/config.h"
#include "scanlio/include/frame.h"
#include "scanlio/include/frontend.h"
#include "scanlio/include/map.h"

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace sad {

struct BackendResult {
    bool attempted = false;       // 是否执行了检测
    bool optimized = false;       // 是否成功求解并落地
    int num_nodes = 0;
    int num_odom_edges = 0;
    int num_loop_edges = 0;
    double max_pose_shift = 0.0;  // 优化前后关键帧最大平移变化，米
    std::string reason;           // 未优化时的原因，便于实机排查
};

// 把 LoopConfig 转成两个后端子系统的 Options。单独暴露，便于单模块验证
// 检查「度→弧度」「omega 分量顺序」这类容易写错的转换。
struct BackendOptions {
    LoopClosure::Options loop;
    PoseGraph::Options graph;
};

BackendOptions MakeBackendOptions(const LoopConfig& cfg, const NDT2D::Options& ndt);

// 按关键帧优化前后的位姿差，把全部帧（含非关键帧）的位姿整体搬移。
//
// 为什么需要：Map 只存关键帧，位姿图也只解关键帧；但占据栅格原本每帧都写，
// 重建时需要全部帧在新世界系下的位姿。非关键帧没有独立的优化结果，用相邻
// 关键帧的修正量按帧序线性插值得到。这是通用机制，不针对特定数据集。
//
//   correction_k = X_k_new ∘ X_k_old⁻¹   （世界系左乘）
//   位于关键帧 a、b 之间的帧 f，按 α 插值 correction_a / correction_b：
//     平移线性，角度先折短弧再加权（与 UndistortScan 的角度插值同一约定）
//
// frames 按帧序排列；kf_old / kf_new 的键为 keyframe_id，两者键集合须相同。
// 关键帧的位姿已由 ApplyOptimizedPoses 改写，且 frames 可能与 Map 共享同一个
// shared_ptr，故本函数跳过 keyframe_id 出现在 kf_old 中的帧，只对非关键帧插值。
// 返回被搬移的帧数。区间外的帧使用最近的修正量。
size_t ApplyFrameCorrections(std::vector<std::shared_ptr<Frame>>& frames,
                             const std::map<size_t, SE2>& kf_old,
                             const std::map<size_t, SE2>& kf_new);

// 完整一次后端动作。all_frames 为节点保存的全部成功配准帧（按帧序），
// current 为当前帧。函数内部会：
//   1. 由 Map::keyframes() 建位姿图节点与里程计边
//   2. LoopClosure::Detect 产出回环边
//   3. 无回环边则不优化（避免无意义重解）
//   4. PoseGraph::Optimize
//   5. Frontend::ApplyOptimizedPoses 回写位姿并重建 NDT
//   6. ApplyFrameCorrections 搬移全部帧，再 Map::RebuildOccupancy
//
// 返回值同 BackendResult::optimized。失败时地图与前端状态不被改动
// （检测与求解都在临时对象上完成，只在成功后才落地）。
bool RunBackend(Frontend& frontend,
                std::vector<std::shared_ptr<Frame>>& all_frames,
                const std::shared_ptr<Frame>& current,
                const LoopConfig& cfg,
                const NDT2D::Options& ndt,
                BackendResult* result);

}  // namespace sad

#endif  // SCANLIO_BACKEND_BACKEND_PIPELINE_H_
