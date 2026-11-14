#ifndef SCANLIO_BACKEND_LOOP_CLOSURE_H_
#define SCANLIO_BACKEND_LOOP_CLOSURE_H_

// 回环检测：从历史关键帧中召回候选，配准后产出回环边。
//
// 理论依据见 .codebuddy/认知管理文档总目录/回环检测/ 的大纲，
// 以及 doc/复习资料/位姿图优化/01 §1.3（回环边是独立观测）。
//
// 职责边界：
//   本模块只产出「边」，不决定边是否被接受、不修改位姿、不重建地图。
//   产出后由调用方 AddEdge 进 PoseGraph，Optimize 后调用
//   Frontend::ApplyOptimizedPoses 与 Map::RebuildOccupancy（P2 已交付）。
//
// 本次实现的范围（按已确认块裁剪）：
//   已实现：几何门控 / 描述子 / 余弦相似度 / 列循环移位 / 局部子图配准 / 边值
//   未实现：群上环闭合差（块 6 搁置）、假回环边剔除（块 7 搁置）
//   因此本模块产出的边全部交给稳健核处理，不做删边判断。
//   调用方若需要剔除，须在 AddEdge 之前自行按马氏残差判定。
//
// 阈值说明：
//   门控与描述子的三个参数（距离、索引间隔、朝向差）是推断的设计值，
//   未在实机数据上验证；改动它们不改变本模块的结构。

#include "scanlio/common/eigen_types.h"
#include "scanlio/include/frame.h"
#include "scanlio/src/ndt_2d.hpp"

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace sad {

class LoopClosure {
 public:
  struct Options {
    // 块 1：几何门控
    double max_distance = 3.0;                          // 米
    int min_index_gap = 20;                             // 关键帧编号之差
    double max_heading_diff = 30.0 * M_PI / 180.0;      // 弧度

    // 块 2：描述子。环按距离等宽切分，扇区按角度等宽切分。
    int rings = 20;
    int sectors = 60;
    double max_range = 20.0;      // 米；超过此距离的点不进入描述子

    // 块 3/4：相似度阈值与移位搜索
    double min_similarity = 0.0;  // 低于此值的候选不送配准
    int max_candidates = 3;       // 送配准的候选上限

    // 块 5：局部子图配准
    int submap_half_window = 2;   // 候选前后各取几帧
    NDT2D::Options ndt_opts;      // 配准参数，与主地图一致
  };

  struct Candidate {
    size_t frame_index = 0;   // 在传入的 keyframes 序列中的下标
    size_t keyframe_id = 0;   // Frame::keyframe_id_
    double distance = 0.0;
    double heading_diff = 0.0;
    double similarity = 0.0;
    int shift = 0;            // 取得最大相似度时的列移位数
  };

  struct Edge {
    size_t keyframe_id = 0;   // 历史帧 id，即 Z_ji 的 j 端
    SE2 z = SE2();            // Z_ji = X_j⁻¹ ∘ T，表达在 j 的车体系
    int effective_pts = 0;    // 配准有效点数
    double similarity = 0.0;
    int shift = 0;
  };

  struct Diagnostics {
    int num_history = 0;      // 参与检索的历史帧数
    int num_gated = 0;        // 通过几何门控的帧数
    int num_described = 0;    // 相似度达阈值的帧数
    int num_verified = 0;     // 配准成功并产出边的帧数
  };

  LoopClosure() = default;
  explicit LoopClosure(const Options& options) : options_(options) {}

  const Options& options() const { return options_; }
  void SetOptions(const Options& options) { options_ = options; }

  // 把一帧点云压成 rings × sectors 的描述子，行优先平铺。
  // 每格取落入该格的点的最小距离；无点的格为 0。
  std::vector<double> Describe(const std::vector<Vec2d>& pts) const;

  // 两张描述子按列循环移位，取最大余弦相似度。
  // best_shift 返回取得最大值时的移位数，单位「扇区」。
  double BestSimilarity(const std::vector<double>& d_cur,
                        const std::vector<double>& d_hist,
                        int* best_shift) const;

  // 完整流程：门控 → 描述子 → 相似度 → 子图配准 → 边。
  //   history：按 keyframe_id 升序排列的历史关键帧
  //   current：当前帧，其 pts_ 为去畸变后的雷达系点云
  //   predicted_pose：当前帧位姿预测值，作为配准初值
  // 返回产出的边（按相似度降序），失败时为空。
  std::vector<Edge> Detect(const std::vector<std::shared_ptr<Frame>>& history,
                           const std::shared_ptr<Frame>& current,
                           const SE2& predicted_pose,
                           Diagnostics* diag,
                           std::string* err) const;

  // 只读的中间结果，供单模块验证核对门控与相似度。
  // 只做几何门控与相似度排序，不做 min_similarity 过滤、不截取候选数。
  std::vector<Candidate> GateAndRank(
      const std::vector<std::shared_ptr<Frame>>& history,
      const std::shared_ptr<Frame>& current,
      const SE2& predicted_pose) const;

 private:
  Options options_;
};

}  // namespace sad

#endif  // SCANLIO_BACKEND_LOOP_CLOSURE_H_
