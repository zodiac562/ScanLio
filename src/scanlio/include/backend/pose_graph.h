#ifndef SCANLIO_BACKEND_POSE_GRAPH_H_
#define SCANLIO_BACKEND_POSE_GRAPH_H_

// 二维位姿图后端。
//
// 理论依据与逐条推导见：
//   slam_ws/doc/复习资料/位姿图优化/01-位姿图与代价函数.md
//   slam_ws/doc/复习资料/位姿图优化/02-求解方法与数值实现.md
//   slam_ws/doc/复习资料/位姿图优化/03-稳健核与代码形态.md
//
// 职责边界：
//   本类只负责「给定节点与边，求一组节点位姿使总代价最小」。
//   它不产生边、不判断回环、不重建地图。边的来源（里程计边 / 回环边）
//   对这里不可见 —— 两者的残差公式与装配流程完全相同，差别只在数值来源
//   与权重（见 01 §1.3）。
//
// 数值实现：
//   迭代、雅可比、稀疏分解、阻尼调整全部由 Ceres 完成（Ceres 为 PRIVATE 依赖，
//   本头文件不暴露任何 ceres 类型）。本类只提供残差定义与数据装配。
//
// 更新约定（必须与残差实现一致，不可与 NDT 配准互相照搬）：
//   节点分量直接相加，即 x += dx、y += dy、theta += dtheta。
//   因此雅可比第三列为 (local_y, -local_x)，与 ndt_2d.cpp 中左乘 exp 扰动的
//   (-p0.y, p0.x) 不同（见 02 §2.5）。

#include "scanlio/common/eigen_types.h"

#include <cstddef>
#include <string>
#include <vector>

namespace sad {

class PoseGraph {
 public:
  struct Options {
    int max_iterations = 50;            // LM 外层迭代上限
    double function_tolerance = 1e-6;   // 代价相对下降量停止阈值
    double parameter_tolerance = 1e-8;  // 步长相对模停止阈值

    // 稳健核。权重已由 EstimateOmega 从实测样本定标（W = Sigma^-1），
    // 故 sqrt(r^T W r) 是无量纲马氏距离。Ceres 比较的是三维残差块的模，
    // 其平方服从自由度 3 的卡方分布，故 delta 取 sqrt(chi2(3, 0.95))，
    // 对应约 5% 的正确边进入降权区；chi2(3, 0.99) 留给删边判据。
    // 默认开启；若调用方传入的 omega_diag 不是 1/sigma^2，须显式关闭，
    // 否则会出现 03 §3.3 描述的「核永不激活」或「全部降权」两种静默失效。
    bool enable_robust = true;
    double huber_delta = 2.795484;  // = sqrt(7.814728) = sqrt(chi2(3, 0.95))
  };

  // 从残差样本估计对角信息矩阵 W = Sigma^-1 的对角元。
  //   samples[t] 是一条同类边的一次残差 (rx, ry, rtheta)，
  //   来源须独立于节点初值（见 P1 块 2 §2.5）。
  // 逐分量用 n-1 分母算样本方差，再取倒数。
  // 样本数 < 2、或任一分量方差非正（例如全零）时返回 false 并写 err，
  // 不静默给出兜底值。
  static bool EstimateOmega(const std::vector<Eigen::Vector3d>& samples,
                            Eigen::Vector3d* omega_diag, std::string* err);

  // 一条边：两个节点序号 + 相对位姿观测 + 三个分量的权重。
  //   z 的三个分量依次为 zx(m)、zy(m)、ztheta(rad)，
  //   其中 zx/zy 表达在节点 i 的车体系下（见 01 §1.3.4）。
  struct Edge {
    int i = 0;
    int j = 0;
    Eigen::Vector3d z = Eigen::Vector3d::Zero();
    Eigen::Vector3d omega_diag = Eigen::Vector3d::Ones();
  };

  PoseGraph() = default;
  explicit PoseGraph(const Options& options) : options_(options) {}

  // 追加一个节点，返回其序号。序号即边的 i、j。
  // 节点 0 在优化中被固定为常数块（见 02 §2.1）。
  int AddNode(const SE2& pose);

  // 追加一条边。z 为「节点 j 在节点 i 坐标系下的相对位姿」。
  // 序号越界、i 等于 j、权重非正，均返回 false 且不写入。
  bool AddEdge(int i, int j, const SE2& z, const Eigen::Vector3d& omega_diag);

  std::size_t num_nodes() const { return params_.size() / 3; }
  std::size_t num_edges() const { return edges_.size(); }

  SE2 pose(std::size_t i) const;
  std::vector<SE2> poses() const;

  // 各边在当前节点值下的马氏残差范数 sqrt(r^T W r)，顺序与 AddEdge 一致。
  // 无量纲，单位为标准差个数；用于核对 Huber 的激活边数占比。
  // 只读，不改变内部状态。
  std::vector<double> EdgeResidualNorms() const;

  // 求解。节点数不足 2 或边数为 0 时返回 false（无事可做）。
  // 返回 Ceres 是否报告收敛；数值结果无论真假都已写回内部状态。
  bool Optimize();

  const Options& options() const { return options_; }
  void SetOptions(const Options& options) { options_ = options; }

 private:
  double* NodePtr(std::size_t k) { return params_.data() + 3 * k; }
  const double* NodePtr(std::size_t k) const { return params_.data() + 3 * k; }

  Options options_;

  // 节点按 (x, y, theta) 平铺存储，不用 std::vector<SE2>：
  // Ceres 要求每个参数块是一段连续的 double 数组，平铺后 NodePtr(k)
  // 直接可用，也避免 Eigen 固定尺寸类型的对齐策略带来的差异。
  std::vector<double> params_;
  std::vector<Edge> edges_;
};

}  // namespace sad

#endif  // SCANLIO_BACKEND_POSE_GRAPH_H_
