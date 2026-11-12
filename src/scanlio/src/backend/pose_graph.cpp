#include "scanlio/include/backend/pose_graph.h"

#include <ceres/ceres.h>
#include <ceres/loss_function.h>

#include <cmath>
#include <limits>

namespace sad {

namespace {

// SE(2) 相对位姿残差（推导见 03 §3.7，定义见 01 §1.5）。
//
// 输入 xi、xj 各是一段 3 个 double 的参数块，依次为 (x, y, theta)。
// 输出 res 是已经乘过 sqrt(W) 的残差，使 Ceres 计算的平方和等于 r^T W r
// （恒等式 r^T W r = ||L^T r||^2，见 03 §3.6）。
//
// 这里必须用 std::cos / std::sin 的重载而非 ::cos：Ceres 传入的是 Jet 类型，
// 需要 ADL 找到 Jet 的三角重载。写成 ::cos 会在编译期因无法把 Jet 转成
// double 而失败。
struct EdgeCost {
  EdgeCost(const Eigen::Vector3d& z, const Eigen::Vector3d& sqrt_omega)
      : z_(z), sw_(sqrt_omega) {}

  template <typename T>
  bool operator()(const T* const xi, const T* const xj, T* res) const {
    using std::cos;
    using std::sin;

    // 第一步：世界系位移
    const T dx = xj[0] - xi[0];
    const T dy = xj[1] - xi[1];

    // 第二步：转到节点 i 的车体系（R(-theta_i) 作用于位移）
    const T c = cos(xi[2]);
    const T s = sin(xi[2]);
    const T lx = c * dx + s * dy;
    const T ly = -s * dx + c * dy;

    // 第三步：角度差折回 (-pi, pi] 后与边值作差
    T dth = xj[2] - xi[2] - T(z_[2]);
    while (dth > T(M_PI)) {
      dth -= T(2.0 * M_PI);
    }
    while (dth < -T(M_PI)) {
      dth += T(2.0 * M_PI);
    }

    res[0] = (lx - T(z_[0])) * T(sw_[0]);
    res[1] = (ly - T(z_[1])) * T(sw_[1]);
    res[2] = dth * T(sw_[2]);
    return true;
  }

  Eigen::Vector3d z_;
  Eigen::Vector3d sw_;
};

}  // namespace

bool PoseGraph::EstimateOmega(const std::vector<Eigen::Vector3d>& samples,
                              Eigen::Vector3d* omega_diag, std::string* err) {
  if (omega_diag == nullptr) {
    if (err) *err = "omega_diag 输出指针为空";
    return false;
  }
  const std::size_t n = samples.size();
  if (n < 2) {
    if (err) *err = "样本数少于 2，无法估计方差（分母 n-1 无意义）";
    return false;
  }

  // 逐分量求样本均值。写全三个分量，不用 Eigen 的均值表达式，
  // 避免与 Jet 求导路径混淆。
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  for (const auto& s : samples) {
    mean += s;
  }
  mean /= static_cast<double>(n);

  Eigen::Vector3d var = Eigen::Vector3d::Zero();
  for (const auto& s : samples) {
    const Eigen::Vector3d d = s - mean;
    var[0] += d[0] * d[0];
    var[1] += d[1] * d[1];
    var[2] += d[2] * d[2];
  }
  var /= static_cast<double>(n - 1);

  if (!(var.array() > 0.0).all()) {
    if (err) {
      *err = "存在方差非正的残差分量；这类样本不含离散信息，"
             "不给出兜底权重（见 P1 块 2 §2.5：样本须独立于节点初值）";
    }
    return false;
  }
  if (!(var.array().isFinite()).all()) {
    if (err) *err = "样本方差含非有限值";
    return false;
  }

  *omega_diag = var.cwiseInverse();
  return true;
}

int PoseGraph::AddNode(const SE2& pose) {
  params_.push_back(pose.translation().x());
  params_.push_back(pose.translation().y());
  params_.push_back(pose.so2().log());
  return static_cast<int>(params_.size() / 3) - 1;
}

bool PoseGraph::AddEdge(int i, int j, const SE2& z,
                        const Eigen::Vector3d& omega_diag) {
  const int n = static_cast<int>(num_nodes());
  if (i < 0 || j < 0 || i >= n || j >= n || i == j) {
    return false;
  }
  if (!(omega_diag.array() > 0.0).all()) {
    return false;
  }

  Edge e;
  e.i = i;
  e.j = j;
  // 注意：SE2::log() 的平移分量是「加权平移」V^-1 * t，不是矩阵中的 t。
  // 边值需要的是原始平移，因此这里必须用 translation() 取，不能对 SE2 直接取 log()。
  e.z = Eigen::Vector3d(z.translation().x(), z.translation().y(),
                        z.so2().log());
  e.omega_diag = omega_diag;
  edges_.push_back(e);
  return true;
}

SE2 PoseGraph::pose(std::size_t i) const {
  if (i >= num_nodes()) {
    return SE2();
  }
  const double* p = NodePtr(i);
  return SE2(SO2(p[2]), Eigen::Vector2d(p[0], p[1]));
}

std::vector<SE2> PoseGraph::poses() const {
  std::vector<SE2> out;
  out.reserve(num_nodes());
  for (std::size_t k = 0; k < num_nodes(); ++k) {
    out.push_back(pose(k));
  }
  return out;
}

std::vector<double> PoseGraph::EdgeResidualNorms() const {
  std::vector<double> out;
  out.reserve(edges_.size());
  for (const Edge& e : edges_) {
    const Eigen::Vector3d sw = e.omega_diag.cwiseSqrt();
    EdgeCost cost(e.z, sw);
    double res[3] = {0.0, 0.0, 0.0};
    // EdgeCost 是模板仿函数，以 double 实例化即得到与 Ceres 内部完全一致的
    // 残差表达式（同一份代码，不另写一份），保证诊断值与实际代价同源。
    cost(params_.data() + 3 * e.i, params_.data() + 3 * e.j, res);
    out.push_back(std::sqrt(res[0] * res[0] + res[1] * res[1] + res[2] * res[2]));
  }
  return out;
}

bool PoseGraph::Optimize() {
  if (num_nodes() < 2 || edges_.empty()) {
    return false;
  }

  ceres::Problem problem;

  for (const Edge& e : edges_) {
    const Eigen::Vector3d sw = e.omega_diag.cwiseSqrt();
    auto* cost =
        new ceres::AutoDiffCostFunction<EdgeCost, 3, 3, 3>(new EdgeCost(e.z, sw));
    ceres::LossFunction* loss = nullptr;
    if (options_.enable_robust) {
      loss = new ceres::HuberLoss(options_.huber_delta);
    }
    // 一个节点一个参数块：稀疏性由边集决定，参数块划分只是把该结构
    // 告知求解器（见 02 §2.6.3）。若把全部节点拼成单个 3N 大小的块传入，
    // 求解器失去耦合信息，只能按稠密处理。
    problem.AddResidualBlock(cost, loss, params_.data() + 3 * e.i,
                             params_.data() + 3 * e.j);
  }

  // 规范自由度：节点 0 固定为单位位姿（见 02 §2.1）。
  // 不固定时 H 有 3 维零空间，整图可整体平移旋转而代价不变。
  problem.SetParameterBlockConstant(params_.data());

  ceres::Solver::Options solver_options;
  solver_options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  solver_options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  solver_options.max_num_iterations = options_.max_iterations;
  solver_options.function_tolerance = options_.function_tolerance;
  solver_options.parameter_tolerance = options_.parameter_tolerance;
  solver_options.minimizer_progress_to_stdout = false;

  ceres::Solver::Summary summary;
  ceres::Solve(solver_options, &problem, &summary);

  return summary.termination_type == ceres::CONVERGENCE;
}

}  // namespace sad
