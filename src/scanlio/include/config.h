#ifndef CONFIG_H_
#define CONFIG_H_

// scanlio 统一配置：把分散在 ESKF::Options / NDT2D::Options / Map::Options /
// Frontend::Options 四处的参数，以及阶段 1 静态标定的初始条件，收拢为一个
// 可从 yaml 文件加载的结构。
//
// 设计约定：
//   1. yaml 中缺省的键 → 保留结构体默认值（向后兼容，不要求写全）
//   2. yaml 中出现未知键 → 加载失败并报出键名（防止拼写错误静默失效）
//   3. 加载失败时调用方必须检查返回值，本模块不会静默回退到默认值

#include "scanlio/include/frontend.h"
#include "scanlio/src/eskf.hpp"
#include "scanlio/src/ndt_2d.hpp"

#include <string>

namespace sad {

// 回环与后端接线的算法参数（P4）。
// 这里不 include backend 头文件：本结构属于 scanlio_core，而 LoopClosure /
// PoseGraph 属于 scanlio_backend，反向依赖会破坏「core 不依赖 Ceres」的分层。
// 节点层把本结构逐字段转换成 LoopClosure::Options 与 PoseGraph::Options。
//
// 权重取 P1 从 imu_static 静止段实测的值（见 test/edge_noise_params.h）。
// 写成代码默认值，yaml 缺省时也能得到正确量级；定标方法见
// doc/复习资料/位姿图优化/01 §1.6。
struct LoopConfig {
    bool enable = false;                 // 总开关，默认关闭，保证默认行为不变
    int optimize_every_n_keyframes = 10; // 每新增多少个关键帧触发一次后端

    double max_distance = 3.0;                       // 米
    int min_index_gap = 20;                          // 关键帧编号之差
    double max_heading_diff_deg = 30.0;              // 度，转换时乘 M_PI/180
    int rings = 20;
    int sectors = 60;
    double max_range = 20.0;                         // 米
    double min_similarity = 0.0;
    int max_candidates = 3;
    int submap_half_window = 2;

    // 里程计边 / 回环边的信息矩阵对角元，单位分别是 1/m² 与 1/rad²
    Vec3d odom_omega = Vec3d(13850773.464537829, 644915.17350063322,
                             17712925.894549124);
    Vec3d loop_omega = Vec3d(417352.9587567249, 151339.27485489356,
                             362679.49452188675);

    int max_iterations = 50;
    double function_tolerance = 1e-6;
    double parameter_tolerance = 1e-8;
    bool enable_robust = true;
    double huber_delta = 2.795484;       // sqrt(chi2(3, 0.95))
};

struct Config {
    // ---- 初始条件（阶段 1 静止标定结果）----
    // 注意：本项目 CSV 中 accel 已扣除重力，ba 为去重力后的线性加速度零偏
    double init_bg = 0.005403;              // 陀螺 z 轴零偏（rad/s）
    Vec2d init_ba = Vec2d(-0.186, 0.054);   // 加计 xy 零偏（m/s^2）

    // ---- 滤波（ESKF::Options）----
    // 语义：[实测] 段内 *_var 键直接作方差使用
    ESKF::Options eskf;

    // ---- 迭代滤波（IESKF2D::Options）----
    // 语义：段内 *_std 键为标准差，代码内部平方成方差。
    // 与 eskf 段分成两段的原因就是这两套语义相反，混在一起会静默改变量级。
    // 启用哪一套由 frontend.filter_type 决定，未选中的那一套不参与计算。
    IESKF2D::Options ieskf;

    // ---- 配准与地图（NDT2D::Options，Map 内部只有一个 NDT2D）----
    NDT2D::Options ndt;

    // ---- 占据栅格（OccupancyMap::Options，Map 内部各持有一个）----
    // 与 ndt 分开成一段的原因：栅格分辨率与配准的体素边长是两个独立量，
    // 前者只影响显示与导出，后者影响配准结果。
    OccupancyMap::Options occu;

    // ---- 前端调度（Frontend::Options 的 5 个自有字段）----
    // 说明：frontend 中另有 eskf_opts_ / map_opts_，由本结构另行填充，
    // yaml 不重复配置，以免两处写法不一致。
    Frontend::Options frontend;

    // ---- 回环与后端接线（P4）----
    // 默认 enable=false，不改变既有行为；开启后由节点在关键帧触发一次
    // 「检测 → 建图 → 求解 → 回写 → 重建」。
    LoopConfig loop;

    // 把 eskf / ndt / frontend 三组参数合成为 Frontend 构造参数
    Frontend::Options ToFrontendOptions() const;

    // 从 yaml 加载。成功返回 true；失败返回 false 并通过 err 给出原因。
    bool LoadFromYaml(const std::string& path, std::string* err);

    // 以 yaml 文本形式导出当前配置（用于自检与存盘）
    std::string ToYamlString() const;
};

}  // namespace sad

#endif  // CONFIG_H_
