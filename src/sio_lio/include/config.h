#ifndef CONFIG_H_
#define CONFIG_H_

// sio_lio 统一配置：把分散在 ESKF::Options / NDT2D::Options / Map::Options /
// Frontend::Options 四处的参数，以及阶段 1 静态标定的初始条件，收拢为一个
// 可从 yaml 文件加载的结构。
//
// 设计约定：
//   1. yaml 中缺省的键 → 保留结构体默认值（向后兼容，不要求写全）
//   2. yaml 中出现未知键 → 加载失败并报出键名（防止拼写错误静默失效）
//   3. 加载失败时调用方必须检查返回值，本模块不会静默回退到默认值

#include "sio_lio/include/frontend.h"
#include "sio_lio/src/eskf.hpp"
#include "sio_lio/src/ndt_2d.hpp"

#include <string>

namespace sad {

struct Config {
    // ---- 初始条件（阶段 1 静止标定结果）----
    // 注意：本项目 CSV 中 accel 已扣除重力，ba 为去重力后的线性加速度零偏
    double init_bg = 0.005403;              // 陀螺 z 轴零偏（rad/s）
    Vec2d init_ba = Vec2d(-0.186, 0.054);   // 加计 xy 零偏（m/s^2）

    // ---- 滤波（ESKF::Options）----
    ESKF::Options eskf;

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

    // 把 eskf / ndt / frontend 三组参数合成为 Frontend 构造参数
    Frontend::Options ToFrontendOptions() const;

    // 从 yaml 加载。成功返回 true；失败返回 false 并通过 err 给出原因。
    bool LoadFromYaml(const std::string& path, std::string* err);

    // 以 yaml 文本形式导出当前配置（用于自检与存盘）
    std::string ToYamlString() const;
};

}  // namespace sad

#endif  // CONFIG_H_
