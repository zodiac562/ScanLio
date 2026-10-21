#ifndef FRONTEND_H_
#define FRONTEND_H_

#include "sio_lio/include/map.h"
#include "sio_lio/src/eskf.hpp"
#include "sio_lio/common/imu.h"
#include "sio_lio/common/odom.h"

#include <deque>
#include <memory>

namespace sad {

// 三路数据的调度器，持有ESKF与Map，闭合预测-配准-修正反馈环
class Frontend {
   public:
    struct Options {
        double keyframe_dist_th_ = 0.1;     // 关键帧平移阈值（米）
        double keyframe_rot_th_ = 15.0 * M_PI / 180.0;  // 关键帧旋转阈值（弧度）
        size_t imu_buffer_size_ = 20;       // IMU 位姿缓冲容量（对齐原版）

        // 入图位姿来源：
        //   true  —— 用滤波后位姿（ESKF 修正完的名义状态），对齐原版 ndtlio
        //            frontend.cpp:35「current_frame_->pose_ = eskf_->getNominalPose()」
        //   false —— 用配准位姿（NDT 输出，未与预测融合）
        // 两者差别 = (I - K) × 配准残差：K 越接近 1（越信任雷达）差别越小。
        bool use_filtered_pose_for_map_ = true;

        // 去畸变总开关：false 时跳过 UndistortScan，点云保持原样。
        // 供对照实验显式关闭，不依赖「pts_time_ 未填」这种隐式失效路径。
        bool enable_undistort_ = true;

        // 内部持有的两个子系统参数。此前 Frontend 用默认值构造它们，
        // 导致 ESKF 与 NDT 的参数无法从外部配置；此处透传以支持 yaml 统一配置。
        ESKF::Options eskf_opts_;
        Map::Options map_opts_;
    };

    explicit Frontend(const Options& options);

    // 三入口（按数据到达顺序调用）
    void ProcessImu(const Imu& imu);
    void ProcessOdom(const Odom& odom);
    bool ProcessScan(const std::shared_ptr<Frame>& frame);   // 返回配准是否成功

    // 当前位姿（来自 ESKF 名义状态）
    SE2 GetPose() const;

    // 当前后验误差协方差（8x8，行列顺序见 eskf.hpp 的 GetP 注释）。
    // 用途：节点发布位姿协方差；只读，不影响滤波器状态。
    Mat8d GetP() const { return eskf_->GetP(); }

    // 内部地图的只读/读写访问（对应原版 IncrementalNDTLO::getMap）。
    // 用途：外部读取体素坐标与占据栅格以发布可视化话题，或在关键帧之外补写占据栅格。
    // 注意：对返回的 Map 调用 add_keyframe / match 会改变前端状态，
    // 只应在两次 ProcessScan 之间、且调用方清楚后果时使用。
    Map& GetMap() { return *map_; }
    const Map& GetMap() const { return *map_; }

    // 静态初始化（阶段 1 产物传入）：透传给内部 ESKF
    void SetInitCondition(double bg, const Vec2d& ba);

   private:
    Options options_;
    std::shared_ptr<ESKF> eskf_;
    std::shared_ptr<Map> map_;

    bool first_scan_ = true;                 // 首帧雷达标志
    SE2 last_keyframe_pose_;                 // 上一关键帧位姿（判定基准）
    std::deque<std::pair<double, SE2>> imu_buffer_;  // IMU 预测位姿缓冲（去畸变用）

    bool UndistortScan(const std::shared_ptr<Frame>& frame);  // 去畸变（内部函数）
};

}

#endif
