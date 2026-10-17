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
    };

    explicit Frontend(const Options& options);

    // 三入口（按数据到达顺序调用）
    void ProcessImu(const Imu& imu);
    void ProcessOdom(const Odom& odom);
    bool ProcessScan(const std::shared_ptr<Frame>& frame);   // 返回配准是否成功

    // 当前位姿（来自 ESKF 名义状态）
    SE2 GetPose() const;

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
