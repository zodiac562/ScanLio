#ifndef ESKF_HPP_
#define ESKF_HPP_

#include "sio_lio/common/eigen_types.h"
#include "sio_lio/common/imu.h"
#include "sio_lio/common/odom.h"

namespace sad{

    class ESKF{
    public:
        struct Options{
            double gyro_var_ = 1e-6;         // 陀螺噪声方差
            double acce_var_ = 1e-3;         // 加计噪声方差
            // 零偏随机游走方差：取实测值（doc/eskf_noise_params.yaml）。
            // bias_gyro_rw_std 3.783749e-02 → 每步 1.430343e-05；
            // bias_acce_rw_std 9.624057e-02 → 每步 9.253627e-05。
            // 原值 1e-4 无实测依据，会使零偏跟随过慢。
            double bias_gyro_var_ = 1.430343e-05;  // 陀螺零偏随机游走方差
            double bias_acce_var_ = 9.253627e-05;  // 加计零偏随机游走方差
            double odom_var_ = 0.1;          // 轮速观测噪声
            double lidar_pos_noise_ = 0.05;  // 雷达位姿观测位置噪声
            double lidar_ang_noise_ = 0.1;   // 雷达位姿观测角度噪声
            double imu_dt_ = 0.01;           // IMU 采样间隔
        };

        ESKF()=default;
        explicit ESKF(const Options& options) : options_(options) {}

        void SetInitCondition(double bg,const Vec2d& ba);
        void Predict(const Imu& imu);
        void ObserveOdom(const Odom& odom);
        void ObserveLidar(const SE2& lidar_pose);
        SE2 GetNominalPose() const;

        // 后验误差协方差（8x8，按值返回）。
        // 各分量的行列顺序与误差状态一致：0~1 = p_x,p_y；2~3 = v_x,v_y；4 = theta；
        // 5 = bg（陀螺 z 轴零偏）；6~7 = ba_x,ba_y（加计 xy 零偏）。
        // 用途：发布位姿协方差、观察各状态的可观测性变化。返回值是副本，
        // 修改它不会影响滤波器内部状态。
        Mat8d GetP() const;

        // 当前名义零偏（陀螺 z 轴、加计 xy）。只读，用于验证零偏估计是否收敛：
        // 给一个与真值不同的初值，观察返回值是否朝真值靠近。
        void GetBias(double* bg, Vec2d* ba) const;

    private:
        Vec2d p_ = Vec2d::Zero();
        Vec2d v_ = Vec2d::Zero();
        double theta_ = 0.0;
        Vec2d ba_ = Vec2d::Zero();
        double bg_ = 0.0;

        Mat8d P_ = Mat8d::Identity();    // 误差协方差

        Options options_;
        double last_imu_time_ = 0.0;     // 上一条 IMU 时间戳
        bool inited_ = false;
    };
}

#endif

