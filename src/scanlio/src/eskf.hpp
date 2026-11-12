#ifndef ESKF_HPP_
#define ESKF_HPP_

#include "scanlio/common/eigen_types.h"
#include "scanlio/common/imu.h"
#include "scanlio/common/odom.h"

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
            // 原 imu_dt_ 已删除（2026-10-29 修复 S-03）：Predict 的时间步长一律由
            // 相邻 IMU 时间戳相减得出，固定采样间隔不参与任何计算，保留只会误导。
        };

        ESKF()=default;
        explicit ESKF(const Options& options) : options_(options) {}

        void SetInitCondition(double bg,const Vec2d& ba);
        void Predict(const Imu& imu);
        void ObserveOdom(const Odom& odom);
        void ObserveLidar(const SE2& lidar_pose);
        SE2 GetNominalPose() const;

        // 外部回写位姿（后端优化结果）。只改位置 p_ 与航向 theta_。
        // 不动 v_、bg_、ba_、P_、inited_、last_imu_time_。
        // 依据 doc/复习资料/ESKF/05-局限与实现要点.md 易混淆点 4：
        // 外部改写不在滤波方程的推导框架内，没有对应的协方差更新式；
        // 后端目前也不提供位姿协方差，故此处不凭空构造 P 的变换。
        // 这是一处已声明的语义缺口，不是遗漏；判据见
        // test/backend_map_rebuild_test.cpp 的 T7。
        void SetPose(const SE2& pose);

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

