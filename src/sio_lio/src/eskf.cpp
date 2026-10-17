#include "sio_lio/src/eskf.hpp"

#include <cmath>

namespace sad {

void ESKF::SetInitCondition(double bg, const Vec2d& ba) {
    bg_ = bg;
    ba_ = ba;
    // 注意：此处不置 inited_。inited_ 只由 Predict 首帧置位，使首帧保护
    // 能捕获真实时间基准 last_imu_time_；若在此置 true 而数据时间戳是
    // Unix 量级（如 1.79e9），首条 IMU 的 dt = t - 0 会达十亿秒级，
    // theta_ += w*dt 直接发散（2026-10-16 由 D2 纯惯导测试暴露的缺陷）
}

void ESKF::Predict(const Imu& imu) {
    if (!inited_) {
        last_imu_time_ = imu.timestamp_;
        inited_ = true;
        return;
    }

    double dt = imu.timestamp_ - last_imu_time_;
    if (dt <= 0) {
        return;
    }

    // 去偏：减去当前零偏估计
    double w = imu.gyro_[2] - bg_;
    Vec2d f = imu.accel_.head<2>() - ba_;

    // 姿态积分
    theta_ += w * dt;

    // 比力转到世界系（SO2 旋转）
    double cos_theta = std::cos(theta_);
    double sin_theta = std::sin(theta_);
    Vec2d a_world(cos_theta * f[0] - sin_theta * f[1],
                  sin_theta * f[0] + cos_theta * f[1]);

    // 半隐式欧拉：先更新速度，再用新速度更新位置
    v_ += a_world * dt;
    p_ += v_ * dt;

    // ---- 误差状态转移矩阵 F（8x8，在本步名义状态处线性化）----
    // 误差传播：ex_k+1 = F ex_k + w_k
    // 各偏导来自对名义积分式求偏导：
    //   p' = p + v dt                → dp'/dv = I·dt
    //   v' = v + R(theta') f dt      → dv'/dtheta = R·[-f_y; f_x]·dt
    //                                   dv'/dba = -R·dt
    //   theta' = theta + (gyro-bg)dt → dtheta'/dbg = -dt
    // 其余对角元为 1（Identity 基底）
    Mat8d F = Mat8d::Identity();
    F.block<2, 2>(0, 2) = Mat2d::Identity() * dt;   // p ← v
    // R·[-f_y; f_x] = [-cos*f_y - sin*f_x; -sin*f_y + cos*f_x]
    F(2, 4) = (-cos_theta * f[1] - sin_theta * f[0]) * dt;  // v_x ← theta
    F(3, 4) = (-sin_theta * f[1] + cos_theta * f[0]) * dt;  // v_y ← theta
    F(2, 6) = -cos_theta * dt;                      // v ← ba（-R(theta)·dt）
    F(2, 7) =  sin_theta * dt;
    F(3, 6) = -sin_theta * dt;
    F(3, 7) = -cos_theta * dt;
    F(4, 5) = -dt;                                  // theta ← bg

    // ---- 过程噪声 Q（对角）----
    // Options 注释约定：*_var_ 直接作方差使用（对照：lidar_*_noise_ 为标准差、使用时平方）
    Mat8d Q = Mat8d::Zero();
    Q(2, 2) = options_.acce_var_;        // v_x
    Q(3, 3) = options_.acce_var_;        // v_y
    Q(4, 4) = options_.gyro_var_;        // theta
    Q(5, 5) = options_.bias_gyro_var_;   // bg
    Q(6, 6) = options_.bias_acce_var_;   // ba_x
    Q(7, 7) = options_.bias_acce_var_;   // ba_y

    // ---- 协方差传播 ----
    P_ = F * P_ * F.transpose() + Q;

    last_imu_time_ = imu.timestamp_;
}

void ESKF::ObserveOdom(const Odom& odom) {
    if (!inited_) {
        return;
    }

    // 观测：车体系前向速度 vel_
    // 预测：把世界系速度 v_ 旋转到车体系，取 x 分量
    double cos_theta = std::cos(theta_);
    double sin_theta = std::sin(theta_);
    double v_body_x = cos_theta * v_[0] + sin_theta * v_[1];

    // 残差（1 维）：r = z - h(x_minus_k)
    double r = odom.vel_ - v_body_x;

    // 观测雅可比 H（1x8），h = cos_theta*v_x + sin_theta*v_y，各偏导在当前名义状态处取值：
    //   dh/dv_x = cos_theta,  dh/dv_y = sin_theta
    //   dh/dtheta = -sin_theta*v_x + cos_theta*v_y（theta 进入旋转投影）
    //   对 p、bg、ba 的偏导为 0
    Eigen::Matrix<double, 1, 8> H = Eigen::Matrix<double, 1, 8>::Zero();
    H(0, 2) = cos_theta;
    H(0, 3) = sin_theta;
    H(0, 4) = -sin_theta * v_[0] + cos_theta * v_[1];

    // 新息方差 S（标量）：S = H P H^T + R
    double S = (H * P_ * H.transpose())(0, 0) + options_.odom_var_;

    // 卡尔曼增益 K（8x1）：K = P H^T / S
    Eigen::Matrix<double, 8, 1> K = P_ * H.transpose() / S;

    // 误差状态估计并注入名义状态
    // （零偏分量 dx(5)、dx(6)、dx(7) 按 Zodiac 决策留待阶段 4 数据流程化接入，此处丢弃）
    Vec8d dx = K * r;
    p_ += dx.head<2>();
    v_ += dx.segment<2>(2);
    theta_ += dx(4);
    while (theta_ > M_PI) theta_ -= 2 * M_PI;
    while (theta_ < -M_PI) theta_ += 2 * M_PI;

    // 协方差更新：P = (I - K H) P
    Mat8d I_KH = Mat8d::Identity() - K * H;
    P_ = I_KH * P_;
}

void ESKF::ObserveLidar(const SE2& lidar_pose) {
    if (!inited_) {
        return;
    }

    // 观测：雷达配准位姿（x, y, theta）
    Vec3d z(lidar_pose.translation().x(), lidar_pose.translation().y(),
            lidar_pose.log()[2]);

    // 预测观测：当前名义位姿
    Vec3d x_pred(p_[0], p_[1], theta_);

    // 残差（3 维）：位置直接相减，角度差 wrap 到 (-pi, pi]
    Vec3d residual = z - x_pred;
    while (residual[2] > M_PI) residual[2] -= 2 * M_PI;
    while (residual[2] < -M_PI) residual[2] += 2 * M_PI;

    // 观测噪声 R（3x3 对角：位置噪声、角度噪声独立）
    Mat3d R = Mat3d::Zero();
    R(0, 0) = options_.lidar_pos_noise_ * options_.lidar_pos_noise_;
    R(1, 1) = options_.lidar_pos_noise_ * options_.lidar_pos_noise_;
    R(2, 2) = options_.lidar_ang_noise_ * options_.lidar_ang_noise_;

    // 观测雅可比 H（3x8）：观测 = 状态的 (p_x, p_y, theta) 分量
    Eigen::Matrix<double, 3, 8> H = Eigen::Matrix<double, 3, 8>::Zero();
    H(0, 0) = 1.0;   // d(z_x) / d(p_x)
    H(1, 1) = 1.0;   // d(z_y) / d(p_y)
    H(2, 4) = 1.0;   // d(z_theta) / d(theta)

    // 新息协方差 S（3x3）：S = H P H^T + R
    Mat3d S = H * P_ * H.transpose() + R;

    // 卡尔曼增益 K_full（8x3）：K = P H^T S^{-1}
    Eigen::Matrix<double, 8, 3> K_full = P_ * H.transpose() * S.inverse();

    // 误差状态最优估计（8 维全量）
    Vec8d dx = K_full * residual;
    p_[0] += dx[0];
    p_[1] += dx[1];
    // 速度分量注入：P 中 v 与 p/theta 的交叉协方差（由 Predict 的 F 通道积累）使
    // K_full 第 2、3 行非零，雷达残差联动修正 v_；若只注入 p/theta 而 P 全量缩减，
    // v 的方差被"已修正"地缩小但状态未变，协方差账本与状态实际不一致
    // （修复于 2026-10-16，此前 dx[2]、dx[3] 被丢弃）
    v_ += dx.segment<2>(2);
    theta_ += dx[4];
    while (theta_ > M_PI) theta_ -= 2 * M_PI;
    while (theta_ < -M_PI) theta_ += 2 * M_PI;

    // 协方差更新：P = (I - K H) P
    Mat8d I_KH = Mat8d::Identity() - K_full * H;
    P_ = I_KH * P_;
}

SE2 ESKF::GetNominalPose() const {
    return SE2(SO2(theta_), p_);
}

}  // namespace sad
