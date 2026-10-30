#include "scanlio/src/ieskf_2d.hpp"

#include <cmath>

namespace sad {

namespace {
// 把角度折回 (-π, π]
double WrapToPi(double theta) {
    while (theta > M_PI) theta -= 2.0 * M_PI;
    while (theta < -M_PI) theta += 2.0 * M_PI;
    return theta;
}
}  // namespace

void IESKF2D::ResetCovariance() {
    P_ = Mat8d::Identity() * 1e-4;
    P_(4, 4) = 0.01;   // 角度初始不确定度
    P_(5, 5) = 0.01;   // 陀螺零偏
    P_(6, 6) = 0.01;   // 加计零偏 x
    P_(7, 7) = 0.01;   // 加计零偏 y
    dx_.setZero();
}

void IESKF2D::SetInitCondition(double bg, const Vec2d& ba) {
    bg_ = bg;
    ba_ = ba;
    // 不置 inited_：与 ESKF::SetInitCondition 同一理由，首帧保护需要捕获
    // 真实的时间基准；若此处置位，首条 IMU 的 dt 会是十亿秒量级。
}

void IESKF2D::SetState(const Vec2d& p, const Vec2d& v, double theta, double bg, const Vec2d& ba,
                       double timestamp) {
    p_ = p;
    v_ = v;
    theta_ = WrapToPi(theta);
    bg_ = bg;
    ba_ = ba;
    last_imu_time_ = timestamp;
    inited_ = true;
    dx_.setZero();
}

bool IESKF2D::Predict(const Imu& imu) {
    if (!inited_) {
        last_imu_time_ = imu.timestamp_;
        inited_ = true;
        return false;
    }

    const double dt = imu.timestamp_ - last_imu_time_;
    if (dt <= 0.0) {
        return false;
    }

    const Vec2d acce(imu.accel_.x(), imu.accel_.y());
    const Vec2d a = acce - ba_;
    const double dtheta = (imu.gyro_.z() - bg_) * dt;

    // ---- 名义状态积分（二阶，对照 ieskf.cpp:74-77）----
    // 先把「世界系加速度 × dt」算成一份，位置更新同时用到它的一半与全部。
    const Vec2d acc_world_dt = SO2(theta_) * a * dt;
    p_ += v_ * dt + 0.5 * acc_world_dt * dt;
    v_ += acc_world_dt;
    theta_ = WrapToPi(theta_ + dtheta);

    // ---- 误差状态雅可比 F（Identity 基底）----
    //   p' = p + v·dt                → ∂p'/∂v = I·dt
    //   v' = v + R(θ)·a·dt           → ∂v'/∂θ = R·[-a_y; a_x]·dt
    //                                  ∂v'/∂ba = -R·dt
    //   θ' = θ + (ω - bg)·dt         → ∂θ'/∂bg = -dt
    // θ 对 θ 的偏导为 1，Identity 基底已给出，不再改写（类注释第 4 条）。
    Mat8d F = Mat8d::Identity();
    const Mat2d R = SO2(theta_).matrix();
    F.block<2, 2>(0, 2) = Mat2d::Identity() * dt;
    F.block<2, 1>(2, 4) = R * Vec2d(-a.y(), a.x()) * dt;
    F.block<2, 2>(2, 6) = -R * dt;
    F(4, 5) = -dt;

    // ---- 过程噪声（由标准差平方而得）----
    Mat8d Q = Mat8d::Zero();
    const double acce_var = options_.acce_std_ * options_.acce_std_;
    const double gyro_var = options_.gyro_std_ * options_.gyro_std_;
    const double bias_g_var = options_.bias_gyro_std_ * options_.bias_gyro_std_;
    const double bias_a_var = options_.bias_acce_std_ * options_.bias_acce_std_;
    Q(2, 2) = acce_var;
    Q(3, 3) = acce_var;
    Q(4, 4) = gyro_var;
    Q(5, 5) = bias_g_var;
    Q(6, 6) = bias_a_var;
    Q(7, 7) = bias_a_var;

    P_ = F * P_ * F.transpose() + Q;

    last_imu_time_ = imu.timestamp_;
    return true;
}

bool IESKF2D::ObserveOdom(const Odom& odom) {
    if (!inited_) {
        return false;
    }

    // 观测 z 与预测 h 都是「世界系速度」二维向量。
    // z 的第二分量由车体系横向速度 0 旋转而来，属非完整约束假设；
    // /robotvel 的 y 字段实测恒 0（car_base_v2 frame_parser.h:23 注释），
    // 因此这里没有引入任何真实横向测量。
    const SO2 R(theta_);
    const Vec2d z = R * Vec2d(odom.vel_, 0.0);
    const Vec2d h = R * Vec2d(v_[0], v_[1]);

    // H 只对 vx、vy 两列非零
    Eigen::Matrix<double, 2, 8> H = Eigen::Matrix<double, 2, 8>::Zero();
    H(0, 2) = 1.0;
    H(1, 3) = 1.0;

    const Mat2d V = Mat2d::Identity() * (options_.odom_std_ * options_.odom_std_);

    const Eigen::Matrix<double, 8, 2> K =
        P_ * H.transpose() * (H * P_ * H.transpose() + V).inverse();

    dx_ = K * (z - h);
    P_ = (Mat8d::Identity() - K * H) * P_;

    Inject();
    ProjectCovariance();
    dx_.setZero();
    return true;
}

bool IESKF2D::UpdateUsingCustomObserve(const CustomObsFunc& obs) {
    if (!inited_ || !obs) {
        return false;
    }

    Mat8d HT_Vinv_H = Mat8d::Zero();
    Vec8d HT_Vinv_r = Vec8d::Zero();
    Mat8d Pk = P_;
    Mat8d Qk = Mat8d::Zero();

    for (int iter = 0; iter < options_.num_iterations_; ++iter) {
        // 用当前名义位姿重新线性化。obs 内部只累加，不更新位姿。
        obs(GetNominalPose(), HT_Vinv_H, HT_Vinv_r);
        HT_Vinv_H *= options_.info_ratio_;
        HT_Vinv_r *= options_.info_ratio_;

        // 本次迭代的 Pk：用上一轮注入的 dx_ 做流形投影
        Pk = P_;
        const double J = 1.0 - 0.5 * dx_(4);
        const double J2 = J * J;
        Pk.block<1, 8>(4, 0) = J2 * P_.block<1, 8>(4, 0);
        Pk.block<8, 1>(0, 4) = J2 * P_.block<8, 1>(0, 4);
        Pk(4, 4) = P_(4, 4) * J2;   // 对角项只乘一次（类注释第 7 条）

        Qk = (Pk.inverse() + HT_Vinv_H).inverse();
        dx_ = Qk * HT_Vinv_r;
        if (std::isnan(dx_(4))) {
            continue;   // 协方差病态时本步结果不可用，跳过注入，进入下一次迭代
        }

        Inject();
        if (dx_.norm() < options_.eps_) {
            break;
        }
    }

    P_ = (Mat8d::Identity() - Qk * HT_Vinv_H) * Pk;
    ProjectCovariance();
    dx_.setZero();
    return true;
}

Vec8d IESKF2D::GetNominal() const {
    Vec8d x = Vec8d::Zero();
    x.block<2, 1>(0, 0) = p_;
    x.block<2, 1>(2, 0) = v_;
    x(4) = theta_;
    x(5) = bg_;
    x.block<2, 1>(6, 0) = ba_;
    return x;
}

void IESKF2D::GetBias(double* bg, Vec2d* ba) const {
    if (bg != nullptr) *bg = bg_;
    if (ba != nullptr) *ba = ba_;
}

void IESKF2D::Inject() {
    p_ += dx_.head<2>();
    v_ += dx_.segment<2>(2);
    theta_ = WrapToPi(theta_ + dx_(4));
    if (options_.update_bias_gyro_) {
        bg_ += dx_(5);
    }
    if (options_.update_bias_acce_) {
        ba_ += dx_.segment<2>(6);
    }
}

void IESKF2D::ProjectCovariance() {
    // 误差状态注入后，角度分量的参考切空间旋转了 dx_(4)。
    // 按 G·P·Gᵀ（G 仅 (4,4) = J）展开：交叉项乘 J²，对角项 (4,4) 也只乘一次 J²。
    const double J = 1.0 - 0.5 * dx_(4);
    const double J2 = J * J;
    for (int i = 0; i < 8; ++i) {
        if (i == 4) {
            continue;
        }
        P_(4, i) *= J2;
        P_(i, 4) *= J2;
    }
    P_(4, 4) *= J2;
}

}  // namespace sad
