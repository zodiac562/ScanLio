#ifndef STATIC_IMU_INIT_H_
#define STATIC_IMU_INIT_H_

#include <cmath>
#include <cstdio>
#include <deque>
#include "sio_lio/common/eigen_types.h"
#include "sio_lio/common/imu.h"
#include "sio_lio/common/odom.h"
#include "sio_lio/common/math_utils.h"

namespace sad {

/// IMU 静止状态初始化器
///
/// 阅读顺序与数据流一致：
///   1. AddOdom  : 每条轮速数据到来，只更新静止标志
///   2. AddIMU   : 每条 IMU 数据到来，静止则入队并计时
///   3. TryInit  : 攒满 10 秒后触发，三次统计 + 噪声检查 + 存结果
///   4. Get 系列 : 初始化成功后，外界从这里取结果
///
/// 生命周期：小车静止前数据被拒绝；静止后攒约 10 秒；
/// TryInit 成功后本对象工作结束，五个结果保持不变直到对象销毁。
class StaticIMUInit {
   public:
    /// 可调参数集合
    struct Options {
        double init_time_seconds_ = 10.0;     // 需要静止采集的时长（秒）
        int init_imu_queue_max_size_ = 2000;  // 缓存队列的条数上限
        double static_vel_th_ = 0.05;         // 判定静止的速度阈值（m/s）
        double max_static_gyro_var_ = 0.2;    // 陀螺噪声方差上限，超过则失败
        double max_static_acce_var_ = 0.05;   // 加计噪声方差上限，超过则失败
        double gravity_norm_ = 9.81;          // 标准重力大小
    };

    // 两个构造函数：无参版委托给带参版（C++ 的委托构造语法）
    // 不写成 "Options options = Options()" 默认实参的原因：C++ 规定嵌套类使用
    // 类内默认成员初始化器时，外层类定义结束前不能生成该默认构造的调用
    StaticIMUInit() : StaticIMUInit(Options()) {}
    explicit StaticIMUInit(const Options& options) : options_(options) {}

    /// 第 1 步：轮速数据到来，只做一件事——更新静止标志
    bool AddOdom(const Odom& odom) {
        if (init_success_) {
            return true;   // 已完成，工作结束
        }

        // 速度低于阈值判为静止
        is_static_ = std::fabs(odom.vel_) < options_.static_vel_th_;

        current_time_ = odom.timestamp_;
        return true;
    }

    /// 第 2 步：IMU 数据到来，静止则入队并计时，攒满 10 秒触发 TryInit
    bool AddIMU(const Imu& imu) {
        if (init_success_) {
            return true;   // 已完成，后续数据不再处理
        }

        // 车在动：清空缓存，等待静止
        if (!is_static_) {
            init_imu_deque_.clear();
            current_time_ = imu.timestamp_;
            return false;
        }

        // 队列为空：这条数据就是静止的开始，记下计时零点
        if (init_imu_deque_.empty()) {
            init_start_time_ = imu.timestamp_;
        }

        // 当前数据进入缓存队列
        init_imu_deque_.push_back(imu);

        // 静止时长足够，触发一次初始化计算
        if (imu.timestamp_ - init_start_time_ > options_.init_time_seconds_) {
            TryInit();
        }

        // 队列超过上限时，从头部丢弃最旧的数据
        while (init_imu_deque_.size() > options_.init_imu_queue_max_size_) {
            init_imu_deque_.pop_front();
        }

        current_time_ = imu.timestamp_;
        return init_success_;
    }

    /// 第 4 步：外界查询初始化是否成功
    bool InitSuccess() const { return init_success_; }

    /// 第 4 步：五个结果出口
    Vec3d GetCovGyro() const { return cov_gyro_; }
    Vec3d GetCovAcce() const { return cov_acce_; }
    Vec3d GetInitBg() const { return init_bg_; }
    Vec3d GetInitBa() const { return init_ba_; }
    Vec3d GetGravity() const { return gravity_; }

   private:
    /// 第 3 步：对缓存队列做统计计算，成功则置位成功标志
    bool TryInit() {
        // 数据太少不做统计
        if (init_imu_deque_.size() < 10) {
            return false;
        }

        // 第一次统计：取 gyro_ 字段，均值即陀螺零偏候选，方差即陀螺噪声
        Vec3d mean_gyro;
        math::ComputeMeanAndCovDiag(init_imu_deque_, mean_gyro, cov_gyro_,
                                    [](const Imu& imu) { return imu.gyro_; });

        // 第二次统计：取 accel_ 字段，均值指向重力反方向
        Vec3d mean_acce;
        math::ComputeMeanAndCovDiag(init_imu_deque_, mean_acce, cov_acce_,
                                    [](const Imu& imu) { return imu.accel_; });

        // 重力方向：加计均值取反，归一化后乘标准重力大小
        gravity_ = -mean_acce / mean_acce.norm() * options_.gravity_norm_;

        // 第三次统计：取 accel_ + gravity_，扣掉重力后，均值才是真正的加计零偏
        math::ComputeMeanAndCovDiag(init_imu_deque_, mean_acce, cov_acce_,
                                    [this](const Imu& imu) { return imu.accel_ + gravity_; });

        // 噪声检查：方差过大说明采集期间存在振动，本次放弃，继续攒数据
        if (cov_gyro_.norm() > options_.max_static_gyro_var_) {
            std::printf("[StaticIMUInit] gyro var too large: %f\n", cov_gyro_.norm());
            return false;
        }
        if (cov_acce_.norm() > options_.max_static_acce_var_) {
            std::printf("[StaticIMUInit] acce var too large: %f\n", cov_acce_.norm());
            return false;
        }

        // 静止时读数均值即零偏
        init_bg_ = mean_gyro;
        init_ba_ = mean_acce;

        std::printf("[StaticIMUInit] init done, bg=(%f, %f, %f), ba=(%f, %f, %f)\n",
                    init_bg_[0], init_bg_[1], init_bg_[2],
                    init_ba_[0], init_ba_[1], init_ba_[2]);
        init_success_ = true;
        return true;
    }

    // ---- 参数与标志 ----
    Options options_;                  // 参数集合
    bool init_success_ = false;        // 初始化成功标志
    bool is_static_ = true;            // 静止标志，由 AddOdom 更新

    // ---- 缓存与计时 ----
    std::deque<Imu> init_imu_deque_;   // 静止数据缓存队列（双端队列：尾部进、头部丢）
    double current_time_ = 0.0;        // 最近一次数据的时间戳
    double init_start_time_ = 0.0;     // 静止计时的零点时刻

    // ---- 结果（TryInit 成功后填写）----
    Vec3d cov_gyro_ = Vec3d::Zero();   // 陀螺噪声方差（三轴各自）
    Vec3d cov_acce_ = Vec3d::Zero();   // 加计噪声方差
    Vec3d init_bg_ = Vec3d::Zero();    // 陀螺零偏
    Vec3d init_ba_ = Vec3d::Zero();    // 加计零偏
    Vec3d gravity_ = Vec3d::Zero();    // 重力向量
};

}  // namespace sad

#endif
