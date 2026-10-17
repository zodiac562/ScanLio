// 阶段 3 ESKF 冒烟测试：手工构造数据，不接 NDT、不读 CSV
// 目标：五个公开接口全部走通 + 数值行为方向正确（三类检查）
//
// 检查 1（静止直测）：p/v/theta 保持近零，P 不发散
// 检查 2（匀速直行）：x 方向速度收敛到真值，位置随时间线性增长
// 检查 3（雷达强观测）：大初始误差被位姿观测拉回
//
// 通过标准：全部 CHECK 输出 PASS；任何 FAIL 则退出码非 0

#include "sio_lio/src/eskf.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>

using namespace sad;

namespace {

const double kDt = 0.01;        // IMU 周期 100Hz
const double kPi = 3.14159265358979323846;

int g_fail_count = 0;

void CheckTrue(const char* name, bool ok) {
    std::printf("[%s] %s\n", ok ? "PASS" : "FAIL", name);
    if (!ok) {
        g_fail_count++;
    }
}

Imu MakeImu(double t, double gyro_z, double ax, double ay) {
    Imu imu;
    imu.timestamp_ = t;
    imu.gyro_ = Vec3d(0.0, 0.0, gyro_z);
    imu.accel_ = Vec3d(ax, ay, 9.81);
    return imu;
}

Odom MakeOdom(double t, double v) {
    Odom odom;
    odom.timestamp_ = t;
    odom.vel_ = v;
    return odom;
}

// 生成 SE2 位姿观测
SE2 MakePose(double x, double y, double theta) {
    return SE2(SO2(theta), Vec2d(x, y));
}

// 检查 1：静止。零 IMU 读数（去偏后为零）、零轮速、位姿观测恒为原点。
// 期望：任意时刻 p 近零、v 近零、theta 近零；P 的迹有界。
void TestStatic() {
    ESKF::Options opt;
    ESKF eskf(opt);
    eskf.SetInitCondition(0.0, Vec2d::Zero());

    double t = 0.0;
    for (int i = 0; i < 5000; ++i) {   // 50 秒静止
        eskf.Predict(MakeImu(t, 0.0, 0.0, 0.0));
        if (i % 10 == 0) {             // 轮速与雷达 10Hz
            eskf.ObserveOdom(MakeOdom(t, 0.0));
            eskf.ObserveLidar(MakePose(0.0, 0.0, 0.0));
        }
        t += kDt;
    }

    SE2 pose = eskf.GetNominalPose();
    double px = pose.translation().x();
    double py = pose.translation().y();
    double theta = pose.so2().log();

    CheckTrue("static: |px| < 1e-3", std::fabs(px) < 1e-3);
    CheckTrue("static: |py| < 1e-3", std::fabs(py) < 1e-3);
    CheckTrue("static: |theta| < 1e-3", std::fabs(theta) < 1e-3);
    std::printf("       static end pose = (%.6f, %.6f, %.6f)\n", px, py, theta);
}

// 检查 2：匀速直行。加计给零（速度不变即无加速度，由轮速直接给速度观测），
// 轮速 2 m/s，位姿观测按真值轨迹给（模拟完美 NDT）。
// 期望：位置跟踪真值，x 速度收敛到 2，theta 保持 0。
void TestConstantVelocity() {
    ESKF::Options opt;
    ESKF eskf(opt);
    eskf.SetInitCondition(0.0, Vec2d::Zero());

    const double v_true = 2.0;
    double t = 0.0;
    for (int i = 0; i < 5000; ++i) {   // 50 秒，真值位置 = v_true * t
        eskf.Predict(MakeImu(t, 0.0, 0.0, 0.0));
        if (i % 10 == 0) {
            eskf.ObserveOdom(MakeOdom(t, v_true));
            eskf.ObserveLidar(MakePose(v_true * t, 0.0, 0.0));
        }
        t += kDt;
    }

    SE2 pose = eskf.GetNominalPose();
    double px = pose.translation().x();
    double theta = pose.so2().log();

    CheckTrue("cruise: |px - 100| < 0.05", std::fabs(px - v_true * 50.0) < 0.05);
    CheckTrue("cruise: |theta| < 1e-3", std::fabs(theta) < 1e-3);
    std::printf("       cruise end px = %.6f (true 100)\n", px);
}

// 检查 3：大初始误差拉回。位姿观测恒为 (1, 0.5, 0.2)，
// 状态从原点出发，期望被观测逐步拉到观测值附近。
void TestPullBack() {
    ESKF::Options opt;
    ESKF eskf(opt);
    eskf.SetInitCondition(0.0, Vec2d::Zero());

    const double tx = 1.0, ty = 0.5, tth = 0.2;
    double t = 0.0;
    for (int i = 0; i < 1000; ++i) {   // 10 秒
        eskf.Predict(MakeImu(t, 0.0, 0.0, 0.0));
        if (i % 10 == 0) {
            eskf.ObserveOdom(MakeOdom(t, 0.0));
            eskf.ObserveLidar(MakePose(tx, ty, tth));
        }
        t += kDt;
    }

    SE2 pose = eskf.GetNominalPose();
    double px = pose.translation().x();
    double py = pose.translation().y();
    double theta = pose.so2().log();

    CheckTrue("pull: |px - 1.0| < 0.02", std::fabs(px - tx) < 0.02);
    CheckTrue("pull: |py - 0.5| < 0.02", std::fabs(py - ty) < 0.02);
    CheckTrue("pull: |theta - 0.2| < 0.02", std::fabs(theta - tth) < 0.02);
    std::printf("       pull end pose = (%.6f, %.6f, %.6f)\n", px, py, theta);
}

}  // namespace

int main() {
    std::printf("===== ESKF smoke test =====\n");
    TestStatic();
    TestConstantVelocity();
    TestPullBack();
    std::printf("===== %s (fail=%d) =====\n", g_fail_count == 0 ? "ALL PASS" : "HAS FAIL", g_fail_count);
    return g_fail_count == 0 ? 0 : 1;
}
