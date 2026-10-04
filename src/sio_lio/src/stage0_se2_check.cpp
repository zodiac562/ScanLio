//
// 阶段0 环境验证程序
// 目的：验证 sio_lio 包中 Sophus 的编译与链接正确，同时数值核对第一课练习3
// 内容：构造 T1、T2，计算 T1^{-1} * T2，提取 theta 与 t，并与手算公式对照
//
#include <cmath>
#include <cstdio>

#include "sio_lio/common/eigen_types.h"

int main() {
    const double deg2rad = M_PI / 180.0;

    // 练习3的已知量：theta1 = 30°, t1 = (1, 0)；theta2 = 90°, t2 = (0, 1)
    const double theta1 = 30.0 * deg2rad;
    const Vec2d t1(1.0, 0.0);
    const double theta2 = 90.0 * deg2rad;
    const Vec2d t2(0.0, 1.0);

    SE2 T1(theta1, t1);
    SE2 T2(theta2, t2);

    // 相对变换，对应 ndtlio frontend.cpp 中
    // delta_pose_with_kf_ = last_keyframe_->pose_.inverse() * current_frame_->pose_;
    SE2 T12 = T1.inverse() * T2;

    // 从结果中提取 theta 与 t
    // log() 返回李代数元素 (v1, v2, theta)，其中 v 是加权平移，不是矩阵中的 t
    // 因此平移部分从矩阵块中取
    const double theta_rel = T12.log()[2];
    const Vec2d t_rel = T12.translation();

    // 手算公式对照：theta_rel = theta2 - theta1，t_rel = R(-theta1) * (t2 - t1)
    const double theta_manual = theta2 - theta1;
    const Mat2d R_neg1 = Sophus::SO2d(-theta1).matrix();
    const Vec2d t_manual = R_neg1 * (t2 - t1);

    std::printf("=== T1^-1 * T2 (Sophus) ===\n");
    std::printf("theta_rel = %.6f rad = %.4f deg\n", theta_rel, theta_rel / deg2rad);
    std::printf("t_rel     = (%.6f, %.6f)\n", t_rel[0], t_rel[1]);

    std::printf("=== manual formula ===\n");
    std::printf("theta_rel = %.6f rad = %.4f deg\n", theta_manual, theta_manual / deg2rad);
    std::printf("t_rel     = (%.6f, %.6f)\n", t_manual[0], t_manual[1]);

    // exp/log 互逆验证：SE2::exp(T12.log()) 应还原 T12
    const double roundtrip_err = (SE2::exp(T12.log()).matrix() - T12.matrix()).norm();
    std::printf("exp(log(T12)) vs T12: ||diff|| = %.2e\n", roundtrip_err);

    const bool ok = std::fabs(theta_rel - theta_manual) < 1e-9 &&
                    (t_rel - t_manual).norm() < 1e-9 &&
                    roundtrip_err < 1e-9;
    std::printf("%s\n", ok ? "STAGE0 CHECK PASSED" : "STAGE0 CHECK FAILED");
    return ok ? 0 : 1;
}
