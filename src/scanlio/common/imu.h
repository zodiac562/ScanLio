#ifndef IMU_H
#define IMU_H

#include "scanlio/common/eigen_types.h"

namespace sad {
    struct Imu{
        Imu() = default;
        Imu(double timestamp, const Eigen::Vector3d& gyro, const Eigen::Vector3d& accel)
        : timestamp_(timestamp), gyro_(gyro), accel_(accel) {}

        double timestamp_ = 0.0;
        Eigen::Vector3d gyro_ = Eigen::Vector3d::Zero();
        Eigen::Vector3d accel_ = Eigen::Vector3d::Zero();
    };
    
}

#endif