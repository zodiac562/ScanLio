#ifndef FRAME_H_
#define FRAME_H_

#include "scanlio/common/eigen_types.h"
#include <vector>

namespace sad{

    // 雷达帧
    struct Frame{
        size_t id_ =0;            // 帧编号
        size_t keyframe_id_ = 0;   // 关键帧编号
        double timestamp_ =0.0;   // 扫描参考时刻（LaserScan.header.stamp；实测为扫描结束）
        SE2 pose_;                // 本帧位姿
        std::vector<Vec2d> pts_;   // 点云：雷达系直角坐标
        // 逐点采集时刻（秒），与 pts_ 一一对应。由导出层按
        // header.stamp - time_increment*(n_ranges-1-idx) 直接算出，
        // 不再依赖「匀速旋转、点均匀分布」这类假设。
        std::vector<double> pts_time_;
    };
       
}

#endif