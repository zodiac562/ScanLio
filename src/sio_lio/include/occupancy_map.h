#ifndef OCCUPANCY_MAP_H_
#define OCCUPANCY_MAP_H_

#include "sio_lio/common/eigen_types.h"

#include <map>
#include <utility>
#include <vector>

namespace sad {

// 2D 占据栅格（射线追踪计数法）
//
// 语义与实现要点：
//   1. 每格记录两个整数计数：hits（射线终点落入该格的次数）、misses（射线途经该格的次数）。
//      占据概率取 p = hits / (hits + misses)。这是「计数比」语义，不是对数几率（log-odds）累加，
//      差别在于多次观测不会把概率推向 0 或 1 的饱和区，便于直接用 0~100 的整数发布。
//   2. 传感器原点取帧位姿的平移分量（本项目的雷达与车体之间为恒等外参）。
//   3. 射线按「半个栅格」为步长采样，沿途每格 misses++，终点格 hits++。
//      采样步长小于栅格边长，保证射线不会跳过格子；步长也不等于边长，避免正好落在格边界
//      时反复计入同一格（相邻采样点在边界附近的归属由 floor 决定，重复计入同一格属可接受误差）。
//   4. 容器用 std::map（有序），不使用 std::unordered_map。原因：遍历顺序影响外部输出
//      （CSV 行序、ROS 消息里的格子顺序）的可复现性，有序容器使同一输入的输出顺序固定。
//
// 与原始 NdtLIO 的差别：原版 OccupancyMap 用 OpenCV 的 cv::Mat 作固定尺寸图像、用 Bresenham
// 直线填充，本实现不依赖 OpenCV（本项目离线验证程序保持纯 C++），栅格按需增长，不设固定边界。
class OccupancyMap {
   public:
    struct Options {
        double resolution_ = 0.1;   // 栅格边长，单位米
    };

    OccupancyMap() = default;
    explicit OccupancyMap(const Options& options) : options_(options) {}

    // 用一帧点云更新栅格。pose 为本帧位姿（车体系→世界系），pts 为车体系点坐标。
    void add_scan(const SE2& pose, const std::vector<Vec2d>& pts);

    // 已写入的格子数量（只统计至少被射线访问过一次的格子）
    size_t num_cells() const { return cells_.size(); }

    // 栅格边长，米
    double resolution() const { return options_.resolution_; }

    // 输出全部已写入格子的索引范围（含端点）。没有任何格子时返回 false。
    bool bounds(int* min_gx, int* min_gy, int* max_gx, int* max_gy) const;

    // 单格占据概率，返回 0~100 的整数；该格从未被观测过时返回 -1（对应 ROS 消息中的未知格）。
    int probability(int gx, int gy) const;

    // 单格原始计数（调试与自检用）。该格不存在时返回 false。
    bool cell_counts(int gx, int gy, int* hits, int* misses) const;

   private:
    struct CellCount {
        int hits = 0;     // 射线终点落入次数
        int misses = 0;   // 射线途经次数
    };

    // 世界系坐标 → 格子索引。用 floor 而非截断，使 (-0.3, 0.3) 分属 -1 与 0 两个格子。
    int grid_index(double v) const;

    Options options_;
    std::map<std::pair<int, int>, CellCount> cells_;
};

}  // namespace sad

#endif  // OCCUPANCY_MAP_H_
