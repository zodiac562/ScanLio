#include "scanlio/include/occupancy_map.h"

#include <cmath>
#include <iterator>

namespace sad {

int OccupancyMap::grid_index(double v) const {
    return static_cast<int>(std::floor(v / options_.resolution_));
}

void OccupancyMap::add_scan(const SE2& pose, const std::vector<Vec2d>& pts) {
    const double res = options_.resolution_;
    if (res <= 0.0) {
        return;   // 非法分辨率：不写入任何格子（配置层已拦截，这里只做防御）
    }

    // 射线起点：本帧位姿的平移分量（车体/雷达在世界系中的位置）
    const Vec2d origin = pose.translation();

    for (const auto& p : pts) {
        // 射线终点：点从车体系变换到世界系
        const Vec2d hit = pose * p;
        const Vec2d d = hit - origin;
        const double len = d.norm();

        // 终点距起点不足一格：不做任何记录。
        // 此时「起点格」与「终点格」可能相同，记录会把传感器所在格标成占据或空闲，
        // 两者都不成立，故直接跳过。
        if (len < res) {
            continue;
        }

        // 采样步长取半格，保证相邻采样点不会跨越一整格
        const int steps = static_cast<int>(len / (res * 0.5));
        const Vec2d dir = d / len;

        // 沿途格子（不含终点）记为穿越
        for (int s = 1; s < steps; ++s) {
            const Vec2d q = origin + dir * (res * 0.5 * static_cast<double>(s));
            cells_[{grid_index(q.x()), grid_index(q.y())}].misses++;
        }

        // 终点格记为命中
        cells_[{grid_index(hit.x()), grid_index(hit.y())}].hits++;
    }
}

bool OccupancyMap::bounds(int* min_gx, int* min_gy, int* max_gx, int* max_gy) const {
    if (cells_.empty()) {
        return false;
    }
    // std::map 按 key 的字典序排列，pair 先比 first 再比 second。
    // 若 key 为 (gx, gy)，则 begin() 是 gx 最小的那一组中 gy 最小的元素，
    // rbegin() 是 gx 最大的那一组中 gy 最大的元素。直接取其分量已覆盖全部格子的
    // 最小/最大值，但严格来说 begin()->first 只保证 first 最小，second 的最小值需另行遍历。
    int mnx = cells_.begin()->first.first;
    int mxx = std::prev(cells_.end())->first.first;
    int mny = cells_.begin()->first.second;
    int mxy = mny;
    for (const auto& kv : cells_) {
        if (kv.first.second < mny) mny = kv.first.second;
        if (kv.first.second > mxy) mxy = kv.first.second;
    }
    if (min_gx) *min_gx = mnx;
    if (min_gy) *min_gy = mny;
    if (max_gx) *max_gx = mxx;
    if (max_gy) *max_gy = mxy;
    return true;
}

int OccupancyMap::probability(int gx, int gy) const {
    auto it = cells_.find({gx, gy});
    if (it == cells_.end()) {
        return -1;   // 未知格
    }
    const int hits = it->second.hits;
    const int total = hits + it->second.misses;
    if (total <= 0) {
        return -1;
    }
    // 计数比 × 100，四舍五入到整数
    return static_cast<int>(std::lround(100.0 * static_cast<double>(hits) / static_cast<double>(total)));
}

bool OccupancyMap::cell_counts(int gx, int gy, int* hits, int* misses) const {
    auto it = cells_.find({gx, gy});
    if (it == cells_.end()) {
        return false;
    }
    if (hits) *hits = it->second.hits;
    if (misses) *misses = it->second.misses;
    return true;
}

}  // namespace sad
