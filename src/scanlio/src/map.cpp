#include "scanlio/include/map.h"

namespace sad{

// 新关键帧入库：保存 Frame，并把其点云按位姿转入增量 NDT 地图
void Map::add_keyframe(const std::shared_ptr<Frame>& frame) {
    // 约束（调用方 Frontend 必须满足）：frame->pose_ 已填入世界系位姿。
    // NDT2D::add_scan 内部执行 pose * p，把雷达系点变换到世界系后写入体素；
    // 若 pose_ 仍是默认构造值，点会被投到错误位置，地图随之损坏。
    keyframes_[frame->keyframe_id_] = frame;
    ndt_.add_scan(frame->pose_, frame->pts_);
}

// 一帧点云写入占据栅格：按本帧位姿做射线追踪
void Map::add_scan_to_occupancy_map(const std::shared_ptr<Frame>& frame) {
    // 与 add_keyframe 相同的约束：frame->pose_ 必须是世界系位姿。
    // 这里传入的是「去畸变之后」的 pts_，与 NDT 配准使用的是同一份点云。
    occu_map_.add_scan(frame->pose_, frame->pts_);
}

size_t Map::UpdateKeyframePoses(const std::map<size_t, SE2>& poses) {
    size_t updated = 0;
    for (auto& kv : keyframes_) {
        const auto it = poses.find(kv.first);
        if (it == poses.end()) {
            continue;
        }
        kv.second->pose_ = it->second;
        ++updated;
    }
    return updated;
}

void Map::RebuildNdt() {
    ndt_.clear();
    // std::map 按 keyframe_id 升序遍历，与首次建图的写入顺序一致；
    // NDT 的 LRU 链表因此恢复到「最近更新的关键帧在头部」的状态。
    for (const auto& kv : keyframes_) {
        ndt_.add_scan(kv.second->pose_, kv.second->pts_);
    }
}

void Map::RebuildOccupancy(const std::vector<std::shared_ptr<Frame>>& frames) {
    occu_map_.clear();
    for (const auto& f : frames) {
        occu_map_.add_scan(f->pose_, f->pts_);
    }
}

// 用当前地图配准：给定初值 pose，返回配准后位姿与配准是否成功
bool Map::match(std::shared_ptr<Frame> frame, SE2& pose) {
    // pose 传入时为初值（来自 ESKF 预测，世界系位姿）；
    // NDT2D::match 内部以它为起点高斯-牛顿迭代，并原地修改为该帧的配准位姿。
    // 返回值为参与迭代的有效点数，点数不足意味着配准不可信（对齐 stage2 判据）。
    int effective_pts = ndt_.match(pose, frame->pts_);
    return effective_pts >= options_.ndt_opts_.min_pts_num_;
}

}  // namespace sad
