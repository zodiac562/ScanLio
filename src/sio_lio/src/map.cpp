#include "sio_lio/include/map.h"

namespace sad{

// 新关键帧入库：保存 Frame，并把其点云按位姿转入增量 NDT 地图
void Map::add_keyframe(const std::shared_ptr<Frame>& frame) {
    // 约束（调用方 Frontend 必须满足）：frame->pose_ 已填入世界系位姿。
    // NDT2D::add_scan 内部执行 pose * p，把雷达系点变换到世界系后写入体素；
    // 若 pose_ 仍是默认构造值，点会被投到错误位置，地图随之损坏。
    keyframes_[frame->keyframe_id_] = frame;
    ndt_.add_scan(frame->pose_, frame->pts_);
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
