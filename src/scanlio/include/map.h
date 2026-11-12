#ifndef MAP_H_
#define MAP_H_

#include "scanlio/common/eigen_types.h"
#include "frame.h"
#include "scanlio/include/occupancy_map.h"
#include "scanlio/src/ndt_2d.hpp"

#include <map>
#include <memory>
#include <vector>

namespace sad {


class Map {
   public:
    struct Options {
        // 透传内部NDT2D
        NDT2D::Options ndt_opts_;
        // 透传内部占据栅格
        OccupancyMap::Options occu_opts_;
    };

    explicit Map(const Options& options)
        : options_(options), ndt_(options.ndt_opts_), occu_map_(options.occu_opts_) {}

    // 新关键帧入库
    void add_keyframe(const std::shared_ptr<Frame>& frame);

    // 当前地图配准
    bool match(std::shared_ptr<Frame> frame, SE2& pose);

    // 把一帧点云写入占据栅格（射线追踪）。
    // 与 add_keyframe 分开的原因：NDT 体素只在关键帧写入（写入会改变配准结果），
    // 占据栅格只用于显示，每帧写入可让边角处的栅格更快细化，且不影响任何算法输出。
    // 调用方要求 frame->pose_ 已填入世界系位姿。
    void add_scan_to_occupancy_map(const std::shared_ptr<Frame>& frame);

    // 关键帧数量
    size_t num_keyframes() const { return keyframes_.size(); }

    // 关键帧库的只读访问（按 keyframe_id 升序）。
    // 用途：后端建位姿图时按此顺序取节点初值；回环检测取候选邻域点云；
    // 优化后重建地图时按此顺序重放。
    // 返回的是 Frame 的 shared_ptr，调用方可读全部字段。写 pose_ 请走
    // UpdateKeyframePoses，不要直接改，以免与 NDT 体素不同步。
    const std::map<size_t, std::shared_ptr<Frame>>& keyframes() const { return keyframes_; }

    // 用后端优化结果改写关键帧位姿。poses 的键为 keyframe_id；
    // 库中不存在的键忽略。返回实际改写的帧数。
    // 只改 pose_，不动 pts_；调用方需随后调用 RebuildNdt。
    size_t UpdateKeyframePoses(const std::map<size_t, SE2>& poses);

    // 清空 NDT 体素，按 keyframes() 当前的 pose_ 全量重放。
    // 位姿未变时，重建结果与重建前逐元素一致（专项测试判据）。
    void RebuildNdt();

    // 清空占据栅格，按给定帧序列全量重放。
    // Map 只保存关键帧，而栅格原本是每帧写入的（见 node/scanlio_node.cpp），
    // 故全部帧序列必须由调用方提供；用关键帧序列重建会丢失非关键帧的贡献。
    void RebuildOccupancy(const std::vector<std::shared_ptr<Frame>>& frames);

    // 暴露内部NDT
    NDT2D& ndt() { return ndt_; }
    // 只读版本：供发布全局点云等只读用途
    const NDT2D& ndt() const { return ndt_; }

    // 暴露内部占据栅格
    OccupancyMap& occupancy_map() { return occu_map_; }
    const OccupancyMap& occupancy_map() const { return occu_map_; }

   private:
    Options options_;
    NDT2D ndt_;                                                    // 增量 NDT 地图
    OccupancyMap occu_map_;                                        // 占据栅格（显示用）
    std::map<size_t, std::shared_ptr<Frame>> keyframes_;           // 关键帧库，按 keyframe_id 索引
};

}

#endif
