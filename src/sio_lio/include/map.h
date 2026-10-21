#ifndef MAP_H_
#define MAP_H_

#include "sio_lio/common/eigen_types.h"
#include "frame.h"
#include "sio_lio/include/occupancy_map.h"
#include "sio_lio/src/ndt_2d.hpp"

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
