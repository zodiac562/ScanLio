#ifndef MAP_H_
#define MAP_H_

#include "sio_lio/common/eigen_types.h"
#include "frame.h"
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
    };

    explicit Map(const Options& options) : options_(options), ndt_(options.ndt_opts_) {}

    // 新关键帧入库
    void add_keyframe(const std::shared_ptr<Frame>& frame);

    // 当前地图配准
    bool match(std::shared_ptr<Frame> frame, SE2& pose);

    // 关键帧数量
    size_t num_keyframes() const { return keyframes_.size(); }

    // 暴露内部NDT
    NDT2D& ndt() { return ndt_; }

   private:
    Options options_;
    NDT2D ndt_;                                                    // 增量 NDT 地图
    std::map<size_t, std::shared_ptr<Frame>> keyframes_;           // 关键帧库，按 keyframe_id 索引
};

}

#endif
