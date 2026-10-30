#include "frontend.h"
#include "frame.h"
#include "map.h"

namespace sad{

Frontend::Frontend(const Options& options) : options_(options) {
    // 用 options 中的嵌套参数构造子系统（此前用默认值，外部无法配置）
    // 滤波器按 filter_type_ 二选一，另一侧保持空指针，供各入口判别。
    if (options_.filter_type_ == Options::FilterType::kIeskf) {
        ieskf_ = std::make_shared<IESKF2D>(options_.ieskf_opts_);
    } else {
        eskf_ = std::make_shared<ESKF>(options_.eskf_opts_);
    }
    map_  = std::make_shared<Map>(options_.map_opts_);
}

// IMU 入口：交给当前滤波器预测，并把预测位姿存入缓冲（去畸变用）
void Frontend::ProcessImu(const Imu& imu) {
    if (ieskf_) {
        ieskf_->Predict(imu);
    } else {
        eskf_->Predict(imu);
    }
    imu_buffer_.push_back({imu.timestamp_, NominalPose()});

    // 缓冲容量守卫：超限时丢最旧的（时间太老的位姿对当前帧去畸变无用）
    if (imu_buffer_.size() > options_.imu_buffer_size_) {
        imu_buffer_.pop_front();
    }
}

// 轮速入口：交给当前滤波器观测
void Frontend::ProcessOdom(const Odom& odom) {
    if (ieskf_) {
        ieskf_->ObserveOdom(odom);
    } else {
        eskf_->ObserveOdom(odom);
    }
}

// 当前滤波器给出的名义位姿（两条路径二选一）
SE2 Frontend::NominalPose() const {
    return ieskf_ ? ieskf_->GetNominalPose() : eskf_->GetNominalPose();
}

// 雷达入口：五步管线
//   1. 首帧分支：直接建图，位姿取恒等（地图坐标系由第一帧定义）
//   2. 去畸变：用 IMU 位姿缓冲，把帧内各点补偿到统一时刻
//   3. 初值推算：从缓冲中取「最接近帧时刻」的 ESKF 预测位姿
//   4. 配准：map_.match 以初值为起点，输出配准位姿
//   5. 修正：配准结果喂回 ESKF（ObserveLidar），并做关键帧判定
bool Frontend::ProcessScan(const std::shared_ptr<Frame>& frame) {
    // ---- 第 1 步：首帧 ----
    // 地图坐标系由第一帧定义：位姿直接取恒等变换，点云入图后返回。
    // 此时无 IMU 历史（或刚起步），不去畸变、不修正、不判关键帧。
    if (first_scan_) {
        frame->pose_ = SE2();                       // 恒等位姿
        map_->add_keyframe(frame);
        last_keyframe_pose_ = frame->pose_;
        first_scan_ = false;
        return true;
    }

    // ---- 第 2 步：去畸变 ----
    // 帧内各点采样时刻不同（0.1s 内车在动），用 IMU 预测位姿把各点
    // 补偿到统一时刻，否则配准与建图带系统误差。
    if (options_.enable_undistort_) {
        UndistortScan(frame);
    }

    // ---- 第 3 步：初值推算 ----
    // 从缓冲中取「时间上最接近本帧时刻」的预测位姿作为初值。
    // 0.1s 内 IMU 已推算约 10 步，初值比直接用上一帧位姿更贴近当前真实位姿。
    SE2 pose = imu_buffer_.back().second;
    SE2 map_pose;

    if (ieskf_) {
        // ---- 第 4/5 步（紧耦合）：配准与滤波合并 ----
        // 先用当前位姿探一次有效点数：不足则直接失败，不改动滤波器状态。
        // 这一步避免「观测已注入但配准不可信」的状态不一致 —— 紧耦合无法回滚。
        Mat8d probe_H = Mat8d::Zero();
        Vec8d probe_r = Vec8d::Zero();
        const int probe_pts =
            map_->ndt().compute_residual_and_jacobians(pose, frame->pts_, probe_H, probe_r);
        if (probe_pts < options_.map_opts_.ndt_opts_.min_pts_num_) {
            return false;
        }

        // 观测函数只累加；迭代、状态注入、协方差更新都在滤波器内部完成。
        auto obs = [this, &frame](const SE2& input_pose, Mat8d& H, Vec8d& r) {
            map_->ndt().compute_residual_and_jacobians(input_pose, frame->pts_, H, r);
        };
        if (!ieskf_->UpdateUsingCustomObserve(obs)) {
            return false;
        }
        pose = ieskf_->GetNominalPose();
        // 紧耦合下不存在「配准位姿」与「滤波位姿」两个量，二者是同一个，
        // 因此 use_filtered_pose_for_map_ 对该路径不产生分支。
        map_pose = pose;
    } else {
        // ---- 第 4 步（松耦合）：NDT 独立配准 ----
        const bool ok = map_->match(frame, pose);
        if (!ok) {
            // 配准失败：保留预测位姿（不跳变），本帧不入图
            pose = imu_buffer_.back().second;
            return false;
        }

        // ---- 第 5 步：修正 ----
        // 配准结果喂回 ESKF（雷达观测修正），滤波器输出位姿更新。
        eskf_->ObserveLidar(pose);

        // 入图位姿来源（Options::use_filtered_pose_for_map_）：
        //   true  -> 滤波后位姿（对齐原版 ndtlio frontend.cpp:35）
        //   false -> 配准位姿（NDT 输出，未与预测融合）
        map_pose = options_.use_filtered_pose_for_map_ ? eskf_->GetNominalPose() : pose;
    }

    // 关键帧判定与 last_keyframe_pose_ 使用同一个位姿，保证判定基准与入库基准一致。
    frame->pose_ = map_pose;

    // 关键帧判定：与上一关键帧比平移与转角，达标才入库
    SE2 delta = last_keyframe_pose_.inverse() * map_pose;
    bool is_keyframe =
        delta.translation().norm() > options_.keyframe_dist_th_ ||
        std::fabs(delta.so2().log()) > options_.keyframe_rot_th_;

    if (is_keyframe) {
        map_->add_keyframe(frame);
        last_keyframe_pose_ = map_pose;
    }

    return true;
}

// 当前位姿（当前滤波器的名义状态，随时可读）
SE2 Frontend::GetPose() const {
    return NominalPose();
}

// 当前后验协方差（两条路径状态维度相同，直接透传）
Mat8d Frontend::GetP() const {
    return ieskf_ ? ieskf_->GetP() : eskf_->GetP();
}

// 静态初始化透传：阶段 1 的零偏结果进入当前滤波器
void Frontend::SetInitCondition(double bg, const Vec2d& ba) {
    if (ieskf_) {
        ieskf_->SetInitCondition(bg, ba);
    } else {
        eskf_->SetInitCondition(bg, ba);
    }
}

// 去畸变实现：
//   1. 逐点采集时刻直接取自 frame->pts_time_（导出层按消息自带的
//      time_increment 与原始 ranges 索引算出），本函数不做任何均匀性假设。
//   2. 基准时刻 t_ref = frame->timestamp_（LaserScan.header.stamp，扫描结束）。
//   3. 在 imu_buffer_ 中找 t_i 所在区间，位置线性插值、角度先折短弧再加权。
//   4. 点补偿：p_out = T_ref^-1 * T_i * p_raw
//      （把点从「t_i 时刻的车体系」搬到「t_ref 时刻的车体系」）
bool Frontend::UndistortScan(const std::shared_ptr<Frame>& frame) {
    // 缓冲不足两条无法插值
    if (imu_buffer_.size() < 2) {
        return false;
    }

    const size_t n = frame->pts_.size();
    if (n == 0) {
        return false;
    }

    // 逐点时刻必须与点云一一对应，否则说明导出层未提供时间信息
    if (frame->pts_time_.size() != n) {
        return false;
    }

    // 补偿目标基准：缓冲最新位姿。其时刻距 t_ref 不超过一个 IMU 周期
    // （约 0.01 s，实测中位数 5 ms），作为 t_ref 时刻位姿的近似。
    const SE2& T_ref = imu_buffer_.back().second;
    const double t_buf_end = imu_buffer_.back().first;
    const SE2 T_ref_inv = T_ref.inverse();

    // 逐点补偿
    for (size_t i = 0; i < n; ++i) {
        const double t_i = frame->pts_time_[i];

        // t_i 晚于缓冲末端：用最新位姿代替（偏差至多一个 IMU 周期），
        // 此时 T_i = T_ref，该点补偿后退化为原值
        if (t_i >= t_buf_end) {
            frame->pts_[i] = T_ref_inv * (T_ref * frame->pts_[i]);
            continue;
        }

        // 在缓冲中找 t_i 的插值区间：prev.t <= t_i < next.t
        auto next = imu_buffer_.begin();
        auto prev = next;
        while (next != imu_buffer_.end() && next->first <= t_i) {
            prev = next;
            ++next;
        }
        if (next == imu_buffer_.begin() || next == imu_buffer_.end()) {
            continue;   // t_i 早于缓冲最早记录，该点不补偿（保持原值）
        }

        const double dt_pair = next->first - prev->first;
        if (dt_pair <= 0.0) {
            continue;   // 时间戳重复，无法插值
        }

        // 位置按比例插值；角度先折到短弧再加权（SO2::log 值域 (-π, π]，
        // 累计朝向跨越 ±π 时直接加权会把朝向算成反向）
        const double alpha = (t_i - prev->first) / dt_pair;
        Vec2d trans = prev->second.translation() * (1.0 - alpha) +
                      next->second.translation() * alpha;
        const double th_prev = prev->second.so2().log();
        double dth = next->second.so2().log() - th_prev;
        while (dth > M_PI) dth -= 2.0 * M_PI;
        while (dth <= -M_PI) dth += 2.0 * M_PI;
        const double theta = th_prev + dth * alpha;
        const SE2 T_i(SO2(theta), trans);

        // 补偿：从 t_i 时刻车体系 → 世界系 → t_ref 时刻车体系
        frame->pts_[i] = T_ref_inv * T_i * frame->pts_[i];
    }
    return true;
}

}  // namespace sad
