#include "frontend.h"
#include "frame.h"
#include "map.h"

namespace sad{

Frontend::Frontend(const Options& options) : options_(options) {
    eskf_ = std::make_shared<ESKF>();
    map_  = std::make_shared<Map>(Map::Options());
}

// IMU 入口：交给 ESKF 预测，并把预测位姿存入缓冲（去畸变用）
void Frontend::ProcessImu(const Imu& imu) {
    eskf_->Predict(imu);
    imu_buffer_.push_back({imu.timestamp_, eskf_->GetNominalPose()});

    // 缓冲容量守卫：超限时丢最旧的（时间太老的位姿对当前帧去畸变无用）
    if (imu_buffer_.size() > options_.imu_buffer_size_) {
        imu_buffer_.pop_front();
    }
}

// 轮速入口：交给 ESKF 观测
void Frontend::ProcessOdom(const Odom& odom) {
    eskf_->ObserveOdom(odom);
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
    UndistortScan(frame);

    // ---- 第 3 步：初值推算 ----
    // 从缓冲中取「时间上最接近本帧时刻」的 ESKF 预测位姿作为初值。
    // 0.1s 内 IMU 已推算约 10 步，初值比直接用上一帧位姿更贴近当前真实位姿。
    SE2 pose = imu_buffer_.back().second;

    // ---- 第 4 步：配准 ----
    bool ok = map_->match(frame, pose);
    if (!ok) {
        // 配准失败：保留 ESKF 预测位姿（不跳变），本帧不入图
        pose = imu_buffer_.back().second;
        return false;
    }

    // ---- 第 5 步：修正与关键帧判定 ----
    // 配准结果喂回 ESKF（雷达观测修正），滤波器输出位姿更新。
    eskf_->ObserveLidar(pose);
    
    frame->pose_ = pose;

    // 关键帧判定：与上一关键帧比平移与转角，达标才入库
    SE2 delta = last_keyframe_pose_.inverse() * pose;
    bool is_keyframe =
        delta.translation().norm() > options_.keyframe_dist_th_ ||
        std::fabs(delta.so2().log()) > options_.keyframe_rot_th_;

    if (is_keyframe) {
        map_->add_keyframe(frame);
        last_keyframe_pose_ = pose;
    }

    return true;
}

// 当前位姿（ESKF 名义状态，随时可读）
SE2 Frontend::GetPose() const {
    return eskf_->GetNominalPose();
}

// 静态初始化透传：阶段 1 的零偏结果进入 ESKF
void Frontend::SetInitCondition(double bg, const Vec2d& ba) {
    eskf_->SetInitCondition(bg, ba);
}

// 去畸变实现：
//   1. 帧内第 i 个点的采样时刻 t_i = 帧起始 + i × 单束周期
//      （单束周期由「上一帧与本帧的时间差 ÷ 点数」推算，雷达匀速旋转）
//   2. 在 imu_buffer_ 中找 t_i 附近的两条位姿，线性插值出该时刻的车体位姿
//   3. 点补偿：p_out = T_end^-1 * T_i * p_raw
//      （把点从「t_i 时刻的车体系」搬到「帧尾时刻的车体系」）
bool Frontend::UndistortScan(const std::shared_ptr<Frame>& frame) {
    // 缓冲不足两条无法插值
    if (imu_buffer_.size() < 2) {
        return false;
    }

    const size_t n = frame->pts_.size();
    if (n == 0) {
        return false;
    }

    // 帧起始时刻（timestamp_ 语义）与帧结束时刻
    const double t_start = frame->timestamp_;
    const double t_end   = imu_buffer_.back().first;

    // 单束周期：帧时长 ÷ 点数
    const double dt_beam = (t_end - t_start) / (double)n;
    if (dt_beam <= 0.0) {
        return false;
    }

    // 帧尾时刻的车体位姿（补偿目标基准）
    const SE2& T_end = imu_buffer_.back().second;

    // 逐点补偿
    for (size_t i = 0; i < n; ++i) {
        const double t_i = t_start + dt_beam * (double)i;

        // 在缓冲中找 t_i 的插值区间：prev.t <= t_i <= next.t
        auto next = imu_buffer_.begin();
        auto prev = next;
        while (next != imu_buffer_.end() && next->first < t_i) {
            prev = next;
            ++next;
        }
        if (next == imu_buffer_.begin() || next == imu_buffer_.end()) {
            continue;   // t_i 超出缓冲覆盖范围，该点不补偿（保持原值）
        }

        // 线性插值：位置按比例，角度按差值的比例（短区间内角度无周期跳变问题，
        // 因相邻 IMU 位姿差远小于 π）
        const double alpha = (t_i - prev->first) / (next->first - prev->first);
        Vec2d trans = prev->second.translation() * (1.0 - alpha) +
                      next->second.translation() * alpha;
        double theta = prev->second.so2().log() * (1.0 - alpha) +
                       next->second.so2().log() * alpha;
        SE2 T_i(SO2(theta), trans);

        // 补偿：从 t_i 时刻车体系 → 世界系 → 帧尾时刻车体系
        frame->pts_[i] = T_end.inverse() * T_i * frame->pts_[i];
    }
    return true;
}

}  // namespace sad
