// sio_lio 的 ROS 2 节点：把 Frontend（ESKF + NDT + Map）接到 ROS 话题上。
//
// 数据流（与原版 ndtlio ros2node/ndt_lio_node.cpp 一致）：
//   /imu      sensor_msgs/Imu       → Frontend::ProcessImu  → ESKF 预测
//   /robotvel origincar_msg/Data    → Frontend::ProcessOdom → ESKF 轮速观测
//   /scan     sensor_msgs/LaserScan → Frame → Frontend::ProcessScan → 配准 + 修正
// 输出：
//   /trajectory  nav_msgs/Path        每成功配准一帧追加一个位姿（关键帧与非关键帧都发）
//   /tf          map → base           每成功配准一帧发布一次
//   /tf_static   base → laser / imu   定时重发（2 s），与驱动侧坐标系保持同一假设
//
// 调度方式（关键设计）：三个话题的消息先进入按（时间戳, 类型）排序的缓冲，再按
// 时间顺序处理，而不是「谁的回调先被触发就先处理谁」。
//   原因：ROS 2 的单线程执行器只保证回调串行，不保证三个订阅之间的触发顺序与
//   消息时间戳顺序一致。实测同一份数据包两次回放，按回调顺序处理会让终点位姿
//   相差约 0.1 m（配准初值来自 ESKF 预测，差一条 IMU 就会改变配准初值）。
//   排序后处理顺序只由消息时间戳决定，同一份数据的输出可复现。
//   缓冲深度由参数 reorder_margin 控制（默认 0.02 s，约两条 IMU）：队列中时间
//   最早的消息，只有等到「已收到的最大时间戳」比它晚 reorder_margin 以上才处理，
//   以此等待可能迟到但时间更早的消息。代价是引入 reorder_margin 的固定时延。
//
// 线程模型：单线程执行器（rclcpp::spin）。所有处理都在同一个线程，Frontend 内部
// 状态只被一个线程访问，不需要加锁。
//
// 本节点不输出占据栅格（/map）与全局体素点云：
//   核心 Map/NDT2D 没有暴露体素坐标与占据栅格的只读接口，占据栅格模块目前只
//   存在于 test/stage4_map_gen.cpp（未进核心）。补这两项需要先改 Zodiac 手写
//   的 NDT2D/Map 接口，故本节点暂不实现，已记入 doc/项目挂起/。

#include "sio_lio/common/imu.h"
#include "sio_lio/common/odom.h"
#include "sio_lio/common/static_imu_init.h"
#include "sio_lio/include/config.h"
#include "sio_lio/include/frame.h"
#include "sio_lio/include/frontend.h"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <origincar_msg/msg/data.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace {

// ros time → 秒（double）。雷达与 IMU 的 header.stamp 均为传感器时基（Unix 秒）。
double StampToSec(const builtin_interfaces::msg::Time& stamp) {
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

builtin_interfaces::msg::Time SecToStamp(double t) {
    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<int32_t>(std::floor(t));
    stamp.nanosec = static_cast<uint32_t>((t - std::floor(t)) * 1e9);
    return stamp;
}

// 2D 位姿 → TF（绕 z 轴旋转 theta）
geometry_msgs::msg::TransformStamped MakeTransform(const std::string& parent,
                                                   const std::string& child, double stamp,
                                                   const SE2& pose) {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id = parent;
    tf.child_frame_id = child;
    tf.header.stamp = SecToStamp(stamp);
    tf.transform.translation.x = pose.translation().x();
    tf.transform.translation.y = pose.translation().y();
    tf.transform.translation.z = 0.0;
    const double theta = pose.so2().log();
    tf.transform.rotation.x = 0.0;
    tf.transform.rotation.y = 0.0;
    tf.transform.rotation.z = std::sin(theta * 0.5);
    tf.transform.rotation.w = std::cos(theta * 0.5);
    return tf;
}

}  // namespace

class SioLioNode : public rclcpp::Node {
   public:
    SioLioNode() : rclcpp::Node("sio_lio_node") {
        // ---- 1. 参数声明 ----
        // 空字符串表示「用安装目录下的 share/sio_lio/config/sio_lio.yaml」
        config_file_ = this->declare_parameter<std::string>("config_file", "");
        const std::string scan_topic = this->declare_parameter<std::string>("scan_topic", "/scan");
        const std::string imu_topic = this->declare_parameter<std::string>("imu_topic", "/imu");
        const std::string odom_topic =
            this->declare_parameter<std::string>("odom_topic", "/robotvel");
        const std::string traj_topic =
            this->declare_parameter<std::string>("trajectory_topic", "/trajectory");
        map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
        base_frame_ = this->declare_parameter<std::string>("base_frame", "base");
        laser_frame_ = this->declare_parameter<std::string>("laser_frame", "laser");
        imu_frame_ = this->declare_parameter<std::string>("imu_frame", "imu");
        publish_tf_ = this->declare_parameter<bool>("publish_tf", true);
        publish_static_tf_ = this->declare_parameter<bool>("publish_static_tf", true);
        use_static_init_ = this->declare_parameter<bool>("use_static_init", false);
        const double static_init_seconds =
            this->declare_parameter<double>("static_init_seconds", 10.0);
        reorder_margin_ = this->declare_parameter<double>("reorder_margin", 0.02);

        // ---- 2. 配置加载（失败即终止，不静默回退到默认值）----
        const std::string path = ResolveConfigPath();
        std::string err;
        if (!cfg_.LoadFromYaml(path, &err)) {
            RCLCPP_FATAL(this->get_logger(), "配置加载失败：%s\n原因：%s", path.c_str(), err.c_str());
            throw std::runtime_error("sio_lio 配置加载失败");
        }
        RCLCPP_INFO(this->get_logger(), "配置来源：%s", path.c_str());

        // ---- 3. 管线构造 ----
        frontend_ = std::make_unique<sad::Frontend>(cfg_.ToFrontendOptions());
        if (use_static_init_) {
            sad::StaticIMUInit::Options opt;
            opt.init_time_seconds_ = static_init_seconds;
            imu_init_ = std::make_unique<sad::StaticIMUInit>(opt);
            RCLCPP_INFO(this->get_logger(),
                        "静止初始化已启用：需轮速 <%.2f m/s 且持续 %.1f s，期间雷达数据被丢弃",
                        opt.static_vel_th_, opt.init_time_seconds_);
        } else {
            // 旁路：直接使用 yaml 中阶段 1 标定的零偏，收到数据即开始计算
            frontend_->SetInitCondition(cfg_.init_bg, cfg_.init_ba);
            init_ready_ = true;
            RCLCPP_INFO(this->get_logger(), "静止初始化已旁路：bg=%.6f ba=(%.4f, %.4f)",
                        cfg_.init_bg, cfg_.init_ba.x(), cfg_.init_ba.y());
        }

        // ---- 4. 发布器与订阅器 ----
        trajectory_.header.frame_id = map_frame_;
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>(traj_topic, rclcpp::QoS(10));

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        tf_static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, rclcpp::QoS(100),
            std::bind(&SioLioNode::ImuCallback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<origincar_msg::msg::Data>(
            odom_topic, rclcpp::QoS(100),
            std::bind(&SioLioNode::OdomCallback, this, std::placeholders::_1));
        // 雷达队列深度 1：一帧 0.1 s，积压的旧帧对当前位姿没有意义
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic, rclcpp::QoS(1),
            std::bind(&SioLioNode::ScanCallback, this, std::placeholders::_1));

        // ---- 5. 静态坐标定时重发 ----
        if (publish_static_tf_) {
            static_tf_timer_ = this->create_wall_timer(
                std::chrono::seconds(2), std::bind(&SioLioNode::PublishStaticTf, this));
        }

        RCLCPP_INFO(this->get_logger(), "订阅：%s / %s / %s；轨迹发布：%s；排序缓冲 %.3f s",
                    imu_topic.c_str(), odom_topic.c_str(), scan_topic.c_str(), traj_topic.c_str(),
                    reorder_margin_);
    }

    ~SioLioNode() override {
        // 冲刷排序缓冲：末尾 reorder_margin 时间段内的消息在正常运行时不会被处理，
        // 退出时按时间顺序补处理，使统计与离线管线可比。
        FlushPending();
        RCLCPP_INFO(this->get_logger(),
                    "退出统计：IMU %zu 条，轮速 %zu 条，雷达帧 %zu，配准成功 %zu，配准失败 %zu，"
                    "有效点不足跳过 %zu",
                    imu_count_, odom_count_, frame_total_, frame_ok_, frame_fail_, frame_skipped_);
    }

   private:
    enum Kind { kImu = 0, kOdom = 1, kScan = 2 };

    struct Pending {
        int kind = kImu;
        double t = 0.0;
        sensor_msgs::msg::Imu::SharedPtr imu;
        origincar_msg::msg::Data::SharedPtr odom;
        sensor_msgs::msg::LaserScan::SharedPtr scan;
    };

    // 配置路径解析：命令行/launch 传入优先；否则用安装目录下的 share/sio_lio/config/
    std::string ResolveConfigPath() {
        if (!config_file_.empty()) {
            return config_file_;
        }
        try {
            // 用 get_package_share_directory 而非 Lyrical 新增的 get_package_share_path：
            // 后者在板载 Humble 上不存在，本仓库需要在两个发行版上都能构建。
            // 该函数在 Lyrical 中被标记弃用，仅产生编译告警，不影响功能。
            const std::string share = ament_index_cpp::get_package_share_directory("sio_lio");
            return share + "/config/sio_lio.yaml";
        } catch (const std::exception& e) {
            RCLCPP_FATAL(this->get_logger(),
                         "无法定位 sio_lio 的共享目录（%s）；请用参数 config_file 显式指定配置路径",
                         e.what());
            throw;
        }
    }

    // ---------------- 三个订阅回调：只做入队 ----------------

    void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        ++imu_count_;
        const double t = StampToSec(msg->header.stamp);
        if (!init_ready_) {
            FeedInitImu(msg, t);
            return;
        }
        latest_sensor_time_ = t;
        has_sensor_time_ = true;
        Pending p;
        p.kind = kImu;
        p.t = t;
        p.imu = msg;
        Enqueue(p);
    }

    void OdomCallback(const origincar_msg::msg::Data::SharedPtr msg) {
        ++odom_count_;
        if (!init_ready_) {
            FeedInitOdom(msg);
            return;
        }
        // Data.msg 没有 header，无法取得传感器时刻。此处用「最近一条 IMU 的时间戳」
        // 作为它的时间，使排序缓冲全部工作在同一时基上（若用节点墙钟，回放加速时
        // 轮速会被排到很久以前，破坏时间顺序）。ESKF 的轮速观测本身不使用时间戳。
        Pending p;
        p.kind = kOdom;
        p.t = has_sensor_time_ ? latest_sensor_time_ : this->now().seconds();
        p.odom = msg;
        Enqueue(p);
    }

    void ScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        if (!init_ready_) {
            return;  // 静止初始化未完成：雷达帧直接丢弃（与原版一致）
        }
        const double t = StampToSec(msg->header.stamp);
        latest_sensor_time_ = t;
        has_sensor_time_ = true;
        Pending p;
        p.kind = kScan;
        p.t = t;
        p.scan = msg;
        Enqueue(p);
    }

    // ---------------- 排序缓冲 ----------------

    void Enqueue(const Pending& p) {
        const double newest = p.t > newest_ts_ ? p.t : newest_ts_;
        newest_ts_ = newest;
        // 键 = (时间戳, 类型)：同一时刻先 IMU、再轮速、最后雷达，与离线管线的
        // 「归并后处理」顺序一致（离线：先喂 imu，再喂 odom，最后给 scan）。
        pending_.emplace(std::make_pair(p.t, p.kind), p);
        DrainReady();
    }

    void DrainReady() {
        while (!pending_.empty()) {
            auto it = pending_.begin();
            // 队首时间戳还不够旧：可能有更早的消息仍在传输中，继续等待
            if (it->first.first > newest_ts_ - reorder_margin_) {
                break;
            }
            Pending p = it->second;
            pending_.erase(it);
            Dispatch(p);
        }
    }

    void FlushPending() {
        while (!pending_.empty()) {
            auto it = pending_.begin();
            Pending p = it->second;
            pending_.erase(it);
            Dispatch(p);
        }
    }

    void Dispatch(const Pending& p) {
        switch (p.kind) {
            case kImu:
                HandleImu(p.imu, p.t);
                break;
            case kOdom:
                HandleOdom(p.odom, p.t);
                break;
            case kScan:
                HandleScan(p.scan, p.t);
                break;
        }
    }

    // ---------------- 实际处理 ----------------

    // 加速度三轴原样透传：2D ESKF 只取 xy，z 参与静止初始化中的重力估计
    static sad::Imu ToImu(const sensor_msgs::msg::Imu& msg, double t) {
        return sad::Imu(t,
                        Vec3d(msg.angular_velocity.x, msg.angular_velocity.y,
                              msg.angular_velocity.z),
                        Vec3d(msg.linear_acceleration.x, msg.linear_acceleration.y,
                              msg.linear_acceleration.z));
    }

    void FeedInitImu(const sensor_msgs::msg::Imu::SharedPtr& msg, double t) {
        if (imu_init_ && imu_init_->AddIMU(ToImu(*msg, t)) && imu_init_->InitSuccess()) {
            FinishStaticInit();
        }
    }

    void FeedInitOdom(const origincar_msg::msg::Data::SharedPtr& msg) {
        if (imu_init_) {
            imu_init_->AddOdom(sad::Odom(this->now().seconds(), msg->x));
        }
    }

    void HandleImu(const sensor_msgs::msg::Imu::SharedPtr& msg, double t) {
        frontend_->ProcessImu(ToImu(*msg, t));
    }

    void HandleOdom(const origincar_msg::msg::Data::SharedPtr& msg, double t) {
        frontend_->ProcessOdom(sad::Odom(t, msg->x));
    }

    void HandleScan(const sensor_msgs::msg::LaserScan::SharedPtr& msg, double t) {
        ++frame_total_;

        // ---- LaserScan → Frame ----
        // 点坐标：极坐标转直角坐标，只保留量程内且有限的读数（与 bag_to_csv.py
        // 的筛点条件相同：range_min <= r <= range_max 且 isfinite）。
        // 逐点时刻：t_i = stamp - time_increment × (n_ranges - 1 - idx)，
        // 其中 n_ranges 是消息中 ranges 的原始长度（含无效点），idx 是该点的原始
        // 索引。相邻有效点的时间间隔按其真实角度间隔计算，不依赖「有效点均匀分布」。
        const size_t n_ranges = msg->ranges.size();
        const bool has_time_increment = msg->time_increment > 0.0f;

        auto frame = std::make_shared<sad::Frame>();
        frame->id_ = frame_total_;
        frame->keyframe_id_ = frame_total_;
        frame->timestamp_ = t;
        frame->pts_.reserve(n_ranges);
        if (has_time_increment) {
            frame->pts_time_.reserve(n_ranges);
        }

        for (size_t idx = 0; idx < n_ranges; ++idx) {
            const float r = msg->ranges[idx];
            if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max) {
                continue;
            }
            const double angle = static_cast<double>(msg->angle_min) +
                                 static_cast<double>(idx) * msg->angle_increment;
            frame->pts_.emplace_back(static_cast<double>(r) * std::cos(angle),
                                     static_cast<double>(r) * std::sin(angle));
            if (has_time_increment) {
                frame->pts_time_.push_back(t - static_cast<double>(msg->time_increment) *
                                                   static_cast<double>(n_ranges - 1 - idx));
            }
        }

        // 有效点过少的帧丢弃（与导出脚本同一判据）
        if (frame->pts_.size() < 3) {
            ++frame_skipped_;
            return;
        }
        if (!has_time_increment && !undistort_warned_) {
            undistort_warned_ = true;
            RCLCPP_WARN(this->get_logger(),
                        "LaserScan.time_increment 为 0，逐点时刻无法计算，去畸变对本帧不生效"
                        "（检查驱动是否填写该字段）");
        }

        const bool ok = frontend_->ProcessScan(frame);
        // 调度诊断：用 --ros-args --log-level debug 可查看前几帧的累计条数，
        // 用于核对节点与离线管线的归并顺序一致
        if (frame_total_ <= 5) {
            RCLCPP_DEBUG(this->get_logger(),
                         "前几帧调度检查：frame=%zu t=%.6f 已处理 IMU=%zu 轮速=%zu 点数=%zu "
                         "time_increment=%.3e",
                         frame_total_, t, imu_count_, odom_count_, frame->pts_.size(),
                         static_cast<double>(msg->time_increment));
        }
        if (!ok) {
            ++frame_fail_;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "配准失败（累计 %zu 帧），本帧不入图", frame_fail_);
            return;
        }
        ++frame_ok_;

        PublishTrajectory(frame->pose_, t);
        if (publish_tf_) {
            tf_broadcaster_->sendTransform(MakeTransform(map_frame_, base_frame_, t, frame->pose_));
        }
    }

    void FinishStaticInit() {
        const Vec3d bg = imu_init_->GetInitBg();
        const Vec3d ba = imu_init_->GetInitBa();
        frontend_->SetInitCondition(bg[2], Vec2d(ba[0], ba[1]));
        init_ready_ = true;
        RCLCPP_INFO(this->get_logger(), "静止初始化完成：bg_z=%.6f ba_xy=(%.4f, %.4f)", bg[2], ba[0],
                    ba[1]);
    }

    void PublishTrajectory(const SE2& pose, double t) {
        trajectory_.header.stamp = SecToStamp(t);

        geometry_msgs::msg::PoseStamped ps;
        ps.header = trajectory_.header;
        ps.pose.position.x = pose.translation().x();
        ps.pose.position.y = pose.translation().y();
        ps.pose.position.z = 0.0;
        const double theta = pose.so2().log();
        ps.pose.orientation.x = 0.0;
        ps.pose.orientation.y = 0.0;
        ps.pose.orientation.z = std::sin(theta * 0.5);
        ps.pose.orientation.w = std::cos(theta * 0.5);
        trajectory_.poses.push_back(ps);
        path_pub_->publish(trajectory_);
    }

    void PublishStaticTf() {
        // 板载 TF 恒等（外参零偏移）：base→laser 与 base→imu 均为单位变换
        const double now = this->now().seconds();
        tf_static_broadcaster_->sendTransform(MakeTransform(base_frame_, laser_frame_, now, SE2()));
        tf_static_broadcaster_->sendTransform(MakeTransform(base_frame_, imu_frame_, now, SE2()));
    }

    // ---- 配置与管线 ----
    sad::Config cfg_;
    std::string config_file_;
    std::unique_ptr<sad::Frontend> frontend_;
    std::unique_ptr<sad::StaticIMUInit> imu_init_;
    bool use_static_init_ = false;
    bool init_ready_ = false;
    bool undistort_warned_ = false;

    // ---- 排序缓冲 ----
    std::map<std::pair<double, int>, Pending> pending_;
    double reorder_margin_ = 0.02;
    double newest_ts_ = 0.0;
    double latest_sensor_time_ = 0.0;
    bool has_sensor_time_ = false;

    // ---- 坐标系与发布 ----
    std::string map_frame_ = "map";
    std::string base_frame_ = "base";
    std::string laser_frame_ = "laser";
    std::string imu_frame_ = "imu";
    bool publish_tf_ = true;
    bool publish_static_tf_ = true;

    nav_msgs::msg::Path trajectory_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
    rclcpp::TimerBase::SharedPtr static_tf_timer_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<origincar_msg::msg::Data>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    // ---- 统计 ----
    size_t imu_count_ = 0;
    size_t odom_count_ = 0;
    size_t frame_total_ = 0;
    size_t frame_ok_ = 0;
    size_t frame_fail_ = 0;
    size_t frame_skipped_ = 0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<SioLioNode>();
        rclcpp::spin(node);
        node.reset();  // 析构时冲刷排序缓冲
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("sio_lio_node"), "节点启动失败：%s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
