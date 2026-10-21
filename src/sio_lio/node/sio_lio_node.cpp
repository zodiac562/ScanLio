// sio_lio 的 ROS 2 节点：把 Frontend（ESKF + NDT + Map）接到 ROS 话题上。
//
// 数据流（与原版 ndtlio ros2node/ndt_lio_node.cpp 一致）：
//   /imu      sensor_msgs/Imu       → Frontend::ProcessImu  → ESKF 预测
//   /robotvel origincar_msg/Data    → Frontend::ProcessOdom → ESKF 轮速观测
//   /scan     sensor_msgs/LaserScan → Frame → Frontend::ProcessScan → 配准 + 修正
// 输出：
//   /trajectory          nav_msgs/Path        每成功配准一帧追加一个位姿（关键帧与非关键帧都发）
//   /tf                  map → base           每成功配准一帧发布一次
//   /tf_static           base → laser / imu   定时重发（2 s），与驱动侧坐标系保持同一假设
//   /map                 nav_msgs/OccupancyGrid    定时发布（publish_period_ms，默认 1 s）
//   /global_point_cloud  sensor_msgs/PointCloud2   定时发布，内容为已生效体素的中心
//   /sio_lio/pose_with_covariance                 每成功配准一帧发布一次，携带 ESKF 后验协方差
//
// 三项可视化输出的数据来源都是核心库的只读接口，节点不自行计算：
//   占据栅格 → Map::occupancy_map() 的 bounds/probability
//   体素点云 → NDT2D::get_voxels()
//   位姿协方差 → ESKF::GetP()
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
// 状态只被一个线程访问，不需要加锁。栅格与点云的定时发布也在同一线程内，
// 与配准串行执行，因此不需要对 Map 加锁。

#include "sio_lio/common/imu.h"
#include "sio_lio/common/odom.h"
#include "sio_lio/common/static_imu_init.h"
#include "sio_lio/include/config.h"
#include "sio_lio/include/frame.h"
#include "sio_lio/include/frontend.h"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <origincar_msg/msg/data.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
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
#include <vector>

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

        // ---- 三项可视化输出 ----
        // 开关与话题名分开声明：便于在同一张地图上并排对比「开/关」两种配置，
        // 也便于回放时避免与数据包内录制的同名话题冲突。
        publish_occupancy_ = this->declare_parameter<bool>("publish_occupancy", true);
        const std::string occ_topic = this->declare_parameter<std::string>("occupancy_topic", "/map");
        publish_cloud_ = this->declare_parameter<bool>("publish_cloud", true);
        const std::string cloud_topic =
            this->declare_parameter<std::string>("cloud_topic", "/global_point_cloud");
        publish_pose_cov_ = this->declare_parameter<bool>("publish_pose_cov", true);
        const std::string pose_cov_topic = this->declare_parameter<std::string>(
            "pose_cov_topic", "/sio_lio/pose_with_covariance");
        const int publish_period_ms = this->declare_parameter<int>("publish_period_ms", 1000);

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

        if (publish_occupancy_) {
            // TRANSIENT_LOCAL + RELIABLE：后启动的 rviz2 或 nav2 也能立刻收到最新地图。
            // 深度 1 足够，地图是全量快照，旧快照没有保留价值。
            rclcpp::QoS map_qos(1);
            map_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
            map_qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
            map_qos.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
            occ_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(occ_topic, map_qos);
        }
        if (publish_cloud_) {
            // 点云每期都是全量快照，用 volatile（不要 latched）：体积随地图增长，
            // 缓存旧快照会长期占用内存，而对后启动的订阅者来说旧点云价值低。
            cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                cloud_topic, rclcpp::QoS(1));
        }
        if (publish_pose_cov_) {
            pose_cov_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
                pose_cov_topic, rclcpp::QoS(10));
        }

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

        // ---- 6. 栅格与点云的定时发布 ----
        // 用 wall_timer 而非跟随帧：地图是全量快照，按固定周期发布使带宽可预期，
        // 也让下游（rviz2、nav2）拿到稳定的刷新节奏，与帧率无关。
        if ((publish_occupancy_ || publish_cloud_) && publish_period_ms > 0) {
            const auto period = std::chrono::milliseconds(publish_period_ms);
            map_timer_ = this->create_wall_timer(period, std::bind(&SioLioNode::PublishMapTick, this));
        } else if (publish_period_ms <= 0) {
            RCLCPP_WARN(this->get_logger(),
                        "publish_period_ms = %d 非正，栅格与点云不发布", publish_period_ms);
        }

        RCLCPP_INFO(this->get_logger(),
                    "订阅：%s / %s / %s；轨迹发布：%s；排序缓冲 %.3f s", imu_topic.c_str(),
                    odom_topic.c_str(), scan_topic.c_str(), traj_topic.c_str(), reorder_margin_);
        RCLCPP_INFO(this->get_logger(),
                    "输出：栅格=%s(%s) 点云=%s(%s) 协方差=%s(%s) 周期=%d ms",
                    occ_topic.c_str(), publish_occupancy_ ? "开" : "关", cloud_topic.c_str(),
                    publish_cloud_ ? "开" : "关", pose_cov_topic.c_str(),
                    publish_pose_cov_ ? "开" : "关", publish_period_ms);
    }

    ~SioLioNode() override {
        // 冲刷排序缓冲：末尾 reorder_margin 时间段内的消息在正常运行时不会被处理，
        // 退出时按时间顺序补处理，使统计与离线管线可比。
        FlushPending();
        RCLCPP_INFO(this->get_logger(),
                    "退出统计：IMU %zu 条，轮速 %zu 条，雷达帧 %zu，配准成功 %zu，配准失败 %zu，"
                    "有效点不足跳过 %zu，无逐点时刻 %zu",
                    imu_count_, odom_count_, frame_total_, frame_ok_, frame_fail_, frame_skipped_,
                    frame_no_time_);
        // 发布开销：栅格与点云的构造耗时（不含 DDS 序列化与传输）
        if (occ_cost_count_ > 0) {
            RCLCPP_INFO(this->get_logger(), "栅格发布开销：%zu 次，平均 %.2f ms，最大 %.2f ms",
                        occ_cost_count_, occ_cost_total_ms_ / static_cast<double>(occ_cost_count_),
                        occ_cost_max_ms_);
        }
        if (cloud_cost_count_ > 0) {
            RCLCPP_INFO(this->get_logger(), "点云发布开销：%zu 次，平均 %.2f ms，最大 %.2f ms",
                        cloud_cost_count_, cloud_cost_total_ms_ / static_cast<double>(cloud_cost_count_),
                        cloud_cost_max_ms_);
        }
        // 地图规模：用于核对 /global_point_cloud 与 /map 的内容是否与内部状态一致
        if (frontend_) {
            RCLCPP_INFO(this->get_logger(),
                        "地图统计：关键帧 %zu，体素 %zu（已生效 %zu），占据栅格格子 %zu",
                        frontend_->GetMap().num_keyframes(),
                        frontend_->GetMap().ndt().num_voxels(),
                        frontend_->GetMap().ndt().get_voxels().size(),
                        frontend_->GetMap().occupancy_map().num_cells());
        }
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
        if (!has_time_increment) {
            // 驱动/记录环节偶尔会写出 time_increment = 0 的帧（实测数据包中存在）。
            // 这类帧的点云不带逐点时刻，UndistortScan 会因 pts_time_ 与 pts_ 数量不符而
            // 直接返回，等价于本帧不做补偿。计入统计并在退出时报告总数。
            ++frame_no_time_;
            if (!undistort_warned_) {
                undistort_warned_ = true;
                RCLCPP_WARN(this->get_logger(),
                            "LaserScan.time_increment 为 0，逐点时刻无法计算，去畸变对这类帧不生效"
                            "（检查驱动是否填写该字段；累计数量在退出统计中报告）");
            }
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
        map_stamp_ = t;   // 栅格与点云消息的时间戳，取最近一帧成功配准的扫描时刻

        // 占据栅格每帧写入：写入不改变配准与位姿（栅格只被读来发布），
        // 因此可以比 NDT 体素（只在关键帧写）更密，让墙角与窄缝更快收敛。
        // 位置放在位姿确定之后、发布之前，保证本帧消息携带的是已含本帧的栅格。
        if (publish_occupancy_) {
            frontend_->GetMap().add_scan_to_occupancy_map(frame);
        }

        PublishTrajectory(frame->pose_, t);
        if (publish_tf_) {
            tf_broadcaster_->sendTransform(MakeTransform(map_frame_, base_frame_, t, frame->pose_));
        }
        if (publish_pose_cov_) {
            PublishPoseCov(frame->pose_, t);
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

    // 定时发布栅格与点云。两者都是「全量快照」，因此共用同一周期。
    void PublishMapTick() {
        if (!init_ready_) {
            return;   // 静止初始化阶段尚未写入任何数据
        }
        // 计时：用于量化发布开销是否影响配准节奏。同线程串行执行，
        // 这段时间内不处理消息，故必须记录最大值。
        if (publish_occupancy_) {
            const auto t0 = std::chrono::steady_clock::now();
            PublishOccupancyGrid();
            const double ms = std::chrono::duration<double, std::milli>(
                                  std::chrono::steady_clock::now() - t0).count();
            occ_cost_total_ms_ += ms;
            if (ms > occ_cost_max_ms_) occ_cost_max_ms_ = ms;
            ++occ_cost_count_;
        }
        if (publish_cloud_) {
            const auto t0 = std::chrono::steady_clock::now();
            PublishVoxelCloud();
            const double ms = std::chrono::duration<double, std::milli>(
                                  std::chrono::steady_clock::now() - t0).count();
            cloud_cost_total_ms_ += ms;
            if (ms > cloud_cost_max_ms_) cloud_cost_max_ms_ = ms;
            ++cloud_cost_count_;
        }
    }

    // 占据栅格：按当前地图的包围盒裁剪后整幅发布
    void PublishOccupancyGrid() {
        const sad::OccupancyMap& occ = frontend_->GetMap().occupancy_map();
        if (occ.num_cells() == 0) {
            return;   // 还没有任何射线，发布全未知的地图没有信息量
        }

        int min_gx = 0, min_gy = 0, max_gx = 0, max_gy = 0;
        if (!occ.bounds(&min_gx, &min_gy, &max_gx, &max_gy)) {
            return;
        }
        const int64_t width = static_cast<int64_t>(max_gx) - min_gx + 1;
        const int64_t height = static_cast<int64_t>(max_gy) - min_gy + 1;

        // 分配上限：20M 格（data 为 int8，约 20 MB）。超限说明包围盒被远处的孤立
        // 点撑大，此时发布整幅图的开销超过收益，跳过并告警由人工判断。
        const int64_t kMaxCells = 20000000;
        if (width <= 0 || height <= 0 || width * height > kMaxCells) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "占据栅格包围盒 %ldx%ld 超出发布上限，跳过本次发布", width, height);
            return;
        }

        nav_msgs::msg::OccupancyGrid msg;
        msg.header.frame_id = map_frame_;
        msg.header.stamp = SecToStamp(map_stamp_);
        msg.info.resolution = static_cast<float>(occ.resolution());
        msg.info.width = static_cast<uint32_t>(width);
        msg.info.height = static_cast<uint32_t>(height);
        // 栅格左下角的世界系坐标：格 (gx, gy) 覆盖 [gx*res, (gx+1)*res)
        msg.info.origin.position.x = static_cast<double>(min_gx) * occ.resolution();
        msg.info.origin.position.y = static_cast<double>(min_gy) * occ.resolution();
        msg.info.origin.position.z = 0.0;
        msg.info.origin.orientation.w = 1.0;

        // ROS 的存储顺序为行主序，index = x_local + y_local * width，
        // 即行号对应 y、列号对应 x。未观测格 probability() 返回 -1，
        // 与 ROS 约定的未知格取值一致，因此直接搬用。
        msg.data.assign(static_cast<size_t>(width * height), -1);
        for (int gy = min_gy; gy <= max_gy; ++gy) {
            const int y_local = gy - min_gy;
            for (int gx = min_gx; gx <= max_gx; ++gx) {
                const int x_local = gx - min_gx;
                const int p = occ.probability(gx, gy);
                if (p >= 0) {
                    msg.data[static_cast<size_t>(x_local) +
                             static_cast<size_t>(y_local) * static_cast<size_t>(width)] =
                        static_cast<int8_t>(p);
                }
            }
        }

        occ_pub_->publish(msg);
    }

    // 全局体素点云：只发已生效体素的中心，用来观察「地图长到哪里、密度如何」
    void PublishVoxelCloud() {
        const std::vector<Vec2d> centers = frontend_->GetMap().ndt().get_voxels();

        sensor_msgs::msg::PointCloud2 msg;
        msg.header.frame_id = map_frame_;
        msg.header.stamp = SecToStamp(map_stamp_);
        msg.height = 1;
        msg.width = static_cast<uint32_t>(centers.size());
        msg.is_bigendian = false;
        msg.is_dense = true;   // 体素中心一定是有限值，没有 NaN

        // 用 Modifier 声明字段与容量，再用迭代器写值，避免手工算偏移
        sensor_msgs::PointCloud2Modifier modifier(msg);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(centers.size());

        sensor_msgs::PointCloud2Iterator<float> it_x(msg, "x");
        sensor_msgs::PointCloud2Iterator<float> it_y(msg, "y");
        sensor_msgs::PointCloud2Iterator<float> it_z(msg, "z");
        for (size_t i = 0; i < centers.size(); ++i, ++it_x, ++it_y, ++it_z) {
            *it_x = static_cast<float>(centers[i].x());
            *it_y = static_cast<float>(centers[i].y());
            *it_z = 0.0f;
        }

        cloud_pub_->publish(msg);
    }

    // 位姿协方差：只填 x、y、yaw 三个对角元，其余为 0
    void PublishPoseCov(const SE2& pose, double t) {
        const Mat8d P = frontend_->GetP();

        geometry_msgs::msg::PoseWithCovarianceStamped msg;
        msg.header.frame_id = map_frame_;
        msg.header.stamp = SecToStamp(t);
        msg.pose.pose.position.x = pose.translation().x();
        msg.pose.pose.position.y = pose.translation().y();
        msg.pose.pose.position.z = 0.0;
        const double theta = pose.so2().log();
        msg.pose.pose.orientation.x = 0.0;
        msg.pose.pose.orientation.y = 0.0;
        msg.pose.pose.orientation.z = std::sin(theta * 0.5);
        msg.pose.pose.orientation.w = std::cos(theta * 0.5);

        // covariance 为 6x6 行主序：索引 = 行*6 + 列。
        // 分量顺序为 x, y, z, rot_x, rot_y, rot_z，故 (x,x)=0、(y,y)=7、(rot_z,rot_z)=35。
        // 本项目为平面运动，z、roll、pitch 不参与估计，保持 0。
        msg.pose.covariance.fill(0.0);
        msg.pose.covariance[0] = P(0, 0);    // x
        msg.pose.covariance[7] = P(1, 1);    // y
        msg.pose.covariance[35] = P(4, 4);   // yaw

        pose_cov_pub_->publish(msg);
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

    // 三项可视化输出的开关
    bool publish_occupancy_ = true;
    bool publish_cloud_ = true;
    bool publish_pose_cov_ = true;
    double map_stamp_ = 0.0;   // 最近一帧成功配准的扫描时刻，用于栅格与点云消息头

    nav_msgs::msg::Path trajectory_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occ_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_cov_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
    rclcpp::TimerBase::SharedPtr static_tf_timer_;
    rclcpp::TimerBase::SharedPtr map_timer_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<origincar_msg::msg::Data>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    // ---- 发布开销统计（毫秒）----
    double occ_cost_total_ms_ = 0.0;
    double occ_cost_max_ms_ = 0.0;
    size_t occ_cost_count_ = 0;
    double cloud_cost_total_ms_ = 0.0;
    double cloud_cost_max_ms_ = 0.0;
    size_t cloud_cost_count_ = 0;

    // ---- 统计 ----
    size_t imu_count_ = 0;
    size_t odom_count_ = 0;
    size_t frame_total_ = 0;
    size_t frame_ok_ = 0;
    size_t frame_fail_ = 0;
    size_t frame_skipped_ = 0;
    size_t frame_no_time_ = 0;   // time_increment 为 0、无法去畸变的帧数
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
