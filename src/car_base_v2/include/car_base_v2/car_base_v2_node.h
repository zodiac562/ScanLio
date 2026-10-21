#ifndef CAR_BASE_V2_NODE_H_
#define CAR_BASE_V2_NODE_H_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <origincar_msg/msg/data.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <serial/serial.h>

#include "car_base_v2/frame_parser.h"

// ---------------------------------------------------------------------------
// 线程结构 (决策节点1: 方案A1; 决策节点5: 增加两个执行器定时器)
//
//   读线程     readThreadLoop: 读串口 -> parser.feed -> tryExtract循环发布
//              异常/断连时执行重连流程 (清缓冲 -> 分片等待 -> 重试 open)
//   主线程     rclcpp::spin:   执行器, 调度以下回调 (同回调组, 互斥执行):
//     cmdVelCallback           更新指令时间戳 -> 加锁写串口
//     watchdogTimerCallback    看门狗: 超时则周期发零速
//     diagnosticsTimerCallback 1Hz 组装发布诊断
//
//   锁的边界 (节点5 变更): serial_mutex_ (原 serial_write_mutex_) 覆盖
//   write/close/open 三类操作 —— 重连把 close/open 挪到运行期, 与写路径互斥.
//   读线程正常数据路径 (read/feed/extract/publish) 仍完全无锁;
//   parser_ 由读线程独占; 重连期间写操作在锁上等待几毫秒.
// ---------------------------------------------------------------------------
class CarBaseV2 : public rclcpp::Node {
public:
  explicit CarBaseV2(const std::string & node_name);
  ~CarBaseV2() override;

private:
  // ---- 读线程 ----
  void readThreadLoop();
  bool tryReconnect();  // 尝试一次重连, 成功返回 true (持 serial_mutex_)

  // ---- 执行器回调 ----
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void watchdogTimerCallback();
  void diagnosticsTimerCallback();

  // ---- 串口写路径 (持 serial_mutex_) ----
  void writeVelocityFrame(float vx, float vy, float wz);  // m/s, rad/s
  void sendStopFrame();

  // ---- 发布 (仅读线程调用) ----
  void publishImu(const protocol::RawFrame& f);
  void publishWheelSpeed(const protocol::RawFrame& f);

  // ---- 成员: 串口与同步 ----
  std::atomic<bool> running_{false};
  std::thread read_thread_;
  std::mutex serial_mutex_;              // 覆盖 write/close/open (节点5)
  serial::Serial serial_;
  protocol::FrameParser parser_;         // 读线程独占

  // ---- 成员: 跨线程状态 (atomic) ----
  std::atomic<bool> connected_{false};   // 读线程维护, 诊断读取
  std::atomic<uint64_t> read_errors_{0};
  std::atomic<uint64_t> reconnects_{0};
  std::atomic<bool> watchdog_stopping_{false};

  // ---- 成员: 看门狗 (仅执行器线程访问, 同回调组互斥, 无需原子) ----
  std::chrono::steady_clock::time_point last_cmd_time_;

  // ---- 成员: 参数 ----
  std::string port_name_;
  std::string cmd_vel_topic_;
  int baud_rate_ = 921600;
  bool watchdog_enable_ = true;
  int watchdog_timeout_ms_ = 500;
  int watchdog_resend_ms_ = 100;
  bool diag_enable_ = true;
  int diag_period_ms_ = 1000;
  bool reconnect_enable_ = true;
  int reconnect_period_ms_ = 1000;

  // ---- 成员: 诊断辅助 (仅执行器线程) ----
  uint64_t last_overflow_seen_ = 0;      // WARN 判定: 本周期溢出是否增加

  // ---- 成员: ROS 接口 ----
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<origincar_msg::msg::Data>::SharedPtr wheel_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::TimerBase::SharedPtr diag_timer_;
};

#endif  // CAR_BASE_V2_NODE_H_
