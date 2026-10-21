#include "car_base_v2/car_base_v2_node.h"

#include <chrono>

namespace {
// 读超时: 静默时读线程的醒转周期 (决策: 200ms, 关机最坏等待此值)
constexpr int    kReadTimeoutMs = 200;
// 单次读取上限: 约 10 帧. 到达更多则下轮再收, 解析器缓冲区兜底
constexpr size_t kReadChunk = 256;
// 串口打开失败时的循环等待 (等待期分片为 100ms, 保证 join 响应)
constexpr int    kOpenRetryMs = 100;
}

// ===========================================================================
// 构造: 参数声明 -> 话题/定时器创建 -> 打开串口 -> 启动读线程
// ===========================================================================
CarBaseV2::CarBaseV2(const std::string & node_name) : Node(node_name) {
  // ---- 参数 ----
  port_name_ = this->declare_parameter<std::string>(
      "usart_port_name", std::string("/dev/ttyACM1"));
  cmd_vel_topic_ = this->declare_parameter<std::string>(
      "cmd_vel", std::string("cmd_vel"));
  baud_rate_ = this->declare_parameter<int>("serial_baud_rate", 921600);
  watchdog_enable_ = this->declare_parameter<bool>("cmd_watchdog.enable", true);
  watchdog_timeout_ms_ = this->declare_parameter<int>(
      "cmd_watchdog.timeout_ms", 500);
  watchdog_resend_ms_ = this->declare_parameter<int>(
      "cmd_watchdog.resend_ms", 100);
  diag_enable_ = this->declare_parameter<bool>("diag.enable", true);
  diag_period_ms_ = this->declare_parameter<int>("diag.period_ms", 1000);
  reconnect_enable_ = this->declare_parameter<bool>(
      "serial_reconnect.enable", true);
  reconnect_period_ms_ = this->declare_parameter<int>(
      "serial_reconnect.period_ms", 1000);

  // ---- 话题 (队列深度1: 定位数据只用最新值) ----
  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 1);
  wheel_pub_ = this->create_publisher<origincar_msg::msg::Data>("robotvel", 1);
  diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "car_base_v2/diagnostics", 1);
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, 1,
      std::bind(&CarBaseV2::cmdVelCallback, this, std::placeholders::_1));

  // ---- 定时器 (默认回调组, 与 cmd_vel 回调互斥执行, 状态访问无需加锁) ----
  watchdog_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(watchdog_resend_ms_),
      std::bind(&CarBaseV2::watchdogTimerCallback, this));
  if (diag_enable_) {
    diag_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(diag_period_ms_),
        std::bind(&CarBaseV2::diagnosticsTimerCallback, this));
  }

  // ---- 看门狗初始状态: "启动即生效" (决策) ----
  // 把指令时间戳回拨到阈值之前, 首个定时器 tick 即进入停车态
  last_cmd_time_ = std::chrono::steady_clock::now() -
                   std::chrono::milliseconds(watchdog_timeout_ms_ + 1000);

  // ---- 串口首连 (失败不终止: connected_ 置 false, 读线程重连流程接管) ----
  try {
    serial_.setPort(port_name_);
    serial_.setBaudrate(static_cast<uint32_t>(baud_rate_));
    // 板端 serial 库的 setTimeout 参数是非 const 左值引用,
    // 不能绑定临时对象, 必须先创建具名变量 (与学长原版写法一致)
    serial::Timeout serial_timeout = serial::Timeout::simpleTimeout(kReadTimeoutMs);
    serial_.setTimeout(serial_timeout);
    serial_.open();
  } catch (serial::IOException&) {
    RCLCPP_ERROR(this->get_logger(), "initial open failed [%s], "
                 "reader thread will keep retrying", port_name_.c_str());
  }
  connected_.store(serial_.isOpen());

  // ---- 启动读线程 (最后一步: 保证线程启动时所有成员已初始化) ----
  running_.store(true);
  read_thread_ = std::thread(&CarBaseV2::readThreadLoop, this);
}

// ===========================================================================
// 析构: 置退出标志 -> join 读线程 -> 发停车帧 -> 关串口
// join 最坏等待 = 100ms 分片 (读线程所有 sleep 均分片检查退出标志)
// 此时执行器已停、读线程已死, 无并发者, close 不需要持锁
// ===========================================================================
CarBaseV2::~CarBaseV2() {
  running_.store(false);
  if (read_thread_.joinable()) {
    read_thread_.join();
  }
  sendStopFrame();  // 单一停车路径: 无自定义信号处理函数
  if (serial_.isOpen()) {
    serial_.close();
  }
}

// ===========================================================================
// 读线程主体: 正常路径 = 事件驱动读取 + 解析发布; 异常路径 = 重连流程
// ===========================================================================
void CarBaseV2::readThreadLoop() {
  std::vector<uint8_t> buf(kReadChunk);
  protocol::RawFrame frame;

  while (running_.load(std::memory_order_relaxed)) {
    // ---- 断连状态: 重连流程 (决策: 固定 1s 无限重试, enable 可关) ----
    if (!connected_.load()) {
      if (!reconnect_enable_) {
        // 重连关闭: 维持旧行为, 空转等待手动重启
        std::this_thread::sleep_for(std::chrono::milliseconds(kOpenRetryMs));
        continue;
      }
      parser_.reset();  // 断连前后是两段不连续字节流, 旧残余不可复用
      // 分片等待: 每 100ms 醒来查退出标志, 保证析构 join 不被卡住
      for (int waited = 0;
           waited < reconnect_period_ms_ && running_.load();
           waited += kOpenRetryMs) {
        std::this_thread::sleep_for(std::chrono::milliseconds(kOpenRetryMs));
      }
      if (!running_.load()) break;
      if (tryReconnect()) {
        RCLCPP_INFO(this->get_logger(), "serial reconnected");
      }
      continue;
    }

    // ---- 正常路径: 事件驱动读取 ----
    size_t n = 0;
    try {
      n = serial_.read(buf.data(), 1);       // 阻塞至多 200ms
      if (n == 0) {
        continue;                            // 静默醒转: 查退出标志
      }
      size_t avail = serial_.available();    // 已到达的字节, 立即返回
      if (avail > 0) {
        if (avail > kReadChunk - n) {
          avail = kReadChunk - n;
        }
        n += serial_.read(buf.data() + n, avail);
      }
    } catch (serial::IOException&) {
      ++read_errors_;
      connected_.store(false);
      RCLCPP_ERROR(this->get_logger(), "serial read error, entering reconnect");
      continue;                              // 下轮进入重连分支
    }

    parser_.feed(buf.data(), n);
    // 一次读入可能含多帧 (积压场景), 循环取完
    while (parser_.tryExtract(frame)) {
      publishImu(frame);
      publishWheelSpeed(frame);
    }
  }
}

// ---------------------------------------------------------------------------
// 重连一次 (读线程调用). close/open 与写路径互斥: 持 serial_mutex_
// (决策节点5: 锁边界从 write 扩展为 write/close/open)
// ---------------------------------------------------------------------------
bool CarBaseV2::tryReconnect() {
  std::lock_guard<std::mutex> lock(serial_mutex_);
  try {
    serial_.close();
    serial_.setPort(port_name_);
    serial_.setBaudrate(static_cast<uint32_t>(baud_rate_));
    serial::Timeout serial_timeout = serial::Timeout::simpleTimeout(kReadTimeoutMs);
    serial_.setTimeout(serial_timeout);
    serial_.open();
  } catch (serial::IOException&) {
    connected_.store(false);
    return false;
  }
  connected_.store(serial_.isOpen());
  if (connected_.load()) {
    ++reconnects_;
    return true;
  }
  return false;
}

// ===========================================================================
// cmd_vel 回调: 刷新指令时间戳 -> 组帧写串口
// ===========================================================================
void CarBaseV2::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  last_cmd_time_ = std::chrono::steady_clock::now();
  writeVelocityFrame(msg->linear.x, msg->linear.y, msg->angular.z);
}

// ---------------------------------------------------------------------------
// 看门狗 (决策节点5): 超时 timeout_ms 未收到指令则周期发零速.
// 与 cmd_vel 回调同回调组互斥执行, last_cmd_time_ 访问无并发.
// ===========================================================================
void CarBaseV2::watchdogTimerCallback() {
  if (!watchdog_enable_) {
    watchdog_stopping_.store(false);
    return;
  }
  auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - last_cmd_time_).count();
  if (elapsed_ms >= watchdog_timeout_ms_) {
    watchdog_stopping_.store(true);
    sendStopFrame();   // 定时器周期 100ms 即重发周期 (决策: 周期重发)
  } else {
    watchdog_stopping_.store(false);
  }
}

// ---------------------------------------------------------------------------
// 诊断 (决策节点5): 1Hz 发布 DiagnosticArray, 8 个键值对.
// level: 断连->ERROR, 本周期溢出增加->WARN, 其余 OK
// ===========================================================================
void CarBaseV2::diagnosticsTimerCallback() {
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = this->now();

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "car_base_v2";
  status.hardware_id = port_name_;

  // ---- level 判定 ----
  const uint64_t overflow_now = parser_.stats().buffer_overflows.load(
      std::memory_order_relaxed);
  if (!connected_.load()) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "serial disconnected";
  } else if (overflow_now > last_overflow_seen_) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "parser buffer overflow";
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "ok";
  }
  last_overflow_seen_ = overflow_now;

  // ---- 键值对 ----
  auto add = [&status](const std::string& key, const std::string& value) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = key;
    kv.value = value;
    status.values.push_back(kv);
  };
  auto& s = parser_.stats();
  add("frames_parsed",
      std::to_string(s.frames_parsed.load(std::memory_order_relaxed)));
  add("candidates_rejected",
      std::to_string(s.candidates_rejected.load(std::memory_order_relaxed)));
  add("buffer_overflows", std::to_string(overflow_now));
  add("serial_read_errors",
      std::to_string(read_errors_.load(std::memory_order_relaxed)));
  add("serial_connected", connected_.load() ? "1" : "0");
  add("serial_reconnects",
      std::to_string(reconnects_.load(std::memory_order_relaxed)));
  add("watchdog_stopping", watchdog_stopping_.load() ? "1" : "0");
  auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - last_cmd_time_).count();
  add("last_cmd_age_ms", std::to_string(age_ms));

  array.status.push_back(status);
  diag_pub_->publish(array);
}

// ---------------------------------------------------------------------------
// 组帧并发送. 协议: [头][控制1][控制2][vx_hi vx_lo][vy_hi vy_lo][wz_hi wz_lo]
//                   [XOR校验(前9字节)][尾]
// 速度放大 1000 倍取整传输, 接收方除以 1000 还原 (协议冻结, 与原版一致)
// ---------------------------------------------------------------------------
void CarBaseV2::writeVelocityFrame(float vx, float vy, float wz) {
  if (!serial_.isOpen()) {
    return;
  }

  uint8_t tx[protocol::kSendSize] = {0};
  tx[0] = protocol::kFrameHeader;

  auto fill_channel = [&tx](size_t hi, size_t lo, float value) {
    short scaled = static_cast<short>(value * 1000.0f);
    tx[hi] = static_cast<uint8_t>(scaled >> 8);  // 高字节在前 (大端)
    tx[lo] = static_cast<uint8_t>(scaled & 0xFF);
  };
  fill_channel(3, 4, vx);
  fill_channel(5, 6, vy);
  fill_channel(7, 8, wz);

  uint8_t checksum = 0;
  for (size_t i = 0; i < 9; ++i) {
    checksum ^= tx[i];
  }
  tx[9]  = checksum;
  tx[10] = protocol::kFrameTail;

  std::lock_guard<std::mutex> lock(serial_mutex_);  // 写路径持锁 (节点5改名)
  try {
    serial_.write(tx, sizeof(tx));
  } catch (serial::IOException&) {
    RCLCPP_ERROR(this->get_logger(), "cannot write velocity frame");
  }
}

void CarBaseV2::sendStopFrame() {
  if (!serial_.isOpen()) {
    return;
  }
  uint8_t tx[protocol::kSendSize] = {0};
  tx[0] = protocol::kFrameHeader;
  uint8_t checksum = 0;
  for (size_t i = 0; i < 9; ++i) {
    checksum ^= tx[i];
  }
  tx[9]  = checksum;
  tx[10] = protocol::kFrameTail;

  std::lock_guard<std::mutex> lock(serial_mutex_);  // 与 cmd_vel 写/重连互斥
  try {
    serial_.write(tx, sizeof(tx));
  } catch (serial::IOException&) {
    // 安全动作发送失败不重试单帧: 看门狗下一周期会重发
  }
}

// ---------------------------------------------------------------------------
// 发布 (仅读线程调用, 无需锁)
// ---------------------------------------------------------------------------
void CarBaseV2::publishImu(const protocol::RawFrame& f) {
  sensor_msgs::msg::Imu msg;
  // TODO(节点2): 固件冻结无采样时刻, stamp 策略为 now() - 固定延迟补偿 (待标定)
  msg.header.stamp = this->now();
  msg.header.frame_id = "imu_link";

  msg.linear_acceleration.x = f.accel[0];
  msg.linear_acceleration.y = f.accel[1];
  msg.linear_acceleration.z = f.accel[2];
  msg.angular_velocity.x = f.gyro[0];
  msg.angular_velocity.y = f.gyro[1];
  msg.angular_velocity.z = f.gyro[2];

  // TODO(节点4): 协方差策略待定 (当前系统无协方差消费者, 决策: 节点4关闭)
  imu_pub_->publish(msg);
}

void CarBaseV2::publishWheelSpeed(const protocol::RawFrame& f) {
  origincar_msg::msg::Data msg;
  msg.x = f.vx;
  msg.y = f.vy;
  msg.z = f.wz;
  wheel_pub_->publish(msg);
}
