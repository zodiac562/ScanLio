#include <rclcpp/rclcpp.hpp>

#include "car_base_v2/car_base_v2_node.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<CarBaseV2>("car_base_v2");

  // 主线程 = 执行器线程: 只调度 cmd_vel 回调, 不接触串口读路径.
  // rclcpp 默认 SIGINT 处理使 spin 返回 (不使用自定义信号处理函数,
  // 原版 sigintHandler 重新打开串口发送的竞态路径随之消失).
  rclcpp::spin(node);

  // 显式析构, 在 rclcpp::shutdown() 之前完成:
  // 停读线程 -> 发停车帧 -> 关串口
  node.reset();
  rclcpp::shutdown();
  return 0;
}
