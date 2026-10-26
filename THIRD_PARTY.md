# 第三方组件与许可

本仓库根目录的 `LICENSE` 为 Apache License 2.0，适用于本项目自行编写的代码。下表列出随仓库一同分发的第三方组件及其原始许可。

## 一、随仓库分发的组件

| 目录 / 文件 | 组件 | 版本 | 许可 | 来源 |
|---|---|---|---|---|
| `src/scanlio/3rdparty/sophus/` | Sophus（李群库） | 源码内嵌 | MIT | https://github.com/strasdat/Sophus |
| `src/serial_ros2/` | serial（ROS 2 串口库） | 1.2.1 | Apache-2.0 | https://github.com/ZhengQuan-ros/serial |
| `src/car_base_v2/` | 底盘驱动 | 0.0.1 | 未声明（`package.xml` 写 `TODO`） | 第三方提供 |
| `src/origincar_msg/` | 轮速消息定义 | 0.0.0 | 未声明（`package.xml` 写 `TODO`） | 第三方提供 |

许可字段的原文取自各包 `package.xml` 的 `<license>` 标签，未作改动。

## 二、已移出仓库的组件

`lslidar_driver` 与 `lslidar_msgs`（镭神 N10 雷达驱动与消息定义，版本 1.2.0）的 `package.xml` 声明为 GNU General Public License V3.0。

二者已从本仓库移除，并由 `.gitignore` 排除。需要实机雷达时，请自行从厂商渠道获取并放入 `src/`，其分发与使用受 GPL-3.0 约束。

这样处理的原因：Apache-2.0 与 GPL-3.0 组合分发有约束，包含 GPL-3.0 代码的整体分发物需按 GPL-3.0 条款提供。将 GPL-3.0 组件移出仓库后，本仓库可整体按 Apache-2.0 分发。

本文件只陈述事实与约束，不构成法律意见。

## 三、未随仓库分发的内容

- 数据集（rosbag）：不包含在本仓库中。
- 验证程序与实验记录：位于被 `.gitignore` 排除的 `test/` 与 `doc/`，不随仓库分发。

## 四、本项目代码

`src/scanlio/` 下的算法实现（NDT 配准、ESKF 滤波、地图与占据栅格、前端调度、ROS 2 节点、配置解析）为自行编写。参照对象为 NdtLIO 的 2D 变体思路，参照的是算法结构与推导，未复制其源码。
