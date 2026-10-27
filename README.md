# scanlio

2D 激光惯性里程计（LiDAR-Inertial Odometry），参照 NdtLIO 的 2D 变体思路实现。不使用第三方 SLAM 库。

当前版本 **0.9.1**。项目用途是原理验证

## 构成

`src/` 下四个 ROS 2 包：

| 包 | 作用 | 是否随仓库分发 |
|---|---|---|
| `scanlio` | 算法本体：NDT 配准、ESKF 滤波、地图与占据栅格、前端调度、ROS 2 节点 | 是 |
| `car_base_v2` | 底盘驱动，发布 IMU 与轮速 | 是 |
| `origincar_msg` | 轮速消息定义 | 是 |
| `serial` | 串口库（目录名 `serial_ros2`，包名 `serial`） | 是 |

`lslidar_driver` 与 `lslidar_msgs`（镭神 N10 雷达驱动与消息定义）采用 GPL-3.0，**不随本仓库分发**，见「雷达驱动」一节。

`scanlio` 内部分三层：

- `src/scanlio/`：算法核心，编译为静态库 `scanlio_core`，不链接任何 ROS 库，可在纯 C++ 环境构建与验证
- `src/scanlio/node/`：ROS 2 节点，只做话题与算法结构体之间的转换
- `src/scanlio/config/`：唯一配置文件 `scanlio.yaml`，五段共 24 项

## 雷达驱动

本仓库不含雷达驱动。使用实机时需自行获取 `lslidar_driver` 与 `lslidar_msgs`，放入 `src/` 后一并构建。二者采用 GPL-3.0，与本仓库的 Apache-2.0 不同，复制或分发前请自行确认条款。

不接实机、仅用已录制数据回放时，不需要这两个包。

## 依赖

ROS 2（在 Lyrical 与 Humble 上均验证过），以及：

```
Eigen3        libeigen3-dev
yaml-cpp      libyaml-cpp-dev
rclcpp / sensor_msgs / nav_msgs / geometry_msgs / tf2_ros / ament_index_cpp
```

驱动包另需 `libpcap-dev`、`libpcl-all-dev`、`libboost-thread-dev`。

Sophus 以源码形式放在 `src/scanlio/3rdparty/sophus/`，无需单独安装。

## 构建

```bash
cd slam_ws
colcon build --packages-select scanlio
```

只构建算法包时用 `--packages-select scanlio`；构建全部包需完整的 ROS 2 与上列系统依赖。

`src/scanlio/CMakeLists.txt` 通过两级判定兼容两个发行版（Lyrical 有聚合导入目标，Humble 只有 `ament_target_dependencies`），因此同一份源码可在两个版本上构建。

## 运行

回放数据包：

```bash
source /opt/ros/<distro>/setup.bash
source install/setup.bash
ros2 launch scanlio scanlio_node.launch.py
```

实机整合启动（雷达驱动 + 底盘驱动 + 算法节点）：

```bash
ros2 launch scanlio scanlio_bringup.launch.py
```

串口设备名颠倒时用参数覆盖：

```bash
ros2 launch scanlio scanlio_bringup.launch.py lidar_port:=/dev/ttyACM1 base_port:=/dev/ttyACM0
```

`scanlio_bringup.launch.py` 会启动 `lslidar_driver`，未放置该包时整合启动失败，此时用单节点 launch 配合已有的雷达驱动。

## 订阅与发布

订阅：

| 话题 | 类型 | 内容 |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | 单线激光雷达 |
| `/imu` | `sensor_msgs/Imu` | 角速度与比力 |
| `/robotvel` | `origincar_msg/Data` | 轮速，字段 `x` 为前向速度 |

发布：

| 话题 | 类型 | 内容 |
|---|---|---|
| `/trajectory` | `nav_msgs/Path` | 轨迹，每帧一个位姿 |
| `/map` | `nav_msgs/OccupancyGrid` | 占据栅格，按周期发布 |
| `/global_point_cloud` | `sensor_msgs/PointCloud2` | NDT 体素中心 |
| `/scanlio/pose_with_covariance` | `geometry_msgs/PoseWithCovarianceStamped` | 位姿与协方差 |

另发布 `map → base_frame` 动态 TF，以及 `base_frame → laser_frame`、`base_frame → imu_frame` 静态 TF。

## 参数

两套，各管各的键：

ROS 参数（21 个）：话题名、坐标系名、是否发布各类输出、发布周期、排序缓冲窗口等对外接口。

yaml 参数（`config/scanlio.yaml`，24 项分五段）：`init` 初始零偏、`eskf` 噪声与协方差、`ndt` 配准参数、`map` 栅格分辨率、`frontend` 关键帧与去畸变开关。

连接点是 ROS 参数 `config_file`：为空时节点用 `get_package_share_directory("scanlio") + "/config/scanlio.yaml"` 定位。

## 算法概要

- 配准：2D NDT。体素内统计均值与协方差，高斯-牛顿迭代求增量，马氏距离做卡方剔除。体素使用 LRU 淘汰。
- 滤波：8 维误差状态 ESKF，状态为 `p(2) v(2) θ bg ba(2)`。观测有两个来源：轮速的前向速度、雷达的配准位姿。零偏作为滤波器状态在线估计，观测更新后写回名义零偏。
- 零偏随机游走方差取实测值（陀螺 1.43e-5、加计 9.25e-5，每步）。
- 去畸变：雷达一帧 0.1 秒内车在运动，按逐点时刻把点统一到帧尾时刻，位姿由 IMU 缓冲插值。
- 地图：占据栅格用射线追踪计数法（hits / hits+misses），容器使用有序 `std::map` 保证输出顺序可复现。
- 调度：三个话题的消息进入按 (时间戳, 类型) 排序的缓冲，等窗口满足后按时间顺序处理，用于抵消回调触发顺序与时间戳顺序不一致。

## 已知限制

- 仅支持单线 2D 雷达，位姿只有 `x, y, θ`，`z` 恒为 0。
- 无回环检测，长距离行驶的累积漂移无法消除。
- 无退化检测与零速更新，几何退化段（如贴墙、空旷）的配准行为未做专门抑制。
- 去畸变的帧内运动模型为线性插值，高速转弯时误差会增大。
- 参数以单线 N10（约 444 点/圈、10 Hz）标定，更换雷达需重新标定 `ndt.voxel_size` 与 `ndt.min_pts_num`。

## 版本与发布

版本号写在本仓库三个位置，改版时同步：

1. `src/scanlio/package.xml` 的 `<version>`（程序可读的来源，`0.9.0`）
2. git tag（`v0.9.0`）与对应的 GitHub Release
3. 本文件顶部的「当前版本」

仓库名不带版本号。版本更迭通过 tag 与 Release 表达，不修改仓库名。

## 许可

本仓库自有代码采用 Apache License 2.0，全文见 `LICENSE`，版权声明见 `NOTICE`。

随仓库分发的第三方组件及其许可见 `THIRD_PARTY.md`。雷达驱动为 GPL-3.0，不随本仓库分发。

## 不随仓库分发的内容

- 数据集（rosbag）：自行录制，不进仓库。
- 验证程序与实验记录：位于 `.gitignore` 排除的 `test/` 与 `doc/`。
- `lslidar_driver`、`lslidar_msgs`：GPL-3.0，需从厂商渠道获取。
