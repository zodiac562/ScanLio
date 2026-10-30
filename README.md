# scanlio

2D 激光惯性里程计。输入为单线激光雷达、IMU 与轮速，输出平面位姿（`x, y, θ`）。

当前版本 **1.0.1**。

## 项目定位

模块划分参照 NdtLIO，代码自行实现，不引入 Cartographer、LIO-SAM 这类完整 SLAM 库。每个模块配验证程序与判定阈值，随源码保留，可随时重跑。

用途有三项：读懂原理、与原版逐模块对照、在原版基础上做可切换的改进。三项都遵守一条规则：程序内部自洽不能证明与真实运动一致，因此只说现象，不说更好，除非有量化对照。

## 仓库构成

| 包 | 内容 | 分发 |
|---|---|---|
| `scanlio` | 算法本体：NDT 配准、误差状态滤波、体素地图与占据栅格、前端调度、ROS 2 节点 | 是 |
| `car_base_v2` | 底盘驱动：串口读取 IMU 与轮速 | 是 |
| `origincar_msg` | 轮速消息定义 | 是 |
| `serial` | 串口库；目录名 `serial_ros2`，包名 `serial` | 是 |
| `lslidar_driver` | 镭神 N10 雷达驱动 | 否，GPL-3.0 |
| `lslidar_msgs` | 镭神雷达消息定义 | 否，GPL-3.0 |

`scanlio` 内部三层：

| 目录 | 内容 | 约束 |
|---|---|---|
| `src/` | 算法核心，编译为静态库 `scanlio_core` | 不链接 ROS 库，可在无 ROS 环境构建 |
| `node/` | ROS 2 节点，负责任务与算法结构体互转 | 不含算法逻辑 |
| `config/` | 唯一配置文件 `scanlio.yaml` | 六段共 35 项 |

使用实机时需自行获取两个雷达包并放入 `src/`；仅回放已录制数据时不需要。

## 依赖

Ubuntu 26.04 + ROS 2 Lyrical 与 Ubuntu 22.04 + Humble 均构建通过。

| 依赖 | 来源 |
|---|---|
| Eigen3 | `libeigen3-dev` |
| yaml-cpp | `libyaml-cpp-dev` |
| `rclcpp` `sensor_msgs` `nav_msgs` `geometry_msgs` `tf2_ros` `ament_index_cpp` | ROS 2 |
| `libpcap-dev` `libpcl-all-dev` `libboost-thread-dev` | 仅两个雷达包需要 |

Sophus 以源码形式位于 `src/scanlio/3rdparty/sophus/`，无需单独安装。

## 构建

```bash
cd slam_ws
colcon build --packages-select scanlio
```

上述命令只构建算法包，不依赖两个雷达包；构建全部包需要完整 ROS 2 环境与上表列出的系统依赖。

`scanlio_link_dep` 兼容两个发行版的 CMake 目标差异：Lyrical 为每个包导出聚合目标 `包名::包名`，Humble 仅提供 `ament_target_dependencies` 宏。该函数先检查聚合目标是否存在，不存在时退回宏调用。

## 运行

回放数据包：

```bash
source /opt/ros/<distro>/setup.bash
source install/setup.bash
ros2 launch scanlio scanlio_node.launch.py
```

实机整合启动（雷达驱动、底盘驱动、算法节点）：

```bash
ros2 launch scanlio scanlio_bringup.launch.py
```

串口设备名颠倒时覆盖默认值：

```bash
ros2 launch scanlio scanlio_bringup.launch.py lidar_port:=/dev/ttyACM1 base_port:=/dev/ttyACM0
```

整合启动在加载阶段调用 `get_package_share_directory("lslidar_driver")`，缺该包时以 `PackageNotFoundError` 中止，此时改用单节点启动配合已独立运行的雷达驱动。

## 接口

### 订阅

| 话题 | 类型 | 内容 |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | 单线激光一圈测距 |
| `/imu` | `sensor_msgs/Imu` | 角速度与比力 |
| `/robotvel` | `origincar_msg/Data` | 轮速；`x` 为车体前向速度，`y`、`z` 在该底盘上恒为 0 |

### 发布

| 话题 | 类型 | 内容 |
|---|---|---|
| `/trajectory` | `nav_msgs/Path` | 每处理一帧追加一个位姿 |
| `/map` | `nav_msgs/OccupancyGrid` | 占据栅格，按固定周期发布 |
| `/global_point_cloud` | `sensor_msgs/PointCloud2` | NDT 体素中心 |
| `/scanlio/pose_with_covariance` | `geometry_msgs/PoseWithCovarianceStamped` | 当前位姿与协方差 |

坐标变换：`map → 车体` 每帧更新；`车体 → 雷达`、`车体 → IMU` 为静态恒等变换，每 2 秒重发。

`/map` 的 QoS 为 `RELIABLE + TRANSIENT_LOCAL`，其余话题为默认 `VOLATILE`。原因是算法节点通常先于 rviz 启动，默认 QoS 下后启动的订阅者需等待下一个发布周期，若该周期无新地图则画面持续空白。

### 参数

两套参数管理的键不重叠。

ROS 参数 21 个，管对外接口：话题名、坐标系名、各输出的开关与话题名、发布周期、排序缓冲窗口。

yaml 参数位于 `config/scanlio.yaml`，六段 35 项：

| 段 | 内容 |
|---|---|
| `init` | 静止标定得到的初始零偏 |
| `eskf` | 松耦合滤波器噪声与协方差，键名 `*_var`，值直接作方差 |
| `ieskf` | 紧耦合滤波器噪声，键名 `*_std`，值为标准差，代码内部平方 |
| `ndt` | 配准参数 |
| `map` | 占据栅格分辨率 |
| `frontend` | 关键帧判定、去畸变开关、滤波器路由 `filter_type` |

两套通过 ROS 参数 `config_file` 连接。该参数为空时节点用 `get_package_share_directory("scanlio")` 定位到 `install/` 下的副本；修改源码树中的 yaml 不影响该默认路径，需重新构建。

`eskf` 与 `ieskf` 分段而不共用数值，原因是两段语义相反（方差与标准差）。共用一组数值会使量级静默变化 3 个数量级。

## 算法

### 配准

2D NDT。地图划分为边长 0.65 m 的体素，每体素统计落入点的均值与协方差，配准以高斯-牛顿迭代求位姿增量。每个点先在自身与周围八个体素中选择马氏距离最小者作为对应，马氏距离超过卡方阈值的点不参与本轮累加，用于抑制外点对迭代方向的影响。体素按最近使用顺序管理，超出数量上限时淘汰最久未更新者。

### 滤波

状态 8 维：`p(2) v(2) θ bg ba(2)`。零偏为滤波器状态而非常量，每次观测更新后写回名义状态。两条路径并列，状态定义相同。

| | `ESKF`（松耦合，默认） | `IESKF2D`（紧耦合） |
|---|---|---|
| NDT 输出 | 迭代收敛后的位姿 | 逐点累加的 `ΣJᵀΩJ` 与 `Σ-JᵀΩr`，不更新位姿 |
| 迭代位置 | `NDT2D::match` 内，`max_iteration_` 次 | `UpdateUsingCustomObserve` 内，`num_iterations_` 次 |
| 雷达观测 | 3 维位姿 | 每点 2 维，8 维状态雅可比 |
| 雷达噪声 | 固定 `lidar_pos_noise_` / `lidar_ang_noise_` | 由体素协方差逐点给出 |
| 轮速观测 | 1 维，车体前向速度 | 2 维，车体 `(v, 0)` 旋转至世界系 |
| 入图位姿 | 由 `use_filtered_pose_for_map` 选择配准位姿或滤波位姿 | 配准与滤波为同一量，该开关无分支 |

紧耦合轮速观测的第二维不是测量值。底盘消息含横向速度字段，但固件不填充，实测恒为 0；该维表达的是"车体不能横移"这一差速底盘运动学假设。

两条路径由 `frontend.filter_type` 切换，默认 `eskf`，默认路径下既有指标与 0.9.3 逐位一致。

### 去畸变

一帧扫描历时 0.1 s，期间车体移动。程序按每点采样时刻从 IMU 缓冲插值出该时刻位姿，将点统一搬至帧尾时刻坐标系，避免把运动位移计入观测形状。

### 地图

射线追踪计数法。射线沿途体素记穿过，终点体素记命中，占据概率为命中次数除以总次数。容器用有序 `std::map` 而非哈希表，使同一输入每次遍历顺序相同，输出可复现。

### 调度

三个话题的消息进入按时间戳排序的缓冲，窗口满足后按时间顺序统一推进。用于消除回调触发顺序与时间戳顺序不一致的问题，代价是引入等长时延。

## 已知限制

| 限制 | 成因 | 影响 |
|---|---|---|
| 仅支持单线 2D 雷达 | 位姿为平面三自由度，`z` 恒为 0 | 无法处理三维场景 |
| 无回环检测 | 未实现 | 长距离累积漂移不可消除 |
| 无退化检测、无零速更新 | 未实现 | 走廊横向约束弱、贴墙、空旷处配准不施加专门抑制；堵转不漂移是"位置仅由雷达位姿观测更新"这一结构的副作用，非专门设计 |
| 去畸变用线性插值 | 帧内运动模型简化 | 高速转弯时误差增大 |
| 参数按 N10 标定 | 444 点/圈、10 Hz | 换雷达需重新标定 `ndt.voxel_size` 与 `ndt.min_pts_num`；后者必须低于单个体素的典型点数，否则大量体素不生效 |
| 紧耦合三项参数未标定 | `num_iterations`、`eps`、`info_ratio` 沿用原版取值 | 本项目体素信息矩阵量级与原版不同，两路径在长走廊末段分歧可达米级；现有数据缺独立真值，无法判定优劣，故紧耦合仅作对照不设为默认 |

## 版本与发布

版本号写在三处，改版时同步：`src/scanlio/package.xml` 的 `<version>`、git tag 与对应 GitHub Release、本文件开头的当前版本。

仓库名不含版本号，版本更迭通过 tag 与 Release 表达。

1.0.1 修正 1.0.0 的文档表述与版本口径，接口与行为未变。

1.0.0 定义为接口稳定版：配置文件的分段与键名、滤波路径切换方式、话题与参数集合在后续版本中保持兼容；新增功能以可选开关加入，不改变默认行为。

## 许可

自有代码采用 Apache License 2.0，全文见 `LICENSE`，版权声明见 `NOTICE`。随仓库分发的第三方组件许可见 `THIRD_PARTY.md`。两个雷达包为 GPL-3.0，不随本仓库分发。

## 不在仓库中的内容

数据集（rosbag）由使用者自行录制。验证程序与实验记录位于被 `.gitignore` 排除的 `test/` 与 `doc/`，仓库可构建的内容为算法本体与节点。
