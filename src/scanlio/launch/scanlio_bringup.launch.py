#!/usr/bin/env python3
"""scanlio 实机整合启动文件：雷达驱动 + 底盘驱动 + 算法节点。

启动方式：
    ros2 launch scanlio scanlio_bringup.launch.py
    # 串口设备名颠倒时（先插底盘后插雷达）：
    ros2 launch scanlio scanlio_bringup.launch.py lidar_port:=/dev/ttyACM1 base_port:=/dev/ttyACM0
    # 不用 yaml 标定值、改做在线静止初始化：
    ros2 launch scanlio scanlio_bringup.launch.py use_static_init:=true

本文件只做装配，不含算法逻辑，也不改变各包的内部默认值。装配处统一指定三类量：
    1. 坐标系名称：base_frame / imu_frame / laser_frame，使算法节点发布的 TF 与
       底盘消息的 header.frame_id、雷达 yaml 的 frame_id、URDF 的根坐标系一致；
    2. 话题名称：/scan、/imu、/robotvel，与两个驱动实际发布的名字一致；
    3. 串口设备：雷达与底盘各一个，暴露为参数以应对插拔顺序变化。

TF 树（启动后应当连通）：
    map -> base_link           算法节点每帧配准后发布
    base_link -> laser         算法节点每 2 s 重发（恒等变换）
    base_link -> imu_link      算法节点每 2 s 重发（恒等变换）

注意：
    - 雷达节点名必须保持 lslidar_driver_node，lsn10.yaml 的顶层键
      /lslidar_driver_node 与之对应，改名会导致参数不加载。
    - 雷达驱动继承 rclcpp::Node（不是 LifecycleNode），启动即开始发布，
      不需要手动 configure / activate。但 launch 里仍用 LifecycleNode
      action，与原 lsn10_launch.py 保持一致。
    - 两个驱动的串口节点打开失败时不会退出进程，只会打印错误并周期性重连；
      若 /scan 或 /imu 无数据，先查串口设备名。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue

LIDAR_NODE_NAME = "lslidar_driver_node"
BASE_NODE_NAME = "car_base_v2_node"
SCANLIO_NODE_NAME = "scanlio_node"


def generate_launch_description():
    # ---- 雷达参数文件：随 lslidar_driver 包安装到 share 目录 ----
    lidar_param_file = os.path.join(
        get_package_share_directory("lslidar_driver"),
        "params",
        "lidar_uart_ros2",
        "lsn10.yaml",
    )

    # ---- 算法配置文件：随 scanlio 包安装到 share/scanlio/config/ ----
    # 留空则节点自行用 ament_index_cpp 定位，这里显式给出便于排查实际加载的文件
    sio_config_file = os.path.join(
        get_package_share_directory("scanlio"), "config", "scanlio.yaml"
    )

    args = [
        # 串口设备：两个驱动各占一个，按实际枚举结果传入
        DeclareLaunchArgument(
            "lidar_port",
            default_value="/dev/ttyACM0",
            description="雷达串口设备（lsn10.yaml 中 serial_port_ 的现值）",
        ),
        DeclareLaunchArgument(
            "base_port",
            default_value="/dev/ttyACM1",
            description="底盘串口设备（car_base_v2.launch.py 中 usart_port_name 的现值）",
        ),
        DeclareLaunchArgument("base_baud", default_value="921600"),
        # 坐标系
        DeclareLaunchArgument(
            "map_frame",
            default_value="map",
            description="地图坐标系，算法节点在此坐标系下输出位姿与地图",
        ),
        DeclareLaunchArgument(
            "base_frame",
            default_value="base_link",
            description="车体坐标系，取 URDF 根 base_link（origincar.urdf:7）",
        ),
        DeclareLaunchArgument(
            "imu_frame",
            default_value="imu_link",
            description="IMU 坐标系，取底盘消息 header.frame_id（car_base_v2_node.cpp:326）",
        ),
        DeclareLaunchArgument(
            "laser_frame",
            default_value="laser",
            description="雷达坐标系，取 lsn10.yaml 的 frame_id",
        ),
        # 话题
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        DeclareLaunchArgument("imu_topic", default_value="/imu"),
        DeclareLaunchArgument("odom_topic", default_value="/robotvel"),
        DeclareLaunchArgument("trajectory_topic", default_value="/trajectory"),
        # 算法启动方式与输出开关
        DeclareLaunchArgument(
            "use_static_init",
            default_value="false",
            description="true=启动时做在线静止初始化；false=用 scanlio.yaml 的标定零偏",
        ),
        DeclareLaunchArgument("static_init_seconds", default_value="10.0"),
        DeclareLaunchArgument("publish_tf", default_value="true"),
        DeclareLaunchArgument("publish_static_tf", default_value="true"),
        DeclareLaunchArgument("publish_occupancy", default_value="true"),
        DeclareLaunchArgument("occupancy_topic", default_value="/map"),
        DeclareLaunchArgument("publish_cloud", default_value="true"),
        DeclareLaunchArgument("cloud_topic", default_value="/global_point_cloud"),
        DeclareLaunchArgument("publish_pose_cov", default_value="true"),
        DeclareLaunchArgument("pose_cov_topic", default_value="/scanlio/pose_with_covariance"),
        DeclareLaunchArgument("publish_period_ms", default_value="1000"),
        DeclareLaunchArgument("reorder_margin", default_value="0.02"),
    ]

    # ---- 1. 雷达驱动 ----
    # parameters 列表顺序：先加载 yaml，再用字典覆盖需要由装配处决定的字段。
    # 覆盖 serial_port_ 是为了让设备名由本文件统一给出，而不是依赖 yaml 中的写死值。
    lidar_node = LifecycleNode(
        package="lslidar_driver",
        executable="lslidar_driver_node",
        name=LIDAR_NODE_NAME,
        namespace="",
        output="screen",
        emulate_tty=True,
        parameters=[
            lidar_param_file,
            {
                "serial_port_": LaunchConfiguration("lidar_port"),
                "frame_id": LaunchConfiguration("laser_frame"),
                "scan_topic": LaunchConfiguration("scan_topic"),
            },
        ],
    )

    # ---- 2. 底盘驱动 ----
    # 参数取自 car_base_v2/launch/car_base_v2.launch.py，保持其原有取值；
    # usart_port_name 与波特率改为由本文件给出。
    base_node = Node(
        package="car_base_v2",
        executable="car_base_v2_node",
        name=BASE_NODE_NAME,
        output="screen",
        parameters=[
            {
                "usart_port_name": LaunchConfiguration("base_port"),
                "serial_baud_rate": ParameterValue(
                    LaunchConfiguration("base_baud"), value_type=int
                ),
                # 看门狗：500 ms 无指令自动停车，100 ms 重发
                "cmd_watchdog.enable": True,
                "cmd_watchdog.timeout_ms": 500,
                "cmd_watchdog.resend_ms": 100,
                # 诊断：1 Hz /car_base_v2/diagnostics
                "diag.enable": True,
                "diag.period_ms": 1000,
                # 断连后 1 s 周期无限重试
                "serial_reconnect.enable": True,
                "serial_reconnect.period_ms": 1000,
                "cmd_vel": "cmd_vel",
            }
        ],
    )

    # ---- 3. 算法节点 ----
    sio_node = Node(
        package="scanlio",
        executable="scanlio_node",
        name=SCANLIO_NODE_NAME,
        output="screen",
        parameters=[
            {
                "config_file": sio_config_file,
                "scan_topic": LaunchConfiguration("scan_topic"),
                "imu_topic": LaunchConfiguration("imu_topic"),
                "odom_topic": LaunchConfiguration("odom_topic"),
                "trajectory_topic": LaunchConfiguration("trajectory_topic"),
                "map_frame": LaunchConfiguration("map_frame"),
                "base_frame": LaunchConfiguration("base_frame"),
                "laser_frame": LaunchConfiguration("laser_frame"),
                "imu_frame": LaunchConfiguration("imu_frame"),
                "publish_tf": ParameterValue(
                    LaunchConfiguration("publish_tf"), value_type=bool
                ),
                "publish_static_tf": ParameterValue(
                    LaunchConfiguration("publish_static_tf"), value_type=bool
                ),
                "use_static_init": ParameterValue(
                    LaunchConfiguration("use_static_init"), value_type=bool
                ),
                "static_init_seconds": ParameterValue(
                    LaunchConfiguration("static_init_seconds"), value_type=float
                ),
                "reorder_margin": ParameterValue(
                    LaunchConfiguration("reorder_margin"), value_type=float
                ),
                "publish_occupancy": ParameterValue(
                    LaunchConfiguration("publish_occupancy"), value_type=bool
                ),
                "occupancy_topic": LaunchConfiguration("occupancy_topic"),
                "publish_cloud": ParameterValue(
                    LaunchConfiguration("publish_cloud"), value_type=bool
                ),
                "cloud_topic": LaunchConfiguration("cloud_topic"),
                "publish_pose_cov": ParameterValue(
                    LaunchConfiguration("publish_pose_cov"), value_type=bool
                ),
                "pose_cov_topic": LaunchConfiguration("pose_cov_topic"),
                "publish_period_ms": ParameterValue(
                    LaunchConfiguration("publish_period_ms"), value_type=int
                ),
            }
        ],
    )

    return LaunchDescription(args + [lidar_node, base_node, sio_node])
