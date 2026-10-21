#!/usr/bin/env python3
"""sio_lio 节点的启动文件。

启动方式：
    ros2 launch sio_lio sio_lio_node.launch.py
    ros2 launch sio_lio sio_lio_node.launch.py config_file:=/abs/path/sio_lio.yaml
    # 回放数据包时避免与包内录制的轨迹冲突：
    ros2 launch sio_lio sio_lio_node.launch.py trajectory_topic:=/sio_lio/trajectory

参数说明：
    config_file        配置文件绝对路径。留空时节点使用安装目录下的
                       share/sio_lio/config/sio_lio.yaml。
    scan_topic         LaserScan 话题，默认 /scan。
    imu_topic          Imu 话题，默认 /imu。
    odom_topic         轮速话题（origincar_msg/Data），默认 /robotvel。
    trajectory_topic   轨迹输出话题，默认 /trajectory。
    use_static_init    true  —— 启动时做静止初始化，需车子静止约 10 s，
                                期间雷达帧被丢弃；
                       false —— 直接使用 yaml 中 init 段的标定零偏（默认）。
    static_init_seconds  静止初始化所需的静止时长，秒。
    reorder_margin     排序缓冲窗口，秒。三个话题的消息按时间戳排序后处理，
                       该窗口用于等待可能迟到的早时间戳消息，代价是等长时延。
    publish_tf         是否发布 map→base 与静态坐标，默认 true。
    map_frame/base_frame/laser_frame/imu_frame  坐标系名称。
    use_sim_time       回放数据包并希望 TF 使用包内时间时可置 true。

注意：节点输出的坐标与轨迹都在 map 坐标系下，地图原点由第一帧扫描位姿定义。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            "config_file",
            default_value="",
            description="配置文件绝对路径；留空则用 share/sio_lio/config/sio_lio.yaml",
        ),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        DeclareLaunchArgument("imu_topic", default_value="/imu"),
        DeclareLaunchArgument("odom_topic", default_value="/robotvel"),
        DeclareLaunchArgument("trajectory_topic", default_value="/trajectory"),
        DeclareLaunchArgument(
            "use_static_init",
            default_value="false",
            description="true=启动时做静止初始化；false=使用 yaml 中的标定零偏",
        ),
        DeclareLaunchArgument("static_init_seconds", default_value="10.0"),
        DeclareLaunchArgument(
            "reorder_margin",
            default_value="0.02",
            description="排序缓冲窗口（秒）。三个话题的消息先按时间戳排序再处理，"
                        "窗口越大越能容忍迟到消息，代价是等长时延",
        ),
        DeclareLaunchArgument("publish_static_tf", default_value="true"),
        DeclareLaunchArgument("publish_tf", default_value="true"),
        DeclareLaunchArgument("map_frame", default_value="map"),
        DeclareLaunchArgument("base_frame", default_value="base"),
        DeclareLaunchArgument("laser_frame", default_value="laser"),
        DeclareLaunchArgument("imu_frame", default_value="imu"),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="回放数据包时置 true，TF 时间戳跟随包内时间",
        ),
    ]

    node = Node(
        package="sio_lio",
        executable="sio_lio_node",
        name="sio_lio_node",
        output="screen",
        parameters=[
            {
                "config_file": LaunchConfiguration("config_file"),
                "scan_topic": LaunchConfiguration("scan_topic"),
                "imu_topic": LaunchConfiguration("imu_topic"),
                "odom_topic": LaunchConfiguration("odom_topic"),
                "trajectory_topic": LaunchConfiguration("trajectory_topic"),
                "use_static_init": ParameterValue(
                    LaunchConfiguration("use_static_init"), value_type=bool
                ),
                "static_init_seconds": ParameterValue(
                    LaunchConfiguration("static_init_seconds"), value_type=float
                ),
                "reorder_margin": ParameterValue(
                    LaunchConfiguration("reorder_margin"), value_type=float
                ),
                "publish_static_tf": ParameterValue(
                    LaunchConfiguration("publish_static_tf"), value_type=bool
                ),
                "publish_tf": ParameterValue(
                    LaunchConfiguration("publish_tf"), value_type=bool
                ),
                "map_frame": LaunchConfiguration("map_frame"),
                "base_frame": LaunchConfiguration("base_frame"),
                "laser_frame": LaunchConfiguration("laser_frame"),
                "imu_frame": LaunchConfiguration("imu_frame"),
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool
                ),
            }
        ],
    )

    return LaunchDescription(args + [node])
