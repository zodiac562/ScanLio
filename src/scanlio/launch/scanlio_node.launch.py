#!/usr/bin/env python3
"""scanlio 节点的启动文件。

启动方式：
    ros2 launch scanlio scanlio_node.launch.py
    ros2 launch scanlio scanlio_node.launch.py config_file:=/abs/path/scanlio.yaml
    # 回放数据包时避免与包内录制的轨迹冲突：
    ros2 launch scanlio scanlio_node.launch.py trajectory_topic:=/scanlio/trajectory

参数说明：
    config_file        配置文件绝对路径。留空时节点使用安装目录下的
                       share/scanlio/config/scanlio.yaml。
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
    publish_occupancy  是否发布 /map 占据栅格，默认 true。栅格每帧写入，
                       按 publish_period_ms 定时整幅发布，未观测格填 -1。
    occupancy_topic    占据栅格话题名，默认 /map。
    publish_cloud      是否发布 /global_point_cloud，默认 true。
    cloud_topic        点云话题名，默认 /global_point_cloud。
    publish_pose_cov   是否发布带协方差的位姿，默认 true，每成功配准一帧一次。
    pose_cov_topic     带协方差位姿话题名，默认 /scanlio/pose_with_covariance。
    publish_period_ms  占据栅格与点云的发布周期（毫秒），默认 1000；
                       0 或负值表示不发布这两项。
    map_frame/base_frame/laser_frame/imu_frame  坐标系名称。
    use_sim_time       回放数据包并希望 TF 使用包内时间时可置 true。
    use_rviz           是否同时启动 rviz2，默认 true。无图形界面时置 false。

注意：节点输出的坐标与轨迹都在 map 坐标系下，地图原点由第一帧扫描位姿定义。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            "config_file",
            default_value="",
            description="配置文件绝对路径；留空则用 share/scanlio/config/scanlio.yaml",
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
        # 三项可视化输出：开关与话题名分开，便于并排对比与避开包内同名话题
        DeclareLaunchArgument("publish_occupancy", default_value="true"),
        DeclareLaunchArgument("occupancy_topic", default_value="/map"),
        DeclareLaunchArgument("publish_cloud", default_value="true"),
        DeclareLaunchArgument("cloud_topic", default_value="/global_point_cloud"),
        DeclareLaunchArgument("publish_pose_cov", default_value="true"),
        DeclareLaunchArgument(
            "pose_cov_topic", default_value="/scanlio/pose_with_covariance"
        ),
        DeclareLaunchArgument(
            "publish_period_ms",
            default_value="1000",
            description="占据栅格与体素点云的发布周期（毫秒）；0 或负值表示不发布",
        ),
        DeclareLaunchArgument("map_frame", default_value="map"),
        DeclareLaunchArgument("base_frame", default_value="base"),
        DeclareLaunchArgument("laser_frame", default_value="laser"),
        DeclareLaunchArgument("imu_frame", default_value="imu"),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="回放数据包时置 true，TF 时间戳跟随包内时间",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="是否同时启动 rviz2；无图形界面（板载 SSH）时置 false",
        ),
    ]

    node = Node(
        package="scanlio",
        executable="scanlio_node",
        name="scanlio_node",
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

    # rviz2：读取包内安装的 scanlio.rviz。无图形界面时由 use_rviz:=false 关闭，
    # 此时 rviz2 进程不启动，算法节点不受影响。
    rviz_config = os.path.join(
        get_package_share_directory("scanlio"), "rviz", "scanlio.rviz"
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": ParameterValue(
            LaunchConfiguration("use_sim_time"), value_type=bool)}],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription(args + [node, rviz])
