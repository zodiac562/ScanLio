from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="car_base_v2",
            executable="car_base_v2_node",
            output="screen",
            parameters=[{
                # 串口
                "usart_port_name": "/dev/ttyACM1",
                "serial_baud_rate": 921600,
                # 看门狗 (节点5): 500ms 无指令自动停车, 100ms 重发
                "cmd_watchdog.enable": True,
                "cmd_watchdog.timeout_ms": 500,
                "cmd_watchdog.resend_ms": 100,
                # 诊断 (节点5): 1Hz /car_base_v2/diagnostics
                "diag.enable": True,
                "diag.period_ms": 1000,
                # 重连 (节点5): 断连后固定 1s 无限重试
                "serial_reconnect.enable": True,
                "serial_reconnect.period_ms": 1000,
                # 话题
                "cmd_vel": "cmd_vel",
            }],
        ),
    ])
