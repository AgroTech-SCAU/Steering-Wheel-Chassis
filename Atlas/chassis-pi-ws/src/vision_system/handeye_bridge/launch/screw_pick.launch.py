"""螺丝抓取完整启动。

用法:
  ros2 launch handeye_bridge screw_pick.launch.py                        # 部署
  ros2 launch handeye_bridge screw_pick.launch.py no_preview:=false      # 调试(带预览)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory("handeye_bridge")
    config_file = os.path.join(pkg_share, "config", "bridge_node.yaml")

    ld = LaunchDescription([
        DeclareLaunchArgument('no_preview', default_value='true',
                              description='关闭 OpenCV 预览窗口'),
    ])

    # ── 视觉检测 ──
    vison_args = [
        '--camera', '0',
        '--conf', '0.55',
        '--process-every-n', '2',
        '--rate-hz', '15',
    ]

    ld.add_action(Node(
        package='vison_topic', executable='vision_detect_server',
        name='vision_detect_server', output='screen',
        arguments=vison_args + ['--no-preview'],
        condition=IfCondition(LaunchConfiguration('no_preview')),
        respawn=True, respawn_delay=2.0,
    ))
    ld.add_action(Node(
        package='vison_topic', executable='vision_detect_server',
        name='vision_detect_server', output='screen',
        arguments=vison_args,
        condition=UnlessCondition(LaunchConfiguration('no_preview')),
        respawn=True, respawn_delay=2.0,
    ))

    # ── 手眼桥 ──
    ld.add_action(Node(
        package='handeye_bridge', executable='bridge_node',
        name='handeye_bridge', output='screen',
        parameters=[config_file],
        respawn=True, respawn_delay=2.0,
    ))

    return ld
