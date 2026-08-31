"""RUI 视觉检测服务启动文件。

用法:
  ros2 launch vison_topic vision_detect.launch.py                       # 默认无预览(部署)
  ros2 launch vison_topic vision_detect.launch.py no_preview:=false     # 带预览窗口(调试)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription([
        DeclareLaunchArgument('camera', default_value='0'),
        DeclareLaunchArgument('conf', default_value='0.55'),
        DeclareLaunchArgument('process_every_n', default_value='2'),
        DeclareLaunchArgument('rate_hz', default_value='15'),
        DeclareLaunchArgument('service_name', default_value='vision_detect'),
        DeclareLaunchArgument('topic_name', default_value='vision_detections'),
        DeclareLaunchArgument('no_preview', default_value='true'),
    ])

    base_args = [
        '--camera', LaunchConfiguration('camera'),
        '--conf', LaunchConfiguration('conf'),
        '--process-every-n', LaunchConfiguration('process_every_n'),
        '--rate-hz', LaunchConfiguration('rate_hz'),
        '--service-name', LaunchConfiguration('service_name'),
        '--topic-name', LaunchConfiguration('topic_name'),
    ]

    # no_preview=true → 加 --no-preview flag
    ld.add_action(Node(
        package='vison_topic', executable='vision_detect_server',
        name='vision_detect_server', output='screen',
        arguments=base_args + ['--no-preview'],
        condition=IfCondition(LaunchConfiguration('no_preview')),
        respawn=True, respawn_delay=2.0,
    ))
    # no_preview=false → 不带 --no-preview
    ld.add_action(Node(
        package='vison_topic', executable='vision_detect_server',
        name='vision_detect_server', output='screen',
        arguments=base_args,
        condition=UnlessCondition(LaunchConfiguration('no_preview')),
        respawn=True, respawn_delay=2.0,
    ))

    return ld
