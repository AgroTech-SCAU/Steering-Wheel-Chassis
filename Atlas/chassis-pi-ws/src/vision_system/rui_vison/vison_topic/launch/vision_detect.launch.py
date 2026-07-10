"""RUI 视觉检测服务启动文件"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory('vison_topic'))
    default_config = str(package_share / 'config' / 'vision_runtime.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument('model', default_value=''),
        DeclareLaunchArgument('labels', default_value=''),
        DeclareLaunchArgument('camera', default_value='/dev/atlas_camera'),
        DeclareLaunchArgument('conf', default_value='0.55'),
        DeclareLaunchArgument('process_every_n', default_value='2'),
        DeclareLaunchArgument('rate_hz', default_value='10'),
        DeclareLaunchArgument('service_name', default_value='vision_detect'),
        DeclareLaunchArgument('topic_name', default_value='vision_detections'),
        Node(
            package='vison_topic',
            executable='vision_detect_server',
            name='vision_detect_server',
            output='screen',
            arguments=[
                '--config', LaunchConfiguration('config'),
                '--model', LaunchConfiguration('model'),
                '--labels', LaunchConfiguration('labels'),
                '--camera', LaunchConfiguration('camera'),
                '--conf', LaunchConfiguration('conf'),
                '--process-every-n', LaunchConfiguration('process_every_n'),
                '--rate-hz', LaunchConfiguration('rate_hz'),
                '--service-name', LaunchConfiguration('service_name'),
                '--topic-name', LaunchConfiguration('topic_name'),
                '--no-preview',
            ],
            respawn=True,
            respawn_delay=2.0,
        ),
    ])
