from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory('atlas_racom_vision_backend'))
    default_config = str(share / 'config' / 'racom_camera_target.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        Node(
            package='atlas_racom_vision_backend',
            executable='racom_camera_target_service',
            name='atlas_racom_camera_target_service',
            output='screen',
            parameters=[LaunchConfiguration('config')],
        ),
    ])
