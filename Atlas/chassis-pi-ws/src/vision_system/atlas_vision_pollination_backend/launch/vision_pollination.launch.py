from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('atlas_vision_pollination_backend'))
    default_pollination_config = str(share / 'config' / 'pollination.yaml')
    default_actions_config = str(share / 'config' / 'pollination_actions.yaml')
    default_camera_config = str(share / 'config' / 'camera_target.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('pollination_config', default_value=default_pollination_config),
        DeclareLaunchArgument('actions_config', default_value=default_actions_config),
        DeclareLaunchArgument('camera_config', default_value=default_camera_config),
        Node(
            package='atlas_vision_pollination_backend',
            executable='camera_target_service',
            name='atlas_camera_target_service',
            output='screen',
            parameters=[LaunchConfiguration('camera_config')],
        ),
        Node(
            package='atlas_vision_pollination_backend',
            executable='vision_pollination_backend',
            name='atlas_vision_pollination_backend',
            output='screen',
            parameters=[
                LaunchConfiguration('pollination_config'),
                {'config_yaml_path': LaunchConfiguration('actions_config')},
            ],
        ),
    ])
