from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('atlas_nav_pseudo_backend'))
    default_config = str(share / 'config' / 'pseudo_nav.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        Node(
            package='atlas_nav_pseudo_backend',
            executable='pseudo_nav_backend',
            name='atlas_nav_pseudo_backend',
            output='screen',
            parameters=[LaunchConfiguration('config')],
        ),
    ])
