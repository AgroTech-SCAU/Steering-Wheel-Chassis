from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory('atlas_nav_full_backend'))
    default_config = str(share / 'config' / 'full_nav.yaml')
    competition_config = LaunchConfiguration('competition_config')

    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument('competition_config', default_value=''),
        Node(
            package='atlas_nav_full_backend',
            executable='full_nav_backend',
            name='atlas_nav_full_backend',
            output='screen',
            parameters=[
                LaunchConfiguration('config'),
                {'competition_config': competition_config},
            ],
        ),
    ])
