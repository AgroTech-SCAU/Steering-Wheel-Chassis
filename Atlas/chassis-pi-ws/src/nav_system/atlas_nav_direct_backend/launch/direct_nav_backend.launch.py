import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("atlas_nav_direct_backend")
    default_config = os.path.join(package_share, "config", "direct_nav.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("competition_config", default_value=""),
        DeclareLaunchArgument("backend_name", default_value="direct_odom_competition"),
        Node(
            package="atlas_nav_direct_backend",
            executable="direct_nav_backend",
            name="atlas_nav_direct_backend",
            output="screen",
            parameters=[
                default_config,
                {
                    "competition_config": LaunchConfiguration("competition_config"),
                    "backend_name": LaunchConfiguration("backend_name"),
                },
            ],
            respawn=True,
            respawn_delay=2.0,
        ),
    ])
