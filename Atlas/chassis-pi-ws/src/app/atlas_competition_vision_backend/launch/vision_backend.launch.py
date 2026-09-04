import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("atlas_competition_vision_backend")
    config_file = os.path.join(pkg_share, "config", "vision_backend.yaml")

    competition_config = LaunchConfiguration("competition_config")

    return LaunchDescription([
        DeclareLaunchArgument("competition_config", default_value=""),
        Node(
            package="atlas_competition_vision_backend",
            executable="vision_backend",
            name="atlas_competition_vision_backend",
            output="screen",
            parameters=[config_file, {"competition_config": competition_config}],
            respawn=True,
            respawn_delay=2.0,
        )
    ])
