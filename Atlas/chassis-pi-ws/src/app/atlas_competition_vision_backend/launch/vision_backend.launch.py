import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("atlas_competition_vision_backend")
    config_file = os.path.join(pkg_share, "config", "vision_backend.yaml")

    return LaunchDescription([
        Node(
            package="atlas_competition_vision_backend",
            executable="vision_backend",
            name="atlas_competition_vision_backend",
            output="screen",
            parameters=[config_file],
            respawn=True,
            respawn_delay=2.0,
        )
    ])
