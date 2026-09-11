"""Cartographer-only one-shot localization for Atlas direct odom navigation."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    at_nav_dir = get_package_share_directory("at_nav2")
    default_pbstream = os.path.join(at_nav_dir, "maps", "ruikang.pbstream")
    pbstream = LaunchConfiguration("pbstream")

    cartographer_node = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        name="atlas_startup_cartographer",
        output="screen",
        parameters=[{"use_sim_time": False}],
        arguments=[
            "-configuration_directory", os.path.join(at_nav_dir, "config"),
            "-configuration_basename", "cartographer_localization.lua",
            "-load_state_filename", pbstream,
        ],
        remappings=[
            ("scan", "/scan"),
            ("odom", "/odom"),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "pbstream",
            default_value=default_pbstream,
            description="Cartographer localization state used only for startup field/odom alignment",
        ),
        cartographer_node,
    ])
