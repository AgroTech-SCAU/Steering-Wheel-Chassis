from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    default_config = str(
        Path(get_package_share_directory("atlas_mission_manager"))
        / "config"
        / "mission_manager.yaml"
    )

    config_arg = DeclareLaunchArgument(
        "config",
        default_value=default_config,
        description="Path to atlas_mission_manager parameter YAML",
    )

    node = Node(
        package="atlas_mission_manager",
        executable="atlas_mission_manager_node",
        name="atlas_mission_manager",
        output="screen",
        parameters=[LaunchConfiguration("config")],
    )

    return LaunchDescription([config_arg, node])
