from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("atlas_mission_manager"))
    default_config = str(share / "config" / "mission_manager.yaml")
    default_route = str(share / "config" / "mission_route.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default_config),
        DeclareLaunchArgument("route", default_value=default_route),
        Node(
            package="atlas_mission_manager",
            executable="atlas_mission_manager_node",
            name="atlas_mission_manager",
            output="screen",
            parameters=[LaunchConfiguration("config"), {"route_yaml_path": LaunchConfiguration("route")}],
        ),
    ])
