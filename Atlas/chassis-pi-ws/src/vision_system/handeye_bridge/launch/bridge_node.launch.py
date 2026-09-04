from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("handeye_bridge")
    config_file = os.path.join(pkg_share, "config", "bridge_node.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("competition_config", default_value=""),
        Node(
            package="handeye_bridge",
            executable="bridge_node",
            name="handeye_bridge",
            output="screen",
            parameters=[config_file, {"competition_config": LaunchConfiguration("competition_config")}],
            respawn=True,
        )
    ])
