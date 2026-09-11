from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('mcu_comm_bridge')
    config = os.path.join(pkg_share, 'config', 'mcu_comm_bridge.yaml')
    output = LaunchConfiguration("output")
    stats_enabled = LaunchConfiguration("stats_enabled")
    log_level = LaunchConfiguration("log_level")

    return LaunchDescription([
        DeclareLaunchArgument("output", default_value="screen"),
        DeclareLaunchArgument("stats_enabled", default_value="true"),
        DeclareLaunchArgument("log_level", default_value="info"),
        Node(
            package='mcu_comm_bridge',
            executable='mcu_comm_bridge_node',
            name='mcu_comm_bridge_node',
            output=output,
            ros_arguments=["--log-level", log_level],
            parameters=[
                config,
                {"stats_enabled": ParameterValue(stats_enabled, value_type=bool)},
            ],
        )
    ])
