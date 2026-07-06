from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    package_share = get_package_share_directory("handeye_calibration_tool")
    config_file = os.path.join(package_share, "config", "handeye_tool.yaml")

    return LaunchDescription([
        Node(
            package="handeye_calibration_tool",
            executable="handeye_tool",
            name="handeye_calibration_tool",
            output="screen",
            emulate_tty=True,
            parameters=[config_file],
        )
    ])
