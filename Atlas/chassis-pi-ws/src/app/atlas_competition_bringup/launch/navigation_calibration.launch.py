"""Atlas 智械争锋导航点位标定入口"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_file(package: str, relpath: str) -> str:
    return os.path.join(get_package_share_directory(package), relpath)


def generate_launch_description():
    competition_config = LaunchConfiguration("competition_config")
    default_competition_config = os.path.join(
        get_package_share_directory("atlas_competition_bringup"),
        "config",
        "competition.yaml",
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable("RCUTILS_LOGGING_USE_STDOUT", "1"),
            DeclareLaunchArgument(
                "competition_config",
                default_value=default_competition_config,
                description="顶层比赛 YAML，必须先填写 map/pbstream 路径",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    _launch_file("mcu_comm_bridge", "launch/mcu_comm_bridge.launch.py")
                ),
                launch_arguments={
                    "output": "log",
                    "stats_enabled": "false",
                    "log_level": "fatal",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    _launch_file(
                        "robot_description", "launch/robot_description.launch.py"
                    )
                ),
                launch_arguments={"output": "log", "log_level": "fatal"}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    _launch_file("lslidar_driver", "launch/lsn10p_launch.py")
                ),
                launch_arguments={"output": "log", "log_level": "fatal"}.items(),
            ),
            Node(
                package="atlas_competition_bringup",
                executable="navigation_calibration.py",
                name="atlas_navigation_calibration",
                output="screen",
                emulate_tty=True,
                parameters=[{"competition_config": competition_config}],
            ),
        ]
    )
