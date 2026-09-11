"""Atlas 智械争锋机械臂运动关键帧标定入口"""

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
from launch_ros.parameter_descriptions import ParameterValue


def _launch_file(package: str, relpath: str) -> str:
    return os.path.join(get_package_share_directory(package), relpath)


def generate_launch_description():
    competition_config = LaunchConfiguration("competition_config")
    no_preview = LaunchConfiguration("no_preview")
    direct_nav_config = os.path.join(
        get_package_share_directory("atlas_nav_direct_backend"), "config", "direct_nav.yaml"
    )
    default_competition_config = os.path.join(
        get_package_share_directory("atlas_competition_bringup"), "config", "competition.yaml"
    )

    return LaunchDescription([
        # rcutils writes ROS logs to stderr by default. Redirect them to stdout
        # so output="log" reliably keeps background output off this terminal.
        SetEnvironmentVariable("RCUTILS_LOGGING_USE_STDOUT", "1"),
        DeclareLaunchArgument(
            "competition_config",
            default_value=default_competition_config,
            description="已有顶层比赛 YAML  标定结果会基于它生成新 YAML",
        ),
        DeclareLaunchArgument(
            "no_preview",
            default_value="false",
            description="true=不显示相机画面 false=自动打开 Vision Detection",
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
                _launch_file("robot_description", "launch/robot_description.launch.py")
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
            package="atlas_nav_direct_backend",
            executable="direct_nav_backend",
            name="atlas_nav_direct_backend",
            output="log",
            ros_arguments=["--log-level", "fatal"],
            parameters=[
                direct_nav_config,
                {
                    "backend_name": "direct_odom_competition",
                    "competition_config": competition_config,
                },
            ],
        ),
        Node(
            package="atlas_competition_bringup",
            executable="arm_motion_calibration.py",
            name="atlas_arm_motion_calibration",
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    "competition_config": competition_config,
                    "no_preview": ParameterValue(no_preview, value_type=bool),
                }
            ],
        ),
    ])
