"""启动智械争锋自动区整套链路，用于单独联调自动任务。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_share = get_package_share_directory("atlas_competition_bringup")
    competition_config = LaunchConfiguration("competition_config")
    no_preview = LaunchConfiguration("no_preview")

    return LaunchDescription([
        DeclareLaunchArgument(
            "competition_config",
            default_value=os.path.join(bringup_share, "config", "competition.yaml"),
            description="自动区测试使用的顶层比赛配置",
        ),
        DeclareLaunchArgument(
            "no_preview",
            default_value="true",
            description="是否关闭相机预览窗口",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, "launch", "competition_stack.launch.py")
            ),
            launch_arguments={
                "competition_config": competition_config,
                "no_preview": no_preview,
                "enable_lidar": "true",
                "enable_navigation": "true",
                "enable_vision": "true",
                "enable_manipulation": "true",
                "enable_mission": "true",
            }.items(),
        ),
    ])
