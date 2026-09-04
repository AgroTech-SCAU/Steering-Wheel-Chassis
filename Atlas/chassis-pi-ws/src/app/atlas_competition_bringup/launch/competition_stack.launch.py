"""Atlas 智械争锋统一启动入口"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_file(package: str, relpath: str) -> str:
    return os.path.join(get_package_share_directory(package), relpath)


def generate_launch_description():
    enable_lidar = LaunchConfiguration('enable_lidar')
    enable_navigation = LaunchConfiguration('enable_navigation')
    enable_vision = LaunchConfiguration('enable_vision')
    enable_manipulation = LaunchConfiguration('enable_manipulation')
    enable_mission = LaunchConfiguration('enable_mission')
    no_preview = LaunchConfiguration('no_preview')
    nav_backend_name = LaunchConfiguration('navigation_backend_name')

    full_nav_config = os.path.join(
        get_package_share_directory('atlas_nav_full_backend'), 'config', 'full_nav.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('enable_lidar', default_value='true'),
        DeclareLaunchArgument('enable_navigation', default_value='true'),
        DeclareLaunchArgument('enable_vision', default_value='true'),
        DeclareLaunchArgument('enable_manipulation', default_value='true'),
        DeclareLaunchArgument('enable_mission', default_value='true'),
        DeclareLaunchArgument('no_preview', default_value='true'),
        DeclareLaunchArgument(
            'navigation_backend_name',
            default_value='nav2_competition',
            description='必须与 atlas_mission_yasmin/config/mission_route.yaml 一致'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                _launch_file('mcu_comm_bridge', 'launch/mcu_comm_bridge.launch.py')),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                _launch_file('robot_description', 'launch/robot_description.launch.py')),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                _launch_file('lslidar_driver', 'launch/lsn10p_launch.py')),
            condition=IfCondition(enable_lidar),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                _launch_file('at_nav2', 'launch/at_nav.launch.py')),
            launch_arguments={'cmd_vel_output': '/atlas/navigation/cmd_vel'}.items(),
            condition=IfCondition(enable_navigation),
        ),
        Node(
            package='atlas_nav_full_backend',
            executable='full_nav_backend',
            name='atlas_nav_full_backend',
            output='screen',
            parameters=[full_nav_config, {'backend_name': nav_backend_name}],
            condition=IfCondition(enable_navigation),
            respawn=True,
            respawn_delay=2.0,
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                _launch_file('handeye_bridge', 'launch/screw_pick.launch.py')),
            launch_arguments={'no_preview': no_preview}.items(),
            condition=IfCondition(enable_vision),
        ),

        TimerAction(
            period=2.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        _launch_file(
                            'atlas_competition_vision_backend',
                            'launch/vision_backend.launch.py')),
                    condition=IfCondition(enable_vision),
                )
            ],
        ),

        TimerAction(
            period=2.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        _launch_file(
                            'atlas_competition_manipulation_backend',
                            'launch/manipulation_backend.launch.py')),
                    condition=IfCondition(enable_manipulation),
                )
            ],
        ),

        TimerAction(
            period=4.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        _launch_file('atlas_mission_yasmin', 'launch/mission_yasmin.launch.py')),
                    condition=IfCondition(enable_mission),
                )
            ],
        ),
    ])
