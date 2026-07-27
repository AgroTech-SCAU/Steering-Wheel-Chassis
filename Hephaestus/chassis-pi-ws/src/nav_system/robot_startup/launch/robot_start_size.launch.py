# "总启动文件：依次启动导航、雷达、MCU 桥接、机器人描述。"
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # ── 1. Nav2 导航栈 ──
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('at_nav2'),
                         'launch', 'at_nav.launch.py')
        )
    ))

    # ── 2. 激光雷达驱动 ──
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('lslidar_driver'),
                         'launch', 'lsn10p_launch.py')
        )
    ))

    # ── 3. MCU 通信桥接 ──
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('mcu_comm_bridge'),
                         'launch', 'mcu_comm_bridge.launch.py')
        )
    ))

    # ── 4. 机器人描述 (robot_state_publisher + joint_state_publisher) ──
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('robot_description'),
                         'launch', 'robot_description.launch.py')
        )
    ))

    # ── 5. 导航目标发送 ──
    ld.add_action(Node(
        package='send_navigation_target',
        executable='send_navigation_target',
        name='send_navigation_target',
        output='screen',
    ))

    return ld
