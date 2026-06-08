"""总启动文件：启动雷达、导航栈、任务管理节点。"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # ── 1. robot_state_publisher ──
    robot_desc_dir = get_package_share_directory('robot_description')
    urdf_file = os.path.join(robot_desc_dir, 'urdf', 'robot_description.urdf')

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}],
    )
    ld.add_action(robot_state_pub)

    # ── 2. LiDAR 驱动 ──
    lslidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('lslidar_driver'),
                         'launch', 'lsn10p_launch.py')
        )
    )
    ld.add_action(lslidar_launch)

    # ── 3. mission_manager ──
    mission_mgr = Node(
        package='mission_manager',
        executable='mission_manager',
        name='mission_manager',
        output='screen',
    )
    ld.add_action(mission_mgr)

    # ── 4. competition_fsm ──
    fsm_node = Node(
        package='competition_fsm',
        executable='competition_fsm_node',
        name='competition_fsm',
        output='screen',
    )
    ld.add_action(fsm_node)

    # ── 5. at_nav2 (Cartographer + Nav2 bringup) ──
    # 延迟启动，等 /scan 和 /odom 就绪
    at_nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('at_nav2'),
                         'launch', 'at_nav.launch.py')
        )
    )
    ld.add_action(TimerAction(period=3.0, actions=[at_nav_launch]))

    return ld
