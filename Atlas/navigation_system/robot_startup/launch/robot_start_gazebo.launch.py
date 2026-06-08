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

    # ── 2. gazebo仿真 ──

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('robot_gazebo'), 'launch', 'gazebo_sim.launch.py')
        )
    )
    ld.add_action(gazebo_launch)

    

    # ── 3. at_nav2 (Cartographer + Nav2 bringup) ──
    # 延迟启动，等 /scan 和 /odom 就绪
    at_nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('at_nav2'),
                         'launch', 'at_nav_gazebo.launch.py')
        )
    )
    ld.add_action(TimerAction(period=5.0, actions=[at_nav_launch]))

    return ld
