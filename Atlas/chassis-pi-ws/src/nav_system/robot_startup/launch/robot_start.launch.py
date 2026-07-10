"""Atlas PI 端整车总启动文件。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def include_launch(package_name: str, relative_path: str, launch_arguments=None):
    package_share = get_package_share_directory(package_name)
    launch_path = os.path.join(package_share, relative_path)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_path),
        launch_arguments=(launch_arguments or {}).items(),
    )


def generate_launch_description():
    navigation_backend = LaunchConfiguration('navigation_backend')
    manipulation_backend = LaunchConfiguration('manipulation_backend')

    return LaunchDescription([
        DeclareLaunchArgument('navigation_backend', default_value='full', description='任务导航后端：full 或 pseudo'),
        DeclareLaunchArgument('manipulation_backend', default_value='racom_vision', description='任务作业后端：racom_vision 或 vision_pollination'),

        # 1. MCU 通信桥：提供 /odom、/imu、/arm/joint_states、/motor_cmd_vel 和机械臂/吸盘服务。
        include_launch('mcu_comm_bridge', 'launch/mcu_comm_bridge.launch.py'),

        # 2. 激光雷达驱动：完整导航需要 /scan。
        include_launch('lslidar_driver', 'launch/lsn10p_launch.py'),

        # 3. 机器人模型与静态 TF。
        include_launch('robot_description', 'launch/robot_description.launch.py'),

        # 4. 任务总栈：内部根据 navigation_backend 启动完整导航或伪导航，默认 full + racom_vision。
        include_launch(
            'atlas_mission_manager',
            'launch/mission_stack.launch.py',
            {
                'navigation_backend': navigation_backend,
                'manipulation_backend': manipulation_backend,
            },
        ),
    ])
