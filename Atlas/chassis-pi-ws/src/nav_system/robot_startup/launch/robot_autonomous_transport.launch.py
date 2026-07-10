"""Atlas 智械争锋全自主运输区整车启动文件。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def include_launch(package_name: str, relative_path: str, launch_arguments=None):
    """按功能包名称包含一个启动文件。"""
    package_share = get_package_share_directory(package_name)
    launch_path = os.path.join(package_share, relative_path)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_path),
        launch_arguments=(launch_arguments or {}).items(),
    )


def generate_launch_description() -> LaunchDescription:
    navigation_backend = LaunchConfiguration('navigation_backend')
    manipulation_backend = LaunchConfiguration('manipulation_backend')
    manager_config = LaunchConfiguration('manager_config')
    enable_voice_player = LaunchConfiguration('enable_voice_player')
    voice_audio_device = LaunchConfiguration('voice_audio_device')

    manager_share = get_package_share_directory(
        'atlas_autonomous_transport_manager'
    )
    default_manager_config = os.path.join(
        manager_share, 'config', 'autonomous_transport.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'navigation_backend',
            default_value='full',
            description='全自主运输导航后端：full 或 pseudo',
        ),
        DeclareLaunchArgument(
            'manipulation_backend',
            default_value='racom_vision',
            description='全自主运输机械臂后端',
        ),
        DeclareLaunchArgument(
            'manager_config',
            default_value=default_manager_config,
            description='全自主运输场地坐标与策略配置',
        ),
        DeclareLaunchArgument('enable_voice_player', default_value='true'),
        DeclareLaunchArgument('voice_audio_device', default_value='default'),

        # MCU 通信桥提供里程计、IMU、机械臂状态、底盘速度入口和执行机构服务。
        include_launch('mcu_comm_bridge', 'launch/mcu_comm_bridge.launch.py'),

        # 激光雷达驱动向完整导航提供 /scan。
        include_launch('lslidar_driver', 'launch/lsn10p_launch.py'),

        # 机器人模型提供底盘与传感器静态 TF。
        include_launch('robot_description', 'launch/robot_description.launch.py'),

        # 全自主运输任务栈内部启动导航、RACOM 视觉、机械臂动作后端和任务状态机。
        include_launch(
            'atlas_autonomous_transport_manager',
            'launch/autonomous_transport_stack.launch.py',
            {
                'navigation_backend': navigation_backend,
                'manipulation_backend': manipulation_backend,
                'manager_config': manager_config,
                'enable_voice_player': enable_voice_player,
                'voice_audio_device': voice_audio_device,
            },
        ),
    ])
