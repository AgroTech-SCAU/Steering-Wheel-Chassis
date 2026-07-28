"""Atlas PI 端整车总启动文件"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def include_launch(package_name: str, relative_path: str, launch_arguments=None):
    package_share = get_package_share_directory(package_name)
    launch_path = os.path.join(package_share, relative_path)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_path),
        launch_arguments=(launch_arguments or {}).items(),
    )


def generate_launch_description():
    camera = LaunchConfiguration('camera')
    asrpro_port = LaunchConfiguration('asrpro_port')

    return LaunchDescription([
        DeclareLaunchArgument('camera', default_value='0', description='rui_vison 摄像头 ID'),
        DeclareLaunchArgument('asrpro_port', default_value='/dev/ttyUSB0', description='ASRPRO TWEN51 USB 串口'),

        # 1. MCU 通信桥：提供 /odom、/imu、/arm/joint_states、/motor_cmd_vel 和机械臂/吸盘服务
        include_launch('mcu_comm_bridge', 'launch/mcu_comm_bridge.launch.py'),

        # 2. 激光雷达驱动：完整导航需要 /scan
        include_launch('lslidar_driver', 'launch/lsn10p_launch.py'),

        # 3. 机器人模型与静态 TF
        include_launch('robot_description', 'launch/robot_description.launch.py'),

        # 4. 完整导航：直接启动已有 at_nav2，不再经过 atlas_nav_full_backend
        include_launch(
            'at_nav2',
            'launch/at_nav.launch.py',
            {'cmd_vel_output': '/motor_cmd_vel'},
        ),

        # 5. 导航目标发送节点：总控只调用 /navigate_to_target
        Node(
            package='send_navigation_target',
            executable='send_navigation_target',
            name='send_navigation_target',
            output='screen',
        ),

        # 6. rui_vison 视觉检测服务，目录名保持现有拼写 rui_vison/
        include_launch(
            'vison_topic',
            'launch/vision_detect.launch.py',
            {
                'camera': camera,
                'service_name': 'vision_detect',
                'topic_name': 'vision_detections',
            },
        ),

        # 7. 智械争锋全自主运输状态机
        Node(
            package='atlas_autonomous_task',
            executable='autonomous_task_node',
            name='atlas_autonomous_task',
            output='screen',
            parameters=[{'asrpro_port': asrpro_port}],
        ),
    ])
