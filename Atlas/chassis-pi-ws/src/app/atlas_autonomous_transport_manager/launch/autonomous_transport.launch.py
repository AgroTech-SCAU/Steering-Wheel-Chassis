"""只启动 Atlas 全自主运输区状态机"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory('atlas_autonomous_transport_manager'))
    default_config = str(share / 'config' / 'autonomous_transport.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=default_config,
            description='全自主运输任务流程、场地坐标和动作名称配置',
        ),
        DeclareLaunchArgument(
            'navigation_backend',
            default_value='full',
            description='导航后端名称：full 或 pseudo',
        ),
        DeclareLaunchArgument(
            'manipulation_backend',
            default_value='racom_vision',
            description='机械臂后端名称',
        ),
        DeclareLaunchArgument(
            'navigation_cmd_vel_topic',
            default_value='/atlas/navigation/cmd_vel',
            description='导航后端速度输出，经过状态机安全门控后转发到底盘',
        ),
        Node(
            package='atlas_autonomous_transport_manager',
            executable='autonomous_transport_manager',
            name='atlas_autonomous_transport_manager',
            output='screen',
            parameters=[
                {
                    'config_yaml_path': LaunchConfiguration('config'),
                    'navigation_backend': LaunchConfiguration('navigation_backend'),
                    'manipulation_backend': LaunchConfiguration('manipulation_backend'),
                    'navigation_cmd_vel_topic': LaunchConfiguration('navigation_cmd_vel_topic'),
                },
            ],
        ),
    ])
