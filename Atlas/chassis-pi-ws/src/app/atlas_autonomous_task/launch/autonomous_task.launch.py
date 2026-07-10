from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('atlas_autonomous_task'))
    default_config = str(share / 'config' / 'autonomous_task.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config, description='全自主任务配置文件'),
        Node(
            package='atlas_autonomous_task',
            executable='autonomous_task_node',
            name='atlas_autonomous_task',
            output='screen',
            parameters=[LaunchConfiguration('config')],
            respawn=True,
            respawn_delay=2.0,
        ),
    ])
