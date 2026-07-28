"""启动 ASRPRO TWEN51 USB 串口桥"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory('atlas_asrpro_bridge'))
    default_config = str(package_share / 'config' / 'asrpro.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        Node(
            package='atlas_asrpro_bridge',
            executable='asrpro_bridge_node',
            name='atlas_asrpro_bridge',
            output='screen',
            parameters=[LaunchConfiguration('config')],
            respawn=True,
            respawn_delay=1.0,
        ),
    ])
