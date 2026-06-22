from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('mcu_comm_bridge')
    config = os.path.join(pkg_share, 'config', 'mcu_comm_bridge.yaml')

    return LaunchDescription([
        Node(
            package='mcu_comm_bridge',
            executable='mcu_comm_bridge_node',
            name='mcu_comm_bridge_node',
            output='screen',
            parameters=[config],
        )
    ])
