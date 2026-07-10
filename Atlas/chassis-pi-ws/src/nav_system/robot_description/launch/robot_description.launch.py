import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    
    pkg_share = get_package_share_directory('robot_description')
    urdf_path = os.path.join(pkg_share, 'urdf', 'robot_description.urdf')

    with open(urdf_path, 'r') as urdf_file:
        robot_description = urdf_file.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': ParameterValue(
            robot_description, value_type=str
        )}],
    )

    return LaunchDescription([
        robot_state_publisher_node,
    ])
