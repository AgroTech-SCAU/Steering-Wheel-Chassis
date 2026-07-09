from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    manager_share = Path(get_package_share_directory('atlas_mission_manager'))
    nav_share = Path(get_package_share_directory('atlas_nav_pseudo_backend'))
    manip_share = Path(get_package_share_directory('atlas_vision_pollination_backend'))

    default_manager_config = str(manager_share / 'config' / 'mission_manager.yaml')
    default_route = str(manager_share / 'config' / 'mission_route.yaml')
    default_nav_config = str(nav_share / 'config' / 'pseudo_nav.yaml')
    default_manip_config = str(manip_share / 'config' / 'pollination.yaml')
    default_actions = str(manip_share / 'config' / 'pollination_actions.yaml')
    default_camera_config = str(manip_share / 'config' / 'camera_target.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('manager_config', default_value=default_manager_config),
        DeclareLaunchArgument('route', default_value=default_route),
        DeclareLaunchArgument('nav_config', default_value=default_nav_config),
        DeclareLaunchArgument('manip_config', default_value=default_manip_config),
        DeclareLaunchArgument('actions', default_value=default_actions),
        DeclareLaunchArgument('camera_config', default_value=default_camera_config),

        Node(
            package='atlas_nav_pseudo_backend',
            executable='pseudo_nav_backend',
            name='atlas_nav_pseudo_backend',
            output='screen',
            parameters=[LaunchConfiguration('nav_config')],
        ),
        Node(
            package='atlas_vision_pollination_backend',
            executable='camera_target_service',
            name='atlas_camera_target_service',
            output='screen',
            parameters=[LaunchConfiguration('camera_config')],
        ),
        Node(
            package='atlas_vision_pollination_backend',
            executable='vision_pollination_backend',
            name='atlas_vision_pollination_backend',
            output='screen',
            parameters=[
                LaunchConfiguration('manip_config'),
                {'config_yaml_path': LaunchConfiguration('actions')},
            ],
        ),
        Node(
            package='atlas_mission_manager',
            executable='atlas_mission_manager_node',
            name='atlas_mission_manager',
            output='screen',
            parameters=[
                LaunchConfiguration('manager_config'),
                {'route_yaml_path': LaunchConfiguration('route')},
            ],
        ),
    ])
