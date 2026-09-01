import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def generate_launch_description():
    at_nav_dir = get_package_share_directory('at_nav2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    at_params_file = os.path.join(at_nav_dir, 'config', 'at_nav2_params.yaml')
    at_map_file = os.path.join(at_nav_dir, 'maps', 'ruikang.yaml')
    pbstream_file = os.path.join(at_nav_dir, 'maps', 'ruikang.pbstream')

    params_file = LaunchConfiguration('params_file')
    map_file = LaunchConfiguration('map')
    cartographer_pbstream = LaunchConfiguration('pbstream')
    cmd_vel_output = LaunchConfiguration('cmd_vel_output')

    # Cartographer 负责纯定位，输出 map -> odom TF。
    # 完整导航后端只发送 NavigateToPose 目标，不直接接触定位实现。
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=[
            '-configuration_directory', os.path.join(at_nav_dir, 'config'),
            '-configuration_basename', 'cartographer_localization.lua',
            '-load_state_filename', cartographer_pbstream,
        ],
        remappings=[
            ('scan', '/scan'),
            ('odom', '/odom'),
        ],
    )

    # map_server 独立启动，只提供静态地图；不启动 AMCL，避免与 Cartographer 争夺 map->odom。
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': False, 'yaml_filename': map_file}],
    )

    map_server_lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{'use_sim_time': False, 'autostart': True, 'node_names': ['map_server']}],
    )

    # Nav2 默认发布 /cmd_vel。
    # 这里统一 remap 到 /atlas/navigation/cmd_vel，再交给 atlas_mission_manager 做安全门控。
    navigation_cmd_vel_remap = SetRemap(src='/cmd_vel', dst=cmd_vel_output)

    navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'False',
            'params_file': params_file,
            'autostart': 'True',
            'use_composition': 'False',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=at_params_file, description='Nav2 参数文件'),
        DeclareLaunchArgument('map', default_value=at_map_file, description='map_server 使用的 YAML 地图'),
        DeclareLaunchArgument('pbstream', default_value=pbstream_file, description='Cartographer 纯定位 pbstream 地图'),
        DeclareLaunchArgument('cmd_vel_output', default_value='/atlas/navigation/cmd_vel', description='Nav2 速度输出 remap 目标'),
        cartographer_node,
        map_server_node,
        map_server_lifecycle_node,
        navigation_cmd_vel_remap,
        navigation_cmd,
    ])
