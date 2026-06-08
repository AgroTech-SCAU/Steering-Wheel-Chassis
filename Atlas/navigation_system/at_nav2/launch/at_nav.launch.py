import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    at_nav_dir = get_package_share_directory('at_nav2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    at_params_file = os.path.join(at_nav_dir, 'config', 'at_nav2_params.yaml')
    at_map_file = os.path.join(at_nav_dir, 'maps', 'map.yaml')
    rviz_config = os.path.join(at_nav_dir, 'rviz2', 'at_nav2.rviz')
    pbstream_file = os.path.join(at_nav_dir, 'maps', 'map.pbstream')

    # Cartographer pure localization 节点
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=[
            '-configuration_directory', os.path.join(at_nav_dir, 'config'),
            '-configuration_basename', 'cartographer_localization.lua',
            '-load_state_filename', pbstream_file,
        ],
        remappings=[
            ('scan', '/scan'),
            ('odom', '/odom'),
        ],
    )

    # ================================================================
    # 架构说明：
    #   - Cartographer 负责纯定位（map -> odom TF）
    #   - map_server 独立启动（提供静态地图）
    #   - navigation_launch.py 只启动 Nav2 导航栈（planner/controller/bt/smoother）
    #   - 不启动 AMCL（避免与 Cartographer 冲突）
    #
    # navigation_launch.py 接收参数：
    #   use_sim_time, params_file, autostart, use_composition,
    #   use_respawn, log_level, namespace, container_name
    # ================================================================
    # controller_server 默认发 /cmd_vel
    # FSM 仲裁后将最终指令发到 /motor_cmd_vel，底盘驱动订阅 /motor_cmd_vel
    
    # map_server 节点（替代 localization_launch.py 中的 map_server，但不启动 AMCL）
    
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[at_params_file, {'use_sim_time': False, 'yaml_filename': at_map_file}],
    )

    # map_server 生命周期管理器
    map_server_lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{'use_sim_time': False, 'autostart': True, 'node_names': ['map_server']}],
    )

    # 只启动 Nav2 导航栈（不含 AMCL），Cartographer 负责提供 map->odom TF
    navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'False',
            'params_file': at_params_file,
            'autostart': 'True',
            'use_composition': 'False',
        }.items(),
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    ld = LaunchDescription()
    ld.add_action(cartographer_node)
    ld.add_action(map_server_node)
    ld.add_action(map_server_lifecycle_node)
    ld.add_action(navigation_cmd)
    ld.add_action(rviz2_node)

    return ld
