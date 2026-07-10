from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def backend_is(name: str, value: LaunchConfiguration) -> IfCondition:
    return IfCondition(PythonExpression(["'", value, "' == '", name, "'"]))


def backend_is_not(name: str, value: LaunchConfiguration) -> IfCondition:
    return IfCondition(PythonExpression(["'", value, "' != '", name, "'"]))


def generate_launch_description() -> LaunchDescription:
    manager_share = Path(get_package_share_directory('atlas_mission_manager'))
    pseudo_nav_share = Path(get_package_share_directory('atlas_nav_pseudo_backend'))
    full_nav_share = Path(get_package_share_directory('atlas_nav_full_backend'))
    at_nav_share = Path(get_package_share_directory('at_nav2'))
    pollination_share = Path(get_package_share_directory('atlas_vision_pollination_backend'))
    racom_adapter_share = Path(get_package_share_directory('atlas_racom_vision_backend'))

    default_manager_config = str(manager_share / 'config' / 'mission_manager.yaml')
    default_route = str(manager_share / 'config' / 'mission_route.yaml')
    default_pseudo_nav_config = str(pseudo_nav_share / 'config' / 'pseudo_nav.yaml')
    default_full_nav_config = str(full_nav_share / 'config' / 'full_nav.yaml')
    default_manip_config = str(pollination_share / 'config' / 'pollination.yaml')
    default_actions = str(pollination_share / 'config' / 'pollination_actions.yaml')
    default_old_camera_config = str(pollination_share / 'config' / 'camera_target.yaml')
    default_racom_camera_config = str(racom_adapter_share / 'config' / 'racom_camera_target.yaml')
    default_at_nav_launch = str(at_nav_share / 'launch' / 'at_nav.launch.py')

    navigation_backend = LaunchConfiguration('navigation_backend')
    manipulation_backend = LaunchConfiguration('manipulation_backend')
    navigation_cmd_vel_topic = LaunchConfiguration('navigation_cmd_vel_topic')

    return LaunchDescription([
        DeclareLaunchArgument('manager_config', default_value=default_manager_config, description='任务状态机参数'),
        DeclareLaunchArgument('route', default_value=default_route, description='任务路线 YAML'),
        DeclareLaunchArgument('navigation_backend', default_value='full', description='导航后端：full 或 pseudo'),
        DeclareLaunchArgument('manipulation_backend', default_value='racom_vision', description='作业后端：racom_vision 或 vision_pollination'),
        DeclareLaunchArgument('navigation_cmd_vel_topic', default_value='/atlas/navigation/cmd_vel', description='导航后端速度输出话题'),
        DeclareLaunchArgument('pseudo_nav_config', default_value=default_pseudo_nav_config),
        DeclareLaunchArgument('full_nav_config', default_value=default_full_nav_config),
        DeclareLaunchArgument('manip_config', default_value=default_manip_config),
        DeclareLaunchArgument('actions', default_value=default_actions),
        DeclareLaunchArgument('old_camera_config', default_value=default_old_camera_config),
        DeclareLaunchArgument('racom_camera_config', default_value=default_racom_camera_config),
        DeclareLaunchArgument('racom_camera_id', default_value='0'),

        # 完整导航：启动 Cartographer 定位 + map_server + Nav2，再启动任务后端适配器。
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(default_at_nav_launch),
            launch_arguments={'cmd_vel_output': navigation_cmd_vel_topic}.items(),
            condition=backend_is('full', navigation_backend),
        ),
        Node(
            package='atlas_nav_full_backend',
            executable='full_nav_backend',
            name='atlas_nav_full_backend',
            output='screen',
            parameters=[LaunchConfiguration('full_nav_config'), {'backend_name': navigation_backend}],
            condition=backend_is('full', navigation_backend),
        ),

        # 伪导航：保留最小实车联调入口，不启动 Nav2。
        Node(
            package='atlas_nav_pseudo_backend',
            executable='pseudo_nav_backend',
            name='atlas_nav_pseudo_backend',
            output='screen',
            parameters=[LaunchConfiguration('pseudo_nav_config'), {'backend_name': navigation_backend}],
            condition=backend_is('pseudo', navigation_backend),
        ),

        # RACOM 视觉：新 ONNX 检测服务 + 兼容旧 DetectCameraTarget 的适配服务。
        Node(
            package='vison_topic',
            executable='vision_detect_server',
            name='racom_vision_detect_server',
            output='screen',
            arguments=[
                '--camera', LaunchConfiguration('racom_camera_id'),
                '--service-name', 'vision_detect',
                '--topic-name', 'vision_detections',
                '--no-preview',
            ],
            condition=backend_is('racom_vision', manipulation_backend),
        ),
        Node(
            package='atlas_racom_vision_backend',
            executable='racom_camera_target_service',
            name='atlas_racom_camera_target_service',
            output='screen',
            parameters=[LaunchConfiguration('racom_camera_config')],
            condition=backend_is('racom_vision', manipulation_backend),
        ),

        # 旧相机目标服务：仅在显式选择旧 vision_pollination 模型时启动。
        Node(
            package='atlas_vision_pollination_backend',
            executable='camera_target_service',
            name='atlas_camera_target_service',
            output='screen',
            parameters=[LaunchConfiguration('old_camera_config')],
            condition=backend_is_not('racom_vision', manipulation_backend),
        ),

        # 动作执行后端继续复用原授粉动作序列；backend_name 改为 racom_vision 即可让任务状态机匹配。
        Node(
            package='atlas_vision_pollination_backend',
            executable='vision_pollination_backend',
            name='atlas_vision_pollination_backend',
            output='screen',
            parameters=[
                LaunchConfiguration('manip_config'),
                {
                    'backend_name': manipulation_backend,
                    'config_yaml_path': LaunchConfiguration('actions'),
                    'vision_service': '/vision/detect_camera_target',
                },
            ],
        ),
        Node(
            package='atlas_mission_manager',
            executable='atlas_mission_manager_node',
            name='atlas_mission_manager',
            output='screen',
            parameters=[
                LaunchConfiguration('manager_config'),
                {
                    'route_yaml_path': LaunchConfiguration('route'),
                    'navigation_backend': navigation_backend,
                    'manipulation_backend': manipulation_backend,
                    'navigation_cmd_vel_topic': navigation_cmd_vel_topic,
                },
            ],
        ),
    ])
