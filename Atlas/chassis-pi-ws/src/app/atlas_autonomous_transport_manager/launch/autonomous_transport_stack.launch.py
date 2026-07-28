"""启动 Atlas 全自主运输区所需的导航、视觉、机械臂和状态机节点。"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def value_equals(value: LaunchConfiguration, expected: str) -> IfCondition:
    """生成字符串启动参数相等条件。"""
    return IfCondition(PythonExpression(["'", value, "' == '", expected, "'"]))


def generate_launch_description() -> LaunchDescription:
    manager_share = Path(
        get_package_share_directory('atlas_autonomous_transport_manager')
    )
    pseudo_nav_share = Path(get_package_share_directory('atlas_nav_pseudo_backend'))
    at_nav_share = Path(get_package_share_directory('at_nav2'))
    manipulation_share = Path(
        get_package_share_directory('atlas_vision_pollination_backend')
    )
    racom_share = Path(get_package_share_directory('atlas_racom_vision_backend'))

    default_manager_config = str(
        manager_share / 'config' / 'autonomous_transport.yaml'
    )
    default_pseudo_nav_config = str(
        pseudo_nav_share / 'config' / 'pseudo_nav.yaml'
    )
    # 全自主运输停车点使用 map 绝对坐标，因此采用本功能包的专用后端参数。
    default_full_nav_config = str(
        manager_share / 'config' / 'autonomous_full_nav.yaml'
    )
    default_manip_config = str(
        manipulation_share / 'config' / 'pollination.yaml'
    )
    default_actions = str(
        manipulation_share / 'config' / 'transport_actions.yaml'
    )
    default_racom_camera_config = str(
        racom_share / 'config' / 'racom_camera_target.yaml'
    )
    default_sorting_rule_config = str(
        racom_share / 'config' / 'sorting_rule.yaml'
    )
    default_at_nav_launch = str(at_nav_share / 'launch' / 'at_nav.launch.py')

    navigation_backend = LaunchConfiguration('navigation_backend')
    manipulation_backend = LaunchConfiguration('manipulation_backend')
    navigation_cmd_vel_topic = LaunchConfiguration('navigation_cmd_vel_topic')

    return LaunchDescription([
        DeclareLaunchArgument(
            'manager_config',
            default_value=default_manager_config,
            description='全自主运输任务配置',
        ),
        DeclareLaunchArgument(
            'navigation_backend',
            default_value='full',
            description='导航后端：full 或 pseudo',
        ),
        DeclareLaunchArgument(
            'manipulation_backend',
            default_value='racom_vision',
            description='机械臂任务后端名称',
        ),
        DeclareLaunchArgument(
            'navigation_cmd_vel_topic',
            default_value='/atlas/navigation/cmd_vel',
            description='导航速度进入状态机安全门控前的话题',
        ),
        DeclareLaunchArgument(
            'pseudo_nav_config', default_value=default_pseudo_nav_config
        ),
        DeclareLaunchArgument(
            'full_nav_config', default_value=default_full_nav_config
        ),
        DeclareLaunchArgument(
            'manip_config', default_value=default_manip_config
        ),
        DeclareLaunchArgument('actions', default_value=default_actions),
        DeclareLaunchArgument(
            'racom_camera_config', default_value=default_racom_camera_config
        ),
        DeclareLaunchArgument(
            'sorting_rule_config', default_value=default_sorting_rule_config
        ),
        DeclareLaunchArgument('racom_camera_id', default_value='0'),
        DeclareLaunchArgument(
            'enable_voice_player',
            default_value='true',
            description='是否启动 espeak + aplay 播报节点',
        ),
        DeclareLaunchArgument(
            'voice_audio_device',
            default_value='default',
            description='aplay 输出设备，例如 default 或 plughw:2,0',
        ),

        # 完整导航模式启动定位、地图服务与 Nav2。
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(default_at_nav_launch),
            launch_arguments={
                'cmd_vel_output': navigation_cmd_vel_topic,
            }.items(),
            condition=value_equals(navigation_backend, 'full'),
        ),
        Node(
            package='atlas_nav_full_backend',
            executable='full_nav_backend',
            name='atlas_nav_full_backend',
            output='screen',
            parameters=[
                LaunchConfiguration('full_nav_config'),
                {'backend_name': navigation_backend},
            ],
            condition=value_equals(navigation_backend, 'full'),
        ),

        # 伪导航用于不启动 Nav2 的接口与状态机联调。
        Node(
            package='atlas_nav_pseudo_backend',
            executable='pseudo_nav_backend',
            name='atlas_nav_pseudo_backend',
            output='screen',
            parameters=[
                LaunchConfiguration('pseudo_nav_config'),
                {'backend_name': navigation_backend},
            ],
            condition=value_equals(navigation_backend, 'pseudo'),
        ),

        # RACOM 检测节点输出分类名称与像素中心。
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
            condition=value_equals(manipulation_backend, 'racom_vision'),
        ),

        # 把 RACOM 像素检测结果转换为机械臂动作后端使用的相机目标接口。
        Node(
            package='atlas_racom_vision_backend',
            executable='racom_camera_target_service',
            name='atlas_racom_camera_target_service',
            output='screen',
            parameters=[LaunchConfiguration('racom_camera_config')],
            condition=value_equals(manipulation_backend, 'racom_vision'),
        ),

        # 识别智能分拣区中齿轮与 T 型螺栓对应的园区映射。
        Node(
            package='atlas_racom_vision_backend',
            executable='sorting_rule_service',
            name='atlas_sorting_rule_service',
            output='screen',
            parameters=[LaunchConfiguration('sorting_rule_config')],
            condition=value_equals(manipulation_backend, 'racom_vision'),
        ),

        # 执行视觉抓取、吸盘控制和园区投放动作序列。
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

        # 播放状态机发布的中转区播报和任务提示。
        Node(
            package='atlas_autonomous_transport_manager',
            executable='voice_player',
            name='atlas_voice_player',
            output='screen',
            parameters=[
                {
                    'text_topic': '/atlas/voice/text',
                    'audio_device': LaunchConfiguration('voice_audio_device'),
                },
            ],
            condition=IfCondition(LaunchConfiguration('enable_voice_player')),
        ),

        # 全自主运输生命周期与任务阶段状态机。
        Node(
            package='atlas_autonomous_transport_manager',
            executable='autonomous_transport_manager',
            name='atlas_autonomous_transport_manager',
            output='screen',
            parameters=[
                {
                    'config_yaml_path': LaunchConfiguration('manager_config'),
                    'navigation_backend': navigation_backend,
                    'manipulation_backend': manipulation_backend,
                    'navigation_cmd_vel_topic': navigation_cmd_vel_topic,
                },
            ],
        ),
    ])
