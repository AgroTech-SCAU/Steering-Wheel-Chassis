"""Atlas 智械争锋全自主运输区唯一整车启动入口"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _package_file(package_name: str, *parts: str) -> str:
    return str(Path(get_package_share_directory(package_name)).joinpath(*parts))


def _resolve_path(value: Any, default_path: str, config_dir: Path) -> str:
    token = str(value or '').strip()
    if not token:
        return default_path
    path = Path(token).expanduser()
    if not path.is_absolute():
        path = config_dir / path
    return str(path.resolve())


def _float(node: Dict[str, Any], key: str, default: float) -> float:
    try:
        return max(0.0, float(node.get(key, default)))
    except (TypeError, ValueError):
        return default


def _delayed(period_s: float, *actions):
    return TimerAction(period=max(0.0, float(period_s)), actions=list(actions))


def _build_stack(context: LaunchContext):
    config_path = Path(LaunchConfiguration('config').perform(context)).expanduser().resolve()
    data = yaml.safe_load(config_path.read_text(encoding='utf-8')) or {}
    system = data.get('system', {}) or {}
    startup = system.get('startup', {}) or {}
    launch_cfg = system.get('launch', {}) or {}
    paths = system.get('paths', {}) or {}
    vision_cfg = system.get('vision', {}) or {}
    runtime_cfg = system.get('runtime', {}) or {}
    config_dir = config_path.parent

    navigation_backend = str(launch_cfg.get('navigation_backend', 'full')).strip().lower()
    manipulation_backend = str(launch_cfg.get('manipulation_backend', 'racom_vision')).strip()
    navigation_cmd_vel_topic = str(
        launch_cfg.get('navigation_cmd_vel_topic', '/atlas/navigation/cmd_vel')
    )
    respawn = bool(runtime_cfg.get('respawn_nodes', True))
    respawn_delay = max(0.5, float(runtime_cfg.get('respawn_delay_s', 2.0)))

    asrpro_delay = _float(startup, 'asrpro_delay_s', 0.0)
    mcu_delay = _float(startup, 'mcu_delay_s', 0.0)
    sensor_delay = _float(startup, 'sensor_delay_s', 1.0)
    navigation_delay = _float(startup, 'navigation_delay_s', 2.0)
    vision_delay = _float(startup, 'vision_delay_s', 3.0)
    manipulation_delay = _float(startup, 'manipulation_delay_s', 4.0)
    manager_delay = _float(startup, 'manager_delay_s', 6.0)

    mcu_config = _resolve_path(
        paths.get('mcu_config'),
        _package_file('mcu_comm_bridge', 'config', 'mcu_comm_bridge.yaml'),
        config_dir,
    )
    asrpro_config = _resolve_path(
        paths.get('asrpro_config'),
        _package_file('atlas_asrpro_bridge', 'config', 'asrpro.yaml'),
        config_dir,
    )
    nav2_params = _resolve_path(
        paths.get('nav2_params'),
        _package_file('at_nav2', 'config', 'at_nav2_params.yaml'),
        config_dir,
    )
    map_yaml = _resolve_path(
        paths.get('map_yaml'),
        _package_file('at_nav2', 'maps', 'ruikang.yaml'),
        config_dir,
    )
    pbstream = _resolve_path(
        paths.get('cartographer_pbstream'),
        _package_file('at_nav2', 'maps', 'ruikang.pbstream'),
        config_dir,
    )
    full_nav_config = _resolve_path(
        paths.get('full_nav_config'),
        _package_file(
            'atlas_autonomous_transport_manager',
            'config',
            'autonomous_full_nav.yaml',
        ),
        config_dir,
    )
    pseudo_nav_config = _resolve_path(
        paths.get('pseudo_nav_config'),
        _package_file('atlas_nav_pseudo_backend', 'config', 'pseudo_nav.yaml'),
        config_dir,
    )
    vision_runtime_config = _resolve_path(
        paths.get('vision_runtime_config'),
        _package_file('vison_topic', 'config', 'vision_runtime.yaml'),
        config_dir,
    )
    vision_model_path = str(paths.get('vision_model_path', '') or '').strip()
    if vision_model_path:
        vision_model_path = _resolve_path(vision_model_path, '', config_dir)
    vision_labels_path = str(paths.get('vision_labels_path', '') or '').strip()
    if vision_labels_path:
        vision_labels_path = _resolve_path(vision_labels_path, '', config_dir)
    racom_camera_config = _resolve_path(
        paths.get('racom_camera_config'),
        _package_file(
            'atlas_racom_vision_backend',
            'config',
            'racom_camera_target.yaml',
        ),
        config_dir,
    )
    sorting_rule_config = _resolve_path(
        paths.get('sorting_rule_config'),
        _package_file(
            'atlas_racom_vision_backend',
            'config',
            'sorting_rule.yaml',
        ),
        config_dir,
    )
    manipulation_config = _resolve_path(
        paths.get('manipulation_config'),
        _package_file(
            'atlas_vision_pollination_backend',
            'config',
            'pollination.yaml',
        ),
        config_dir,
    )
    transport_actions = _resolve_path(
        paths.get('transport_actions'),
        _package_file(
            'atlas_vision_pollination_backend',
            'config',
            'transport_actions.yaml',
        ),
        config_dir,
    )

    actions = []

    # MCU 通信桥提供里程计；IMU；机械臂状态；底盘速度入口和执行机构服务
    # 激光雷达驱动向完整导航提供 /scan
    # 机器人模型提供底盘与传感器静态 TF
    # 全自主运输任务栈统一启动导航；视觉；机械臂动作后端和任务状态机
    # ASRPRO 和 MCU 先建立 USB 串口及状态发布；两者互不阻塞
    actions.append(_delayed(
        asrpro_delay,
        Node(
            package='atlas_asrpro_bridge',
            executable='asrpro_bridge_node',
            name='atlas_asrpro_bridge',
            output='screen',
            parameters=[asrpro_config],
            respawn=respawn,
            respawn_delay=respawn_delay,
        ),
    ))
    actions.append(_delayed(
        mcu_delay,
        Node(
            package='mcu_comm_bridge',
            executable='mcu_comm_bridge_node',
            name='mcu_comm_bridge_node',
            output='screen',
            parameters=[mcu_config],
            respawn=respawn,
            respawn_delay=respawn_delay,
        ),
    ))

    if bool(launch_cfg.get('enable_lidar', True)):
        actions.append(_delayed(
            sensor_delay,
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    _package_file('lslidar_driver', 'launch', 'lsn10p_launch.py')
                )
            ),
        ))

    if bool(launch_cfg.get('enable_robot_description', True)):
        actions.append(_delayed(
            sensor_delay,
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    _package_file(
                        'robot_description',
                        'launch',
                        'robot_description.launch.py',
                    )
                )
            ),
        ))

    if bool(launch_cfg.get('enable_navigation', True)):
        if navigation_backend == 'full':
            actions.append(_delayed(
                navigation_delay,
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        _package_file('at_nav2', 'launch', 'at_nav.launch.py')
                    ),
                    launch_arguments={
                        'params_file': nav2_params,
                        'map': map_yaml,
                        'pbstream': pbstream,
                        'cmd_vel_output': navigation_cmd_vel_topic,
                    }.items(),
                ),
                Node(
                    package='atlas_nav_full_backend',
                    executable='full_nav_backend',
                    name='atlas_nav_full_backend',
                    output='screen',
                    parameters=[
                        full_nav_config,
                        {'backend_name': navigation_backend},
                    ],
                    respawn=respawn,
                    respawn_delay=respawn_delay,
                ),
            ))
        elif navigation_backend == 'pseudo':
            actions.append(_delayed(
                navigation_delay,
                Node(
                    package='atlas_nav_pseudo_backend',
                    executable='pseudo_nav_backend',
                    name='atlas_nav_pseudo_backend',
                    output='screen',
                    parameters=[
                        pseudo_nav_config,
                        {'backend_name': navigation_backend},
                    ],
                    respawn=respawn,
                    respawn_delay=respawn_delay,
                ),
            ))
        else:
            raise RuntimeError(
                f'不支持的 system.launch.navigation_backend={navigation_backend}'
            )

    if bool(launch_cfg.get('enable_vision', True)):
        camera = str(vision_cfg.get('camera', '/dev/atlas_camera'))
        confidence = str(float(vision_cfg.get('confidence_threshold', 0.55)))
        process_every_n = str(max(1, int(vision_cfg.get('process_every_n', 2))))
        rate_hz = str(max(1.0, float(vision_cfg.get('rate_hz', 10.0))))
        service_name = str(vision_cfg.get('service_name', 'vision_detect'))
        topic_name = str(vision_cfg.get('topic_name', 'vision_detections'))
        vision_arguments = [
            '--config', vision_runtime_config,
            '--camera', camera,
            '--conf', confidence,
            '--process-every-n', process_every_n,
            '--rate-hz', rate_hz,
            '--service-name', service_name,
            '--topic-name', topic_name,
            '--no-preview',
        ]
        # 只有显式填写路径时才传入命令行；留空时保留 vision_runtime.yaml 和包内默认值的优先级
        if vision_model_path:
            vision_arguments.extend(['--model', vision_model_path])
        if vision_labels_path:
            vision_arguments.extend(['--labels', vision_labels_path])

        actions.append(_delayed(
            vision_delay,
            Node(
                package='vison_topic',
                executable='vision_detect_server',
                name='racom_vision_detect_server',
                output='screen',
                arguments=vision_arguments,
                respawn=respawn,
                respawn_delay=respawn_delay,
            ),
            Node(
                package='atlas_racom_vision_backend',
                executable='racom_camera_target_service',
                name='atlas_racom_camera_target_service',
                output='screen',
                parameters=[racom_camera_config],
                respawn=respawn,
                respawn_delay=respawn_delay,
            ),
            Node(
                package='atlas_racom_vision_backend',
                executable='sorting_rule_service',
                name='atlas_sorting_rule_service',
                output='screen',
                parameters=[sorting_rule_config],
                respawn=respawn,
                respawn_delay=respawn_delay,
            ),
        ))

    actions.append(_delayed(
        manipulation_delay,
        Node(
            package='atlas_vision_pollination_backend',
            executable='vision_pollination_backend',
            name='atlas_vision_pollination_backend',
            output='screen',
            parameters=[
                manipulation_config,
                {
                    'backend_name': manipulation_backend,
                    'config_yaml_path': transport_actions,
                    'vision_service': '/vision/detect_camera_target',
                },
            ],
            respawn=respawn,
            respawn_delay=respawn_delay,
        ),
    ))

    # 状态机最后启动；节点内部还会等待 MCU；ASRPRO；导航；视觉和机械臂服务全部就绪
    actions.append(_delayed(
        manager_delay,
        Node(
            package='atlas_autonomous_transport_manager',
            executable='autonomous_transport_manager',
            name='atlas_autonomous_transport_manager',
            output='screen',
            parameters=[
                {
                    'config_yaml_path': str(config_path),
                    'navigation_backend': navigation_backend,
                    'manipulation_backend': manipulation_backend,
                    'navigation_cmd_vel_topic': navigation_cmd_vel_topic,
                },
            ],
            respawn=respawn,
            respawn_delay=respawn_delay,
        ),
    ))
    return actions


def generate_launch_description() -> LaunchDescription:
    default_config = _package_file(
        'atlas_autonomous_transport_manager',
        'config',
        'autonomous_transport.yaml',
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=default_config,
            description='统一整车启动与全自主运输状态机 YAML',
        ),
        OpaqueFunction(function=_build_stack),
    ])
