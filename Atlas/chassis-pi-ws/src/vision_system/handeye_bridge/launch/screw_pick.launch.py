"""螺丝抓取完整启动。

用法:
  ros2 launch handeye_bridge screw_pick.launch.py                        # 部署
  ros2 launch handeye_bridge screw_pick.launch.py no_preview:=false      # 调试(带预览)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from atlas_competition_config.config import load_optional_competition_config


def _bridge_node(context, config_file):
    competition_path = context.perform_substitution(LaunchConfiguration('competition_config'))
    competition = load_optional_competition_config(competition_path)
    handeye_parameters = {} if competition is None else competition.handeye_bridge
    runtime_parameters = {'competition_config': competition_path}
    auto_move = context.perform_substitution(
        LaunchConfiguration('auto_move_to_initial_on_start')).strip().lower()
    if auto_move:
        if auto_move not in ('true', 'false'):
            raise ValueError('auto_move_to_initial_on_start must be true or false')
        runtime_parameters['auto_move_to_initial_on_start'] = auto_move == 'true'
    return [Node(
        package='handeye_bridge', executable='bridge_node',
        name='handeye_bridge', output='screen',
        parameters=[
            config_file,
            handeye_parameters,
            runtime_parameters,
        ],
        respawn=True, respawn_delay=2.0,
    )]


def generate_launch_description():
    pkg_share = get_package_share_directory("handeye_bridge")
    config_file = os.path.join(pkg_share, "config", "bridge_node.yaml")
    try:
        competition_share = get_package_share_directory("atlas_competition_bringup")
        default_competition_config = os.path.join(
            competition_share, "config", "competition.yaml")
    except LookupError:
        default_competition_config = ''

    ld = LaunchDescription([
        DeclareLaunchArgument('no_preview', default_value='true',
                              description='关闭 OpenCV 预览窗口'),
        DeclareLaunchArgument('competition_config', default_value=default_competition_config,
                              description='顶层比赛 YAML；默认使用已安装的比赛配置'),
        DeclareLaunchArgument('auto_move_to_initial_on_start', default_value='',
                              description='空=使用顶层配置；true/false=覆盖初始观察位自动移动'),
    ])

    # ── 视觉检测 ──
    vison_args = [
        '--camera', '0',
        '--conf', '0.55',
        '--process-every-n', '2',
        '--rate-hz', '15',
    ]

    ld.add_action(Node(
        package='vison_topic', executable='vision_detect_server',
        name='vision_detect_server', output='screen',
        arguments=vison_args + ['--no-preview'],
        condition=IfCondition(LaunchConfiguration('no_preview')),
        respawn=True, respawn_delay=2.0,
    ))
    ld.add_action(Node(
        package='vison_topic', executable='vision_detect_server',
        name='vision_detect_server', output='screen',
        arguments=vison_args,
        condition=UnlessCondition(LaunchConfiguration('no_preview')),
        respawn=True, respawn_delay=2.0,
    ))

    # ── 手眼桥 ──
    ld.add_action(OpaqueFunction(function=_bridge_node, args=[config_file]))

    return ld
