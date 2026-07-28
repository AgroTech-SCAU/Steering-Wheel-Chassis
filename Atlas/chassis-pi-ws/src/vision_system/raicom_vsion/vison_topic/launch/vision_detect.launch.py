"""RACOM/RAICOM 视觉检测服务启动文件。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera', default_value='0', description='摄像头 ID'),
        DeclareLaunchArgument('conf', default_value='0.7', description='ONNX 检测置信度阈值'),
        DeclareLaunchArgument('process_every_n', default_value='1', description='每 N 帧推理一次'),
        DeclareLaunchArgument('service_name', default_value='vision_detect', description='检测启停服务名'),
        DeclareLaunchArgument('topic_name', default_value='vision_detections', description='检测结果话题名'),
        Node(
            package='vison_topic',
            executable='vision_detect_server',
            name='vision_detect_server',
            output='screen',
            arguments=[
                '--camera', LaunchConfiguration('camera'),
                '--conf', LaunchConfiguration('conf'),
                '--process-every-n', LaunchConfiguration('process_every_n'),
                '--service-name', LaunchConfiguration('service_name'),
                '--topic-name', LaunchConfiguration('topic_name'),
                '--no-preview',
            ],
            respawn=True,
            respawn_delay=2.0,
        ),
    ])
