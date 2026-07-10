"""RUI 视觉检测服务启动文件

用法:
  ros2 launch vison_topic vision_detect.launch.py                  # 默认有预览
  ros2 launch vison_topic vision_detect.launch.py camera:=2        # 指定摄像头
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera', default_value='0',
                              description='摄像头 ID'),
        DeclareLaunchArgument('conf', default_value='0.55',
                              description='ONNX 检测置信度阈值'),
        DeclareLaunchArgument('process_every_n', default_value='2',
                              description='每 N 帧推理一次'),
        DeclareLaunchArgument('rate_hz', default_value='10',
                              description='检测定时器频率 Hz'),
        DeclareLaunchArgument('service_name', default_value='vision_detect',
                              description='检测启停服务名'),
        DeclareLaunchArgument('topic_name', default_value='vision_detections',
                              description='检测结果话题名'),
        DeclareLaunchArgument('no_preview', default_value='true',
                              description='是否关闭 OpenCV 预览窗口'),

        Node(
            package='vison_topic',
            executable='vision_detect_server',
            name='vision_detect_server',
            output='screen',
            arguments=[
                '--camera', LaunchConfiguration('camera'),
                '--conf', LaunchConfiguration('conf'),
                '--process-every-n', LaunchConfiguration('process_every_n'),
                '--rate-hz', LaunchConfiguration('rate_hz'),
                '--service-name', LaunchConfiguration('service_name'),
                '--topic-name', LaunchConfiguration('topic_name'),
                '--no-preview',
            ],
            respawn=True,
            respawn_delay=2.0,
        ),
    ])
