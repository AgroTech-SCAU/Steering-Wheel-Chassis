from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = LaunchConfiguration('config')
    competition_config = LaunchConfiguration('competition_config')
    default_config = PathJoinSubstitution([
        FindPackageShare('atlas_competition_manipulation_backend'),
        'config',
        'manipulation.yaml',
    ])
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument('competition_config', default_value=''),
        Node(
            package='atlas_competition_manipulation_backend',
            executable='competition_manipulation_backend',
            name='atlas_competition_manipulation_backend',
            output='screen',
            parameters=[config, {'competition_config': competition_config}],
            respawn=True,
            respawn_delay=2.0,
        ),
    ])
