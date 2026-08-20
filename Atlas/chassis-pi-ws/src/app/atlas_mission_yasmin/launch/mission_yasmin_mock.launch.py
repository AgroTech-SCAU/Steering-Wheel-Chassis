from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    route_file = LaunchConfiguration("route_file")

    default_config_file = PathJoinSubstitution(
        [FindPackageShare("atlas_mission_yasmin"), "config", "mission_yasmin.yaml"]
    )
    default_route_file = PathJoinSubstitution(
        [FindPackageShare("atlas_mission_yasmin"), "config", "mission_route.yaml"]
    )

    mock_mcu = Node(
        package="atlas_mission_yasmin",
        executable="mock_mcu.py",
        name="atlas_mock_mcu",
        output="screen",
    )
    mock_backends = Node(
        package="atlas_mission_yasmin",
        executable="mock_backends.py",
        name="atlas_mock_backends",
        output="screen",
    )
    mission_node = Node(
        package="atlas_mission_yasmin",
        executable="atlas_mission_yasmin_node",
        name="atlas_mission_yasmin",
        output="screen",
        parameters=[
            {"config_file": config_file},
            {"route_file": route_file},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config_file,
                description="Runtime topic, service and timeout configuration",
            ),
            DeclareLaunchArgument(
                "route_file",
                default_value=default_route_file,
                description="Mission route configuration",
            ),
            mock_mcu,
            mock_backends,
            mission_node,
        ]
    )
