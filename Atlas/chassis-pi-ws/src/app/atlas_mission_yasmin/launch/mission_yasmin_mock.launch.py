# Copyright 2026 yangxuan
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch the Atlas YASMIN mission runtime with mock skeleton nodes."""

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
            config_file,
            {"route_yaml_path": route_file},
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
