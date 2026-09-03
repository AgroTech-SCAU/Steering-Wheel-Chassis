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

"""Launch competition YASMIN with deterministic mock MCU/navigation/vision/arm."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    arena = LaunchConfiguration("arena")
    scenario = LaunchConfiguration("scenario")
    config_file = PathJoinSubstitution(
        [FindPackageShare("atlas_mission_yasmin"), "config", "mission_yasmin.yaml"])
    route_file = PathJoinSubstitution(
        [FindPackageShare("atlas_mission_yasmin"), "config", "mission_route.yaml"])

    return LaunchDescription([
        DeclareLaunchArgument("arena", default_value="A"),
        DeclareLaunchArgument("scenario", default_value="normal"),
        Node(
            package="atlas_mission_yasmin",
            executable="mock_mcu.py",
            name="atlas_mock_mcu",
            output="screen",
            parameters=[{"scenario": scenario}],
        ),
        Node(
            package="atlas_mission_yasmin",
            executable="mock_backends.py",
            name="atlas_mock_backends",
            output="screen",
            parameters=[{"arena": arena, "scenario": scenario}],
        ),
        Node(
            package="atlas_mission_yasmin",
            executable="atlas_mission_yasmin_node",
            name="atlas_mission_yasmin",
            output="screen",
            parameters=[
                config_file,
                {"route_yaml_path": route_file},
                {"mcu_status_timeout_s": 0.3},
                {"enable_viewer": False},
            ],
        ),
    ])
