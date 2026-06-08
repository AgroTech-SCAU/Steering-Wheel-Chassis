#!/usr/bin/env python3

import os
import platform

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_gazebo')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Ensure system Gazebo paths are available. gzserver.launch.py only
    # appends them if they already exist in the environment (via
    # GazeboRosPaths.get_paths() + os.environ). Without this, Gazebo
    # can't find shaders, system models (sun, ground_plane), or plugins.
    _arch = platform.machine()  # x86_64, aarch64, etc.
    os.environ.setdefault('GAZEBO_RESOURCE_PATH', '/usr/share/gazebo-11')
    os.environ.setdefault('GAZEBO_MODEL_PATH', '/usr/share/gazebo-11/models')
    os.environ.setdefault(
        'GAZEBO_PLUGIN_PATH',
        f'/usr/lib/{_arch}-linux-gnu/gazebo-11/plugins'
    )

    # Paths
    world_path = LaunchConfiguration(
        'world_path',
        default=os.path.join(pkg_share, 'worlds', 'competition.world')
    )
    xacro_path = os.path.join(pkg_share, 'urdf', 'robot_sim.xacro')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pos = LaunchConfiguration('x_pos', default='1.57')
    y_pos = LaunchConfiguration('y_pos', default='1.4')
    z_pos = LaunchConfiguration('z_pos', default='0.1')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock'
    )
    declare_x_pos = DeclareLaunchArgument(
        'x_pos', default_value='1.57',
        description='Robot spawn X position (m)'
    )
    declare_y_pos = DeclareLaunchArgument(
        'y_pos', default_value='1.4',
        description='Robot spawn Y position (m)'
    )
    declare_z_pos = DeclareLaunchArgument(
        'z_pos', default_value='0.1',
        description='Robot spawn Z position (m)'
    )
    declare_world_path = DeclareLaunchArgument(
        'world_path',
        default_value=os.path.join(pkg_share, 'worlds', 'competition.world'),
        description='Path to Gazebo world file'
    )

    # 1. Gazebo server (loads the world)
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_path}.items()
    )

    # 2. Gazebo client (GUI) — optional
    #    Launched directly (not via gzclient.launch.py) to skip the
    #    --gui-client-plugin=libgazebo_ros_eol_gui.so flag, which causes
    #    a boost shared_ptr assertion crash on NVIDIA + Wayland systems.
    gui = LaunchConfiguration('gui', default='true')
    declare_gui = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Launch Gazebo GUI client (set to false for headless mode)'
    )

    gzclient_env = {
        'GDK_BACKEND': 'x11',           # Force X11 backend (Wayland → XWayland GLX fix)
        '__GL_SYNC_TO_VBLANK': '0',     # Disable vsync (NVIDIA OGRE workaround)
    }

    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        additional_env=gzclient_env,
        condition=IfCondition(gui),
    )

    # 3. robot_state_publisher (TF from /joint_states)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': ParameterValue(
                Command(['xacro ', xacro_path]),
                value_type=str
            ),
        }],
    )

    # 4. Spawn robot into Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'robot_description',
            '-topic', 'robot_description',
            '-x', x_pos,
            '-y', y_pos,
            '-z', z_pos,
        ],
        output='screen',
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_x_pos)
    ld.add_action(declare_y_pos)
    ld.add_action(declare_z_pos)
    ld.add_action(declare_world_path)
    ld.add_action(declare_gui)

    ld.add_action(gzserver)
    ld.add_action(gzclient)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_entity)

    return ld
