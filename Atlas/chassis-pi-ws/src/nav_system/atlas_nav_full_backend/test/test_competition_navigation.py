from atlas_nav_full_backend.competition_navigation import Nav2StackLauncher


def test_nav2_stack_launcher_builds_arena_specific_launch_command():
    """Catches starting Nav2 with package defaults for arena maps."""
    launcher = Nav2StackLauncher(
        launch_package="at_nav2",
        launch_file="at_nav.launch.py",
        params_file="/maps/nav2_params.yaml",
        cmd_vel_output="/atlas/navigation/cmd_vel",
    )

    command = launcher.build_command(
        "/maps/arena_a.yaml",
        "/maps/arena_a.pbstream",
    )

    assert command == [
        "ros2",
        "launch",
        "at_nav2",
        "at_nav.launch.py",
        "map:=/maps/arena_a.yaml",
        "pbstream:=/maps/arena_a.pbstream",
        "params_file:=/maps/nav2_params.yaml",
        "cmd_vel_output:=/atlas/navigation/cmd_vel",
    ]
