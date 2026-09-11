from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
BRINGUP = ROOT / "app" / "atlas_competition_bringup"
MCU_LAUNCH = ROOT / "mcu_comm_bridge" / "launch" / "mcu_comm_bridge.launch.py"
ROBOT_LAUNCH = ROOT / "nav_system" / "robot_description" / "launch" / "robot_description.launch.py"
LIDAR_LAUNCH = ROOT / "nav_system" / "lslidar_driver" / "launch" / "lsn10p_launch.py"


def test_calibration_launch_routes_background_nodes_to_log_files():
    text = (BRINGUP / "launch" / "arm_motion_calibration.launch.py").read_text(encoding="utf-8")

    assert 'output="log"' in text
    assert 'SetEnvironmentVariable("RCUTILS_LOGGING_USE_STDOUT", "1")' in text
    assert '"stats_enabled": "false"' in text
    assert text.count('"log_level": "fatal"') == 3
    assert 'ros_arguments=["--log-level", "fatal"]' in text


def test_background_launches_accept_output_override_without_changing_default():
    for path in (MCU_LAUNCH, ROBOT_LAUNCH, LIDAR_LAUNCH):
        text = path.read_text(encoding="utf-8")
        assert 'DeclareLaunchArgument("output", default_value="screen")' in text
        assert 'LaunchConfiguration("output")' in text
        assert 'DeclareLaunchArgument("log_level", default_value="info")' in text
        assert 'LaunchConfiguration("log_level")' in text


def test_terminal_prompt_uses_launch_ancestor_tty_instead_of_stdin():
    text = (BRINGUP / "scripts" / "arm_motion_calibration.py").read_text(encoding="utf-8")
    terminal_input = text[text.index("def terminal_input"):text.index("\n\ndef quaternion", text.index("def terminal_input"))]

    assert "def _terminal_candidates():" in text
    assert 'os.readlink(f"/proc/{pid}/fd/{fd}")' in text
    assert "os.open(terminal, os.O_RDWR | os.O_NOCTTY)" in terminal_input
    assert "os.write(fd," in terminal_input
    assert "os.read(fd, 4096)" in terminal_input
    assert "return input()" not in terminal_input


def test_no_preview_launch_argument_controls_camera_without_prompt():
    launch_text = (BRINGUP / "launch" / "arm_motion_calibration.launch.py").read_text(encoding="utf-8")
    script_text = (BRINGUP / "scripts" / "arm_motion_calibration.py").read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument(\n            "no_preview"' in launch_text
    assert '"no_preview": ParameterValue(no_preview, value_type=bool)' in launch_text
    assert 'self.declare_parameter("no_preview", False)' in script_text
    assert "if not self.no_preview:" in script_text
    assert "是否打开摄像头预览" not in script_text


def test_calibration_ui_is_bound_to_the_input_terminal():
    text = (BRINGUP / "scripts" / "arm_motion_calibration.py").read_text(encoding="utf-8")

    assert "def bind_terminal_output()" in text
    assert "os.dup2(fd, sys.stdout.fileno())" in text
    assert "bind_terminal_output()\n    rclpy.init" in text


def test_navigation_service_wait_covers_nav2_stack_activation():
    text = (BRINGUP / "scripts" / "arm_motion_calibration.py").read_text(encoding="utf-8")

    assert 'self.declare_parameter("navigation_start_timeout_s", 40.0)' in text
    assert "deadline = time.monotonic() + self.navigation_start_timeout_s" in text
    assert "deadline = time.monotonic() + 8.0" not in text


def test_calibration_cleanup_never_publishes_after_rclpy_context_shutdown():
    text = (BRINGUP / "scripts" / "arm_motion_calibration.py").read_text(encoding="utf-8")
    cleanup = text[text.index("    def cleanup(self)"):text.index("\n\ndef main", text.index("    def cleanup(self)"))]

    assert "if rclpy.ok():" in cleanup
    assert cleanup.index("if rclpy.ok():") < cleanup.index("self._publish_zero()")


def test_camera_subprocess_does_not_inherit_calibration_terminal_output():
    text = (BRINGUP / "scripts" / "arm_motion_calibration.py").read_text(encoding="utf-8")
    camera = text[text.index("    def _start_camera(self)"):text.index("\n    def _stop_camera", text.index("    def _start_camera(self)"))]

    assert "stdout=" in camera
    assert "stderr=" in camera
    assert "subprocess.DEVNULL" in camera
    assert '"--allow-unprepared", "--auto-start"' in camera


def test_selected_arena_captures_only_its_sorting_scan_pose():
    text = (BRINGUP / "scripts" / "arm_motion_calibration.py").read_text(encoding="utf-8")
    run = text[text.index("    def run_interactive"):text.index("    def cleanup", text.index("    def run_interactive"))]

    assert 'scan_key = f"sorting_scan_{arena.lower()}"' in run
    assert 'fixed[scan_key] = self._capture_pose' in run
    assert 'fixed["sorting_scan_a"] = self._capture_pose' not in run
    assert 'fixed["sorting_scan_b"] = self._capture_pose' not in run
    assert '"1/8 zero"' in run
    assert '"3/8 navigation_safe"' in run
