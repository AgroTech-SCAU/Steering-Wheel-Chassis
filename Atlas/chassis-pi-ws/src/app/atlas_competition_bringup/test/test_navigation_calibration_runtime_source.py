from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
BRINGUP = ROOT / "app" / "atlas_competition_bringup"
SCRIPT = BRINGUP / "scripts" / "navigation_calibration.py"
LAUNCH = BRINGUP / "launch" / "navigation_calibration.launch.py"
CMAKE = BRINGUP / "CMakeLists.txt"
PACKAGE = BRINGUP / "package.xml"


def _read(path: Path) -> str:
    assert path.is_file(), f"missing {path.name}"
    return path.read_text(encoding="utf-8")


def test_navigation_calibration_launch_starts_only_required_background_hardware():
    text = _read(LAUNCH)
    assert 'mcu_comm_bridge.launch.py' in text
    assert 'robot_description.launch.py' in text
    assert 'lsn10p_launch.py' in text
    assert 'navigation_calibration.py' in text
    assert 'package="atlas_nav_direct_backend"' in text
    assert 'executable="direct_nav_backend"' in text
    assert 'full_nav_backend' not in text
    assert 'at_nav.launch.py' not in text


def test_runtime_checks_yaml_map_assets_before_arena_selection_and_warns_shared_maps():
    text = _read(SCRIPT)
    run = text[text.index("    def run_interactive"):text.index("    def cleanup", text.index("    def run_interactive"))]
    assert "inspect_navigation_maps" in run
    assert run.index("inspect_navigation_maps") < run.index("_select_arena")
    assert "A/B 当前指向同一组地图资产" in text
    assert "中转区最优位姿" in text


def test_runtime_uses_direct_backend_verified_map_alignment_without_second_localizer():
    text = _read(SCRIPT)
    assert "_start_localization" not in text
    assert "_localization_process" not in text
    assert 'NavigateToPose' not in text
    assert 'self._align_and_return_origin(arena)' in text
    assert 'request.waypoint_id = "origin"' in text
    assert '"/atlas/navigation/cmd_vel"' in text
    assert '"/motor_cmd_vel"' in text


def test_runtime_samples_map_to_base_link_and_confirms_three_waypoints():
    text = _read(SCRIPT)
    assert 'self.tf_buffer.lookup_transform(' in text
    assert 'self.map_frame, self.base_frame, Time()' in text
    assert 'self.declare_parameter("map_frame", "map")' in text
    assert 'self.declare_parameter("base_frame", "base_link")' in text
    assert 'for waypoint, label in (' in text
    for waypoint in ("pickup", "park_1", "park_2"):
        assert f'("{waypoint}"' in text
    assert "确认当前坐标和位姿" in text


def test_bringup_installs_navigation_calibration_and_declares_tf2_dependency():
    cmake = _read(CMAKE)
    package = _read(PACKAGE)
    assert "scripts/navigation_calibration.py" in cmake
    assert "scripts/navigation_calibration_model.py" in cmake
    assert "navigation_calibration_model_test" in cmake
    assert "navigation_calibration_runtime_source_test" in cmake
    assert "<exec_depend>tf2_ros</exec_depend>" in package


def test_navigation_calibration_displays_current_offset_from_map_origin_before_waypoints():
    text = _read(SCRIPT)
    run = text[text.index("    def run_interactive"):text.index("    def cleanup", text.index("    def run_interactive"))]
    assert "def _show_origin_deviation" in text
    assert "当前相对地图原点偏差" in text
    assert "distance=" in text
    assert run.index("self._align_and_return_origin(arena)") < run.index("self._wait_localization_ready()")
    assert run.index("self._wait_localization_ready()") < run.index("self._show_origin_deviation()")
    assert run.index("self._show_origin_deviation()") < run.index("for waypoint, label in (")
