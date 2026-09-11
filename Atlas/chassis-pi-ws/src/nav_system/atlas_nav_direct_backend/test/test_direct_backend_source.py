from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
NAV_SYSTEM = ROOT.parent


def test_startup_localization_launch_is_cartographer_only():
    text = (NAV_SYSTEM / "at_nav2/launch/startup_localization.launch.py").read_text()
    assert "cartographer_node" in text
    assert "cartographer_localization.lua" in text
    assert "nav2_bringup" not in text
    assert "map_server" not in text
    assert "navigation_launch.py" not in text


def test_direct_backend_has_one_shot_alignment_and_semantic_odom_tracking():
    text = (ROOT / "atlas_nav_direct_backend/direct_nav_backend.py").read_text()
    assert 'waypoint_id == "origin"' in text
    assert "evaluate_localization_candidate" in text
    assert "frozen_map_to_odom" in text
    assert "target_map_to_odom" in text
    assert "resolve_navigation_waypoint" in text
    assert "shutdown()" in text
    assert "/atlas/navigation/cmd_vel" in text
    assert "compute_body_tracking_command" in text


def test_direct_backend_config_enables_fail_closed_startup_localization():
    text = (ROOT / "config/direct_nav.yaml").read_text()
    assert "startup_localization:" in text
    assert "sample_count:" in text
    assert "max_origin_offset_m:" in text
    assert "position_tolerance_m:" in text
    assert "require_yaw_reached: true" in text


def test_startup_localization_never_accepts_timeout_with_too_few_samples():
    text = (ROOT / "atlas_nav_direct_backend/direct_nav_backend.py").read_text()
    assert "required_samples = max(3, self.localization_sample_count)" in text
    assert "insufficient localization samples" in text
