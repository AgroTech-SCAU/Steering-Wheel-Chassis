from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
NAV_SYSTEM = ROOT.parent


def test_startup_localization_launch_is_cartographer_only():
    text = (NAV_SYSTEM / "at_nav2/launch/startup_localization.launch.py").read_text()
    assert "cartographer_node" in text
    assert "cartographer_startup_localization.lua" in text
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


def test_origin_request_can_limit_startup_localization_to_explicit_arena():
    text = (ROOT / "atlas_nav_direct_backend/direct_nav_backend.py").read_text()
    assert "begin_startup_localization(str(request.arena or \"\").strip().upper())" in text
    assert "localization_candidates(" in text
    assert "arena=arena" in text


def test_startup_localization_requires_real_cartographer_map_connection():
    text = (ROOT / "atlas_nav_direct_backend/direct_nav_backend.py").read_text()
    assert 'Inter constraints, different trajectories' in text
    assert 'waiting for Cartographer global map constraint' in text
    assert 'localization_global_constraint_seen_at is None' in text
    assert 'self.localization_tf_samples.clear()' in text
    assert 'localization_global_constraint_baseline' in text
    assert 'count - self.localization_global_constraint_baseline' in text


def test_startup_cartographer_config_forces_early_global_search_only_for_one_shot_localizer():
    startup = (NAV_SYSTEM / "at_nav2/config/cartographer_startup_localization.lua").read_text()
    persistent = (NAV_SYSTEM / "at_nav2/config/cartographer_localization.lua").read_text()
    assert "POSE_GRAPH.global_constraint_search_after_n_seconds = 2.0" in startup
    assert "POSE_GRAPH.global_sampling_ratio = 1.0" in startup
    assert "POSE_GRAPH.constraint_builder.sampling_ratio = 1.0" in startup
    assert "POSE_GRAPH.global_constraint_search_after_n_seconds = 2.0" not in persistent
