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


def test_direct_backend_keeps_laser_alignment_during_semantic_odom_tracking():
    text = (ROOT / "atlas_nav_direct_backend/direct_nav_backend.py").read_text()
    assert 'waypoint_id == "origin"' in text
    assert "evaluate_localization_candidate" in text
    assert "frozen_map_to_odom" in text
    assert "target_map_to_odom" in text
    assert "resolve_navigation_waypoint" in text
    assert "shutdown()" in text
    assert "/atlas/navigation/cmd_vel" in text
    assert "compute_body_tracking_command" in text
    assert "refresh_live_localization" in text
    assert "max_correction_speed_m_s" in text
    assert "rejecting localization jump" in text


def test_direct_backend_config_enables_fail_closed_startup_localization():
    text = (ROOT / "config/direct_nav.yaml").read_text()
    assert "startup_localization:" in text
    assert "sample_count:" in text
    assert "max_origin_offset_m:" in text
    assert "position_tolerance_m:" in text
    assert "require_yaw_reached: true" in text
    assert "continuous_localization:" in text
    assert "enabled: true" in text


def test_startup_localization_never_accepts_timeout_with_too_few_samples():
    text = (ROOT / "atlas_nav_direct_backend/direct_nav_backend.py").read_text()
    assert "required_samples = max(3, self.localization_sample_count)" in text
    assert "insufficient localization samples" in text


def test_origin_request_requires_and_locks_explicit_arena_before_map_matching():
    text = (ROOT / "atlas_nav_direct_backend/direct_nav_backend.py").read_text()
    assert "arena = self.arena_lock.accept(request.arena)" in text
    assert "begin_startup_localization(arena)" in text
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


def test_startup_cartographer_config_forces_early_global_search_for_initial_alignment():
    startup = (NAV_SYSTEM / "at_nav2/config/cartographer_startup_localization.lua").read_text()
    persistent = (NAV_SYSTEM / "at_nav2/config/cartographer_localization.lua").read_text()
    assert "POSE_GRAPH.global_constraint_search_after_n_seconds = 2.0" in startup
    assert "POSE_GRAPH.global_sampling_ratio = 1.0" in startup
    assert "POSE_GRAPH.constraint_builder.sampling_ratio = 1.0" in startup
    assert "POSE_GRAPH.global_constraint_search_after_n_seconds = 2.0" not in persistent


def test_localization_scan_range_and_search_window_match_mapping_profile():
    startup = (NAV_SYSTEM / "at_nav2/config/cartographer_startup_localization.lua").read_text()
    persistent = (NAV_SYSTEM / "at_nav2/config/cartographer_localization.lua").read_text()
    for config in (startup, persistent):
        assert "TRAJECTORY_BUILDER_2D.max_range = 8.0" in config
        assert "linear_search_window = 0.2" in config
