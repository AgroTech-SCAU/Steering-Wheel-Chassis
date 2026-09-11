import math
import pathlib
import sys

PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from atlas_nav_direct_backend.direct_nav_model import (  # noqa: E402
    Pose2D,
    compose_pose,
    compute_body_tracking_command,
    evaluate_localization_candidate,
    inverse_pose,
    target_map_to_odom,
)


def assert_pose_close(actual: Pose2D, expected: Pose2D, tol: float = 1e-6):
    assert actual.x == pytest.approx(expected.x, abs=tol)
    assert actual.y == pytest.approx(expected.y, abs=tol)
    delta = math.atan2(math.sin(actual.yaw - expected.yaw), math.cos(actual.yaw - expected.yaw))
    assert delta == pytest.approx(0.0, abs=tol)


import pytest  # noqa: E402


def test_se2_inverse_round_trip():
    transform = Pose2D(1.2, -0.4, 1.1)
    identity = compose_pose(transform, inverse_pose(transform))
    assert_pose_close(identity, Pose2D(0.0, 0.0, 0.0))


def test_target_map_to_odom_uses_frozen_map_to_odom_transform():
    map_to_odom = Pose2D(0.5, -0.2, math.pi / 2.0)
    target_map = Pose2D(0.5, 0.8, math.pi / 2.0)
    target_odom = target_map_to_odom(map_to_odom, target_map)
    assert_pose_close(target_odom, Pose2D(1.0, 0.0, 0.0))


def test_localization_candidate_accepts_stable_pose_near_origin_and_handles_yaw_wrap():
    tf_samples = [
        Pose2D(0.10, -0.04, math.pi - 0.010),
        Pose2D(0.11, -0.05, -math.pi + 0.012),
        Pose2D(0.10, -0.05, math.pi - 0.008),
        Pose2D(0.11, -0.04, -math.pi + 0.009),
    ]
    odom_samples = [
        Pose2D(0.10, -0.04, -math.pi + 0.01),
        Pose2D(0.11, -0.05, math.pi - 0.01),
        Pose2D(0.10, -0.05, -math.pi + 0.008),
        Pose2D(0.11, -0.04, math.pi - 0.009),
    ]
    result = evaluate_localization_candidate(
        tf_samples,
        odom_samples,
        max_origin_offset_m=0.30,
        max_origin_yaw_rad=0.20,
        stability_xy_m=0.03,
        stability_yaw_rad=0.03,
    )
    assert result.valid, result.reason
    assert math.hypot(result.robot_pose_map.x, result.robot_pose_map.y) < 0.05
    assert abs(result.robot_pose_map.yaw) < 0.05


def test_localization_candidate_rejects_pose_outside_transfer_zone_origin_window():
    tf_samples = [Pose2D(0.7, 0.0, 0.0)] * 5
    odom_samples = [Pose2D(0.0, 0.0, 0.0)] * 5
    result = evaluate_localization_candidate(
        tf_samples,
        odom_samples,
        max_origin_offset_m=0.50,
        max_origin_yaw_rad=0.30,
        stability_xy_m=0.03,
        stability_yaw_rad=0.03,
    )
    assert not result.valid
    assert "origin offset" in result.reason


def test_localization_candidate_rejects_unstable_scan_match():
    tf_samples = [
        Pose2D(0.00, 0.00, 0.00),
        Pose2D(0.10, 0.00, 0.00),
        Pose2D(-0.08, 0.00, 0.00),
        Pose2D(0.07, 0.00, 0.00),
    ]
    odom_samples = [Pose2D(0.0, 0.0, 0.0)] * 4
    result = evaluate_localization_candidate(
        tf_samples,
        odom_samples,
        max_origin_offset_m=0.50,
        max_origin_yaw_rad=0.30,
        stability_xy_m=0.03,
        stability_yaw_rad=0.03,
    )
    assert not result.valid
    assert "unstable" in result.reason


def test_body_tracking_command_rotates_world_error_into_robot_frame():
    current = Pose2D(0.0, 0.0, math.pi / 2.0)
    target = Pose2D(1.0, 0.0, math.pi / 2.0)
    cmd = compute_body_tracking_command(
        current,
        target,
        kp_xy=1.0,
        kp_yaw=1.0,
        max_linear_speed_m_s=0.5,
        max_angular_speed_rad_s=0.5,
        slowdown_distance_m=0.25,
    )
    assert cmd.vx == pytest.approx(0.0, abs=1e-6)
    assert cmd.vy == pytest.approx(-0.5, abs=1e-6)
    assert cmd.wz == pytest.approx(0.0, abs=1e-6)
