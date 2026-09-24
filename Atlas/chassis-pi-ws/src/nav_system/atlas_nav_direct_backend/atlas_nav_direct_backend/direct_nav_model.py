from __future__ import annotations

from dataclasses import dataclass
import math
import statistics
from typing import Sequence


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class VelocityCommand:
    vx: float
    vy: float
    wz: float


@dataclass(frozen=True)
class LocalizationEvaluation:
    valid: bool
    score: float
    frozen_map_to_odom: Pose2D
    robot_pose_map: Pose2D
    xy_spread_m: float
    yaw_spread_rad: float
    reason: str


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def compose_pose(lhs: Pose2D, rhs: Pose2D) -> Pose2D:
    """Return T_a_c = T_a_b * T_b_c for planar poses."""
    c = math.cos(lhs.yaw)
    s = math.sin(lhs.yaw)
    return Pose2D(
        lhs.x + c * rhs.x - s * rhs.y,
        lhs.y + s * rhs.x + c * rhs.y,
        normalize_angle(lhs.yaw + rhs.yaw),
    )


def inverse_pose(pose: Pose2D) -> Pose2D:
    c = math.cos(pose.yaw)
    s = math.sin(pose.yaw)
    return Pose2D(
        -c * pose.x - s * pose.y,
        s * pose.x - c * pose.y,
        normalize_angle(-pose.yaw),
    )


def target_map_to_odom(map_to_odom: Pose2D, target_map: Pose2D) -> Pose2D:
    """Convert a fixed field/map target to the MCU odom frame."""
    return compose_pose(inverse_pose(map_to_odom), target_map)


def slew_pose(
    previous: Pose2D,
    measured: Pose2D,
    dt_s: float,
    *,
    max_linear_rate_m_s: float,
    max_angular_rate_rad_s: float,
) -> Pose2D:
    """Rate-limit an SE(2) correction without breaking yaw wrap-around."""
    dt = max(0.0, dt_s)
    dx = measured.x - previous.x
    dy = measured.y - previous.y
    distance = math.hypot(dx, dy)
    max_distance = max(0.0, max_linear_rate_m_s) * dt
    if distance > max_distance > 0.0:
        scale = max_distance / distance
        dx *= scale
        dy *= scale
    elif max_distance <= 0.0:
        dx = 0.0
        dy = 0.0

    dyaw = normalize_angle(measured.yaw - previous.yaw)
    max_yaw = max(0.0, max_angular_rate_rad_s) * dt
    dyaw = clamp(dyaw, -max_yaw, max_yaw)
    return Pose2D(
        previous.x + dx,
        previous.y + dy,
        normalize_angle(previous.yaw + dyaw),
    )


def circular_mean(values: Sequence[float]) -> float:
    if not values:
        return 0.0
    s = sum(math.sin(v) for v in values)
    c = sum(math.cos(v) for v in values)
    if abs(s) < 1e-12 and abs(c) < 1e-12:
        return normalize_angle(values[0])
    return math.atan2(s, c)


def _median_pose(samples: Sequence[Pose2D]) -> Pose2D:
    return Pose2D(
        statistics.median(v.x for v in samples),
        statistics.median(v.y for v in samples),
        circular_mean([v.yaw for v in samples]),
    )


def _pose_spread(samples: Sequence[Pose2D], center: Pose2D) -> tuple[float, float]:
    xy = max(
        (math.hypot(v.x - center.x, v.y - center.y) for v in samples),
        default=0.0,
    )
    yaw = max(
        (abs(normalize_angle(v.yaw - center.yaw)) for v in samples),
        default=0.0,
    )
    return xy, yaw


def evaluate_localization_candidate(
    map_to_odom_samples: Sequence[Pose2D],
    odom_to_base_samples: Sequence[Pose2D],
    *,
    max_origin_offset_m: float,
    max_origin_yaw_rad: float,
    stability_xy_m: float,
    stability_yaw_rad: float,
) -> LocalizationEvaluation:
    if not map_to_odom_samples or len(map_to_odom_samples) != len(odom_to_base_samples):
        zero = Pose2D(0.0, 0.0, 0.0)
        return LocalizationEvaluation(False, math.inf, zero, zero, math.inf, math.inf, "missing localization samples")

    frozen = _median_pose(map_to_odom_samples)
    robot_samples = [
        compose_pose(map_to_odom, odom_to_base)
        for map_to_odom, odom_to_base in zip(map_to_odom_samples, odom_to_base_samples)
    ]
    robot = _median_pose(robot_samples)
    xy_spread, yaw_spread = _pose_spread(robot_samples, robot)

    if xy_spread > stability_xy_m or yaw_spread > stability_yaw_rad:
        return LocalizationEvaluation(
            False,
            math.inf,
            frozen,
            robot,
            xy_spread,
            yaw_spread,
            "localization unstable",
        )

    origin_offset = math.hypot(robot.x, robot.y)
    yaw_offset = abs(normalize_angle(robot.yaw))
    if origin_offset > max_origin_offset_m:
        return LocalizationEvaluation(
            False,
            math.inf,
            frozen,
            robot,
            xy_spread,
            yaw_spread,
            f"origin offset {origin_offset:.3f}m exceeds limit",
        )
    if yaw_offset > max_origin_yaw_rad:
        return LocalizationEvaluation(
            False,
            math.inf,
            frozen,
            robot,
            xy_spread,
            yaw_spread,
            f"origin yaw offset {yaw_offset:.3f}rad exceeds limit",
        )

    score = origin_offset + 0.25 * yaw_offset + 2.0 * xy_spread + 0.25 * yaw_spread
    return LocalizationEvaluation(
        True,
        score,
        frozen,
        robot,
        xy_spread,
        yaw_spread,
        "localization stable",
    )


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def compute_body_tracking_command(
    current_odom: Pose2D,
    target_odom: Pose2D,
    *,
    kp_xy: float,
    kp_yaw: float,
    max_linear_speed_m_s: float,
    max_angular_speed_rad_s: float,
    slowdown_distance_m: float,
) -> VelocityCommand:
    dx = target_odom.x - current_odom.x
    dy = target_odom.y - current_odom.y
    distance = math.hypot(dx, dy)
    c = math.cos(current_odom.yaw)
    s = math.sin(current_odom.yaw)
    ex_body = c * dx + s * dy
    ey_body = -s * dx + c * dy

    vx = kp_xy * ex_body
    vy = kp_xy * ey_body
    speed = math.hypot(vx, vy)
    # Use the slowdown ramp as a speed ceiling. Multiplying the proportional
    # command by distance/slowdown_distance makes speed quadratic in the
    # remaining error and caused the final few decimetres to crawl.
    speed_limit = max_linear_speed_m_s
    if slowdown_distance_m > 1e-9:
        speed_limit *= clamp(distance / slowdown_distance_m, 0.15, 1.0)
    if speed > speed_limit > 0.0:
        factor = speed_limit / speed
        vx *= factor
        vy *= factor

    yaw_error = normalize_angle(target_odom.yaw - current_odom.yaw)
    wz = clamp(kp_yaw * yaw_error, -max_angular_speed_rad_s, max_angular_speed_rad_s)
    return VelocityCommand(vx, vy, wz)


def limit_acceleration(
    desired: VelocityCommand,
    previous: VelocityCommand,
    dt_s: float,
    *,
    max_linear_accel_m_s2: float,
    max_angular_accel_rad_s2: float,
) -> VelocityCommand:
    dt = max(0.001, dt_s)
    dv = max_linear_accel_m_s2 * dt
    dw = max_angular_accel_rad_s2 * dt
    delta_vx = desired.vx - previous.vx
    delta_vy = desired.vy - previous.vy
    delta_speed = math.hypot(delta_vx, delta_vy)
    if delta_speed > dv > 0.0:
        scale = dv / delta_speed
        delta_vx *= scale
        delta_vy *= scale
    elif dv <= 0.0:
        delta_vx = 0.0
        delta_vy = 0.0
    return VelocityCommand(
        previous.vx + delta_vx,
        previous.vy + delta_vy,
        clamp(desired.wz, previous.wz - dw, previous.wz + dw),
    )
