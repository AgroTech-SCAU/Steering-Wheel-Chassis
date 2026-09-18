from __future__ import annotations

import math
import os
import subprocess
from dataclasses import replace
from pathlib import Path
from typing import Optional

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener
from visualization_msgs.msg import MarkerArray

from atlas_competition_config.config import (
    ArenaLock,
    CompetitionConfigError,
    load_competition_config,
    resolve_navigation_waypoint,
)
from atlas_mission_interfaces.msg import NavigationStatus
from atlas_mission_interfaces.srv import CancelNavigation, StartNavigation

from .direct_nav_model import (
    LocalizationEvaluation,
    Pose2D,
    VelocityCommand,
    compute_body_tracking_command,
    evaluate_localization_candidate,
    limit_acceleration,
    normalize_angle,
    target_map_to_odom,
    compose_pose,
)
from .map_scan_match import MapScanMatch
from .startup_localization import (
    LocalizationCandidate,
    StartupLocalizationLauncher,
    localization_candidates,
)


def yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class DirectNavBackend(Node):
    PHASE_IDLE = "idle"
    PHASE_LOCALIZING = "localizing"
    PHASE_TRACKING = "tracking"
    PHASE_BRAKE = "brake_hold"

    def __init__(self) -> None:
        super().__init__("atlas_nav_direct_backend")

        self.backend_name = str(self.declare_parameter("backend_name", "direct_odom_competition").value)
        self.competition_config_path = str(self.declare_parameter("competition_config", "").value)
        self.odom_topic = str(self.declare_parameter("odom_topic", "/odom").value)
        self.cmd_vel_topic = str(self.declare_parameter("cmd_vel_topic", "/atlas/navigation/cmd_vel").value)
        self.status_topic = str(self.declare_parameter("status_topic", "/atlas/navigation/status").value)
        self.start_service = str(self.declare_parameter("start_service", "/atlas/navigation/start").value)
        self.cancel_service = str(self.declare_parameter("cancel_service", "/atlas/navigation/cancel").value)
        self.map_frame = str(self.declare_parameter("map_frame", "map").value)
        self.odom_frame = str(self.declare_parameter("odom_frame", "odom").value)
        self.base_frame = str(self.declare_parameter("base_frame", "base_link").value)
        self.control_rate_hz = float(self.declare_parameter("control_rate_hz", 50.0).value)
        self.odom_timeout_s = float(self.declare_parameter("odom_timeout_s", 0.30).value)
        self.default_waypoint_timeout_s = float(self.declare_parameter("default_waypoint_timeout_s", 30.0).value)

        self.localization_warmup_s = float(self.declare_parameter("startup_localization.warmup_s", 1.0).value)
        self.localization_candidate_timeout_s = float(self.declare_parameter("startup_localization.candidate_timeout_s", 30.0).value)
        self.localization_sample_count = int(self.declare_parameter("startup_localization.sample_count", 20).value)
        self.max_origin_offset_m = float(self.declare_parameter("startup_localization.max_origin_offset_m", 1000.0).value)
        self.max_origin_yaw_rad = float(self.declare_parameter("startup_localization.max_origin_yaw_rad", math.pi).value)
        self.localization_stability_xy_m = float(self.declare_parameter("startup_localization.stability_xy_m", 0.030).value)
        self.localization_stability_yaw_rad = float(self.declare_parameter("startup_localization.stability_yaw_rad", 0.040).value)
        self.map_scan_min_agreement = float(self.declare_parameter("startup_localization.min_map_scan_agreement", 0.35).value)
        self.map_scan_min_hits = int(self.declare_parameter("startup_localization.min_map_scan_hits", 25).value)
        self.localization_constraint_topic = str(
            self.declare_parameter("startup_localization.constraint_topic", "/constraint_list").value
        )
        self.localization_require_global_constraint = bool(
            self.declare_parameter("startup_localization.require_global_constraint", True).value
        )
        self.localization_min_global_constraints = max(
            1, int(self.declare_parameter("startup_localization.min_global_constraints", 1).value)
        )
        self.localization_post_constraint_settle_s = max(
            0.0,
            float(self.declare_parameter("startup_localization.post_constraint_settle_s", 0.8).value),
        )

        self.kp_xy = float(self.declare_parameter("control.kp_xy", 1.20).value)
        self.kp_yaw = float(self.declare_parameter("control.kp_yaw", 1.50).value)
        self.max_linear_speed = float(self.declare_parameter("control.max_linear_speed_m_s", 0.45).value)
        self.max_angular_speed = float(self.declare_parameter("control.max_angular_speed_rad_s", 0.60).value)
        self.max_linear_accel = float(self.declare_parameter("control.max_linear_accel_m_s2", 0.50).value)
        self.max_angular_accel = float(self.declare_parameter("control.max_angular_accel_rad_s2", 1.00).value)
        self.slowdown_distance = float(self.declare_parameter("control.slowdown_distance_m", 0.30).value)
        self.position_tolerance = float(self.declare_parameter("control.position_tolerance_m", 0.025).value)
        self.yaw_tolerance = float(self.declare_parameter("control.yaw_tolerance_rad", 0.060).value)
        self.require_yaw_reached = bool(self.declare_parameter("control.require_yaw_reached", True).value)
        self.brake_hold_s = float(self.declare_parameter("control.brake_hold_s", 0.30).value)

        if not self.competition_config_path:
            raise RuntimeError("competition_config is required for direct competition navigation")
        self.competition = load_competition_config(self.competition_config_path)
        nav_cfg = self.competition.navigation
        startup_cfg = nav_cfg.get("startup_localization", {})
        if isinstance(startup_cfg, dict):
            self.localization_warmup_s = float(startup_cfg.get("warmup_s", self.localization_warmup_s))
            self.localization_candidate_timeout_s = float(startup_cfg.get("candidate_timeout_s", self.localization_candidate_timeout_s))
            self.localization_sample_count = int(startup_cfg.get("sample_count", self.localization_sample_count))
            self.max_origin_offset_m = float(startup_cfg.get("max_origin_offset_m", self.max_origin_offset_m))
            self.max_origin_yaw_rad = float(startup_cfg.get("max_origin_yaw_rad", self.max_origin_yaw_rad))
            self.localization_stability_xy_m = float(startup_cfg.get("stability_xy_m", self.localization_stability_xy_m))
            self.localization_stability_yaw_rad = float(startup_cfg.get("stability_yaw_rad", self.localization_stability_yaw_rad))
            self.map_scan_min_agreement = float(startup_cfg.get("min_map_scan_agreement", self.map_scan_min_agreement))
            self.map_scan_min_hits = int(startup_cfg.get("min_map_scan_hits", self.map_scan_min_hits))
            self.localization_require_global_constraint = bool(
                startup_cfg.get("require_global_constraint", self.localization_require_global_constraint)
            )
            self.localization_min_global_constraints = max(
                1,
                int(startup_cfg.get("min_global_constraints", self.localization_min_global_constraints)),
            )
            self.localization_post_constraint_settle_s = max(
                0.0,
                float(
                    startup_cfg.get(
                        "post_constraint_settle_s",
                        self.localization_post_constraint_settle_s,
                    )
                ),
            )
        control_cfg = nav_cfg.get("direct_control", {})
        if isinstance(control_cfg, dict):
            self.kp_xy = float(control_cfg.get("kp_xy", self.kp_xy))
            self.kp_yaw = float(control_cfg.get("kp_yaw", self.kp_yaw))
            self.max_linear_speed = float(control_cfg.get("max_linear_speed_m_s", self.max_linear_speed))
            self.max_angular_speed = float(control_cfg.get("max_angular_speed_rad_s", self.max_angular_speed))
            self.max_linear_accel = float(control_cfg.get("max_linear_accel_m_s2", self.max_linear_accel))
            self.max_angular_accel = float(control_cfg.get("max_angular_accel_rad_s2", self.max_angular_accel))
            self.slowdown_distance = float(control_cfg.get("slowdown_distance_m", self.slowdown_distance))
            self.position_tolerance = float(control_cfg.get("position_tolerance_m", self.position_tolerance))
            self.yaw_tolerance = float(control_cfg.get("yaw_tolerance_rad", self.yaw_tolerance))
            self.require_yaw_reached = bool(control_cfg.get("require_yaw_reached", self.require_yaw_reached))
            self.brake_hold_s = float(control_cfg.get("brake_hold_s", self.brake_hold_s))
        self.localization_candidates = localization_candidates(
            self.competition.navigation,
            self.competition.source_path,
        )
        self.arena_lock = ArenaLock()

        self.latest_odom: Optional[Pose2D] = None
        self.latest_odom_stamp: Optional[Time] = None
        self.latest_scan: Optional[LaserScan] = None
        self.frozen_map_to_odom: Optional[Pose2D] = None
        self.target_map: Optional[Pose2D] = None
        self.target_odom: Optional[Pose2D] = None
        self.active_waypoint = ""
        self.active_timeout_s = self.default_waypoint_timeout_s
        self.active_started_at: Optional[Time] = None
        self.phase = self.PHASE_IDLE
        self.state = NavigationStatus.STATE_IDLE
        self.error_code = 0
        self.message = "idle"
        self.distance_error = 0.0
        self.yaw_error = 0.0
        self.last_cmd = VelocityCommand(0.0, 0.0, 0.0)
        self.last_update_time = self.get_clock().now()
        self.brake_started_at: Optional[Time] = None

        self.localization_launcher = StartupLocalizationLauncher()
        self.localization_index = -1
        self.localization_candidate: Optional[LocalizationCandidate] = None
        self.localization_candidate_started_at: Optional[Time] = None
        self.localization_retry_after: Optional[Time] = None
        self.localization_mismatch_logged = False
        self.localization_tf_samples: list[Pose2D] = []
        self.localization_odom_samples: list[Pose2D] = []
        self.localization_results: list[tuple[LocalizationCandidate, LocalizationEvaluation]] = []
        self.localization_global_constraint_baseline: Optional[int] = None
        self.localization_global_constraint_count = 0
        self.localization_global_constraint_seen_at: Optional[Time] = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.map_tf_broadcaster = TransformBroadcaster(self)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.status_pub = self.create_publisher(NavigationStatus, self.status_topic, 10)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.on_odom, 20)
        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.on_scan, 10)
        self.constraint_sub = self.create_subscription(
            MarkerArray,
            self.localization_constraint_topic,
            self.on_constraint_list,
            10,
        )
        self.start_srv = self.create_service(StartNavigation, self.start_service, self.on_start)
        self.cancel_srv = self.create_service(CancelNavigation, self.cancel_service, self.on_cancel)
        self.timer = self.create_timer(1.0 / max(1.0, self.control_rate_hz), self.on_timer)
        self.get_logger().info(
            f"direct odom backend ready backend={self.backend_name}; "
            f"startup candidates={len(self.localization_candidates)}"
        )

    def on_odom(self, msg: Odometry) -> None:
        self.latest_odom = Pose2D(
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            yaw_from_quaternion(msg.pose.pose.orientation),
        )
        self.latest_odom_stamp = (
            Time.from_msg(msg.header.stamp)
            if msg.header.stamp.sec or msg.header.stamp.nanosec
            else self.get_clock().now()
        )

    def on_scan(self, msg: LaserScan) -> None:
        self.latest_scan = msg

    def on_constraint_list(self, msg: MarkerArray) -> None:
        # Cartographer visualizes loop-closure constraints on /constraint_list.
        # A constraint between different trajectories is the evidence we need:
        # the newly started localization trajectory has actually connected to
        # the frozen trajectory loaded from the pbstream. Stable map->odom TF
        # alone is not sufficient because Cartographer can publish a stable
        # identity-like transform before global relocalization has happened.
        if self.phase != self.PHASE_LOCALIZING:
            return
        count = 0
        for marker in msg.markers:
            if marker.ns == "Inter constraints, different trajectories":
                count += len(marker.points) // 2

        # The pbstream itself may already contain more than one frozen
        # trajectory. Capture that pre-existing count first and only accept
        # constraints added after this startup-localization process began.
        if self.localization_global_constraint_baseline is None:
            self.localization_global_constraint_baseline = count
            self.message = f"global constraint baseline={count}; waiting for new map constraint"
            return

        new_constraints = max(0, count - self.localization_global_constraint_baseline)
        if new_constraints > self.localization_global_constraint_count:
            self.localization_global_constraint_count = new_constraints
        if (
            self.localization_global_constraint_seen_at is None
            and self.localization_global_constraint_count >= self.localization_min_global_constraints
        ):
            self.localization_global_constraint_seen_at = self.get_clock().now()
            # Discard every TF sample collected before loop closure. The pose
            # graph can jump when the first cross-trajectory constraint lands.
            self.localization_tf_samples.clear()
            self.localization_odom_samples.clear()
            self.localization_retry_after = None
            self.localization_mismatch_logged = False
            self.message = (
                f"global map constraint observed "
                f"({self.localization_global_constraint_count}); settling"
            )
            self.get_logger().info(self.message)

    def odom_fresh(self, now: Time) -> bool:
        if self.latest_odom is None or self.latest_odom_stamp is None:
            return False
        age = (now - self.latest_odom_stamp).nanoseconds * 1e-9
        return 0.0 <= age <= self.odom_timeout_s

    def on_start(self, request: StartNavigation.Request, response: StartNavigation.Response):
        now = self.get_clock().now()
        if request.backend and request.backend != self.backend_name:
            response.success = False
            response.message = f"backend mismatch: requested {request.backend}, this is {self.backend_name}"
            return response
        if self.state == NavigationStatus.STATE_RUNNING:
            response.success = False
            response.message = "navigation is already running"
            return response
        if not self.odom_fresh(now):
            response.success = False
            response.message = "no fresh odom"
            self.fail(3102, response.message)
            return response

        waypoint_id = str(request.waypoint_id or "").strip()
        if not waypoint_id:
            response.success = False
            response.message = "waypoint_id is required"
            return response

        timeout = float(request.timeout_s) if request.timeout_s > 0.0 else self.default_waypoint_timeout_s
        self.active_waypoint = waypoint_id
        self.active_timeout_s = timeout
        self.active_started_at = now
        self.error_code = 0
        self.distance_error = 0.0
        self.yaw_error = 0.0
        self.last_cmd = VelocityCommand(0.0, 0.0, 0.0)
        self.brake_started_at = None
        self.state = NavigationStatus.STATE_RUNNING

        if waypoint_id == "origin":
            self.target_map = Pose2D(0.0, 0.0, 0.0)
            if self.frozen_map_to_odom is None:
                if not self.begin_startup_localization(str(request.arena or "").strip().upper()):
                    response.success = False
                    response.message = self.message
                    return response
                response.success = True
                response.message = "startup localization accepted"
                self.publish_status()
                return response
            self.begin_tracking_target()
            response.success = True
            response.message = "origin correction accepted"
            return response

        if self.frozen_map_to_odom is None:
            response.success = False
            response.message = "startup origin localization has not completed"
            self.fail(3103, response.message)
            return response

        try:
            arena = self.arena_lock.accept(request.arena)
            semantic = resolve_navigation_waypoint(
                self.competition.navigation,
                arena,
                waypoint_id,
                base_path=self.competition.source_path,
            )
        except CompetitionConfigError as exc:
            response.success = False
            response.message = str(exc)
            self.fail(3104, response.message)
            return response

        self.target_map = Pose2D(semantic.x_m, semantic.y_m, semantic.yaw_rad)
        self.begin_tracking_target()
        response.success = True
        response.message = "direct odom navigation accepted"
        return response

    def on_cancel(self, request: CancelNavigation.Request, response: CancelNavigation.Response):
        reason = str(request.reason or "cancel requested")
        self.localization_launcher.shutdown()
        self.stop_with_state(NavigationStatus.STATE_CANCELLED, reason, 0)
        response.success = True
        response.message = reason
        return response

    def begin_startup_localization(self, arena: str = "") -> bool:
        self.localization_candidates = localization_candidates(
            self.competition.navigation,
            self.competition.source_path,
            arena=arena,
        )
        existing = [
            c for c in self.localization_candidates
            if Path(c.map_path).is_file() and Path(c.pbstream_path).is_file()
        ]
        if not existing:
            self.fail(3110, "no complete map/pbstream asset is available for startup localization")
            return False
        self.localization_candidates = existing
        self.localization_results.clear()
        self.localization_index = -1
        self.phase = self.PHASE_LOCALIZING
        self.message = "startup localization"
        return self.start_next_localization_candidate()

    def start_next_localization_candidate(self) -> bool:
        self.localization_launcher.shutdown()
        self.localization_index += 1
        self.localization_tf_samples.clear()
        self.localization_odom_samples.clear()
        self.localization_retry_after = None
        self.localization_mismatch_logged = False
        self.localization_global_constraint_baseline = None
        self.localization_global_constraint_count = 0
        self.localization_global_constraint_seen_at = None
        if self.localization_index >= len(self.localization_candidates):
            return self.finish_startup_localization()

        candidate = self.localization_candidates[self.localization_index]
        self.localization_candidate = candidate
        try:
            self.localization_launcher.start(candidate)
        except (OSError, subprocess.SubprocessError) as exc:  # type: ignore[name-defined]
            self.get_logger().error(f"failed to start startup localization: {exc}")
            return self.start_next_localization_candidate()
        self.localization_candidate_started_at = self.get_clock().now()
        self.message = f"localizing with candidate {candidate.label}"
        self.get_logger().info(
            f"startup localization candidate={candidate.label} pbstream={candidate.pbstream_path}"
        )
        return True

    def sample_startup_localization(self, now: Time) -> None:
        if self.localization_candidate is None or self.localization_candidate_started_at is None:
            self.fail(3111, "localization candidate state missing")
            return
        elapsed = (now - self.localization_candidate_started_at).nanoseconds * 1e-9
        if elapsed > self.localization_candidate_timeout_s:
            self.finish_current_candidate("candidate timeout")
            return
        if elapsed < self.localization_warmup_s or not self.odom_fresh(now):
            return
        if self.localization_retry_after is not None and now < self.localization_retry_after:
            return
        if not self.localization_launcher.running():
            self.finish_current_candidate("cartographer exited")
            return
        if self.localization_require_global_constraint:
            if self.localization_global_constraint_seen_at is None:
                self.message = "waiting for Cartographer global map constraint"
                return
            settled_s = (now - self.localization_global_constraint_seen_at).nanoseconds * 1e-9
            if settled_s < self.localization_post_constraint_settle_s:
                self.message = (
                    f"global map constraint observed; settling "
                    f"{settled_s:.1f}/{self.localization_post_constraint_settle_s:.1f}s"
                )
                return

        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.odom_frame, Time())
        except TransformException:
            return

        tf_stamp = Time.from_msg(tf.header.stamp)
        if (
            tf_stamp.nanoseconds
            and tf_stamp.nanoseconds < self.localization_candidate_started_at.nanoseconds
        ):
            return
        t = tf.transform.translation
        q = tf.transform.rotation
        map_to_odom = Pose2D(float(t.x), float(t.y), yaw_from_quaternion(q))
        assert self.latest_odom is not None
        self.localization_tf_samples.append(map_to_odom)
        self.localization_odom_samples.append(self.latest_odom)
        if len(self.localization_tf_samples) >= max(3, self.localization_sample_count):
            self.finish_current_candidate("")

    def finish_current_candidate(self, fallback_reason: str) -> None:
        candidate = self.localization_candidate
        if candidate is None:
            self.fail(3112, "localization candidate missing")
            return
        required_samples = max(3, self.localization_sample_count)
        if (
            self.localization_require_global_constraint
            and self.localization_global_constraint_seen_at is None
        ):
            reason = fallback_reason or "no cross-trajectory Cartographer constraint observed"
            self.get_logger().warn(f"candidate {candidate.label} rejected: {reason}")
            self.start_next_localization_candidate()
            return
        if len(self.localization_tf_samples) < required_samples:
            reason = fallback_reason or (
                f"insufficient localization samples: "
                f"{len(self.localization_tf_samples)}/{required_samples}"
            )
            self.get_logger().warn(f"candidate {candidate.label} rejected: {reason}")
            self.start_next_localization_candidate()
            return
        evaluation = evaluate_localization_candidate(
            self.localization_tf_samples,
            self.localization_odom_samples,
            max_origin_offset_m=self.max_origin_offset_m,
            max_origin_yaw_rad=self.max_origin_yaw_rad,
            stability_xy_m=self.localization_stability_xy_m,
            stability_yaw_rad=self.localization_stability_yaw_rad,
        )
        now = self.get_clock().now()
        elapsed = (now - self.localization_candidate_started_at).nanoseconds * 1e-9
        if not evaluation.valid and elapsed + 2.0 < self.localization_candidate_timeout_s:
            self.localization_tf_samples.clear()
            self.localization_odom_samples.clear()
            self.localization_retry_after = now + Duration(seconds=2.0)
            if not self.localization_mismatch_logged:
                self.get_logger().warn(f"candidate {candidate.label} awaiting stable localization: {evaluation.reason}")
                self.localization_mismatch_logged = True
            return
        if evaluation.valid:
            agreement, hits = self.map_scan_agreement(candidate, evaluation.frozen_map_to_odom)
            if hits < self.map_scan_min_hits or agreement < self.map_scan_min_agreement:
                evaluation = replace(
                    evaluation,
                    valid=False,
                    reason=(
                        f"laser does not match saved map: {agreement:.2f} agreement "
                        f"over {hits} hits (need {self.map_scan_min_agreement:.2f}/{self.map_scan_min_hits})"
                    ),
                )
                if elapsed + 2.0 < self.localization_candidate_timeout_s:
                    self.localization_tf_samples.clear()
                    self.localization_odom_samples.clear()
                    self.localization_retry_after = now + Duration(seconds=2.0)
                    if not self.localization_mismatch_logged:
                        self.get_logger().warn(f"candidate {candidate.label} awaiting map match: {evaluation.reason}")
                        self.localization_mismatch_logged = True
                    return
            else:
                # Select the saved map whose observed walls fit best. Distance
                # from map origin cannot be a quality score: the robot may
                # legitimately start away from (0, 0, 0).
                evaluation = replace(
                    evaluation,
                    score=1.0 - agreement + evaluation.xy_spread_m + evaluation.yaw_spread_rad,
                )
        if evaluation.valid:
            self.localization_results.append((candidate, evaluation))
            self.get_logger().info(
                f"candidate {candidate.label} accepted score={evaluation.score:.4f} "
                f"robot=({evaluation.robot_pose_map.x:.3f},{evaluation.robot_pose_map.y:.3f},"
                f"{evaluation.robot_pose_map.yaw:.3f})"
            )
        else:
            reason = evaluation.reason if self.localization_tf_samples else fallback_reason or evaluation.reason
            self.get_logger().warn(f"candidate {candidate.label} rejected: {reason}")
        self.start_next_localization_candidate()

    def map_scan_agreement(self, candidate: LocalizationCandidate, map_to_odom: Pose2D) -> tuple[float, int]:
        scan = self.latest_scan
        if scan is None or self.latest_odom is None or self.latest_odom_stamp is None:
            self.get_logger().warn("saved map scan check unavailable: missing /scan or /odom")
            return 0.0, 0
        now = self.get_clock().now()
        stamp = Time.from_msg(scan.header.stamp)
        scan_age_s = (now - stamp).nanoseconds * 1e-9
        if abs(scan_age_s) > 0.5:
            self.get_logger().warn(
                f"saved map scan check unavailable: /scan timestamp age {scan_age_s:.2f}s"
            )
            return 0.0, 0
        if not self.odom_fresh(now):
            self.get_logger().warn("saved map scan check unavailable: /odom is stale")
            return 0.0, 0
        try:
            transform = self.tf_buffer.lookup_transform(self.base_frame, scan.header.frame_id, Time())
            offset = transform.transform.translation
            orientation = transform.transform.rotation
            base_to_laser = Pose2D(float(offset.x), float(offset.y), yaw_from_quaternion(orientation))
            map_to_laser = compose_pose(compose_pose(map_to_odom, self.latest_odom), base_to_laser)
            return MapScanMatch(candidate.map_path).agreement(scan, map_to_laser)
        except (OSError, ValueError, KeyError, IndexError, StopIteration, TransformException) as exc:
            self.get_logger().warn(f"saved map scan check unavailable: {exc}")
            return 0.0, 0

    def finish_startup_localization(self) -> bool:
        self.localization_launcher.shutdown()
        if not self.localization_results:
            self.fail(3113, "no startup localization candidate matched the saved map")
            return False
        candidate, evaluation = min(self.localization_results, key=lambda item: item[1].score)
        self.frozen_map_to_odom = evaluation.frozen_map_to_odom
        self.publish_frozen_map_tf()
        self.get_logger().info(
            f"startup alignment frozen from candidate {candidate.label}: "
            f"map->odom=({self.frozen_map_to_odom.x:.3f},"
            f"{self.frozen_map_to_odom.y:.3f},{self.frozen_map_to_odom.yaw:.3f})"
        )
        self.begin_tracking_target()
        return True

    def publish_frozen_map_tf(self) -> None:
        if self.frozen_map_to_odom is None:
            return
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.map_frame
        transform.child_frame_id = self.odom_frame
        transform.transform.translation.x = self.frozen_map_to_odom.x
        transform.transform.translation.y = self.frozen_map_to_odom.y
        half_yaw = self.frozen_map_to_odom.yaw / 2.0
        transform.transform.rotation.z = math.sin(half_yaw)
        transform.transform.rotation.w = math.cos(half_yaw)
        self.map_tf_broadcaster.sendTransform(transform)

    def begin_tracking_target(self) -> None:
        if self.frozen_map_to_odom is None or self.target_map is None:
            self.fail(3120, "cannot track without frozen field/odom alignment and target")
            return
        self.target_odom = target_map_to_odom(self.frozen_map_to_odom, self.target_map)
        self.phase = self.PHASE_TRACKING
        self.message = "moving by frozen field/odom alignment"
        self.brake_started_at = None

    def on_timer(self) -> None:
        now = self.get_clock().now()
        self.publish_frozen_map_tf()
        dt = max(0.001, (now - self.last_update_time).nanoseconds * 1e-9)
        self.last_update_time = now
        if self.state != NavigationStatus.STATE_RUNNING:
            self.publish_status()
            return
        if self.active_started_at is not None:
            elapsed = (now - self.active_started_at).nanoseconds * 1e-9
            if elapsed > self.active_timeout_s:
                self.localization_launcher.shutdown()
                self.fail(3121, "waypoint timeout")
                return
        if self.phase == self.PHASE_LOCALIZING:
            self.sample_startup_localization(now)
        elif self.phase == self.PHASE_TRACKING:
            self.update_tracking(now, dt)
        elif self.phase == self.PHASE_BRAKE:
            self.update_brake(now)
        self.publish_status()

    def update_tracking(self, now: Time, dt: float) -> None:
        if not self.odom_fresh(now):
            self.fail(3122, "odom timeout")
            return
        if self.latest_odom is None or self.target_odom is None:
            self.fail(3123, "tracking target missing")
            return
        dx = self.target_odom.x - self.latest_odom.x
        dy = self.target_odom.y - self.latest_odom.y
        self.distance_error = math.hypot(dx, dy)
        self.yaw_error = normalize_angle(self.target_odom.yaw - self.latest_odom.yaw)
        position_ok = self.distance_error <= self.position_tolerance
        yaw_ok = (not self.require_yaw_reached) or abs(self.yaw_error) <= self.yaw_tolerance
        if position_ok and yaw_ok:
            self.publish_zero()
            self.last_cmd = VelocityCommand(0.0, 0.0, 0.0)
            self.phase = self.PHASE_BRAKE
            self.brake_started_at = now
            self.message = "brake hold"
            return

        desired = compute_body_tracking_command(
            self.latest_odom,
            self.target_odom,
            kp_xy=self.kp_xy,
            kp_yaw=self.kp_yaw,
            max_linear_speed_m_s=self.max_linear_speed,
            max_angular_speed_rad_s=self.max_angular_speed,
            slowdown_distance_m=self.slowdown_distance,
        )
        cmd = limit_acceleration(
            desired,
            self.last_cmd,
            dt,
            max_linear_accel_m_s2=self.max_linear_accel,
            max_angular_accel_rad_s2=self.max_angular_accel,
        )
        self.last_cmd = cmd
        msg = Twist()
        msg.linear.x = cmd.vx
        msg.linear.y = cmd.vy
        msg.angular.z = cmd.wz
        self.cmd_pub.publish(msg)
        self.message = "tracking"

    def update_brake(self, now: Time) -> None:
        self.publish_zero()
        if self.brake_started_at is None:
            self.brake_started_at = now
            return
        if (now - self.brake_started_at).nanoseconds * 1e-9 >= self.brake_hold_s:
            self.state = NavigationStatus.STATE_SUCCEEDED
            self.phase = self.PHASE_IDLE
            self.message = "waypoint reached with brake hold"
            self.error_code = 0
            self.publish_status()

    def publish_zero(self) -> None:
        self.cmd_pub.publish(Twist())

    def fail(self, error_code: int, message: str) -> None:
        self.localization_launcher.shutdown()
        self.publish_zero()
        self.state = NavigationStatus.STATE_FAILED
        self.phase = self.PHASE_IDLE
        self.error_code = int(error_code)
        self.message = message
        self.last_cmd = VelocityCommand(0.0, 0.0, 0.0)
        self.publish_status()

    def stop_with_state(self, state: int, message: str, error_code: int) -> None:
        self.publish_zero()
        self.state = state
        self.phase = self.PHASE_IDLE
        self.error_code = int(error_code)
        self.message = message
        self.last_cmd = VelocityCommand(0.0, 0.0, 0.0)
        self.publish_status()

    def publish_status(self) -> None:
        msg = NavigationStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.backend = self.backend_name
        msg.state = self.state
        msg.waypoint_id = self.active_waypoint
        if self.target_map is not None:
            msg.target_x_m = float(self.target_map.x)
            msg.target_y_m = float(self.target_map.y)
            msg.target_yaw_rad = float(self.target_map.yaw)
        msg.distance_error_m = float(self.distance_error)
        msg.yaw_error_rad = float(self.yaw_error)
        msg.error_code = int(self.error_code)
        msg.message = self.message
        self.status_pub.publish(msg)

    def destroy_node(self):
        self.localization_launcher.shutdown()
        self.publish_zero()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DirectNavBackend()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
