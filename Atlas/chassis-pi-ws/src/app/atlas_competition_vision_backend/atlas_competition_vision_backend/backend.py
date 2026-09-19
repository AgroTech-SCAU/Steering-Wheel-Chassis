from __future__ import annotations

from dataclasses import dataclass, field
import time
from typing import Callable, Iterable, Optional

import yaml

from atlas_competition_config.config import (
    apply_vision_backend_overrides,
    load_optional_competition_config,
)

try:
    import rclpy
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Bool
    from std_srvs.srv import Trigger
    from atlas_mission_interfaces.srv import ClassifySortingRule, DetectCameraTarget
    from vison_topic_interfaces.msg import DetectionCenterArray
    from vison_topic_interfaces.srv import VisionDetect
except ImportError:  # Unit tests exercise the pure helpers without a ROS environment.
    rclpy = None
    ReentrantCallbackGroup = None
    MultiThreadedExecutor = None
    Node = object
    QoSProfile = None
    ReliabilityPolicy = None
    DurabilityPolicy = None
    Bool = None
    Trigger = None
    ClassifySortingRule = None
    DetectCameraTarget = None
    DetectionCenterArray = None
    VisionDetect = None


@dataclass(frozen=True)
class Detection:
    cls_name: str
    u: float
    v: float
    conf: float = 1.0
    corner_index: int = -1


@dataclass(frozen=True)
class BackendConfig:
    class_aliases: dict[str, str] = field(default_factory=dict)
    sorting_enabled: bool = False

    @classmethod
    def from_dict(cls, data: dict) -> "BackendConfig":
        rule = data.get("sorting_rule", {})
        return cls(
            class_aliases={
                str(k): str(v) for k, v in data.get("class_aliases", {}).items()
            },
            sorting_enabled=bool(rule.get("enabled", False)),
        )


@dataclass(frozen=True)
class SortingRuleResult:
    success: bool
    arena: str = ""
    park_1_cargo: str = ""
    park_2_cargo: str = ""
    message: str = ""


@dataclass(frozen=True)
class DetectTargetResult:
    success: bool
    cargo_class: str = ""
    layer_ok: bool = False
    complete: bool = False
    message: str = ""
    target_count: int = 0


def _canonical_class(cls_name: str, aliases: dict[str, str]) -> Optional[str]:
    cargo = aliases.get(cls_name, cls_name)
    return cargo if cargo in {"gear", "t_bolt"} else None


def _best_detection_by_cargo(
    detections: Iterable[Detection],
    aliases: dict[str, str],
) -> dict[str, Detection]:
    best: dict[str, Detection] = {}
    for detection in detections:
        cargo = _canonical_class(detection.cls_name, aliases)
        if cargo is None:
            continue
        current = best.get(cargo)
        if current is None or detection.conf > current.conf:
            best[cargo] = detection
    return best


def sorting_reframe_direction(
    detections: Iterable[Detection],
    aliases: dict[str, str],
    image_center_u_px: float,
) -> Optional[str]:
    """只有一个有效分类标志时，根据其横向位置给出 joint1 追视方向。

    现场约定：单个标志位于画面中间/右侧时，底座 joint1 向右微调；
    位于左侧时向左微调。若不是“恰好一个有效标志”，不做方向猜测。
    """
    valid = [
        detection
        for detection in detections
        if _canonical_class(detection.cls_name, aliases) is not None
    ]
    if len(valid) != 1:
        return None
    return "right" if float(valid[0].u) >= float(image_center_u_px) else "left"


def resolve_sorting_rule(
    detections: Iterable[Detection],
    config: BackendConfig,
    arena: str,
) -> SortingRuleResult:
    if not config.sorting_enabled:
        return SortingRuleResult(False, message="sorting_rule.enabled=false")

    parts = list(detections)
    if len(parts) != 2:
        return SortingRuleResult(
            False, message="sorting view must contain exactly two parts"
        )
    best = _best_detection_by_cargo(parts, config.class_aliases)
    if "gear" not in best or "t_bolt" not in best:
        return SortingRuleResult(
            False,
            message="sorting view must contain both gear and t_bolt",
        )

    gear = best["gear"]
    t_bolt = best["t_bolt"]
    if gear.u == t_bolt.u:
        return SortingRuleResult(
            False,
            message="sorting left/right order is ambiguous",
        )

    if arena not in {"A", "B"}:
        return SortingRuleResult(False, message=f"unknown arena {arena}")
    left, right = ("gear", "t_bolt") if gear.u < t_bolt.u else ("t_bolt", "gear")
    # A/B 两个场区都按同一画面语义：左侧=园区一，右侧=园区二。
    park_1_cargo, park_2_cargo = left, right

    return SortingRuleResult(
        True,
        park_1_cargo=park_1_cargo,
        park_2_cargo=park_2_cargo,
        message=f"arena {arena} sorting rule decoded by image left/right order",
    )


def classify_with_scan_sequence(
    scan: Callable[[str], Optional[Iterable[Detection]]],
    config: BackendConfig,
) -> SortingRuleResult:
    for arena, scan_name in (("A", "sorting_scan_a"), ("B", "sorting_scan_b")):
        detections = scan(scan_name)
        if detections is None:
            return SortingRuleResult(False, message=f"{scan_name} scan failed")
        result = resolve_sorting_rule(detections, config, arena)
        if result.success:
            return SortingRuleResult(
                True,
                arena=arena,
                park_1_cargo=result.park_1_cargo,
                park_2_cargo=result.park_2_cargo,
                message=f"{scan_name} decoded sorting rule",
            )
    return SortingRuleResult(False, message="no valid sorting rule in scan A or B")


def detect_camera_target_from_centers(
    request,
    centers: Iterable[Detection],
    aliases: dict[str, str],
) -> DetectTargetResult:
    slot = int(request.slot)
    expected_layer = int(request.expected_layer)
    if slot not in (0, 1, 2, 3):
        return DetectTargetResult(False, message="slot must be 0..3")
    if expected_layer not in (1, 2):
        return DetectTargetResult(False, message="expected_layer must be 1..2")

    detections = list(centers)
    matching = [d for d in detections if int(d.corner_index) == slot]
    if (
        not matching
        and len(detections) == 1
        and str(getattr(request, "waypoint_id", "pickup")) == "pickup"
    ):
        # A calibrated slot view can show only its one remaining cargo. The
        # detector labels a lone object corner 0 regardless of physical slot.
        matching = detections
    max_targets = int(getattr(request, "max_targets", 1) or 1)
    target_count = min(len(matching), max_targets)
    if not matching:
        return DetectTargetResult(
            True,
            layer_ok=False,
            complete=False,
            message=f"no detection for corner_index={slot}",
            target_count=0,
        )

    best = max(matching, key=lambda detection: detection.conf)
    cargo_class = _canonical_class(best.cls_name, aliases)
    if cargo_class is None:
        return DetectTargetResult(
            True,
            layer_ok=False,
            complete=False,
            message=f"unknown cargo class {best.cls_name}",
            target_count=target_count,
        )

    target_class = str(getattr(request, "target_class", "") or "")
    target_ok = not target_class or cargo_class == _canonical_class(
        target_class, aliases
    )
    return DetectTargetResult(
        True,
        cargo_class=cargo_class,
        layer_ok=target_ok,
        complete=target_ok,
        message="target detected" if target_ok else "target class mismatch",
        target_count=target_count,
    )


class CompetitionVisionBackend(Node):
    def __init__(self) -> None:
        if rclpy is None:
            raise RuntimeError(
                "rclpy is required to run the competition vision backend"
            )
        super().__init__("atlas_competition_vision_backend")
        self._group = ReentrantCallbackGroup()

        self.declare_parameter("service_timeout_s", 3.0)
        self.declare_parameter("sorting_scan_timeout_s", 40.0)
        self.declare_parameter("vision_pose_ready_timeout_s", 8.0)
        self.declare_parameter("detection_window_s", 1.0)
        self.declare_parameter("final_centers_wait_s", 0.2)
        self.declare_parameter("pose_settle_s", 0.3)
        self.declare_parameter("sorting_reframe_attempts", 2)
        self.declare_parameter("sorting_image_center_u_px", 320.0)
        self.declare_parameter("topics.vision_pose_ready", "/vision_pose_ready")
        self.declare_parameter("topics.detection_centers", "/detection_centers")
        self.declare_parameter(
            "services.classify_sorting", "/atlas/vision/classify_sorting_rule"
        )
        self.declare_parameter("services.detect_target", "/atlas/vision/detect_target")
        self.declare_parameter("services.vision_detect", "/vision_detect")
        self.declare_parameter(
            "services.move_to_sorting_scan_a",
            "/atlas/manipulation/move_to_sorting_scan_a",
        )
        self.declare_parameter(
            "services.move_to_sorting_scan_b",
            "/atlas/manipulation/move_to_sorting_scan_b",
        )
        self.declare_parameter(
            "services.adjust_sorting_scan_left",
            "/atlas/manipulation/adjust_sorting_scan_left",
        )
        self.declare_parameter(
            "services.adjust_sorting_scan_right",
            "/atlas/manipulation/adjust_sorting_scan_right",
        )
        self.declare_parameter("competition_config", "")
        self.declare_parameter("class_aliases.chilun", "gear")
        self.declare_parameter("class_aliases.luosi", "t_bolt")
        self.declare_parameter("sorting_rule.enabled", False)

        self._config = self._load_config()
        self._vision_pose_ready = False
        self._latest_centers: list[Detection] = []
        self._final_centers: Optional[list[Detection]] = None
        self._capture_start_ns = 0

        ready_qos = QoSProfile(depth=1)
        ready_qos.reliability = ReliabilityPolicy.RELIABLE
        ready_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            Bool,
            str(self.get_parameter("topics.vision_pose_ready").value),
            self._on_vision_pose_ready,
            ready_qos,
            callback_group=self._group,
        )
        self.create_subscription(
            DetectionCenterArray,
            str(self.get_parameter("topics.detection_centers").value),
            self._on_detection_centers,
            10,
            callback_group=self._group,
        )

        self._vision_detect = self.create_client(
            VisionDetect,
            str(self.get_parameter("services.vision_detect").value),
            callback_group=self._group,
        )
        self._move_scan_a = self.create_client(
            Trigger,
            str(self.get_parameter("services.move_to_sorting_scan_a").value),
            callback_group=self._group,
        )
        self._move_scan_b = self.create_client(
            Trigger,
            str(self.get_parameter("services.move_to_sorting_scan_b").value),
            callback_group=self._group,
        )
        self._adjust_scan_left = self.create_client(
            Trigger,
            str(self.get_parameter("services.adjust_sorting_scan_left").value),
            callback_group=self._group,
        )
        self._adjust_scan_right = self.create_client(
            Trigger,
            str(self.get_parameter("services.adjust_sorting_scan_right").value),
            callback_group=self._group,
        )

        self.create_service(
            ClassifySortingRule,
            str(self.get_parameter("services.classify_sorting").value),
            self._on_classify_sorting,
            callback_group=self._group,
        )
        self.create_service(
            DetectCameraTarget,
            str(self.get_parameter("services.detect_target").value),
            self._on_detect_target,
            callback_group=self._group,
        )
        self.get_logger().info("competition vision backend ready")

    def _load_config(self) -> BackendConfig:
        base = {
            "class_aliases": {
                "chilun": str(self.get_parameter("class_aliases.chilun").value),
                "luosi": str(self.get_parameter("class_aliases.luosi").value),
            },
            "sorting_rule": {
                "enabled": bool(self.get_parameter("sorting_rule.enabled").value),
            },
        }
        competition = load_optional_competition_config(
            str(self.get_parameter("competition_config").value)
        )
        if competition is not None:
            base = apply_vision_backend_overrides(base, competition.vision)
        return BackendConfig.from_dict(base)

    def _on_vision_pose_ready(self, msg) -> None:
        self._vision_pose_ready = bool(msg.data)

    def _on_detection_centers(self, msg) -> None:
        centers = [
            Detection(
                d.cls_name, float(d.u), float(d.v), float(d.conf), int(d.corner_index)
            )
            for d in msg.detections
        ]
        self._latest_centers = centers
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(
            msg.header.stamp.nanosec
        )
        if (
            msg.is_final_best
            and self._capture_start_ns
            and stamp_ns >= self._capture_start_ns
        ):
            self._final_centers = centers

    def _wait_for_service(self, client, label: str) -> Optional[str]:
        timeout_s = float(self.get_parameter("service_timeout_s").value)
        if not client.wait_for_service(timeout_sec=timeout_s):
            return f"{label} service unavailable"
        return None

    def _call_trigger(
        self, client, label: str, timeout_s: Optional[float] = None
    ) -> tuple[bool, str]:
        error = self._wait_for_service(client, label)
        if error:
            return False, error
        future = client.call_async(Trigger.Request())
        if not self._wait_future(future, timeout_s):
            return False, f"{label} service timeout"
        result = future.result()
        return bool(result and result.success), str(result.message if result else "")

    def _wait_future(self, future, timeout_s: Optional[float] = None) -> bool:
        timeout = (
            timeout_s
            if timeout_s is not None
            else float(self.get_parameter("service_timeout_s").value)
        )
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if future.done():
                return True
            time.sleep(0.01)
        return future.done()

    def _wait_vision_pose_ready(self) -> bool:
        deadline = time.monotonic() + float(
            self.get_parameter("vision_pose_ready_timeout_s").value
        )
        while time.monotonic() < deadline:
            if self._vision_pose_ready:
                return True
            time.sleep(0.02)
        return False

    def _wait_observation_pose(self) -> bool:
        if not self._wait_vision_pose_ready():
            return False
        time.sleep(max(0.0, float(self.get_parameter("pose_settle_s").value)))
        return self._vision_pose_ready

    def _set_vision_detect(self, start: bool):
        error = self._wait_for_service(self._vision_detect, "vision_detect")
        if error:
            return None, error
        request = VisionDetect.Request()
        request.start = bool(start)
        future = self._vision_detect.call_async(request)
        if not self._wait_future(future):
            return None, "vision_detect service timeout"
        return future.result(), ""

    def _capture_sorting_view(self) -> Optional[list[Detection]]:
        start_response, error = self._set_vision_detect(True)
        if error or not start_response or not start_response.success:
            self.get_logger().warn(error or start_response.message)
            return None
        time.sleep(float(self.get_parameter("detection_window_s").value))
        stop_response, error = self._set_vision_detect(False)
        if error or not stop_response or not stop_response.success:
            self.get_logger().warn(error or stop_response.message)
            return None
        return [
            Detection(cls_name, u, v, 1.0)
            for cls_name, u, v in zip(
                stop_response.cls_names, stop_response.u_px, stop_response.v_px
            )
        ]

    def _scan_view(self, scan_name: str) -> Optional[list[Detection]]:
        client = (
            self._move_scan_a if scan_name == "sorting_scan_a" else self._move_scan_b
        )
        scan_timeout = float(self.get_parameter("sorting_scan_timeout_s").value)
        ok, message = self._call_trigger(client, scan_name, scan_timeout)
        if not ok:
            self.get_logger().warn(f"{scan_name} move failed: {message}")
            return None
        if not self._wait_observation_pose():
            self.get_logger().warn(
                f"{scan_name} did not publish vision_pose_ready=true"
            )
            return None

        detections = self._capture_sorting_view()
        if detections is None:
            return None
        if resolve_sorting_rule(detections, self._config, "A").success:
            return detections

        attempts = max(0, int(self.get_parameter("sorting_reframe_attempts").value))
        center_u = float(self.get_parameter("sorting_image_center_u_px").value)
        for attempt in range(attempts):
            direction = sorting_reframe_direction(
                detections, self._config.class_aliases, center_u
            )
            if direction is None:
                break
            adjust_client = (
                self._adjust_scan_right if direction == "right" else self._adjust_scan_left
            )
            ok, message = self._call_trigger(
                adjust_client,
                f"adjust_sorting_scan_{direction}",
                scan_timeout,
            )
            if not ok:
                self.get_logger().warn(
                    f"{scan_name} adaptive joint1 adjust failed: {message}"
                )
                return None
            if not self._wait_observation_pose():
                self.get_logger().warn(
                    f"{scan_name} joint1 adjust did not remain vision-ready"
                )
                return None
            self.get_logger().info(
                f"{scan_name} only one marker visible; joint1 {direction} "
                f"reframe attempt {attempt + 1}/{attempts}"
            )
            detections = self._capture_sorting_view()
            if detections is None:
                return None
            # 成功看到一齿轮一 T 型螺栓后立即结束补偿，不继续晃动机械臂。
            if resolve_sorting_rule(detections, self._config, "A").success:
                break
        return detections

    def _capture_target_centers(self) -> tuple[Optional[list[Detection]], str]:
        if not self._wait_observation_pose():
            return None, "vision pose not ready or unstable"
        self._final_centers = None
        self._capture_start_ns = self.get_clock().now().nanoseconds
        start_response, error = self._set_vision_detect(True)
        if error or not start_response or not start_response.success:
            self.get_logger().warn(error or start_response.message)
            self._capture_start_ns = 0
            return None, error or str(start_response.message)
        time.sleep(float(self.get_parameter("detection_window_s").value))
        stop_response, error = self._set_vision_detect(False)
        if error or not stop_response or not stop_response.success:
            self.get_logger().warn(error or stop_response.message)
            self._capture_start_ns = 0
            return None, error or str(stop_response.message)
        deadline = time.monotonic() + float(
            self.get_parameter("final_centers_wait_s").value
        )
        while self._final_centers is None and time.monotonic() < deadline:
            time.sleep(0.01)
        if self._final_centers is None and int(stop_response.count) > 0:
            self._capture_start_ns = 0
            return None, "final detection centers not received"
        result = list(self._final_centers or [])
        self._capture_start_ns = 0
        return result, ""

    def _on_classify_sorting(self, _request, response):
        if not self._config.sorting_enabled:
            result = resolve_sorting_rule([], self._config, "A")
        else:
            result = classify_with_scan_sequence(self._scan_view, self._config)
        response.success = result.success
        response.arena = result.arena
        response.park_1_cargo = result.park_1_cargo
        response.park_2_cargo = result.park_2_cargo
        response.message = result.message
        return response

    def _on_detect_target(self, request, response):
        centers, error = self._capture_target_centers()
        if centers is None:
            response.success = False
            response.layer_ok = False
            response.complete = False
            response.message = error
            response.target_count = 0
            return response
        result = detect_camera_target_from_centers(
            request,
            centers,
            self._config.class_aliases,
        )
        response.success = result.success
        response.cargo_class = result.cargo_class
        response.layer_ok = result.layer_ok
        response.complete = result.complete
        response.message = result.message
        response.target_count = result.target_count
        return response


def load_yaml_config(path: str) -> BackendConfig:
    with open(path, encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    if "competition" in data:
        return BackendConfig.from_dict(
            data.get("competition", {}).get("vision", {}) or {}
        )
    params = data.get("atlas_competition_vision_backend", {}).get(
        "ros__parameters", data
    )
    return BackendConfig.from_dict(params)


def main() -> None:
    rclpy.init()
    node = CompetitionVisionBackend()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
