#!/usr/bin/env python3
"""Atlas 智械争锋机械臂动作后端"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Optional

import yaml

from atlas_competition_config.config import (
    apply_manipulation_placement_overrides,
    load_optional_competition_config,
    resolve_arm_pose,
    resolve_pickup_layer_z,
    resolve_placement_reference,
)

try:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import JointState
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Bool
    from std_srvs.srv import SetBool, Trigger

    from atlas_mission_interfaces.msg import ManipulationStatus
    from atlas_mission_interfaces.srv import CancelManipulation, StartManipulation
    from mcu_comm_bridge.srv import SetArmJoints, SetArmPosition
    from vison_topic_interfaces.msg import PickTarget
except ImportError:  # Unit tests exercise pure config helpers without ROS.
    rclpy = None
    PoseStamped = None
    JointState = None
    MutuallyExclusiveCallbackGroup = None
    Node = object
    MultiThreadedExecutor = None
    DurabilityPolicy = None
    QoSProfile = None
    ReliabilityPolicy = None
    Bool = None
    SetBool = None
    Trigger = None
    ManipulationStatus = None
    CancelManipulation = None
    StartManipulation = None
    SetArmJoints = None
    SetArmPosition = None
    PickTarget = None


@dataclass
class XYZ:
    x: float
    y: float
    z: float

    def distance(self, other: "XYZ") -> float:
        return math.sqrt(
            (self.x - other.x) ** 2
            + (self.y - other.y) ** 2
            + (self.z - other.z) ** 2
        )


def pickup_target_spec(arm_motion: dict, arena: str, layer: int, slot: int = 0) -> dict[str, float]:
    # 抓取姿态与观察位角度解耦：吸盘沿基座 -Z 轴垂直向下 (pitch=0, yaw=0)。
    # 如需补偿工具安装偏角，在 competition.yaml 的 arm_motion.pick_orientation 微调。
    orientation = arm_motion.get("pick_orientation") or {}
    return {
        "target_z_m": resolve_pickup_layer_z(arm_motion, arena, layer, slot),
        "pitch_rad": float(orientation.get("pitch_rad", 0.0)),
        "yaw_rad": float(orientation.get("yaw_rad", 0.0)),
    }


def compute_placement_target(
    arm_motion: dict,
    placement: dict,
    arena: str,
    park: str,
    slot: int,
    existing_layer: int,
) -> XYZ:
    if not bool(placement.get("enabled", False)):
        raise RuntimeError(
            "placement.enabled=false：请先完成园区放置标定，再开启自动放置"
        )
    if slot not in (0, 1, 2, 3):
        raise ValueError(f"park slot 非法: {slot}")
    if existing_layer < 0:
        raise ValueError(f"existing layer 非法: {existing_layer}")
    park_cfg = arm_motion.get("arenas", {}).get(arena, {}).get(park, {})
    reference = resolve_placement_reference(arm_motion, arena, park, slot)
    if "placement_points" in park_cfg:
        dx = dy = 0.0
    else:
        offsets = list(placement.get("slot_offsets_xy_m", []) or [])
        if len(offsets) != 8:
            raise ValueError("placement.slot_offsets_xy_m 必须正好包含 8 个数")
        dx = float(offsets[slot * 2])
        dy = float(offsets[slot * 2 + 1])
    step = float(placement.get("layer_step_m", 0.050))
    return XYZ(
        reference["x_m"] + dx,
        reference["y_m"] + dy,
        reference["first_layer_z_m"] + float(existing_layer) * step,
    )


def compute_pick_contact_target(above: XYZ, target_z_m: float) -> XYZ:
    return XYZ(above.x, above.y, float(target_z_m))


class CompetitionManipulationBackend(Node):
    def __init__(self) -> None:
        if rclpy is None:
            raise RuntimeError("rclpy is required to run the competition manipulation backend")
        super().__init__("atlas_competition_manipulation_backend")
        # Sorting callbacks wait for MCU service replies and joint feedback.
        # Keep them off the default group used by those clients/subscriptions.
        self._sorting_group = MutuallyExclusiveCallbackGroup()

        self.backend_name = str(self.declare_parameter("backend_name", "vision_arm").value)
        self.start_service = str(
            self.declare_parameter("start_service", "/atlas/manipulation/start").value
        )
        self.cancel_service = str(
            self.declare_parameter("cancel_service", "/atlas/manipulation/cancel").value
        )
        self.status_topic = str(
            self.declare_parameter("status_topic", "/atlas/manipulation/status").value
        )
        self.initial_pose_service = str(
            self.declare_parameter("initial_pose_service", "/move_to_initial_pose").value
        )
        self.initial_pose_ready_topic = str(
            self.declare_parameter("initial_pose_ready_topic", "/initial_pose_ready").value
        )
        self.pick_target_topic = str(
            self.declare_parameter("pick_target_topic", "/pick_target").value
        )
        self.arm_pose_topic = str(
            self.declare_parameter("arm_pose_topic", "/arm/pose").value
        )
        self.arm_joint_state_topic = str(
            self.declare_parameter("arm_joint_state_topic", "/arm/joint_states").value
        )
        self.arm_joints_service = str(
            self.declare_parameter("arm_joints_service", "/mcu/set_arm_joints").value
        )
        self.arm_position_service = str(
            self.declare_parameter("arm_position_service", "/mcu/set_arm_position").value
        )
        self.suction_service = str(
            self.declare_parameter("suction_service", "/mcu/set_suction").value
        )
        self.declare_parameter("competition_config", "")

        self.service_timeout_s = float(self.declare_parameter("service_timeout_s", 2.0).value)
        self.motion_timeout_s = float(self.declare_parameter("motion_timeout_s", 12.0).value)
        self.initial_pose_timeout_s = float(
            self.declare_parameter("initial_pose_timeout_s", 15.0).value
        )
        self.position_tolerance_m = float(
            self.declare_parameter("position_tolerance_m", 0.015).value
        )
        self.stable_delta_m = float(self.declare_parameter("stable_delta_m", 0.004).value)
        self.stable_samples = int(self.declare_parameter("stable_samples", 5).value)
        self.min_pick_target_motion_m = float(
            self.declare_parameter("min_pick_target_motion_m", 0.005).value
        )
        self.pick_target_settle_s = float(
            self.declare_parameter("pick_target_settle_s", 0.25).value
        )
        self.pick_approach_m = float(
            self.declare_parameter("pick_approach_m", 0.050).value
        )
        self.suction_settle_s = float(
            self.declare_parameter("suction_settle_s", 0.45).value
        )
        # 抓取接触后额外保持吸盘开启的时间。与放置释放后的 settle 分离，
        # 避免为了延长吸取等待而拖慢每次放置动作。
        self.pick_suction_hold_s = float(
            self.declare_parameter("pick_suction_hold_s", 1.50).value
        )
        self.default_speed_rad_s = float(
            self.declare_parameter("default_speed_rad_s", 0.8).value
        )
        self.joint_tolerance_rad = float(
            self.declare_parameter("joint_tolerance_rad", 0.06).value
        )
        self.pose_validation_tolerance_m = float(
            self.declare_parameter("pose_validation_tolerance_m", 0.08).value
        )
        # 到位后静置时间：机械臂完全静止后再开始视觉检测，避免"还没稳定就开"
        self.settle_before_observe_s = float(
            self.declare_parameter("settle_before_observe_s", 0.5).value
        )

        self.view_scan_enabled = bool(
            self.declare_parameter("view_scan.enabled", False).value
        )
        self.view_scan_dx_m = float(
            self.declare_parameter("view_scan.dx_m", 0.0).value
        )
        self.view_scan_dy_m = float(
            self.declare_parameter("view_scan.dy_m", 0.025).value
        )
        self.view_scan_dz_m = float(
            self.declare_parameter("view_scan.dz_m", 0.0).value
        )
        self.view_scan_offsets_m = self._load_view_scan_offsets()

        # 智能分拣区观察补偿：只调整底座 joint1(q0)，不改其它关节和末端姿态约束。
        # 方向由视觉后端根据“当前只看到的一个标志”位于画面左右侧来决定。
        self.sorting_joint1_step_rad = abs(float(
            self.declare_parameter("sorting_scan.joint1_step_rad", 0.07).value
        ))
        self.sorting_joint1_max_offset_rad = abs(float(
            self.declare_parameter("sorting_scan.joint1_max_offset_rad", 0.14).value
        ))
        self.sorting_joint1_settle_s = max(0.0, float(
            self.declare_parameter("sorting_scan.joint1_settle_s", 0.35).value
        ))
        self.sorting_joint1_right_sign = 1.0 if float(
            self.declare_parameter("sorting_scan.joint1_right_sign", 1.0).value
        ) >= 0.0 else -1.0

        self.competition = load_optional_competition_config(
            str(self.get_parameter("competition_config").value)
        )
        self.arm_motion = {} if self.competition is None else self.competition.arm_motion
        placement_config = self._load_placement_config(self.competition)
        self.placement_config = placement_config
        self.place_enabled = bool(placement_config["enabled"])
        self.place_approach_m = float(placement_config["approach_m"])

        self._state_lock = threading.Lock()
        self._pose_cv = threading.Condition()
        self._joint_cv = threading.Condition()
        self._initial_cv = threading.Condition()
        self._latest_pose: Optional[XYZ] = None
        self._latest_joints: Optional[list[float]] = None
        self._latest_pose_time = 0.0
        self._initial_ready = False
        self._worker: Optional[threading.Thread] = None
        self._cancel_event = threading.Event()
        self._active_sorting_scan = ""
        self._sorting_joint1_offset_rad = 0.0

        self._status_state = ManipulationStatus.STATE_IDLE
        self._status_waypoint = ""
        self._status_task = ""
        self._status_step = "idle"
        self._status_error = 0
        self._status_message = "空闲"

        self.status_pub = self.create_publisher(ManipulationStatus, self.status_topic, 10)
        self.start_srv = self.create_service(StartManipulation, self.start_service, self._on_start)
        self.cancel_srv = self.create_service(CancelManipulation, self.cancel_service, self._on_cancel)

        ready_qos = QoSProfile(depth=1)
        ready_qos.reliability = ReliabilityPolicy.RELIABLE
        ready_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.initial_sub = self.create_subscription(
            Bool, self.initial_pose_ready_topic, self._on_initial_ready, ready_qos
        )
        self.pose_sub = self.create_subscription(
            PoseStamped, self.arm_pose_topic, self._on_arm_pose, 20
        )
        self.joint_sub = self.create_subscription(
            JointState, self.arm_joint_state_topic, self._on_joint_state, 20
        )
        self.pick_target_pub = self.create_publisher(PickTarget, self.pick_target_topic, 10)

        self.initial_pose_client = self.create_client(Trigger, self.initial_pose_service)
        self.arm_joints_client = self.create_client(SetArmJoints, self.arm_joints_service)
        self.arm_position_client = self.create_client(SetArmPosition, self.arm_position_service)
        self.suction_client = self.create_client(SetBool, self.suction_service)
        self.sorting_scan_a_srv = self.create_service(
            Trigger, "/atlas/manipulation/move_to_sorting_scan_a", self._on_sorting_scan_a,
            callback_group=self._sorting_group,
        )
        self.sorting_scan_b_srv = self.create_service(
            Trigger, "/atlas/manipulation/move_to_sorting_scan_b", self._on_sorting_scan_b,
            callback_group=self._sorting_group,
        )
        self.sorting_scan_left_srv = self.create_service(
            Trigger,
            "/atlas/manipulation/adjust_sorting_scan_left",
            self._on_sorting_scan_left,
            callback_group=self._sorting_group,
        )
        self.sorting_scan_right_srv = self.create_service(
            Trigger,
            "/atlas/manipulation/adjust_sorting_scan_right",
            self._on_sorting_scan_right,
            callback_group=self._sorting_group,
        )

        self.status_timer = self.create_timer(0.2, self._publish_status)
        self.get_logger().info(
            f"机械臂动作后端已启动 backend={self.backend_name}; "
            f"place_enabled={self.place_enabled}; view_scan_enabled={self.view_scan_enabled}"
        )

    def _load_placement_config(self, competition=None) -> dict:
        base = {
            "placement": {
                "enabled": bool(self.declare_parameter("placement.enabled", False).value),
                "approach_m": float(self.declare_parameter("placement.approach_m", 0.060).value),
                "layer_step_m": float(
                    self.declare_parameter("placement.layer_step_m", 0.050).value
                ),
                "park_1": {
                    "x_m": float(self.declare_parameter("placement.park_1.x_m", 0.0).value),
                    "y_m": float(self.declare_parameter("placement.park_1.y_m", 0.0).value),
                    "first_layer_z_m": float(
                        self.declare_parameter(
                            "placement.park_1.first_layer_z_m", 0.0
                        ).value
                    ),
                },
                "park_2": {
                    "x_m": float(self.declare_parameter("placement.park_2.x_m", 0.0).value),
                    "y_m": float(self.declare_parameter("placement.park_2.y_m", 0.0).value),
                    "first_layer_z_m": float(
                        self.declare_parameter(
                            "placement.park_2.first_layer_z_m", 0.0
                        ).value
                    ),
                },
                "slot_offsets_xy_m": list(
                    self.declare_parameter(
                        "placement.slot_offsets_xy_m",
                        [0.0, 0.0, 0.05, 0.0, 0.05, 0.05, 0.0, 0.05],
                    ).value
                ),
            }
        }
        if competition is None:
            competition = load_optional_competition_config(
                str(self.get_parameter("competition_config").value)
            )
        if competition is not None:
            base = apply_manipulation_placement_overrides(base, competition.manipulation)
        return base["placement"]

    # ---------------- ROS 状态 ----------------
    def _on_initial_ready(self, msg: Bool) -> None:
        with self._initial_cv:
            self._initial_ready = bool(msg.data)
            self._initial_cv.notify_all()

    def _on_arm_pose(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        xyz = XYZ(float(p.x), float(p.y), float(p.z))
        with self._pose_cv:
            self._latest_pose = xyz
            self._latest_pose_time = time.monotonic()
            self._pose_cv.notify_all()

    def _on_joint_state(self, msg: JointState) -> None:
        values = [float(v) for v in list(msg.position)[:5]]
        if len(values) != 5:
            return
        with self._joint_cv:
            self._latest_joints = values
            self._joint_cv.notify_all()

    def _set_status(
        self,
        state: int,
        *,
        waypoint: Optional[str] = None,
        task: Optional[str] = None,
        step: Optional[str] = None,
        error: Optional[int] = None,
        message: Optional[str] = None,
    ) -> None:
        with self._state_lock:
            self._status_state = int(state)
            if waypoint is not None:
                self._status_waypoint = waypoint
            if task is not None:
                self._status_task = task
            if step is not None:
                self._status_step = step
            if error is not None:
                self._status_error = int(error)
            if message is not None:
                self._status_message = str(message)
        self._publish_status()

    def _publish_status(self) -> None:
        with self._state_lock:
            msg = ManipulationStatus()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.state = self._status_state
            msg.backend = self.backend_name
            msg.waypoint_id = self._status_waypoint
            msg.task_id = self._status_task
            msg.step_name = self._status_step
            msg.error_code = self._status_error
            msg.message = self._status_message
        self.status_pub.publish(msg)

    # ---------------- 服务入口 ----------------
    def _on_start(self, request: StartManipulation.Request, response: StartManipulation.Response):
        if request.backend and request.backend != self.backend_name:
            response.success = False
            response.message = (
                f"后端不匹配: request={request.backend}, current={self.backend_name}"
            )
            return response

        with self._state_lock:
            if self._status_state == ManipulationStatus.STATE_RUNNING:
                response.success = False
                response.message = "已有机械臂任务正在运行"
                return response

            self._status_state = ManipulationStatus.STATE_RUNNING
            self._status_waypoint = request.waypoint_id
            self._status_task = request.arrival_task
            self._status_step = "accepted"
            self._status_error = 0
            self._status_message = "任务已接受"

        self._cancel_event.clear()
        request_copy = StartManipulation.Request()
        request_copy.backend = request.backend
        if hasattr(request_copy, "arena"):
            request_copy.arena = getattr(request, "arena", "")
        request_copy.waypoint_id = request.waypoint_id
        request_copy.prepare_action = request.prepare_action
        request_copy.arrival_task = request.arrival_task
        request_copy.slot = request.slot
        request_copy.layer = request.layer
        request_copy.cargo_class = request.cargo_class

        self._worker = threading.Thread(
            target=self._run_task, args=(request_copy,), daemon=True
        )
        self._worker.start()
        response.success = True
        response.message = "机械臂任务已提交"
        self._publish_status()
        return response

    def _on_cancel(self, request: CancelManipulation.Request, response: CancelManipulation.Response):
        self._cancel_event.set()
        self._set_status(
            ManipulationStatus.STATE_CANCELLED,
            step="cancelled",
            error=0,
            message=request.reason or "收到取消请求",
        )
        response.success = True
        response.message = request.reason or "已请求取消机械臂任务"
        return response

    # ---------------- 通用等待 ----------------
    def _cancelled(self) -> bool:
        return self._cancel_event.is_set() or not rclpy.ok()

    def _call_service(self, client, request, timeout_s: Optional[float] = None):
        timeout = float(timeout_s if timeout_s is not None else self.service_timeout_s)
        if not client.wait_for_service(timeout_sec=max(0.1, timeout)):
            raise RuntimeError(f"服务不可用: {client.srv_name}")
        future = client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout):
            raise TimeoutError(f"服务调用超时: {client.srv_name}")
        if future.exception() is not None:
            raise RuntimeError(f"服务异常 {client.srv_name}: {future.exception()}")
        return future.result()

    def _current_pose(self) -> Optional[XYZ]:
        with self._pose_cv:
            return None if self._latest_pose is None else XYZ(
                self._latest_pose.x, self._latest_pose.y, self._latest_pose.z
            )

    def _wait_initial_ready(self, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        with self._initial_cv:
            while time.monotonic() < deadline:
                if self._cancelled():
                    return False
                if self._initial_ready:
                    return True
                self._initial_cv.wait(timeout=min(0.1, max(0.0, deadline - time.monotonic())))
        return False

    def _wait_pose_target(self, target: XYZ, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        stable = 0
        with self._pose_cv:
            while time.monotonic() < deadline:
                if self._cancelled():
                    return False
                if self._latest_pose is not None:
                    if self._latest_pose.distance(target) <= self.position_tolerance_m:
                        stable += 1
                        if stable >= max(1, self.stable_samples):
                            return True
                    else:
                        stable = 0
                self._pose_cv.wait(timeout=0.05)
        return False

    def _wait_for_motion_then_stable(self, start: XYZ, timeout_s: float) -> Optional[XYZ]:
        deadline = time.monotonic() + timeout_s
        moved = False
        stable = 0
        last: Optional[XYZ] = None
        with self._pose_cv:
            while time.monotonic() < deadline:
                if self._cancelled():
                    return None
                current = self._latest_pose
                if current is not None:
                    current_copy = XYZ(current.x, current.y, current.z)
                    if current_copy.distance(start) >= self.min_pick_target_motion_m:
                        moved = True
                    if moved and last is not None and current_copy.distance(last) <= self.stable_delta_m:
                        stable += 1
                    elif moved:
                        stable = 0
                    last = current_copy
                    if moved and stable >= max(1, self.stable_samples):
                        return current_copy
                self._pose_cv.wait(timeout=0.05)
        return None

    def _move_position(self, target: XYZ, *, suction_valid: bool, suction_enable: bool) -> bool:
        if self._cancelled():
            return False
        req = SetArmPosition.Request()
        req.x_m = float(target.x)
        req.y_m = float(target.y)
        req.z_m = float(target.z)
        req.speed_rad_s = float(self.default_speed_rad_s)
        req.suction_valid = bool(suction_valid)
        req.suction_enable = bool(suction_enable)
        result = self._call_service(self.arm_position_client, req)
        if result is None or not result.success:
            return False
        return self._wait_pose_target(target, self.motion_timeout_s)

    def _wait_joint_target(self, target: list[float], timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        stable = 0
        with self._joint_cv:
            while time.monotonic() < deadline:
                if self._cancelled():
                    return False
                if self._latest_joints is not None:
                    error = max(abs(a - b) for a, b in zip(self._latest_joints, target))
                    if error <= self.joint_tolerance_rad:
                        stable += 1
                        if stable >= max(1, self.stable_samples):
                            return True
                    else:
                        stable = 0
                self._joint_cv.wait(timeout=0.05)
        return False

    def _move_named_pose(self, pose_name: str, arena: str = "", area: str = "", slot: int = 0) -> bool:
        pose = resolve_arm_pose(
            self.arm_motion, pose_name, arena=arena or None, area=area or None, slot=slot
        )
        req = SetArmJoints.Request()
        req.joints_rad = [float(v) for v in pose["joints_rad"]]
        req.speed_rad_s = float(pose["speed_rad_s"])
        req.suction_valid = False
        req.suction_enable = False
        result = self._call_service(self.arm_joints_client, req)
        if result is None or not result.success:
            return False
        if not self._wait_joint_target(req.joints_rad, self.motion_timeout_s):
            return False
        current = self._current_pose()
        expected = XYZ(float(pose["x_m"]), float(pose["y_m"]), float(pose["z_m"]))
        if current is not None and current.distance(expected) > self.pose_validation_tolerance_m:
            self.get_logger().error(
                f"{pose_name} 关节已到位但 TCP 偏差 {current.distance(expected):.3f} m 超限"
            )
            return False
        return True

    def _on_sorting_scan_a(self, _request, response):
        try:
            response.success = self._move_named_pose("sorting_scan_a")
            if response.success:
                self._active_sorting_scan = "sorting_scan_a"
                self._sorting_joint1_offset_rad = 0.0
            response.message = "sorting_scan_a 到位" if response.success else "sorting_scan_a 到位失败"
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = str(exc)
        return response

    def _on_sorting_scan_b(self, _request, response):
        try:
            response.success = self._move_named_pose("sorting_scan_b")
            if response.success:
                self._active_sorting_scan = "sorting_scan_b"
                self._sorting_joint1_offset_rad = 0.0
            response.message = "sorting_scan_b 到位" if response.success else "sorting_scan_b 到位失败"
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = str(exc)
        return response

    def _adjust_sorting_joint1(self, direction: str) -> tuple[bool, str]:
        """在当前分拣观察位基础上只微调 joint1(q0)。

        direction 是相机画面语义方向：right 表示底座向右追视，left 表示向左。
        joint1_right_sign 用来适配实机正负方向；默认 +1，现场若方向相反只改 YAML。
        """
        if self._active_sorting_scan not in {"sorting_scan_a", "sorting_scan_b"}:
            return False, "尚未进入 sorting_scan_a/b 固定观察位"
        if self.sorting_joint1_step_rad <= 0.0:
            return False, "sorting_scan.joint1_step_rad 必须 > 0"

        pose = resolve_arm_pose(self.arm_motion, self._active_sorting_scan)
        joints = [float(v) for v in pose["joints_rad"]]
        if len(joints) != 5:
            return False, "sorting scan joints_rad 必须包含 5 个关节"

        sign = self.sorting_joint1_right_sign
        if direction == "left":
            sign *= -1.0
        elif direction != "right":
            return False, f"未知 joint1 调整方向: {direction}"

        next_offset = self._sorting_joint1_offset_rad + sign * self.sorting_joint1_step_rad
        limit = self.sorting_joint1_max_offset_rad
        if limit > 0.0:
            next_offset = max(-limit, min(limit, next_offset))
        if math.isclose(next_offset, self._sorting_joint1_offset_rad, abs_tol=1e-9):
            return False, "joint1 已达到分拣观察补偿上限"

        joints[0] += next_offset
        req = SetArmJoints.Request()
        req.joints_rad = joints
        req.speed_rad_s = float(pose["speed_rad_s"])
        req.suction_valid = False
        req.suction_enable = False
        result = self._call_service(self.arm_joints_client, req)
        if result is None or not result.success:
            return False, "joint1 微调命令下发失败"
        if not self._wait_joint_target(joints, self.motion_timeout_s):
            return False, "joint1 微调后未在超时内到位"

        self._sorting_joint1_offset_rad = next_offset
        time.sleep(self.sorting_joint1_settle_s)
        return True, (
            f"{direction} 微调完成: joint1_offset="
            f"{self._sorting_joint1_offset_rad:+.3f} rad"
        )

    def _on_sorting_scan_left(self, _request, response):
        try:
            response.success, response.message = self._adjust_sorting_joint1("left")
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = str(exc)
        return response

    def _on_sorting_scan_right(self, _request, response):
        try:
            response.success, response.message = self._adjust_sorting_joint1("right")
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = str(exc)
        return response

    # ---------------- 比赛动作 ----------------
    def _run_task(self, request: StartManipulation.Request) -> None:
        task = (request.arrival_task or request.prepare_action or "").strip()
        try:
            arena = str(getattr(request, "arena", "") or "").strip().upper()
            if task == "pre_recognition":
                ok = self._do_pre_recognition(arena, request.waypoint_id, int(request.slot))
            elif task == "view_scan":
                ok = self._do_view_scan(
                    arena, request.waypoint_id, int(request.slot), int(request.layer)
                )
            elif task in {"zero", "sorting_scan_a", "sorting_scan_b", "navigation_safe"}:
                ok = self._move_named_pose(task)
            elif task == "pickup_observe":
                ok = self._move_named_pose("pickup_observe", arena=arena, slot=int(request.slot))
            elif task == "park_prepare":
                ok = self._move_named_pose("park_prepare", arena=arena, area=request.waypoint_id)
            elif task == "pick":
                ok = self._do_pick(arena, int(request.slot), int(request.layer))
            elif task == "place":
                ok = self._do_place(
                    arena, request.waypoint_id, int(request.slot), int(request.layer)
                )
            else:
                raise ValueError(f"不支持的机械臂任务: {task}")

            if self._cancelled():
                self._set_status(
                    ManipulationStatus.STATE_CANCELLED,
                    step="cancelled",
                    message="任务已取消",
                )
                return
            if ok:
                self._set_status(
                    ManipulationStatus.STATE_SUCCEEDED,
                    step="done",
                    error=0,
                    message=f"{task} 完成",
                )
            else:
                self._set_status(
                    ManipulationStatus.STATE_FAILED,
                    step="failed",
                    error=2101,
                    message=f"{task} 执行失败",
                )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().exception(f"机械臂任务异常: {exc}")
            if not self._cancelled():
                self._set_status(
                    ManipulationStatus.STATE_FAILED,
                    step="exception",
                    error=2199,
                    message=str(exc),
                )

    def _do_pre_recognition(self, arena: str, area: str, slot: int = 0) -> bool:
        self._set_status(
            ManipulationStatus.STATE_RUNNING,
            step="move_to_observe_pose",
            message=f"移动到 {area} 固定观察或预备位",
        )
        if area == "pickup":
            ok = self._move_named_pose("pickup_observe", arena=arena, slot=slot)
        elif area in {"park_1", "park_2"}:
            ok = self._move_named_pose("park_prepare", arena=arena, area=area)
        else:
            raise ValueError(f"pre_recognition 不支持区域: {area}")
        if ok:
            time.sleep(self.settle_before_observe_s)
        return ok

    def _load_view_scan_offsets(self) -> list[tuple[float, float, float]]:
        # offsets_m 是扁平 (dx,dy,dz) 三元组序列；每 3 个数一组，逐次自检换视角。
        raw = self.declare_parameter(
            "view_scan.offsets_m",
            [0.0, 0.0, 0.02, 0.0, 0.03, 0.0, -0.03, 0.0, 0.0],
        ).value
        values = [float(v) for v in (list(raw) if raw is not None else [])]
        if len(values) >= 3 and len(values) % 3 == 0:
            return [
                (values[i], values[i + 1], values[i + 2])
                for i in range(0, len(values), 3)
            ]
        return []

    def _view_scan_offset(self, attempt: int) -> tuple[float, float, float]:
        if self.view_scan_offsets_m:
            return self.view_scan_offsets_m[int(attempt) % len(self.view_scan_offsets_m)]
        return (self.view_scan_dx_m, self.view_scan_dy_m, self.view_scan_dz_m)

    def _do_view_scan(self, arena: str, area: str, slot: int = 0, attempt: int = 0) -> bool:
        if not self.view_scan_enabled:
            self._set_status(
                ManipulationStatus.STATE_RUNNING,
                step="view_scan_fallback",
                message="view_scan 未标定，退化为重新回固定观察位",
            )
            return self._do_pre_recognition(arena, area, slot)

        start = self._current_pose()
        if start is None:
            return False
        dx, dy, dz = self._view_scan_offset(attempt)
        target = XYZ(start.x + dx, start.y + dy, start.z + dz)
        self._set_status(
            ManipulationStatus.STATE_RUNNING,
            step="view_scan_move",
            message=f"换视角#{attempt} 到 ({target.x:.3f},{target.y:.3f},{target.z:.3f})",
        )
        ok = self._move_position(target, suction_valid=False, suction_enable=False)
        if ok:
            time.sleep(self.settle_before_observe_s)
        return ok

    def _do_pick(self, arena: str, slot: int, layer: int) -> bool:
        if slot not in (0, 1, 2, 3):
            raise ValueError(f"pickup slot 非法: {slot}")
        if layer not in (1, 2):
            raise ValueError(f"pickup layer 非法: {layer}")

        start = self._current_pose()
        if start is None:
            return False

        self._set_status(
            ManipulationStatus.STATE_RUNNING,
            step="select_pick_target",
            message=f"选择角点 slot={slot}, layer={layer}",
        )
        spec = pickup_target_spec(self.arm_motion, arena, layer, slot)
        msg = PickTarget()
        msg.corner_index = int(slot)
        msg.layer = int(layer)
        if hasattr(msg, "use_target_z"):
            msg.use_target_z = True
            msg.target_z_m = float(spec["target_z_m"])
        if hasattr(msg, "use_orientation"):
            msg.use_orientation = True
            msg.pitch_rad = float(spec["pitch_rad"])
            msg.yaw_rad = float(spec["yaw_rad"])
        if hasattr(msg, "use_approach"):
            msg.use_approach = True
            msg.approach_m = float(self.pick_approach_m)
        self.pick_target_pub.publish(msg)
        time.sleep(max(0.0, self.pick_target_settle_s))

        above = self._wait_for_motion_then_stable(start, self.motion_timeout_s)
        if above is None:
            self.get_logger().error("/pick_target 后机械臂未检测到有效移动并稳定")
            return False

        contact = compute_pick_contact_target(above, spec["target_z_m"])
        self._set_status(
            ManipulationStatus.STATE_RUNNING,
            step="pick_descend",
            message=f"下降到标定吸取高度 z={contact.z:.3f} m 并打开吸盘",
        )
        if not self._move_position(contact, suction_valid=True, suction_enable=True):
            return False

        # 到达接触高度后不要立刻抬升，给真空建立和吸盘贴合留出时间。
        time.sleep(max(0.0, self.pick_suction_hold_s))
        self._set_status(
            ManipulationStatus.STATE_RUNNING,
            step="pick_lift",
            message=f"吸附后回到抓取接近位 z={above.z:.3f} m",
        )
        return self._move_position(above, suction_valid=True, suction_enable=True)

    def _place_target(
        self, arena: str, park: str, slot: int, existing_layer: int
    ) -> XYZ:
        return compute_placement_target(
            self.arm_motion, self.placement_config, arena, park, slot, existing_layer
        )

    def _do_place(
        self, arena: str, park: str, slot: int, existing_layer: int
    ) -> bool:
        target = self._place_target(arena, park, slot, existing_layer)
        above = XYZ(target.x, target.y, target.z + self.place_approach_m)

        self._set_status(
            ManipulationStatus.STATE_RUNNING,
            step="place_approach",
            message=(
                f"{park} slot={slot} 当前已有层={existing_layer}，到放置点上方"
            ),
        )
        if not self._move_position(above, suction_valid=True, suction_enable=True):
            return False

        self._set_status(
            ManipulationStatus.STATE_RUNNING,
            step="place_descend",
            message="下降到释放高度",
        )
        if not self._move_position(target, suction_valid=True, suction_enable=True):
            return False

        req = SetBool.Request()
        req.data = False
        result = self._call_service(self.suction_client, req)
        if result is None or not result.success:
            return False
        time.sleep(max(0.0, self.suction_settle_s))

        self._set_status(
            ManipulationStatus.STATE_RUNNING,
            step="place_retreat",
            message="释放完成并抬起",
        )
        return self._move_position(above, suction_valid=False, suction_enable=False)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CompetitionManipulationBackend()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


def load_placement_config(path: str) -> dict:
    with open(path, encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    if "competition" in data:
        return dict(
            data.get("competition", {})
            .get("manipulation", {})
            .get("placement", {})
            or {}
        )
    return dict(
        data.get("atlas_competition_manipulation_backend", {})
        .get("ros__parameters", {})
        .get("placement", {})
        or {}
    )
