from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node

from mcu_comm_bridge.msg import AutoTaskEvent, McuStatus
from mcu_comm_bridge.srv import ReportMissionResult
from send_navigation_target.srv import NavigateToTarget
from vison_topic_interfaces.srv import VisionDetect


def xor_checksum(payload: str) -> int:
    value = 0
    for char in payload.encode("utf-8"):
        value ^= char
    return value & 0xFF


@dataclass
class AsrEvent:
    sequence: int
    event_type: str
    payload: str


class AsrproClient:
    """ASRPRO TWEN51 串口协议客户端"""

    def __init__(self, node: Node, port: str, baud: int, timeout_s: float):
        self._node = node
        self._port = port
        self._baud = baud
        self._timeout_s = timeout_s
        self._serial = None
        self._sequence = 1
        self._lock = threading.Lock()

    def open(self) -> bool:
        try:
            import serial
            self._serial = serial.Serial(self._port, self._baud, timeout=self._timeout_s)
            self._node.get_logger().info(f"ASRPRO serial opened: {self._port}@{self._baud}")
            return True
        except Exception as exc:
            self._node.get_logger().error(f"ASRPRO serial open failed: {exc}")
            self._serial = None
            return False

    def close(self) -> None:
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
        self._serial = None

    def _next_sequence(self) -> int:
        with self._lock:
            value = self._sequence
            self._sequence = 1 if self._sequence >= 65535 else self._sequence + 1
            return value

    def send(self, command: str, *arguments: str, sequence: Optional[int] = None) -> int:
        if self._serial is None:
            raise RuntimeError("ASRPRO serial is not open")
        seq = self._next_sequence() if sequence is None else sequence
        tokens = ["A2P", "1", str(seq), command]
        tokens.extend(str(item) for item in arguments if item is not None)
        payload = ",".join(tokens)
        frame = f"@{payload}*{xor_checksum(payload):02X}\r\n"
        self._serial.write(frame.encode("utf-8"))
        return seq

    def read_frame(self) -> Optional[List[str]]:
        if self._serial is None:
            return None
        try:
            raw = self._serial.readline()
        except Exception as exc:
            self._node.get_logger().warn(f"ASRPRO read failed: {exc}")
            return None
        if not raw:
            return None
        try:
            line = raw.decode("utf-8", errors="ignore").strip()
        except Exception:
            return None
        if not line.startswith("@") or "*" not in line:
            return None
        payload, checksum_text = line[1:].rsplit("*", 1)
        try:
            received = int(checksum_text[:2], 16)
        except ValueError:
            return None
        if xor_checksum(payload) != received:
            self._node.get_logger().warn(f"ASRPRO checksum mismatch: {line}")
            return None
        tokens = payload.split(",")
        if len(tokens) < 4 or tokens[0] != "P2A" or tokens[1] != "1":
            return None
        return tokens

    def wait_hello(self, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and rclpy.ok():
            tokens = self.read_frame()
            if not tokens:
                continue
            command = tokens[3]
            if command == "HELLO" and len(tokens) >= 6:
                seq = int(tokens[2])
                boot_id = tokens[5]
                self.send("HELLO_ACK", boot_id, sequence=seq)
                self._node.get_logger().info(f"ASRPRO handshake ok: {boot_id}")
                return True
        self._node.get_logger().warn("ASRPRO HELLO timeout, continue with degraded voice flow")
        return False

    def wait_ack(self, sequence: int, command: str, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and rclpy.ok():
            tokens = self.read_frame()
            if not tokens:
                continue
            if tokens[3] == "ACK" and int(tokens[2]) == sequence and len(tokens) >= 5 and tokens[4] == command:
                return True
            if tokens[3] == "NACK" and int(tokens[2]) == sequence:
                reason = tokens[5] if len(tokens) >= 6 else "UNKNOWN"
                self._node.get_logger().warn(f"ASRPRO NACK command={command} reason={reason}")
                return False
        return False

    def speak(self, phrase_id: str, timeout_s: float) -> bool:
        seq = self.send("SPEAK", phrase_id)
        if not self.wait_ack(seq, "SPEAK", 1.0):
            return False
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and rclpy.ok():
            event = self.poll_event()
            if event and event.event_type == "SPEAK_DONE" and event.payload == phrase_id:
                return True
        self._node.get_logger().warn(f"ASRPRO SPEAK_DONE timeout: {phrase_id}")
        return False

    def listen(self, enabled: bool) -> None:
        seq = self.send("LISTEN", "1" if enabled else "0")
        (void_ok := self.wait_ack(seq, "LISTEN", 1.0))
        if not void_ok:
            self._node.get_logger().warn(f"ASRPRO LISTEN ack timeout: {enabled}")

    def poll_event(self) -> Optional[AsrEvent]:
        tokens = self.read_frame()
        if not tokens or tokens[3] != "EVENT" or len(tokens) < 6:
            return None
        event = AsrEvent(sequence=int(tokens[2]), event_type=tokens[4], payload=tokens[5])
        self.send("EVENT_ACK", event.event_type, sequence=event.sequence)
        return event


class AtlasAutonomousTask(Node):
    """智械争锋全自主运输状态机"""

    def __init__(self):
        super().__init__("atlas_autonomous_task")

        self.mcu_status_topic = self.declare_parameter("mcu_status_topic", "/mcu/status").value
        self.auto_task_event_topic = self.declare_parameter("auto_task_event_topic", "/mcu/auto_task_event").value
        self.navigate_service = self.declare_parameter("navigate_service", "/navigate_to_target").value
        self.vision_service = self.declare_parameter("vision_service", "/vision_detect").value
        self.mission_result_service = self.declare_parameter("mission_result_service", "/mcu/report_mission_result").value
        self.log_dir = self.declare_parameter("log_dir", "log").value

        self.navigate_timeout_s = float(self.declare_parameter("navigate_timeout_s", 300.0).value)
        self.vision_scan_duration_s = float(self.declare_parameter("vision_scan_duration_s", 2.0).value)
        self.vision_service_timeout_s = float(self.declare_parameter("vision_service_timeout_s", 5.0).value)
        self.asr_start_timeout_s = float(self.declare_parameter("asr_start_timeout_s", -1.0).value)
        self.speak_timeout_s = float(self.declare_parameter("speak_timeout_s", 8.0).value)
        self.asrpro_handshake_timeout_s = float(self.declare_parameter("asrpro_handshake_timeout_s", 3.0).value)

        self.gear_class_names = set(self.declare_parameter("gear_class_names", ["chilun", "gear"]).value)
        self.bolt_class_names = set(self.declare_parameter("bolt_class_names", ["luosi", "bolt", "t_bolt"]).value)
        self.fallback_zone = self.declare_parameter("fallback_zone", "zone_1").value
        self.image_mid_u = float(self.declare_parameter("image_mid_u", 320.0).value)
        self.pre_detect_action_1 = self.declare_parameter("pre_detect_action_1", "pre_detect_left").value
        self.pre_detect_action_2 = self.declare_parameter("pre_detect_action_2", "pre_detect_right").value
        self.pre_detect_rounds = max(1, int(self.declare_parameter("pre_detect_rounds", 2).value))
        self.delivery_cargo_sequence = [
            str(item).lower() for item in self.declare_parameter("delivery_cargo_sequence", ["chilun"]).value
        ]

        self.phrase_transition_complete = self.declare_parameter("phrase_transition_complete", "transition_complete").value
        self.phrase_voice_prompt = self.declare_parameter("phrase_voice_prompt", "voice_prompt").value
        self.phrase_autonomous_start = self.declare_parameter("phrase_autonomous_start", "autonomous_start").value
        self.phrase_delivery_complete = self.declare_parameter("phrase_delivery_complete", "delivery_complete").value
        self.phrase_task_complete = self.declare_parameter("phrase_task_complete", "task_complete").value
        self.phrase_task_skipped = self.declare_parameter("phrase_task_skipped", "task_skipped").value

        self.waypoint_pickup = self.declare_parameter("waypoint_pickup", "P2").value
        self.waypoint_zone_1 = self.declare_parameter("waypoint_zone_1", "P3").value
        self.waypoint_zone_2 = self.declare_parameter("waypoint_zone_2", "P4").value
        self.skip_failed_stage = bool(self.declare_parameter("skip_failed_stage", True).value)

        asrpro_port = self.declare_parameter("asrpro_port", "/dev/ttyUSB0").value
        asrpro_baud = int(self.declare_parameter("asrpro_baud", 115200).value)
        asrpro_timeout_s = float(self.declare_parameter("asrpro_timeout_s", 0.02).value)
        self.asrpro = AsrproClient(self, asrpro_port, asrpro_baud, asrpro_timeout_s)
        self._log_file = self._open_log_file()

        self._latest_mcu_status: Optional[McuStatus] = None
        self._task_lock = threading.Lock()
        self._task_active = False
        self._abort_event = threading.Event()

        self.navigate_client = self.create_client(NavigateToTarget, self.navigate_service)
        self.vision_client = self.create_client(VisionDetect, self.vision_service)
        self.result_client = self.create_client(ReportMissionResult, self.mission_result_service)

        self.create_subscription(McuStatus, self.mcu_status_topic, self._on_mcu_status, 10)
        self.create_subscription(AutoTaskEvent, self.auto_task_event_topic, self._on_auto_task_event, 10)

        self.get_logger().info("Atlas autonomous task node started")
        self._log("Atlas autonomous task node started")

    def _open_log_file(self):
        try:
            log_dir = Path(str(self.log_dir))
            log_dir.mkdir(parents=True, exist_ok=True)
            path = log_dir / f"{datetime.now().strftime('%Y%m%d-%H%M%S')}-log.txt"
            handle = path.open("a", encoding="utf-8")
            self.get_logger().info(f"Full task log: {path}")
            return handle
        except Exception as exc:
            self.get_logger().warn(f"Failed to open file log: {exc}")
            return None

    def _log(self, message: str) -> None:
        if self._log_file is None:
            return
        stamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        try:
            self._log_file.write(f"[{stamp}] {message}\n")
            self._log_file.flush()
        except Exception:
            pass

    def _on_mcu_status(self, msg: McuStatus) -> None:
        self._latest_mcu_status = msg
        if self._task_active:
            if msg.app_state != McuStatus.STATE_AUTO_PI or not msg.auto_start_latched:
                self._abort_event.set()

    def _on_auto_task_event(self, msg: AutoTaskEvent) -> None:
        if msg.event == AutoTaskEvent.EVENT_RESET:
            self.get_logger().info("MCU RESET event received")
            self._abort_event.set()
            return
        if msg.event != AutoTaskEvent.EVENT_START:
            return
        status = self._latest_mcu_status
        if status is None or status.app_state != McuStatus.STATE_AUTO_PI or not status.auto_start_latched:
            self.get_logger().warn("START event ignored because MCU is not confirmed AutoPi")
            return
        with self._task_lock:
            if self._task_active:
                self.get_logger().warn("START event ignored because task is already active")
                return
            self._task_active = True
            self._abort_event.clear()
        threading.Thread(target=self._run_task, daemon=True).start()

    def _run_task(self) -> None:
        self.get_logger().info("AutoPi task started")
        self._log("AutoPi task started")
        try:
            self._prepare_asrpro()
            self._speak(self.phrase_transition_complete)
            self._speak(self.phrase_voice_prompt)
            if not self._wait_asr_start():
                self._skip("ASR start timeout")
            self._speak(self.phrase_autonomous_start)

            cargo_zone_map = self._detect_area_and_cargo_map()
            for cargo in self.delivery_cargo_sequence:
                zone = cargo_zone_map.get(cargo, self.fallback_zone)
                self._navigate_or_skip(self.waypoint_pickup, f"pickup area for {cargo}")
                target = self._waypoint_for_zone(zone)
                self._navigate_or_skip(target, f"{zone} for {cargo}")
                self._speak(self.phrase_delivery_complete)
            self._speak(self.phrase_task_complete)
            self._report_result(ReportMissionResult.Request.RESULT_DONE, 0)
        except RuntimeError as exc:
            self.get_logger().warn(f"Task aborted: {exc}")
            self._log(f"Task aborted: {exc}")
        except Exception as exc:
            self.get_logger().error(f"Task fatal error: {exc}")
            self._log(f"Task fatal error: {exc}")
            self._report_result(ReportMissionResult.Request.RESULT_FAIL, 1)
        finally:
            try:
                self.asrpro.listen(False)
            except Exception:
                pass
            with self._task_lock:
                self._task_active = False
            self.get_logger().info("AutoPi task stopped")
            self._log("AutoPi task stopped")

    def _check_abort(self) -> None:
        if self._abort_event.is_set():
            raise RuntimeError("MCU left AutoPi or RESET was received")

    def _prepare_asrpro(self) -> None:
        if self.asrpro.open():
            self.asrpro.wait_hello(self.asrpro_handshake_timeout_s)
            self.asrpro.listen(True)

    def _speak(self, phrase_id: str) -> None:
        self._check_abort()
        if self.asrpro._serial is None:
            return
        if not self.asrpro.speak(phrase_id, self.speak_timeout_s):
            self.get_logger().warn(f"Speak skipped or timeout: {phrase_id}")

    def _wait_asr_start(self) -> bool:
        if self.asrpro._serial is None:
            self.get_logger().warn("ASRPRO is unavailable, skip voice start wait")
            return True
        deadline = None if self.asr_start_timeout_s < 0.0 else time.monotonic() + self.asr_start_timeout_s
        self.get_logger().info("Waiting ASR event: atlas_start")
        self._log("Waiting ASR event: atlas_start")
        while rclpy.ok():
            self._check_abort()
            if deadline is not None and time.monotonic() > deadline:
                return False
            event = self.asrpro.poll_event()
            if event and event.event_type == "ASR" and event.payload == "atlas_start":
                self.get_logger().info("ASR start received")
                self._log("ASR start received")
                return True

    def _wait_future(self, future, timeout_s: float):
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and not future.done():
            self._check_abort()
            if time.monotonic() > deadline:
                return None
            time.sleep(0.02)
        return future.result() if future.done() else None

    def _navigate_or_skip(self, waypoint_id: str, label: str) -> bool:
        self._check_abort()
        if not self.navigate_client.wait_for_service(timeout_sec=2.0):
            return self._skip(f"navigate service unavailable: {label}")
        req = NavigateToTarget.Request()
        req.waypoint_id = waypoint_id
        self.get_logger().info(f"Navigate {label}: {waypoint_id}")
        self._log(f"Navigate {label}: {waypoint_id}")
        result = self._wait_future(self.navigate_client.call_async(req), self.navigate_timeout_s + 2.0)
        if result is not None and result.success:
            self.get_logger().info(f"Navigation succeeded: {waypoint_id}")
            self._log(f"Navigation succeeded: {waypoint_id}")
            return True
        message = result.message if result is not None else "timeout"
        return self._skip(f"navigation failed {waypoint_id}: {message}")

    def _detect_area_and_cargo_map(self) -> Dict[str, str]:
        for round_index in range(self.pre_detect_rounds):
            self._log(f"Pre-detect round {round_index + 1}")
            self._run_pre_detect_action(self.pre_detect_action_1)
            mapping = self._detect_cargo_zone_map_for_area("A")
            if self._mapping_is_complete(mapping):
                self.get_logger().info("Pre-detect action 1 succeeded, field side is A")
                self._log(f"Field side=A mapping={mapping}")
                return mapping

            self._run_pre_detect_action(self.pre_detect_action_2)
            mapping = self._detect_cargo_zone_map_for_area("B")
            if self._mapping_is_complete(mapping):
                self.get_logger().info("Pre-detect action 2 succeeded, field side is B")
                self._log(f"Field side=B mapping={mapping}")
                return mapping

        fallback = {"chilun": "zone_1", "luosi": "zone_2"}
        self._skip("sort sign recognition incomplete after all pre-detect rounds, fallback to field side A")
        self._log(f"Fallback mapping={fallback}")
        return fallback

    def _run_pre_detect_action(self, action_name: str) -> None:
        # 这里保留动作 hook，后续接入机械臂服务时只需要替换这一处
        self._check_abort()
        self.get_logger().info(f"Pre-detect action hook: {action_name}")
        self._log(f"Pre-detect action hook: {action_name}")

    def _detect_cargo_zone_map_for_area(self, detected_area: str) -> Dict[str, str]:
        self._check_abort()
        if not self.vision_client.wait_for_service(timeout_sec=2.0):
            self._skip("vision service unavailable")
            return {}
        start = VisionDetect.Request()
        start.start = True
        self._wait_future(self.vision_client.call_async(start), self.vision_service_timeout_s)
        end_time = time.monotonic() + self.vision_scan_duration_s
        while time.monotonic() < end_time:
            self._check_abort()
            time.sleep(0.05)
        stop = VisionDetect.Request()
        stop.start = False
        result = self._wait_future(self.vision_client.call_async(stop), self.vision_service_timeout_s)
        if result is None or not result.success or result.count <= 0:
            self._skip("vision result empty")
            return {}
        mapping: Dict[str, str] = {}
        observations = []
        for name, u_px in zip(result.cls_names, result.u_px):
            cargo = self._normalize_cargo_name(str(name).lower())
            if cargo is None:
                continue
            side = "left" if float(u_px) < self.image_mid_u else "right"
            zone = self._zone_from_camera_side(detected_area, side)
            mapping[cargo] = zone
            observations.append(f"{cargo}@{side}->{zone}")
        self.get_logger().info(f"Sort sign mapping: {observations}")
        self._log(f"Sort sign mapping: {observations}")
        if not mapping:
            self._skip("sort sign mapping empty")
        return mapping

    def _mapping_is_complete(self, mapping: Dict[str, str]) -> bool:
        return "chilun" in mapping and "luosi" in mapping

    def _normalize_cargo_name(self, class_name: str) -> Optional[str]:
        if class_name in self.gear_class_names:
            return "chilun"
        if class_name in self.bolt_class_names:
            return "luosi"
        return None

    def _zone_from_camera_side(self, detected_area: str, camera_side: str) -> str:
        if detected_area == "B":
            return "zone_2" if camera_side == "left" else "zone_1"
        return "zone_1" if camera_side == "left" else "zone_2"

    def _waypoint_for_zone(self, zone: str) -> str:
        return self.waypoint_zone_2 if zone == "zone_2" else self.waypoint_zone_1

    def _skip(self, reason: str) -> bool:
        self.get_logger().warn(f"Stage skipped: {reason}")
        self._log(f"Stage skipped: {reason}")
        try:
            self._speak(self.phrase_task_skipped)
        except Exception:
            pass
        return False if self.skip_failed_stage else (_ for _ in ()).throw(RuntimeError(reason))

    def _report_result(self, result: int, code: int) -> None:
        if not self.result_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("mission result service unavailable")
            return
        req = ReportMissionResult.Request()
        req.result = result
        req.code = code
        future = self.result_client.call_async(req)
        deadline = time.monotonic() + 2.0
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if future.done() and future.result() and future.result().success:
            self.get_logger().info("mission result reported")
            self._log("mission result reported")
        else:
            self.get_logger().warn("mission result report failed or timeout")
            self._log("mission result report failed or timeout")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AtlasAutonomousTask()
    try:
        rclpy.spin(node)
    finally:
        if node._log_file is not None:
            try:
                node._log_file.close()
            except Exception:
                pass
        node.asrpro.close()
        node.destroy_node()
        rclpy.shutdown()
