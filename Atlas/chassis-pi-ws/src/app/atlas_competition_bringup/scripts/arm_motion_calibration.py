#!/usr/bin/env python3
from __future__ import annotations

import math
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Optional

import yaml

from arm_motion_calibration_model import dump_clean_yaml, merge_calibration

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool

from atlas_mission_interfaces.msg import NavigationStatus
from atlas_mission_interfaces.srv import StartNavigation


def _parent_pid(pid: int) -> int:
    """Read an ancestor PID without depending on ps or a shell."""
    try:
        for line in Path(f"/proc/{pid}/status").read_text(encoding="utf-8").splitlines():
            if line.startswith("PPid:"):
                return int(line.split()[1])
    except (OSError, ValueError, IndexError):
        pass
    return 0


def _terminal_candidates():
    """Yield the terminal used by ros2 launch, even if child stdin is a pipe."""
    yield "/dev/tty"
    pid = os.getpid()
    seen = {"/dev/tty"}
    for _ in range(8):
        if pid <= 1:
            break
        for fd in (0, 1, 2):
            try:
                target = os.readlink(f"/proc/{pid}/fd/{fd}")
            except OSError:
                continue
            if target.startswith("/dev/") and target not in seen:
                seen.add(target)
                yield target
        pid = _parent_pid(pid)


def terminal_input(prompt: str) -> str:
    # ros2 launch does not reliably forward stdin to Node processes. Read and
    # write the launch command's real terminal directly, bypassing captured
    # stdout and the child's non-interactive stdin pipe.
    errors = []
    for terminal in _terminal_candidates():
        fd = -1
        try:
            # Do not use TextIO r+ here. Switching a TextIOWrapper from write
            # to read calls seek(), but Linux TTY/PTY streams are not seekable.
            fd = os.open(terminal, os.O_RDWR | os.O_NOCTTY)
            os.write(fd, f"\n{prompt.rstrip()} ".encode("utf-8"))
            data = os.read(fd, 4096)
            if not data:
                raise EOFError(f"{terminal} closed")
            return data.decode("utf-8", errors="replace").rstrip("\r\n")
        except (OSError, EOFError) as exc:
            errors.append(f"{terminal}: {exc}")
        finally:
            if fd >= 0:
                os.close(fd)
    raise RuntimeError(
        "找不到可交互终端，无法读取标定输入"
        "请直接在树莓派桌面终端或 SSH 终端运行 ros2 launch"
        + (f" ({'; '.join(errors)})" if errors else "")
    )


def bind_terminal_output() -> None:
    """Send calibration UI directly to the same TTY used for keyboard input."""
    for terminal in _terminal_candidates():
        fd = -1
        try:
            fd = os.open(terminal, os.O_WRONLY | os.O_NOCTTY)
            os.dup2(fd, sys.stdout.fileno())
            try:
                sys.stdout.reconfigure(line_buffering=True, write_through=True)
            except (AttributeError, OSError):
                pass
            return
        except OSError:
            continue
        finally:
            if fd >= 0 and fd != sys.stdout.fileno():
                os.close(fd)


def quaternion_to_pitch_yaw(x: float, y: float, z: float, w: float) -> tuple[float, float]:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-8:
        raise ValueError("机械臂末端四元数无效")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return pitch, yaw


class ArmMotionCalibration(Node):
    def __init__(self) -> None:
        super().__init__("atlas_arm_motion_calibration")
        self.declare_parameter("competition_config", "")
        self.declare_parameter("navigation_backend", "nav2_competition")
        self.declare_parameter("navigation_timeout_s", 90.0)
        self.declare_parameter("navigation_start_timeout_s", 40.0)
        self.declare_parameter("feedback_timeout_s", 2.0)
        self.declare_parameter("navigation_safe_joint_tolerance_rad", 0.08)
        self.declare_parameter("pose_speed_rad_s", 0.5)
        self.declare_parameter("no_preview", False)

        self.config_path = Path(str(self.get_parameter("competition_config").value)).expanduser()
        self.navigation_backend = str(self.get_parameter("navigation_backend").value)
        self.navigation_timeout_s = float(self.get_parameter("navigation_timeout_s").value)
        self.navigation_start_timeout_s = float(
            self.get_parameter("navigation_start_timeout_s").value
        )
        self.feedback_timeout_s = float(self.get_parameter("feedback_timeout_s").value)
        self.safe_joint_tolerance = float(
            self.get_parameter("navigation_safe_joint_tolerance_rad").value
        )
        self.pose_speed = float(self.get_parameter("pose_speed_rad_s").value)
        self.no_preview = bool(self.get_parameter("no_preview").value)

        self._lock = threading.Lock()
        self._nav_cv = threading.Condition(self._lock)
        self._latest_pose: Optional[PoseStamped] = None
        self._latest_pose_time = 0.0
        self._latest_joints: Optional[list[float]] = None
        self._latest_joints_time = 0.0
        self._latest_nav_status: Optional[NavigationStatus] = None
        self._nav_active = False
        self._camera_process: Optional[subprocess.Popen] = None

        self.create_subscription(PoseStamped, "/arm/pose", self._on_pose, 20)
        self.create_subscription(JointState, "/arm/joint_states", self._on_joints, 20)
        self.create_subscription(
            NavigationStatus, "/atlas/navigation/status", self._on_nav_status, 20
        )
        self.create_subscription(Twist, "/atlas/navigation/cmd_vel", self._on_nav_cmd, 20)
        self.motor_pub = self.create_publisher(Twist, "/motor_cmd_vel", 20)
        self.nav_client = self.create_client(StartNavigation, "/atlas/navigation/start")
        self.brake_client = self.create_client(SetBool, "/mcu/set_brake")

    def _on_pose(self, msg: PoseStamped) -> None:
        with self._lock:
            self._latest_pose = msg
            self._latest_pose_time = time.monotonic()

    def _on_joints(self, msg: JointState) -> None:
        values = [float(v) for v in list(msg.position)[:5]]
        if len(values) != 5:
            return
        with self._lock:
            self._latest_joints = values
            self._latest_joints_time = time.monotonic()

    def _on_nav_status(self, msg: NavigationStatus) -> None:
        with self._nav_cv:
            self._latest_nav_status = msg
            self._nav_cv.notify_all()

    def _on_nav_cmd(self, msg: Twist) -> None:
        with self._lock:
            active = self._nav_active
        if active:
            self.motor_pub.publish(msg)

    def _snapshot(self) -> tuple[list[float], PoseStamped]:
        deadline = time.monotonic() + self.feedback_timeout_s
        while time.monotonic() < deadline:
            with self._lock:
                pose = self._latest_pose
                joints = None if self._latest_joints is None else list(self._latest_joints)
                fresh = (
                    pose is not None
                    and joints is not None
                    and time.monotonic() - self._latest_pose_time <= self.feedback_timeout_s
                    and time.monotonic() - self._latest_joints_time <= self.feedback_timeout_s
                )
            if fresh:
                return joints, pose
            time.sleep(0.05)
        raise RuntimeError("未收到新鲜的 /arm/pose 和 /arm/joint_states")

    def _pose_dict(self) -> dict:
        joints, pose = self._snapshot()
        p = pose.pose.position
        q = pose.pose.orientation
        pitch, yaw = quaternion_to_pitch_yaw(q.x, q.y, q.z, q.w)
        return {
            "configured": True,
            "joints_rad": [round(v, 9) for v in joints],
            "x_m": round(float(p.x), 6),
            "y_m": round(float(p.y), 6),
            "z_m": round(float(p.z), 6),
            "pitch_rad": round(float(pitch), 9),
            "yaw_rad": round(float(yaw), 9),
            "speed_rad_s": self.pose_speed,
        }

    def _capture_pose(self, title: str, instruction: str) -> dict:
        while True:
            print(f"\n[{title}]")
            print(instruction)
            value = terminal_input("Enter=记录  R=重读  Q=退出 > ").strip().lower()
            if value == "q":
                raise KeyboardInterrupt
            if value not in {"", "r"}:
                print("请输入 Enter R 或 Q")
                continue
            try:
                result = self._pose_dict()
            except Exception as exc:  # noqa: BLE001
                print(f"读取失败: {exc}")
                continue
            print(
                "记录完成  joints="
                + str([round(v, 4) for v in result["joints_rad"]])
                + f"  xyz=({result['x_m']:.4f},{result['y_m']:.4f},{result['z_m']:.4f})"
            )
            if value == "r":
                continue
            return result

    def _capture_z(self, title: str) -> float:
        while True:
            print(f"\n[{title}]")
            terminal_input("将吸盘移动到该层实际吸取接触高度后按 Enter > ")
            try:
                _joints, pose = self._snapshot()
                z = round(float(pose.pose.position.z), 6)
                print(f"记录 Z={z:.6f} m")
                return z
            except Exception as exc:  # noqa: BLE001
                print(f"读取失败: {exc}")

    def _capture_reference(self, title: str) -> dict[str, float]:
        while True:
            print(f"\n[{title}]")
            terminal_input("将末端移动到 slot0 第一层标准释放位置后按 Enter > ")
            try:
                _joints, pose = self._snapshot()
                p = pose.pose.position
                result = {
                    "configured": True,
                    "x_m": round(float(p.x), 6),
                    "y_m": round(float(p.y), 6),
                    "first_layer_z_m": round(float(p.z), 6),
                }
                print(f"记录完成  {result}")
                return result
            except Exception as exc:  # noqa: BLE001
                print(f"读取失败: {exc}")

    def _joints_close(self, target: list[float]) -> tuple[bool, float]:
        joints, _pose = self._snapshot()
        error = max(abs(a - b) for a, b in zip(joints, target))
        return error <= self.safe_joint_tolerance, error

    def _require_recorded_pose(self, pose: dict, label: str) -> None:
        target = [float(v) for v in pose["joints_rad"]]
        while True:
            ok, error = self._joints_close(target)
            if ok:
                print(f"{label} 已确认  最大关节误差 {error:.4f} rad")
                return
            value = terminal_input(
                f"当前不在 {label}  最大关节误差 {error:.4f} rad\n"
                f"请拖动机械臂到 {label} 后按 Enter  Q=退出 > "
            ).strip().lower()
            if value == "q":
                raise KeyboardInterrupt

    def _require_navigation_safe(self, safe_pose: dict) -> None:
        self._require_recorded_pose(safe_pose, "navigation_safe")

    def _call_brake(self, enabled: bool) -> None:
        if not self.brake_client.wait_for_service(timeout_sec=0.5):
            return
        req = SetBool.Request()
        req.data = bool(enabled)
        future = self.brake_client.call_async(req)
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline and not future.done():
            time.sleep(0.02)

    def _publish_zero(self) -> None:
        zero = Twist()
        for _ in range(5):
            self.motor_pub.publish(zero)
            time.sleep(0.02)

    def _navigate(self, arena: str, waypoint: str, safe_pose: dict) -> None:
        value = terminal_input(f"\n是否导航到 {waypoint} [y/N] > ").strip().lower()
        if value != "y":
            raise RuntimeError(f"用户取消前往 {waypoint}")

        self._require_navigation_safe(safe_pose)
        if not self.nav_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("/atlas/navigation/start 服务不可用")

        request = StartNavigation.Request()
        request.backend = self.navigation_backend
        request.arena = arena
        request.waypoint_id = waypoint
        request.x_m = 0.0
        request.y_m = 0.0
        request.yaw_rad = 0.0
        request.reset_origin = False
        request.timeout_s = self.navigation_timeout_s

        with self._nav_cv:
            self._latest_nav_status = None
            self._nav_active = True
        self._call_brake(False)
        future = self.nav_client.call_async(request)
        # The first request may need to launch and activate the complete Nav2
        # stack (configured for up to 30 s). Do not report failure while that
        # same request can still complete and make the chassis move.
        deadline = time.monotonic() + self.navigation_start_timeout_s
        while time.monotonic() < deadline and not future.done():
            time.sleep(0.02)
        if not future.done() or future.result() is None or not future.result().success:
            with self._nav_cv:
                self._nav_active = False
            self._publish_zero()
            self._call_brake(True)
            message = "导航请求超时" if not future.done() else future.result().message
            raise RuntimeError(message)

        print(f"正在导航到 {waypoint}")
        terminal = {
            NavigationStatus.STATE_SUCCEEDED,
            NavigationStatus.STATE_FAILED,
            NavigationStatus.STATE_CANCELLED,
        }
        nav_deadline = time.monotonic() + self.navigation_timeout_s + 10.0
        result_state = None
        result_message = ""
        with self._nav_cv:
            while time.monotonic() < nav_deadline:
                status = self._latest_nav_status
                if status is not None and status.waypoint_id == waypoint and status.state in terminal:
                    result_state = int(status.state)
                    result_message = str(status.message)
                    break
                self._nav_cv.wait(timeout=0.1)
            self._nav_active = False

        self._publish_zero()
        self._call_brake(True)
        if result_state != NavigationStatus.STATE_SUCCEEDED:
            raise RuntimeError(result_message or f"导航到 {waypoint} 失败或超时")
        print(f"已到达 {waypoint}")

    def _start_camera(self) -> None:
        display = os.environ.get("DISPLAY", "")
        try:
            display_ready = bool(display) and subprocess.run(
                ["xset", "q"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=1.0,
                check=False,
            ).returncode == 0
        except (OSError, subprocess.SubprocessError):
            display_ready = False
        if not display_ready:
            print(
                "无法打开相机预览：当前终端没有可用的 X11 画面\n"
                "请在树莓派桌面终端运行，或使用 ssh -X 连接后再启动标定"
            )
            return
        try:
            self._camera_process = subprocess.Popen(
                [
                    "ros2", "run", "vison_topic", "vision_detect_server",
                    "--allow-unprepared", "--auto-start",
                ],
                start_new_session=True,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            time.sleep(0.5)
            if self._camera_process.poll() is not None:
                self._camera_process = None
                print("相机预览启动失败，请单独运行 vision_detect_server 检查相机")
                return
            print("已打开相机预览窗口 Vision Detection  按 Q 或 Esc 可关闭")
        except Exception as exc:  # noqa: BLE001
            print(f"相机启动失败: {exc}")

    def _stop_camera(self) -> None:
        proc = self._camera_process
        if proc is not None and proc.poll() is None:
            try:
                os.killpg(proc.pid, signal.SIGTERM)
                proc.wait(timeout=3.0)
            except Exception:  # noqa: BLE001
                try:
                    os.killpg(proc.pid, signal.SIGKILL)
                except Exception:
                    pass
        self._camera_process = None

    @staticmethod
    def _select_arena() -> str:
        while True:
            value = terminal_input("请选择标定区域 [A/B] > ").strip().upper()
            if value in {"A", "B"}:
                return value
            print("请输入 A 或 B")

    def run_interactive(self) -> None:
        if not self.config_path.is_file():
            raise FileNotFoundError(f"competition_config 不存在: {self.config_path}")
        source = yaml.safe_load(self.config_path.read_text(encoding="utf-8")) or {}

        print("\nAtlas 智械争锋机械臂运动标定")
        print("起始条件  机器人位于中转区  机械臂允许人工拖拽")
        arena = self._select_arena()
        if not self.no_preview:
            self._start_camera()

        fixed = {}
        fixed["zero"] = self._capture_pose("1/8 zero", "确认机械臂处于零位")
        scan_key = f"sorting_scan_{arena.lower()}"
        fixed[scan_key] = self._capture_pose(
            f"2/8 {scan_key}", f"拖动机械臂到 {arena} 区分拣观察位"
        )
        fixed["navigation_safe"] = self._capture_pose(
            "3/8 navigation_safe", "拖动机械臂到安全运输位"
        )

        self._navigate(arena, "pickup", fixed["navigation_safe"])
        pickup_observe = self._capture_pose(
            "4/8 pickup.observe", "拖动机械臂到货物区固定观察位"
        )
        layer_z = [
            self._capture_z(f"pickup 第 {layer} 层基准高度")
            for layer in (1, 2, 3)
        ]
        self._require_recorded_pose(pickup_observe, "pickup.observe")
        self._require_navigation_safe(fixed["navigation_safe"])

        self._navigate(arena, "park_1", fixed["navigation_safe"])
        park1_prepare = self._capture_pose(
            "5/8 park_1.prepare", "拖动机械臂到园区一预备位"
        )
        park1_reference = self._capture_reference("6/8 park_1.placement_reference")
        self._require_recorded_pose(park1_prepare, "park_1.prepare")
        self._require_navigation_safe(fixed["navigation_safe"])

        self._navigate(arena, "park_2", fixed["navigation_safe"])
        park2_prepare = self._capture_pose(
            "7/8 park_2.prepare", "拖动机械臂到园区二预备位"
        )
        park2_reference = self._capture_reference("8/8 park_2.placement_reference")
        self._require_recorded_pose(park2_prepare, "park_2.prepare")
        self._require_navigation_safe(fixed["navigation_safe"])

        calibration = {
            "fixed_poses": fixed,
            "pickup": {
                "observe": pickup_observe,
                "layer_z_configured": True,
                "layer_z_m": layer_z,
            },
            "park_1": {
                "prepare": park1_prepare,
                "placement_reference": park1_reference,
            },
            "park_2": {
                "prepare": park2_prepare,
                "placement_reference": park2_reference,
            },
        }
        merged = merge_calibration(source, arena, calibration)
        preview = dump_clean_yaml(merged)
        print("\n================ YAML 预览 ================")
        print(preview, end="")
        print("================ 预览结束 ================")
        if terminal_input("确认以上标定结果 [y/N] > ").strip().lower() != "y":
            print("用户取消导出")
            return

        while True:
            raw = terminal_input("新的 YAML 导出到哪里 > ").strip()
            if not raw:
                print("路径不能为空")
                continue
            output = Path(raw).expanduser()
            if output.exists():
                if terminal_input(f"{output} 已存在  是否覆盖 [y/N] > ").strip().lower() != "y":
                    continue
            output.parent.mkdir(parents=True, exist_ok=True)
            output.write_text(preview, encoding="utf-8")
            print(f"已导出 {output}")
            break

    def cleanup(self) -> None:
        with self._lock:
            self._nav_active = False
        if rclpy.ok():
            try:
                self._publish_zero()
                self._call_brake(True)
            except Exception:  # noqa: BLE001
                pass
        self._stop_camera()


def main(args=None) -> None:
    bind_terminal_output()
    rclpy.init(args=args)
    node = ArmMotionCalibration()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        node.run_interactive()
    except KeyboardInterrupt:
        print("\n标定已取消")
    except Exception as exc:  # noqa: BLE001
        print(f"\n标定失败: {exc}")
    finally:
        node.cleanup()
        try:
            executor.shutdown()
        except Exception:  # noqa: BLE001
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
