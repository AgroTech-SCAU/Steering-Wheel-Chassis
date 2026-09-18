#!/usr/bin/env python3
from __future__ import annotations

import math
import os
import statistics
import sys
import threading
import time
from pathlib import Path
from typing import Optional

import yaml

from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformException, TransformListener

from atlas_mission_interfaces.msg import NavigationStatus
from atlas_mission_interfaces.srv import StartNavigation

from navigation_calibration_model import (
    NavigationMapStatus,
    dump_clean_yaml,
    inspect_navigation_maps,
    merge_navigation_calibration,
)


def _parent_pid(pid: int) -> int:
    try:
        for line in Path(f"/proc/{pid}/status").read_text(encoding="utf-8").splitlines():
            if line.startswith("PPid:"):
                return int(line.split()[1])
    except (OSError, ValueError, IndexError):
        pass
    return 0


def _terminal_candidates():
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
    errors = []
    for terminal in _terminal_candidates():
        fd = -1
        try:
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
        "找不到可交互终端，请直接在树莓派桌面终端或 SSH 终端运行 ros2 launch"
        + (f" ({'; '.join(errors)})" if errors else "")
    )


def bind_terminal_output() -> None:
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


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-8:
        raise ValueError("TF 四元数无效")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def circular_mean(values: list[float]) -> float:
    return math.atan2(
        sum(math.sin(value) for value in values),
        sum(math.cos(value) for value in values),
    )


class NavigationCalibration(Node):
    def __init__(self) -> None:
        super().__init__("atlas_navigation_calibration")
        self.declare_parameter("competition_config", "")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("pose_sample_count", 20)
        self.declare_parameter("pose_sample_interval_s", 0.05)
        self.declare_parameter("localization_ready_timeout_s", 20.0)
        self.declare_parameter("navigation_timeout_s", 90.0)
        self.declare_parameter("navigation_start_timeout_s", 40.0)
        self.declare_parameter("origin_position_tolerance_m", 0.10)
        self.declare_parameter("origin_yaw_tolerance_rad", 0.15)

        self.config_path = Path(str(self.get_parameter("competition_config").value)).expanduser()
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.pose_sample_count = max(3, int(self.get_parameter("pose_sample_count").value))
        self.pose_sample_interval_s = max(
            0.02, float(self.get_parameter("pose_sample_interval_s").value)
        )
        self.localization_ready_timeout_s = max(
            2.0, float(self.get_parameter("localization_ready_timeout_s").value)
        )
        self.navigation_timeout_s = float(self.get_parameter("navigation_timeout_s").value)
        self.navigation_start_timeout_s = float(
            self.get_parameter("navigation_start_timeout_s").value
        )
        self.origin_position_tolerance_m = float(
            self.get_parameter("origin_position_tolerance_m").value
        )
        self.origin_yaw_tolerance_rad = float(
            self.get_parameter("origin_yaw_tolerance_rad").value
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._localization_started_ns = 0
        self._nav_cv = threading.Condition()
        self._latest_nav_status: Optional[NavigationStatus] = None
        self._nav_active = False
        self.motor_pub = self.create_publisher(Twist, "/motor_cmd_vel", 20)
        self.create_subscription(
            NavigationStatus, "/atlas/navigation/status", self._on_nav_status, 20
        )
        self.create_subscription(Twist, "/atlas/navigation/cmd_vel", self._on_nav_cmd, 20)
        self.nav_client = self.create_client(StartNavigation, "/atlas/navigation/start")
        self.brake_client = self.create_client(SetBool, "/mcu/set_brake")

    def _on_nav_status(self, msg: NavigationStatus) -> None:
        with self._nav_cv:
            self._latest_nav_status = msg
            self._nav_cv.notify_all()

    def _on_nav_cmd(self, msg: Twist) -> None:
        with self._nav_cv:
            active = self._nav_active
        if active:
            self.motor_pub.publish(msg)

    def _publish_zero(self) -> None:
        for _ in range(5):
            self.motor_pub.publish(Twist())
            time.sleep(0.02)

    def _call_brake(self, enabled: bool) -> None:
        if not self.brake_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("/mcu/set_brake 服务不可用，无法确认底盘刹车状态")
        request = SetBool.Request()
        request.data = bool(enabled)
        future = self.brake_client.call_async(request)
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and not future.done():
            time.sleep(0.02)
        if not future.done() or future.result() is None:
            raise RuntimeError("/mcu/set_brake 响应超时")
        if not future.result().success:
            raise RuntimeError(
                f"/mcu/set_brake {'启用' if enabled else '解除'}失败: "
                f"{future.result().message}"
            )

    def _align_and_return_origin(self, arena: str) -> None:
        print("\n[激光重定位与回地图原点]")
        answer = terminal_input(
            "确认机械臂已收拢、底盘周围安全，允许定位后自动回 (0,0,0) [y/N] > "
        ).strip().lower()
        if answer != "y":
            raise RuntimeError("用户取消自动回原点")
        if not self.nav_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("/atlas/navigation/start 服务不可用")

        request = StartNavigation.Request()
        request.backend = "direct_odom_competition"
        request.arena = arena
        request.waypoint_id = "origin"
        request.x_m = 0.0
        request.y_m = 0.0
        request.yaw_rad = 0.0
        request.reset_origin = False
        request.timeout_s = self.navigation_timeout_s

        with self._nav_cv:
            self._latest_nav_status = None
            self._nav_active = True
        self._localization_started_ns = self.get_clock().now().nanoseconds
        try:
            self._call_brake(False)
            future = self.nav_client.call_async(request)
            deadline = time.monotonic() + self.navigation_start_timeout_s
            while time.monotonic() < deadline and not future.done():
                time.sleep(0.02)
            if not future.done() or future.result() is None or not future.result().success:
                message = "原点对齐请求超时" if not future.done() else (
                    future.result().message if future.result() is not None else "原点对齐无响应"
                )
                raise RuntimeError(message)

            print("正在激光重定位并自动回到地图原点...")
            terminal_states = {
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
                    if (status is not None and status.waypoint_id == "origin"
                            and status.state in terminal_states):
                        result_state = int(status.state)
                        result_message = str(status.message)
                        break
                    self._nav_cv.wait(timeout=0.1)
            if result_state != NavigationStatus.STATE_SUCCEEDED:
                raise RuntimeError(result_message or "激光重定位或自动回原点失败/超时")
            print("已到达地图原点，开始导航点位标定")
        finally:
            with self._nav_cv:
                self._nav_active = False
            self._publish_zero()
            try:
                self._call_brake(True)
            except RuntimeError as exc:
                self.get_logger().error(f"回原点结束后刹车未确认: {exc}")

    @staticmethod
    def _print_map_statuses(statuses: dict[str, NavigationMapStatus]) -> None:
        print("\n地图资产检查")
        print("要求：导航组必须在中转区最优位姿和最优朝向开始建图；该位姿作为地图原点，")
        print("并作为后续 sorting_scan_a / sorting_scan_b 的底盘基准位姿")
        for arena in ("A", "B"):
            status = statuses[arena]
            if status.ready:
                print(
                    f"  {arena}: READY  map={status.map_path}  "
                    f"pbstream={status.pbstream_path}"
                )
            else:
                print(f"  {arena}: NOT READY")
                for reason in status.errors:
                    print(f"      - {reason}")
        print("地图未就绪时，请先把该半场 map YAML 和 pbstream 路径写入 competition.yaml")

    @staticmethod
    def _same_assets(
        first: NavigationMapStatus, second: NavigationMapStatus
    ) -> bool:
        if not first.ready or not second.ready:
            return False
        try:
            return (
                first.map_path.resolve() == second.map_path.resolve()
                and first.pbstream_path.resolve() == second.pbstream_path.resolve()
            )
        except OSError:
            return False

    @staticmethod
    def _select_arena(statuses: dict[str, NavigationMapStatus]) -> str:
        while True:
            value = terminal_input("请选择导航标定区域 [A/B] > ").strip().upper()
            if value not in {"A", "B"}:
                print("请输入 A 或 B")
                continue
            status = statuses[value]
            if not status.ready:
                print(f"{value} 区地图未就绪，不能开始标定")
                for reason in status.errors:
                    print(f"  - {reason}")
                continue
            return value

    def _wait_localization_ready(self) -> None:
        print("正在等待已验证的地图定位 TF...")
        deadline = time.monotonic() + self.localization_ready_timeout_s
        last_error = ""
        while time.monotonic() < deadline:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.map_frame, self.base_frame, Time()
                )
                if Time.from_msg(transform.header.stamp).nanoseconds < self._localization_started_ns:
                    time.sleep(0.1)
                    continue
                print(f"定位已就绪  TF: {self.map_frame} -> {self.base_frame}")
                return
            except TransformException as exc:
                last_error = str(exc)
            time.sleep(0.1)
        raise RuntimeError(f"等待 map TF 超时: {last_error}")

    def _sample_pose(self) -> dict[str, float | bool]:
        xs: list[float] = []
        ys: list[float] = []
        yaws: list[float] = []
        deadline = time.monotonic() + max(3.0, self.pose_sample_count * self.pose_sample_interval_s * 3.0)
        while len(xs) < self.pose_sample_count and time.monotonic() < deadline:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.map_frame, self.base_frame, Time()
                )
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                xs.append(float(translation.x))
                ys.append(float(translation.y))
                yaws.append(
                    quaternion_to_yaw(rotation.x, rotation.y, rotation.z, rotation.w)
                )
            except TransformException:
                pass
            time.sleep(self.pose_sample_interval_s)
        if len(xs) < max(3, self.pose_sample_count // 2):
            raise RuntimeError("有效 map 位姿样本不足，请检查激光定位是否稳定")
        return {
            "x": round(float(statistics.median(xs)), 6),
            "y": round(float(statistics.median(ys)), 6),
            "yaw": round(float(circular_mean(yaws)), 9),
            "configured": True,
        }

    def _show_origin_deviation(self) -> None:
        pose = self._sample_pose()
        x = float(pose["x"])
        y = float(pose["y"])
        yaw = float(pose["yaw"])
        distance = math.hypot(x, y)
        print("\n地图原点到位复核")
        print(
            f"当前相对地图原点偏差: dx={x:+.4f} m  dy={y:+.4f} m  "
            f"distance={distance:.4f} m  yaw={yaw:+.4f} rad "
            f"({math.degrees(yaw):+.2f} deg)"
        )
        if (distance > self.origin_position_tolerance_m
                or abs(yaw) > self.origin_yaw_tolerance_rad):
            raise RuntimeError(
                "底盘未到标定地图原点："
                f"位置偏差 {distance:.3f} m，朝向偏差 {yaw:.3f} rad；"
                "请检查地图定位和 direct navigation 状态"
            )
        print("接下来由用户遥控到货物区和两个放置区记录点位")

    def _capture_waypoint(self, waypoint: str, label: str) -> dict[str, float | bool]:
        while True:
            terminal_input(
                f"\n请遥控机器人到{label}，调整到最终执行任务时的底盘朝向，停车后按 Enter > "
            )
            pose = self._sample_pose()
            yaw_deg = math.degrees(float(pose["yaw"]))
            print(
                f"当前 {label} map 位姿: x={pose['x']:.6f} m  y={pose['y']:.6f} m  "
                f"yaw={pose['yaw']:.9f} rad ({yaw_deg:.2f} deg)"
            )
            answer = terminal_input(
                f"确认当前坐标和位姿为{label}点位 ({waypoint}) [y/N] > "
            ).strip().lower()
            if answer == "y":
                return pose
            print("未确认，继续遥控调整后重新记录")

    def _export(self, source: dict, arena: str, waypoints: dict) -> None:
        merged = merge_navigation_calibration(source, arena, waypoints)
        preview = dump_clean_yaml(merged)
        print("\n================ YAML 预览 ================")
        print(preview, end="")
        print("================ 预览结束 ================")
        if terminal_input("确认以上导航标定结果 [y/N] > ").strip().lower() != "y":
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
            return

    def run_interactive(self) -> None:
        if not self.config_path.is_file():
            raise FileNotFoundError(f"competition_config 不存在: {self.config_path}")
        source = yaml.safe_load(self.config_path.read_text(encoding="utf-8")) or {}

        print("\nAtlas 智械争锋导航点位标定")
        print("本流程只使用 Cartographer 纯定位记录坐标，不启动 Nav2 规划/控制器")
        statuses = inspect_navigation_maps(source, self.config_path)
        self._print_map_statuses(statuses)
        if not any(status.ready for status in statuses.values()):
            raise RuntimeError("A/B 均没有完整地图资产，无法开始导航标定")

        arena = self._select_arena(statuses)
        if arena == "B" and self._same_assets(statuses["A"], statuses["B"]):
            print("\n警告：A/B 当前指向同一组地图资产")
            print("如果这是临时占位配置，不要用 A 地图去标定 B 区")
            if terminal_input(f"确认这组地图可以用于 {arena} 区正式标定 [y/N] > ").strip().lower() != "y":
                raise RuntimeError("用户取消：请先更新所选区域地图路径")

        self._align_and_return_origin(arena)
        self._wait_localization_ready()
        self._show_origin_deviation()

        waypoints = {}
        for waypoint, label in (
            ("pickup", "货物区"),
            ("park_1", "园区一"),
            ("park_2", "园区二"),
        ):
            waypoints[waypoint] = self._capture_waypoint(waypoint, label)

        self._export(source, arena, waypoints)

    def cleanup(self) -> None:
        with self._nav_cv:
            self._nav_active = False
        if rclpy.ok():
            self._publish_zero()
            try:
                self._call_brake(True)
            except RuntimeError as exc:
                self.get_logger().error(f"标定退出时刹车未确认: {exc}")


def main(args=None) -> None:
    bind_terminal_output()
    rclpy.init(args=args)
    node = NavigationCalibration()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        node.run_interactive()
    except KeyboardInterrupt:
        print("\n导航标定已取消")
    except Exception as exc:  # noqa: BLE001
        print(f"\n导航标定失败: {exc}")
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
