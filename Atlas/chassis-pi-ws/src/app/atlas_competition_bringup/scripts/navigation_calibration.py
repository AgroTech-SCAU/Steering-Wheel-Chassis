#!/usr/bin/env python3
from __future__ import annotations

import math
import os
import signal
import statistics
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Optional

import yaml

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

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

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._localization_process: Optional[subprocess.Popen] = None

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

    def _start_localization(self, pbstream_path: Path) -> None:
        config_dir = Path(get_package_share_directory("at_nav2")) / "config"
        localization_lua = config_dir / "cartographer_localization.lua"
        if not localization_lua.is_file():
            raise FileNotFoundError(f"Cartographer 定位配置不存在: {localization_lua}")

        command = [
            "ros2", "run", "cartographer_ros", "cartographer_node",
            "-configuration_directory", str(config_dir),
            "-configuration_basename", "cartographer_localization.lua",
            "-load_state_filename", str(pbstream_path),
            "--ros-args",
            "-r", "scan:=/scan",
            "-r", "odom:=/odom",
        ]
        self._localization_process = subprocess.Popen(
            command,
            start_new_session=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        time.sleep(0.5)
        if self._localization_process.poll() is not None:
            self._localization_process = None
            raise RuntimeError("Cartographer 纯定位启动失败，请单独检查 /scan、/odom 和 TF")

    def _wait_localization_ready(self) -> None:
        print("正在等待 Cartographer 纯定位和 map TF...")
        deadline = time.monotonic() + self.localization_ready_timeout_s
        last_error = ""
        while time.monotonic() < deadline:
            if self._localization_process is not None and self._localization_process.poll() is not None:
                raise RuntimeError("Cartographer 纯定位进程提前退出")
            try:
                self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
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
        print("\n地图原点检查（仅显示，不自动移动底盘）")
        print(
            f"当前相对地图原点偏差: dx={x:+.4f} m  dy={y:+.4f} m  "
            f"distance={distance:.4f} m  yaw={yaw:+.4f} rad "
            f"({math.degrees(yaw):+.2f} deg)"
        )
        print("导航点位标定继续由用户遥控；这里不会自动回到 (0,0,0)")

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

        status = statuses[arena]
        self._start_localization(status.pbstream_path)
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
        proc = self._localization_process
        if proc is not None and proc.poll() is None:
            try:
                os.killpg(proc.pid, signal.SIGTERM)
                proc.wait(timeout=3.0)
            except Exception:  # noqa: BLE001
                try:
                    os.killpg(proc.pid, signal.SIGKILL)
                except Exception:  # noqa: BLE001
                    pass
        self._localization_process = None


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
