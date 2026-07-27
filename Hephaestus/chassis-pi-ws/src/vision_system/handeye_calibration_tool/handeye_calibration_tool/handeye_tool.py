#!/usr/bin/env python3
"""用于 UVC 相机与 ROS 2 机械臂的交互式手眼标定工具"""

from __future__ import annotations

import copy
import math
import os
import shutil
import sys
import threading
import time
from collections import deque
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple


def _configure_qt_font_directory() -> None:
    """为视觉库窗口指定系统中可用的字体目录"""
    if os.environ.get("QT_QPA_FONTDIR"):
        return
    candidates = (
        "/usr/share/fonts/truetype/wqy",
        "/usr/share/fonts/truetype/dejavu",
        "/usr/share/fonts/truetype",
        "/usr/share/fonts",
    )
    for candidate in candidates:
        if Path(candidate).is_dir():
            os.environ["QT_QPA_FONTDIR"] = candidate
            return


_configure_qt_font_directory()

import cv2
import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

from mcu_comm_bridge.srv import SetArmJoints, SetArmPose


# ----------------------------- 数学工具 -----------------------------


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """返回按照偏航，俯仰，横滚顺序组合的旋转矩阵"""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def matrix_to_rpy(rotation: np.ndarray) -> Tuple[float, float, float]:
    """将旋转矩阵转换为横滚，俯仰，偏航"""
    sy = math.sqrt(rotation[0, 0] ** 2 + rotation[1, 0] ** 2)
    singular = sy < 1e-9
    if not singular:
        roll = math.atan2(rotation[2, 1], rotation[2, 2])
        pitch = math.atan2(-rotation[2, 0], sy)
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    else:
        roll = math.atan2(-rotation[1, 2], rotation[1, 1])
        pitch = math.atan2(-rotation[2, 0], sy)
        yaw = 0.0
    return roll, pitch, yaw


def quaternion_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        raise ValueError("zero-length quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def matrix_to_quaternion(rotation: np.ndarray) -> Tuple[float, float, float, float]:
    """返回四元数，顺序为 x，y，z，w"""
    trace = float(np.trace(rotation))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (rotation[2, 1] - rotation[1, 2]) / s
        y = (rotation[0, 2] - rotation[2, 0]) / s
        z = (rotation[1, 0] - rotation[0, 1]) / s
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        s = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
        w = (rotation[2, 1] - rotation[1, 2]) / s
        x = 0.25 * s
        y = (rotation[0, 1] + rotation[1, 0]) / s
        z = (rotation[0, 2] + rotation[2, 0]) / s
    elif rotation[1, 1] > rotation[2, 2]:
        s = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
        w = (rotation[0, 2] - rotation[2, 0]) / s
        x = (rotation[0, 1] + rotation[1, 0]) / s
        y = 0.25 * s
        z = (rotation[1, 2] + rotation[2, 1]) / s
    else:
        s = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
        w = (rotation[1, 0] - rotation[0, 1]) / s
        x = (rotation[0, 2] + rotation[2, 0]) / s
        y = (rotation[1, 2] + rotation[2, 1]) / s
        z = 0.25 * s

    q = np.array([x, y, z, w], dtype=np.float64)
    q /= np.linalg.norm(q)
    return tuple(float(v) for v in q)


def make_transform(rotation: np.ndarray, translation: Sequence[float]) -> np.ndarray:
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    transform[:3, 3] = np.asarray(translation, dtype=np.float64).reshape(3)
    return transform


def invert_transform(transform: np.ndarray) -> np.ndarray:
    rotation = transform[:3, :3]
    translation = transform[:3, 3]
    inverse = np.eye(4, dtype=np.float64)
    inverse[:3, :3] = rotation.T
    inverse[:3, 3] = -rotation.T @ translation
    return inverse


def rotation_angle_deg(rotation: np.ndarray) -> float:
    value = (float(np.trace(rotation)) - 1.0) * 0.5
    return math.degrees(math.acos(max(-1.0, min(1.0, value))))


def mean_rotation(rotations: Sequence[np.ndarray]) -> np.ndarray:
    summed = np.sum(np.asarray(rotations), axis=0)
    u, _, vt = np.linalg.svd(summed)
    correction = np.eye(3)
    correction[2, 2] = np.linalg.det(u @ vt)
    return u @ correction @ vt


def matrix_to_list(transform: np.ndarray) -> List[List[float]]:
    return [[float(value) for value in row] for row in transform.tolist()]


def parse_float_list(text: str, expected: int) -> List[float]:
    values = [float(item.strip()) for item in text.replace(";", ",").split(",") if item.strip()]
    if len(values) != expected:
        raise ValueError(f"需要 {expected} 个数，实际得到 {len(values)} 个")
    return values


class InteractiveConsole:
    """从控制终端读取命令，即使在启动文件下运行也可用

    ROS 2 launch captures a node's standard streams. ``emulate_tty=True`` only
    makes the output look like a terminal and does not reliably forward keyboard
    input to Python's ``input()``. When standard input is not a TTY, this class
    opens ``/dev/tty`` so menu commands still come from the terminal that started
    the launch process.
    """

    def __init__(self) -> None:
        self._stream = None
        self._owns_stream = False
        self.source = "unavailable"

        if sys.stdin is not None and sys.stdin.isatty():
            self._stream = sys.stdin
            self.source = "stdin"
            return

        try:
            self._stream = open(
                "/dev/tty",
                mode="r",
                encoding="utf-8",
                errors="replace",
                buffering=1,
            )
            self._owns_stream = True
            self.source = "/dev/tty"
        except OSError:
            # 保留最后的回退方式，便于重定向输入时直接运行
            self._stream = sys.stdin
            self.source = "stdin (non-tty)"

    def input(self, prompt: str = "") -> str:
        if self._stream is None:
            raise EOFError("没有可用的交互式终端输入")
        print(prompt, end="", flush=True)
        line = self._stream.readline()
        if line == "":
            raise EOFError("终端输入已关闭")
        return line.rstrip("\r\n")

    def close(self) -> None:
        if self._owns_stream and self._stream is not None:
            try:
                self._stream.close()
            except OSError:
                pass
        self._stream = None


class MenuCancelled(Exception):
    """用户取消配置向导时抛出"""


# ------------------------------- 数据模型 -------------------------------


@dataclass
class AppConfig:
    camera_index: int = 0
    camera_backend: str = "v4l2"
    camera_width: int = 1280
    camera_height: int = 720
    camera_fps: float = 30.0
    camera_fourcc: str = "MJPG"
    camera_window_name: str = "Hand-Eye Calibration Camera"
    intrinsics_file: str = ""

    # 标定板类型：普通棋盘格表示普通黑白棋盘格，编码棋盘格表示带编码标定板
    board_type: str = "chessboard"

    # 普通棋盘格参数这里填写的是内角点数量，不是方格数量
    # 例如 10 x 7 个黑白方格，对应 9 x 6 个内角点
    chessboard_inner_corners_x: int = 9
    chessboard_inner_corners_y: int = 6
    chessboard_square_length_m: float = 0.025
    chessboard_use_sb: bool = True

    # 编码棋盘格参数表示完整方格数量
    charuco_dictionary: str = "DICT_4X4_50"
    charuco_squares_x: int = 7
    charuco_squares_y: int = 5
    charuco_square_length_m: float = 0.030
    charuco_marker_length_m: float = 0.022
    min_charuco_corners: int = 8

    max_reprojection_error_px: float = 1.5

    handeye_mode: str = "eye_in_hand"
    base_frame: str = "arm_base_link"
    gripper_frame: str = "tool0"
    camera_frame: str = "camera_optical_frame"
    target_frame: str = "handeye_target"

    arm_pose_source: str = "auto"
    arm_pose_topic: str = "/arm/pose"
    arm_joint_state_topic: str = "/arm/joint_states"
    arm_fk_position_topic: str = "/arm/pose_position"
    orientation_rpy_joint_coeffs: List[float] = None
    orientation_rpy_offset_rad: List[float] = None

    max_robot_state_age_s: float = 0.5
    robot_sync_tolerance_s: float = 0.20
    require_stable_before_capture: bool = True
    stable_duration_s: float = 0.5
    stable_joint_range_rad: float = 0.008
    stable_wait_timeout_s: float = 8.0

    arm_joints_service: str = "/mcu/set_arm_joints"
    arm_pose_service: str = "/mcu/set_arm_pose"
    default_arm_speed_rad_s: float = 0.8
    output_directory: str = "~/handeye_calibration"

    def __post_init__(self) -> None:
        if self.orientation_rpy_joint_coeffs is None:
            self.orientation_rpy_joint_coeffs = [
                0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 1.0, 1.0, 1.0,
                1.0, 0.0, 0.0, 0.0, 0.0,
            ]
        if self.orientation_rpy_offset_rad is None:
            self.orientation_rpy_offset_rad = [0.0, 0.0, 0.0]


@dataclass
class CameraIntrinsics:
    matrix: np.ndarray
    distortion: np.ndarray
    source: str


@dataclass
class DetectionResult:
    timestamp_monotonic: float
    success: bool
    corner_count: int
    reprojection_error_px: Optional[float]
    target_to_camera: Optional[np.ndarray]
    raw_frame: np.ndarray
    message: str
    pattern_detected: bool = False
    expected_corner_count: int = 0
    board_type: str = ""


@dataclass
class RobotStateSnapshot:
    timestamp_monotonic: float
    source: str
    base_to_gripper: np.ndarray
    joints_rad: Optional[List[float]]
    frame_id: str


# ------------------------------ 机械臂节点 ------------------------------


class ArmStateNode(Node):
    """采集机械臂状态，并提供微控制器机械臂控制服务"""

    def __init__(self, config: AppConfig) -> None:
        super().__init__("handeye_calibration_tool")
        self._config = config
        self._lock = threading.RLock()
        self._latest_pose: Optional[Tuple[float, np.ndarray, str]] = None
        self._latest_position: Optional[Tuple[float, np.ndarray, str]] = None
        self._latest_joints: Optional[Tuple[float, np.ndarray]] = None
        self._joint_history: deque[Tuple[float, np.ndarray]] = deque(maxlen=1000)

        self.create_subscription(PoseStamped, config.arm_pose_topic, self._pose_callback, 20)
        self.create_subscription(PointStamped, config.arm_fk_position_topic, self._position_callback, 20)
        self.create_subscription(JointState, config.arm_joint_state_topic, self._joint_callback, 50)

        self._joints_client = self.create_client(SetArmJoints, config.arm_joints_service)
        self._pose_client = self.create_client(SetArmPose, config.arm_pose_service)

    def _pose_callback(self, message: PoseStamped) -> None:
        try:
            rotation = quaternion_to_matrix(
                message.pose.orientation.x,
                message.pose.orientation.y,
                message.pose.orientation.z,
                message.pose.orientation.w,
            )
        except ValueError:
            return
        transform = make_transform(
            rotation,
            [message.pose.position.x, message.pose.position.y, message.pose.position.z],
        )
        with self._lock:
            self._latest_pose = (time.monotonic(), transform, message.header.frame_id)

    def _position_callback(self, message: PointStamped) -> None:
        position = np.array([message.point.x, message.point.y, message.point.z], dtype=np.float64)
        with self._lock:
            self._latest_position = (time.monotonic(), position, message.header.frame_id)

    def _joint_callback(self, message: JointState) -> None:
        if len(message.position) < 5:
            return

        name_to_position = {name: value for name, value in zip(message.name, message.position)}
        expected_names = ["q0", "q1", "q2", "q3", "q4"]
        if all(name in name_to_position for name in expected_names):
            joints = np.array([name_to_position[name] for name in expected_names], dtype=np.float64)
        else:
            joints = np.asarray(message.position[:5], dtype=np.float64)

        now = time.monotonic()
        with self._lock:
            self._latest_joints = (now, joints)
            self._joint_history.append((now, joints.copy()))

    def get_robot_state(self, config: AppConfig) -> Tuple[Optional[RobotStateSnapshot], str]:
        now = time.monotonic()
        with self._lock:
            pose = copy.deepcopy(self._latest_pose)
            position = copy.deepcopy(self._latest_position)
            joints = copy.deepcopy(self._latest_joints)

        if config.arm_pose_source in ("auto", "pose_topic") and pose is not None:
            pose_time, transform, frame_id = pose
            age = now - pose_time
            if age <= config.max_robot_state_age_s:
                joint_values = None if joints is None else joints[1].tolist()
                return RobotStateSnapshot(
                    timestamp_monotonic=pose_time,
                    source="pose_topic",
                    base_to_gripper=transform,
                    joints_rad=joint_values,
                    frame_id=frame_id,
                ), ""
            if config.arm_pose_source == "pose_topic":
                return None, f"末端位姿话题数据过期：{age:.3f}s"

        if config.arm_pose_source not in ("auto", "joint_fk"):
            return None, f"未知 arm_pose_source: {config.arm_pose_source}"
        if position is None:
            return None, f"尚未收到 {config.arm_fk_position_topic}"
        if joints is None:
            return None, f"尚未收到 {config.arm_joint_state_topic}"

        position_time, xyz, frame_id = position
        joints_time, joint_values = joints
        position_age = now - position_time
        joints_age = now - joints_time
        if position_age > config.max_robot_state_age_s:
            return None, f"末端位置数据过期：{position_age:.3f}s"
        if joints_age > config.max_robot_state_age_s:
            return None, f"关节数据过期：{joints_age:.3f}s"
        if abs(position_time - joints_time) > config.robot_sync_tolerance_s:
            return None, (
                "末端位置与关节状态时间差过大："
                f"{abs(position_time - joints_time):.3f}s > {config.robot_sync_tolerance_s:.3f}s"
            )

        coefficients = np.asarray(config.orientation_rpy_joint_coeffs, dtype=np.float64).reshape(3, 5)
        offset = np.asarray(config.orientation_rpy_offset_rad, dtype=np.float64).reshape(3)
        roll, pitch, yaw = coefficients @ joint_values + offset
        transform = make_transform(rpy_to_matrix(float(roll), float(pitch), float(yaw)), xyz)
        return RobotStateSnapshot(
            timestamp_monotonic=min(position_time, joints_time),
            source="joint_fk",
            base_to_gripper=transform,
            joints_rad=joint_values.tolist(),
            frame_id=frame_id,
        ), ""

    def wait_until_stable(self, config: AppConfig) -> Tuple[bool, str]:
        if not config.require_stable_before_capture:
            return True, "稳定性检查已关闭"

        deadline = time.monotonic() + config.stable_wait_timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            now = time.monotonic()
            with self._lock:
                recent_samples = [
                    (sample_time, joints.copy())
                    for sample_time, joints in self._joint_history
                    if now - sample_time <= config.stable_duration_s
                ]
                latest = copy.deepcopy(self._latest_joints)

            if latest is None:
                time.sleep(0.05)
                continue
            if now - latest[0] > config.max_robot_state_age_s:
                time.sleep(0.05)
                continue
            if len(recent_samples) >= 3:
                stacked = np.vstack([sample[1] for sample in recent_samples])
                joint_range = np.max(stacked, axis=0) - np.min(stacked, axis=0)
                recent_span = recent_samples[-1][0] - recent_samples[0][0]
                history_span_ok = recent_span >= config.stable_duration_s * 0.8
                if history_span_ok and float(np.max(joint_range)) <= config.stable_joint_range_rad:
                    return True, f"机械臂稳定，最大关节波动 {float(np.max(joint_range)):.6f} rad"
            time.sleep(0.05)

        return False, f"等待机械臂稳定超时（{config.stable_wait_timeout_s:.1f}s）"

    @staticmethod
    def _wait_future(future: Any, timeout_s: float) -> Tuple[bool, Any]:
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            if future.done():
                try:
                    return True, future.result()
                except Exception as exc:  # noqa: BLE001
                    return False, str(exc)
            time.sleep(0.02)
        return False, "service call timeout"

    def command_joints(self, joints_rad: Sequence[float], speed_rad_s: float) -> Tuple[bool, str]:
        if len(joints_rad) != 5:
            return False, "关节目标必须包含 5 个值"
        if not self._joints_client.wait_for_service(timeout_sec=2.0):
            return False, f"服务不可用：{self._config.arm_joints_service}"
        request = SetArmJoints.Request()
        request.joints_rad = [float(value) for value in joints_rad]
        request.speed_rad_s = float(speed_rad_s)
        ok, response = self._wait_future(self._joints_client.call_async(request), 5.0)
        if not ok:
            return False, str(response)
        return bool(response.success), f"{response.message}; command_seq={response.command_seq}"

    def command_pose(
        self,
        x_m: float,
        y_m: float,
        z_m: float,
        pitch_rad: float,
        yaw_rad: float,
        speed_rad_s: float,
    ) -> Tuple[bool, str]:
        if not self._pose_client.wait_for_service(timeout_sec=2.0):
            return False, f"服务不可用：{self._config.arm_pose_service}"
        request = SetArmPose.Request()
        request.x_m = float(x_m)
        request.y_m = float(y_m)
        request.z_m = float(z_m)
        request.pitch_rad = float(pitch_rad)
        request.yaw_rad = float(yaw_rad)
        request.speed_rad_s = float(speed_rad_s)
        ok, response = self._wait_future(self._pose_client.call_async(request), 5.0)
        if not ok:
            return False, str(response)
        return bool(response.success), f"{response.message}; command_seq={response.command_seq}"


# ------------------------------- 相机界面 -------------------------------


class CameraWorker:
    def __init__(
        self,
        config_getter: Callable[[], AppConfig],
        intrinsics_getter: Callable[[], Optional[CameraIntrinsics]],
        capture_callback: Callable[[], None],
    ) -> None:
        self._config_getter = config_getter
        self._intrinsics_getter = intrinsics_getter
        self._capture_callback = capture_callback
        self._stop_event = threading.Event()
        self._restart_event = threading.Event()
        self._lock = threading.RLock()
        self._latest_detection: Optional[DetectionResult] = None
        self._camera_open = False
        self._status = "camera not started"
        self._thread = threading.Thread(target=self._run, name="handeye_camera", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        self._thread.join(timeout=3.0)

    def restart(self) -> None:
        self._restart_event.set()

    def status(self) -> Tuple[bool, str]:
        with self._lock:
            return self._camera_open, self._status

    def latest_detection(self) -> Optional[DetectionResult]:
        with self._lock:
            return copy.deepcopy(self._latest_detection)

    def is_alive(self) -> bool:
        return self._thread.is_alive()

    @staticmethod
    def _create_dictionary_and_board(config: AppConfig) -> Tuple[Any, Any]:
        """需要时创建编码棋盘格对象，普通棋盘格不需要该对象"""
        board_type = config.board_type.strip().lower()
        if board_type == "chessboard":
            return None, None
        if board_type != "charuco":
            raise ValueError(f"未知标定板类型：{config.board_type}，应为 chessboard 或 charuco")
        if not hasattr(cv2, "aruco"):
            raise RuntimeError("当前 OpenCV 未包含 aruco 模块，请安装带 contrib 的 OpenCV")
        dictionary_id = getattr(cv2.aruco, config.charuco_dictionary, None)
        if dictionary_id is None:
            raise ValueError(f"未知 ArUco 字典：{config.charuco_dictionary}")
        dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)

        if hasattr(cv2.aruco, "CharucoBoard"):
            try:
                board = cv2.aruco.CharucoBoard(
                    (config.charuco_squares_x, config.charuco_squares_y),
                    config.charuco_square_length_m,
                    config.charuco_marker_length_m,
                    dictionary,
                )
                return dictionary, board
            except Exception:  # noqa: BLE001
                pass
        board = cv2.aruco.CharucoBoard_create(
            config.charuco_squares_x,
            config.charuco_squares_y,
            config.charuco_square_length_m,
            config.charuco_marker_length_m,
            dictionary,
        )
        return dictionary, board

    @staticmethod
    def _board_corners(board: Any) -> np.ndarray:
        if hasattr(board, "getChessboardCorners"):
            return np.asarray(board.getChessboardCorners(), dtype=np.float64)
        return np.asarray(board.chessboardCorners, dtype=np.float64)

    @staticmethod
    def _solve_board_pose(
        object_points: np.ndarray,
        image_points: np.ndarray,
        intrinsics: CameraIntrinsics,
        axis_length_m: float,
        annotated: np.ndarray,
    ) -> Tuple[Optional[np.ndarray], Optional[float], str, bool]:
        ok, rvec, tvec = cv2.solvePnP(
            np.asarray(object_points, dtype=np.float64).reshape(-1, 1, 3),
            np.asarray(image_points, dtype=np.float64).reshape(-1, 1, 2),
            intrinsics.matrix,
            intrinsics.distortion,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not ok:
            return None, None, "PnP failed", False

        projected, _ = cv2.projectPoints(
            np.asarray(object_points, dtype=np.float64).reshape(-1, 1, 3),
            rvec,
            tvec,
            intrinsics.matrix,
            intrinsics.distortion,
        )
        residual = projected.reshape(-1, 2) - np.asarray(image_points).reshape(-1, 2)
        reprojection_error = float(np.sqrt(np.mean(np.sum(residual * residual, axis=1))))
        rotation, _ = cv2.Rodrigues(rvec)
        target_to_camera = make_transform(rotation, tvec.reshape(3))
        cv2.drawFrameAxes(
            annotated,
            intrinsics.matrix,
            intrinsics.distortion,
            rvec,
            tvec,
            max(float(axis_length_m), 1e-4),
        )
        return target_to_camera, reprojection_error, "PnP OK", True

    def _detect_chessboard(
        self,
        frame: np.ndarray,
        config: AppConfig,
        intrinsics: Optional[CameraIntrinsics],
    ) -> Tuple[DetectionResult, np.ndarray]:
        annotated = frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        pattern_size = (
            int(config.chessboard_inner_corners_x),
            int(config.chessboard_inner_corners_y),
        )
        expected = pattern_size[0] * pattern_size[1]
        found = False
        corners = None

        if config.chessboard_use_sb and hasattr(cv2, "findChessboardCornersSB"):
            flags = 0
            flags |= getattr(cv2, "CALIB_CB_NORMALIZE_IMAGE", 0)
            flags |= getattr(cv2, "CALIB_CB_EXHAUSTIVE", 0)
            flags |= getattr(cv2, "CALIB_CB_ACCURACY", 0)
            found, corners = cv2.findChessboardCornersSB(gray, pattern_size, flags=flags)
        else:
            flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
            found, corners = cv2.findChessboardCorners(gray, pattern_size, flags=flags)
            if found and corners is not None:
                criteria = (
                    cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                    40,
                    1e-4,
                )
                corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

        corner_count = 0 if corners is None else int(len(corners))
        target_to_camera = None
        reprojection_error = None
        success = False

        if not found or corners is None:
            message = (
                f"chessboard not found; expected {pattern_size[0]}x{pattern_size[1]} inner corners"
            )
            pattern_detected = False
        else:
            pattern_detected = True
            cv2.drawChessboardCorners(annotated, pattern_size, corners, True)
            if intrinsics is None:
                message = "corners FOUND; load camera intrinsics in menu 4 for PnP"
            else:
                object_points = np.zeros((expected, 3), dtype=np.float64)
                grid = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
                object_points[:, :2] = grid * float(config.chessboard_square_length_m)
                target_to_camera, reprojection_error, _, pnp_ok = self._solve_board_pose(
                    object_points,
                    corners,
                    intrinsics,
                    config.chessboard_square_length_m * 2.0,
                    annotated,
                )
                if not pnp_ok or reprojection_error is None:
                    message = "corners FOUND, but PnP failed"
                else:
                    success = reprojection_error <= config.max_reprojection_error_px
                    message = (
                        f"VALID chessboard pose, reproj={reprojection_error:.3f}px"
                        if success
                        else f"corners FOUND, reprojection too high: {reprojection_error:.3f}px"
                    )

        return DetectionResult(
            timestamp_monotonic=time.monotonic(),
            success=success,
            corner_count=corner_count,
            reprojection_error_px=reprojection_error,
            target_to_camera=target_to_camera,
            raw_frame=frame.copy(),
            message=message,
            pattern_detected=pattern_detected,
            expected_corner_count=expected,
            board_type="chessboard",
        ), annotated

    def _detect_charuco(
        self,
        frame: np.ndarray,
        config: AppConfig,
        dictionary: Any,
        board: Any,
        intrinsics: Optional[CameraIntrinsics],
    ) -> Tuple[DetectionResult, np.ndarray]:
        annotated = frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 兼容不同版本视觉库的角点检测接口
        if hasattr(cv2.aruco, "CharucoDetector"):
            detector = cv2.aruco.CharucoDetector(board)
            charuco_corners, charuco_ids, marker_corners, marker_ids = detector.detectBoard(gray)
        else:
            marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(gray, dictionary)
            if marker_ids is not None and len(marker_ids) > 0:
                interpolation = cv2.aruco.interpolateCornersCharuco(
                    marker_corners,
                    marker_ids,
                    gray,
                    board,
                    cameraMatrix=None if intrinsics is None else intrinsics.matrix,
                    distCoeffs=None if intrinsics is None else intrinsics.distortion,
                )
                if interpolation is not None and len(interpolation) >= 3:
                    _, charuco_corners, charuco_ids = interpolation
                else:
                    charuco_corners, charuco_ids = None, None
            else:
                charuco_corners, charuco_ids = None, None

        corner_count = 0
        target_to_camera = None
        reprojection_error = None
        message = "ChArUco board not detected"
        success = False
        pattern_detected = False

        if marker_ids is not None and len(marker_ids) > 0:
            cv2.aruco.drawDetectedMarkers(annotated, marker_corners, marker_ids)

        if charuco_ids is not None and charuco_corners is not None:
            pattern_detected = True
            corner_count = int(len(charuco_ids))
            cv2.aruco.drawDetectedCornersCharuco(annotated, charuco_corners, charuco_ids)
            if corner_count < config.min_charuco_corners:
                message = f"only {corner_count} ChArUco corners"
            elif intrinsics is None:
                message = "corners FOUND; load camera intrinsics in menu 4 for PnP"
            else:
                all_object_points = self._board_corners(board)
                indices = charuco_ids.reshape(-1).astype(np.int64)
                object_points = all_object_points[indices].reshape(-1, 3)
                image_points = np.asarray(charuco_corners, dtype=np.float64).reshape(-1, 2)
                target_to_camera, reprojection_error, _, pnp_ok = self._solve_board_pose(
                    object_points,
                    image_points,
                    intrinsics,
                    config.charuco_square_length_m * 2.0,
                    annotated,
                )
                if not pnp_ok or reprojection_error is None:
                    message = "ChArUco corners FOUND, but PnP failed"
                else:
                    success = reprojection_error <= config.max_reprojection_error_px
                    message = (
                        f"VALID ChArUco pose, reproj={reprojection_error:.3f}px"
                        if success
                        else f"corners FOUND, reprojection too high: {reprojection_error:.3f}px"
                    )
        expected = max(0, (config.charuco_squares_x - 1) * (config.charuco_squares_y - 1))
        return DetectionResult(
            timestamp_monotonic=time.monotonic(),
            success=success,
            corner_count=corner_count,
            reprojection_error_px=reprojection_error,
            target_to_camera=target_to_camera,
            raw_frame=frame.copy(),
            message=message,
            pattern_detected=pattern_detected,
            expected_corner_count=expected,
            board_type="charuco",
        ), annotated

    def _detect(
        self,
        frame: np.ndarray,
        config: AppConfig,
        dictionary: Any,
        board: Any,
        intrinsics: Optional[CameraIntrinsics],
    ) -> Tuple[DetectionResult, np.ndarray]:
        board_type = config.board_type.strip().lower()
        if board_type == "chessboard":
            return self._detect_chessboard(frame, config, intrinsics)
        if board_type == "charuco":
            return self._detect_charuco(frame, config, dictionary, board, intrinsics)
        raise ValueError(f"unknown board_type: {config.board_type}")

    @staticmethod
    def _open_capture(config: AppConfig) -> cv2.VideoCapture:
        backend = cv2.CAP_ANY
        if config.camera_backend.lower() == "v4l2" and hasattr(cv2, "CAP_V4L2"):
            backend = cv2.CAP_V4L2
        capture = cv2.VideoCapture(config.camera_index, backend)
        if config.camera_fourcc and len(config.camera_fourcc) == 4:
            capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*config.camera_fourcc))
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, config.camera_width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, config.camera_height)
        capture.set(cv2.CAP_PROP_FPS, config.camera_fps)
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return capture

    @staticmethod
    def _draw_overlay(
        image: np.ndarray,
        lines: Sequence[Tuple[str, Tuple[int, int, int]]],
    ) -> None:
        y = 26
        for line, color in lines:
            cv2.putText(
                image,
                line,
                (12, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.58,
                color,
                2,
                cv2.LINE_AA,
            )
            y += 24

    def _run(self) -> None:
        capture: Optional[cv2.VideoCapture] = None
        window_name = "Hand-Eye Calibration Camera"
        dictionary = None
        board = None

        while not self._stop_event.is_set():
            config = self._config_getter()
            window_name = config.camera_window_name
            try:
                dictionary, board = self._create_dictionary_and_board(config)
                capture = self._open_capture(config)
                if not capture.isOpened():
                    raise RuntimeError(f"cannot open camera index {config.camera_index}")
                with self._lock:
                    self._camera_open = True
                    self._status = (
                        f"opened index={config.camera_index}, "
                        f"{int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))}x"
                        f"{int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))}"
                    )
                gui_enabled = True
                try:
                    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
                except cv2.error as exc:
                    gui_enabled = False
                    with self._lock:
                        self._status = f"camera opened without GUI window: {exc}"

                while not self._stop_event.is_set() and not self._restart_event.is_set():
                    ok, frame = capture.read()
                    if not ok or frame is None:
                        with self._lock:
                            self._status = "camera read failed"
                        time.sleep(0.05)
                        continue

                    config = self._config_getter()
                    intrinsics = self._intrinsics_getter()
                    try:
                        result, annotated = self._detect(frame, config, dictionary, board, intrinsics)
                    except Exception as exc:  # noqa: BLE001
                        result = DetectionResult(
                            timestamp_monotonic=time.monotonic(),
                            success=False,
                            corner_count=0,
                            reprojection_error_px=None,
                            target_to_camera=None,
                            raw_frame=frame.copy(),
                            message=f"detection error: {exc}",
                        )
                        annotated = frame.copy()

                    with self._lock:
                        self._latest_detection = result
                        self._status = result.message

                    intrinsics_text = "intrinsics: LOADED" if intrinsics is not None else "intrinsics: MISSING (menu 4)"
                    intrinsics_color = (0, 255, 0) if intrinsics is not None else (0, 165, 255)
                    detect_text = (
                        f"pattern: FOUND {result.corner_count}/{result.expected_corner_count}"
                        if result.pattern_detected
                        else f"pattern: NOT FOUND 0/{result.expected_corner_count}"
                    )
                    detect_color = (0, 255, 0) if result.pattern_detected else (0, 0, 255)
                    pose_text = "pose: VALID" if result.success else f"pose: NOT READY | {result.message}"
                    pose_color = (0, 255, 0) if result.success else (0, 165, 255)
                    board_desc = (
                        f"chessboard {config.chessboard_inner_corners_x}x{config.chessboard_inner_corners_y} INNER corners"
                        if config.board_type == "chessboard"
                        else f"charuco {config.charuco_squares_x}x{config.charuco_squares_y} squares"
                    )
                    self._draw_overlay(
                        annotated,
                        [
                            (f"board: {board_desc}", (255, 255, 255)),
                            (detect_text, detect_color),
                            (intrinsics_text, intrinsics_color),
                            (pose_text, pose_color),
                            ("C: capture | Q: close tool", (255, 255, 255)),
                        ],
                    )
                    if gui_enabled:
                        try:
                            cv2.imshow(window_name, annotated)
                            key = cv2.waitKey(1) & 0xFF
                        except cv2.error:
                            gui_enabled = False
                            key = -1
                    else:
                        key = -1
                    if key in (ord("c"), ord("C")):
                        threading.Thread(target=self._capture_callback, daemon=True).start()
                    elif key in (ord("q"), ord("Q"), 27):
                        self._stop_event.set()
                        break
            except Exception as exc:  # noqa: BLE001
                with self._lock:
                    self._camera_open = False
                    self._status = str(exc)
                time.sleep(1.0)
            finally:
                if capture is not None:
                    capture.release()
                    capture = None
                try:
                    cv2.destroyWindow(window_name)
                except cv2.error:
                    pass
                with self._lock:
                    self._camera_open = False
                self._restart_event.clear()

        try:
            cv2.destroyAllWindows()
        except cv2.error:
            pass


# ---------------------------- 标定应用 ----------------------------


class HandEyeApplication:
    def __init__(self, node: ArmStateNode, config: AppConfig) -> None:
        self.node = node
        self._config = config
        self._config_lock = threading.RLock()
        self._intrinsics_lock = threading.RLock()
        self._intrinsics: Optional[CameraIntrinsics] = None
        self._samples_lock = threading.RLock()
        self._capture_lock = threading.Lock()
        self.console = InteractiveConsole()
        self.samples: List[Dict[str, Any]] = []
        self.session_dir = self._create_session_dir()
        self.images_dir = self.session_dir / "images"
        self.images_dir.mkdir(parents=True, exist_ok=True)
        self.camera = CameraWorker(self.get_config, self.get_intrinsics, self.capture_sample)

        if config.intrinsics_file:
            try:
                self.load_intrinsics(config.intrinsics_file)
            except Exception as exc:  # noqa: BLE001
                print(f"[WARN] 自动加载相机内参失败：{exc}")

    def _create_session_dir(self) -> Path:
        base = Path(os.path.expanduser(self._config.output_directory)).resolve()
        session = base / datetime.now().strftime("%Y%m%d_%H%M%S")
        session.mkdir(parents=True, exist_ok=True)
        return session

    def get_config(self) -> AppConfig:
        with self._config_lock:
            return copy.deepcopy(self._config)

    def update_config(self, **kwargs: Any) -> None:
        with self._config_lock:
            for key, value in kwargs.items():
                if not hasattr(self._config, key):
                    raise AttributeError(key)
                setattr(self._config, key, value)

    def get_intrinsics(self) -> Optional[CameraIntrinsics]:
        with self._intrinsics_lock:
            return copy.deepcopy(self._intrinsics)

    def set_intrinsics(self, intrinsics: CameraIntrinsics) -> None:
        with self._intrinsics_lock:
            self._intrinsics = intrinsics

    @staticmethod
    def _load_intrinsics_opencv(path: str) -> Optional[CameraIntrinsics]:
        try:
            storage = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
        except Exception:  # ROS camera_info YAML is not OpenCV FileStorage format
            return None
        if not storage.isOpened():
            return None
        try:
            matrix = None
            distortion = None
            for name in ("camera_matrix", "K", "cameraMatrix"):
                value = storage.getNode(name).mat()
                if value is not None and value.size == 9:
                    matrix = np.asarray(value, dtype=np.float64).reshape(3, 3)
                    break
            for name in ("distortion_coefficients", "D", "dist_coeffs", "distCoeffs"):
                value = storage.getNode(name).mat()
                if value is not None and value.size >= 4:
                    distortion = np.asarray(value, dtype=np.float64).reshape(-1, 1)
                    break
            if matrix is not None and distortion is not None:
                return CameraIntrinsics(matrix=matrix, distortion=distortion, source=path)
        finally:
            storage.release()
        return None

    @staticmethod
    def _extract_yaml_data(value: Any) -> Any:
        if isinstance(value, dict) and "data" in value:
            return value["data"]
        return value

    @classmethod
    def _load_intrinsics_yaml(cls, path: str) -> Optional[CameraIntrinsics]:
        text = Path(path).read_text(encoding="utf-8")
        text = text.replace("%YAML:1.0", "")
        text = text.replace("!!opencv-matrix", "")
        data = yaml.safe_load(text)
        if not isinstance(data, dict):
            return None

        matrix_value = None
        distortion_value = None
        for name in ("camera_matrix", "K", "cameraMatrix"):
            if name in data:
                matrix_value = cls._extract_yaml_data(data[name])
                break
        for name in ("distortion_coefficients", "D", "dist_coeffs", "distCoeffs"):
            if name in data:
                distortion_value = cls._extract_yaml_data(data[name])
                break
        if matrix_value is None or distortion_value is None:
            return None
        matrix = np.asarray(matrix_value, dtype=np.float64).reshape(3, 3)
        distortion = np.asarray(distortion_value, dtype=np.float64).reshape(-1, 1)
        return CameraIntrinsics(matrix=matrix, distortion=distortion, source=path)

    def load_intrinsics(self, path: str) -> CameraIntrinsics:
        expanded = str(Path(os.path.expanduser(path)).resolve())
        if not Path(expanded).is_file():
            raise FileNotFoundError(expanded)
        intrinsics = self._load_intrinsics_opencv(expanded)
        if intrinsics is None:
            intrinsics = self._load_intrinsics_yaml(expanded)
        if intrinsics is None:
            raise ValueError("未找到 camera_matrix/K 和 distortion_coefficients/D")
        self.set_intrinsics(intrinsics)
        self.update_config(intrinsics_file=expanded)
        return intrinsics

    def set_manual_intrinsics(self, matrix_values: Sequence[float], distortion: Sequence[float]) -> None:
        matrix = np.asarray(matrix_values, dtype=np.float64).reshape(3, 3)
        distortion_array = np.asarray(distortion, dtype=np.float64).reshape(-1, 1)
        path = self.session_dir / "camera_intrinsics.yaml"
        payload = {
            "camera_matrix": {
                "rows": 3,
                "cols": 3,
                "data": matrix.reshape(-1).tolist(),
            },
            "distortion_coefficients": {
                "rows": 1,
                "cols": int(distortion_array.size),
                "data": distortion_array.reshape(-1).tolist(),
            },
        }
        path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")
        intrinsics = CameraIntrinsics(
            matrix=matrix,
            distortion=distortion_array,
            source=str(path),
        )
        self.set_intrinsics(intrinsics)
        self.update_config(intrinsics_file=str(path))

    def save_runtime_config(self) -> Path:
        path = self.session_dir / "config.yaml"
        config_dict = asdict(self.get_config())
        intrinsics = self.get_intrinsics()
        if intrinsics is not None:
            config_dict["resolved_camera_matrix"] = intrinsics.matrix.reshape(-1).tolist()
            config_dict["resolved_distortion_coefficients"] = intrinsics.distortion.reshape(-1).tolist()
            config_dict["resolved_intrinsics_source"] = intrinsics.source
        payload = {"handeye_calibration_tool": {"ros__parameters": config_dict}}
        path.write_text(yaml.safe_dump(payload, sort_keys=False, allow_unicode=True), encoding="utf-8")
        return path

    def _save_samples(self) -> Path:
        path = self.session_dir / "samples.yaml"
        temporary = path.with_suffix(".yaml.tmp")
        payload = {
            "session": str(self.session_dir),
            "handeye_mode": self.get_config().handeye_mode,
            "sample_count": len(self.samples),
            "samples": self.samples,
        }
        temporary.write_text(yaml.safe_dump(payload, sort_keys=False, allow_unicode=True), encoding="utf-8")
        temporary.replace(path)
        return path

    def capture_sample(self) -> None:
        if not self._capture_lock.acquire(blocking=False):
            print("\n[WARN] 已有采样任务正在执行")
            return
        try:
            config = self.get_config()
            print("\n[采样] 等待机械臂稳定...")
            stable, stable_message = self.node.wait_until_stable(config)
            if not stable:
                print(f"[采样失败] {stable_message}")
                return

            detection = self.camera.latest_detection()
            if detection is None:
                print("[采样失败] 尚未收到相机图像")
                return
            detection_age = time.monotonic() - detection.timestamp_monotonic
            if detection_age > 0.5:
                print(f"[采样失败] 视觉检测结果过期：{detection_age:.3f}s")
                return
            if not detection.success or detection.target_to_camera is None:
                print(f"[采样失败] {detection.message}")
                return

            robot_state, robot_error = self.node.get_robot_state(config)
            if robot_state is None:
                print(f"[采样失败] {robot_error}")
                return

            with self._samples_lock:
                sample_id = max((int(item["id"]) for item in self.samples), default=0) + 1
                image_name = f"sample_{sample_id:04d}.png"
                image_path = self.images_dir / image_name
                if not cv2.imwrite(str(image_path), detection.raw_frame):
                    print(f"[采样失败] 无法保存图像：{image_path}")
                    return

                record = {
                    "id": sample_id,
                    "captured_at": datetime.now().isoformat(timespec="milliseconds"),
                    "robot_pose_source": robot_state.source,
                    "robot_frame_id": robot_state.frame_id,
                    "joints_rad": robot_state.joints_rad,
                    "base_to_gripper": matrix_to_list(robot_state.base_to_gripper),
                    "target_to_camera": matrix_to_list(detection.target_to_camera),
                    "board_type": detection.board_type or config.board_type,
                    "board_corner_count": detection.corner_count,
                    # 保留该字段以兼容已有样本读取器
                    "charuco_corner_count": detection.corner_count,
                    "reprojection_error_px": float(detection.reprojection_error_px),
                    "image": str(Path("images") / image_name),
                    "stable_message": stable_message,
                }
                self.samples.append(record)
                samples_path = self._save_samples()
                self.save_runtime_config()

            print(
                f"[采样成功] #{sample_id}, corners={detection.corner_count}, "
                f"reproj={detection.reprojection_error_px:.3f}px"
            )
            print(f"[数据文件] {samples_path}")
        finally:
            self._capture_lock.release()

    def delete_sample(self, sample_id: int) -> bool:
        with self._samples_lock:
            for index, sample in enumerate(self.samples):
                if int(sample["id"]) == sample_id:
                    image_path = self.session_dir / sample["image"]
                    if image_path.exists():
                        image_path.unlink()
                    self.samples.pop(index)
                    self._save_samples()
                    return True
        return False

    def clear_samples(self) -> None:
        with self._samples_lock:
            self.samples.clear()
            if self.images_dir.exists():
                shutil.rmtree(self.images_dir)
            self.images_dir.mkdir(parents=True, exist_ok=True)
            self._save_samples()

    @staticmethod
    def _consistency_metrics(
        samples: Sequence[Dict[str, Any]],
        handeye_mode: str,
        result_transform: np.ndarray,
    ) -> Dict[str, float]:
        constants = []
        for sample in samples:
            base_to_gripper = np.asarray(sample["base_to_gripper"], dtype=np.float64)
            target_to_camera = np.asarray(sample["target_to_camera"], dtype=np.float64)
            if handeye_mode == "eye_in_hand":
                # 基座到目标 = 基座到末端 * 末端到相机 * 相机到目标
                constant_transform = base_to_gripper @ result_transform @ target_to_camera
            else:
                # 末端到目标 = 末端到基座 * 基座到相机 * 相机到目标
                constant_transform = invert_transform(base_to_gripper) @ result_transform @ target_to_camera
            constants.append(constant_transform)

        translations = np.asarray([value[:3, 3] for value in constants])
        mean_translation = np.mean(translations, axis=0)
        translation_errors = np.linalg.norm(translations - mean_translation, axis=1)
        mean_rot = mean_rotation([value[:3, :3] for value in constants])
        rotation_errors = np.asarray(
            [rotation_angle_deg(mean_rot.T @ value[:3, :3]) for value in constants], dtype=np.float64
        )
        return {
            "translation_rms_m": float(np.sqrt(np.mean(translation_errors ** 2))),
            "translation_max_m": float(np.max(translation_errors)),
            "rotation_rms_deg": float(np.sqrt(np.mean(rotation_errors ** 2))),
            "rotation_max_deg": float(np.max(rotation_errors)),
        }

    def solve(self) -> Optional[Dict[str, Any]]:
        with self._samples_lock:
            samples = copy.deepcopy(self.samples)
        if len(samples) < 3:
            print("[求解失败] OpenCV 理论最低需要 3 组姿态；建议采集 15～25 组")
            return None
        if len(samples) < 10:
            print(f"[警告] 当前只有 {len(samples)} 组样本，结果可能不稳定，建议至少 15 组")

        config = self.get_config()
        robot_rotations: List[np.ndarray] = []
        robot_translations: List[np.ndarray] = []
        target_rotations: List[np.ndarray] = []
        target_translations: List[np.ndarray] = []

        for sample in samples:
            base_to_gripper = np.asarray(sample["base_to_gripper"], dtype=np.float64)
            target_to_camera = np.asarray(sample["target_to_camera"], dtype=np.float64)
            robot_input = base_to_gripper if config.handeye_mode == "eye_in_hand" else invert_transform(base_to_gripper)
            robot_rotations.append(robot_input[:3, :3])
            robot_translations.append(robot_input[:3, 3].reshape(3, 1))
            target_rotations.append(target_to_camera[:3, :3])
            target_translations.append(target_to_camera[:3, 3].reshape(3, 1))

        methods = {
            "TSAI": cv2.CALIB_HAND_EYE_TSAI,
            "PARK": cv2.CALIB_HAND_EYE_PARK,
            "HORAUD": cv2.CALIB_HAND_EYE_HORAUD,
            "ANDREFF": cv2.CALIB_HAND_EYE_ANDREFF,
            "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS,
        }
        results = []
        for name, method in methods.items():
            try:
                rotation, translation = cv2.calibrateHandEye(
                    robot_rotations,
                    robot_translations,
                    target_rotations,
                    target_translations,
                    method=method,
                )
                transform = make_transform(rotation, translation.reshape(3))
                if not np.all(np.isfinite(transform)):
                    raise ValueError("result contains NaN/Inf")
                metrics = self._consistency_metrics(samples, config.handeye_mode, transform)
                normalized_score = (
                    metrics["translation_rms_m"] / 0.005
                    + metrics["rotation_rms_deg"] / 1.0
                )
                results.append(
                    {
                        "method": name,
                        "transform": transform,
                        "metrics": metrics,
                        "score": float(normalized_score),
                    }
                )
            except Exception as exc:  # noqa: BLE001
                print(f"[求解器 {name}] 失败：{exc}")

        if not results:
            print("[求解失败] 所有算法均失败，请检查坐标方向与姿态激励")
            return None

        results.sort(key=lambda item: item["score"])
        print("\n========== 手眼标定求解结果 ==========")
        for result in results:
            metrics = result["metrics"]
            print(
                f"{result['method']:<10} "
                f"trans_rms={metrics['translation_rms_m'] * 1000.0:8.3f} mm, "
                f"rot_rms={metrics['rotation_rms_deg']:7.3f} deg, "
                f"trans_max={metrics['translation_max_m'] * 1000.0:8.3f} mm, "
                f"rot_max={metrics['rotation_max_deg']:7.3f} deg"
            )

        best = results[0]
        transform = best["transform"]
        x, y, z, w = matrix_to_quaternion(transform[:3, :3])
        roll, pitch, yaw = matrix_to_rpy(transform[:3, :3])
        parent_frame = config.gripper_frame if config.handeye_mode == "eye_in_hand" else config.base_frame
        child_frame = config.camera_frame
        result_name = "gripper_to_camera" if config.handeye_mode == "eye_in_hand" else "base_to_camera"

        payload = {
            "created_at": datetime.now().isoformat(timespec="seconds"),
            "handeye_mode": config.handeye_mode,
            "sample_count": len(samples),
            "selected_method": best["method"],
            "result_name": result_name,
            "parent_frame": parent_frame,
            "child_frame": child_frame,
            "transform_matrix": matrix_to_list(transform),
            "translation_m": transform[:3, 3].tolist(),
            "quaternion_xyzw": [x, y, z, w],
            "rpy_rad": [roll, pitch, yaw],
            "metrics": best["metrics"],
            "all_methods": [
                {
                    "method": item["method"],
                    "transform_matrix": matrix_to_list(item["transform"]),
                    "metrics": item["metrics"],
                    "score": item["score"],
                }
                for item in results
            ],
        }
        result_path = self.session_dir / "handeye_result.yaml"
        result_path.write_text(yaml.safe_dump(payload, sort_keys=False, allow_unicode=True), encoding="utf-8")

        translation = transform[:3, 3]
        print(f"\n[推荐结果] {best['method']} ({result_name})")
        print(np.array2string(transform, precision=9, suppress_small=True))
        print(
            "translation [m]: "
            f"x={translation[0]:.9f}, y={translation[1]:.9f}, z={translation[2]:.9f}"
        )
        print(f"quaternion xyzw: [{x:.9f}, {y:.9f}, {z:.9f}, {w:.9f}]")
        print(f"rpy [deg]: [{math.degrees(roll):.4f}, {math.degrees(pitch):.4f}, {math.degrees(yaw):.4f}]")
        print(f"[结果文件] {result_path}")
        print("[静态 TF 命令]")
        print(
            "ros2 run tf2_ros static_transform_publisher "
            f"--x {translation[0]:.9f} --y {translation[1]:.9f} --z {translation[2]:.9f} "
            f"--qx {x:.9f} --qy {y:.9f} --qz {z:.9f} --qw {w:.9f} "
            f"--frame-id {parent_frame} --child-frame-id {child_frame}"
        )
        return payload

    # ------------------------------ 终端界面 ------------------------------

    def _input(self, prompt: str = "") -> str:
        return self.console.input(prompt)

    def _ask(self, prompt: str, current: Any, converter: Callable[[str], Any] = str) -> Any:
        print(f"\n{prompt}")
        print(f"当前值：{current}；直接按 Enter 保留当前值")
        text = self._input(">>> ").strip()
        if not text:
            return current
        return converter(text)

    def _ask_bool(self, prompt: str, current: bool) -> bool:
        default = "y" if current else "n"
        print(f"\n{prompt}")
        print(f"请输入 y 或 n；当前值：{default}；直接按 Enter 保留")
        text = self._input(">>> ").strip().lower()
        if not text:
            return current
        if text in ("y", "yes", "1", "true", "是"):
            return True
        if text in ("n", "no", "0", "false", "否"):
            return False
        raise ValueError("请输入 y 或 n")

    def _guided_value(
        self,
        title: str,
        current: Any,
        converter: Callable[[str], Any],
        explanation: str,
        example: str,
    ) -> Any:
        print(f"\n{title}")
        print(f"说明：{explanation}")
        print(f"当前值：{current}")
        print(f"示例输入：{example}")
        print("直接按 Enter 保留当前值；输入 q 返回上一级")
        text = self._input(">>> ").strip()
        if text.lower() in ("q", "quit", "cancel", "取消"):
            raise MenuCancelled
        if not text:
            return current
        return converter(text)

    def _guided_choice(
        self,
        title: str,
        choices: Dict[str, str],
        current_key: Optional[str] = None,
    ) -> str:
        print(f"\n{title}")
        for key, description in choices.items():
            suffix = "  [当前]" if current_key == key else ""
            print(f"  {key}. {description}{suffix}")
        print("请输入编号并按 Enter；输入 q 返回上一级")
        while True:
            text = self._input(">>> ").strip().lower()
            if text in ("q", "quit", "cancel", "取消"):
                raise MenuCancelled
            if not text and current_key is not None:
                return current_key
            if text in choices:
                return text
            print(f"[输入错误] 只能输入：{', '.join(choices.keys())}")

    def print_status(self) -> None:
        config = self.get_config()
        camera_open, camera_status = self.camera.status()
        intrinsics = self.get_intrinsics()
        robot_state, robot_error = self.node.get_robot_state(config)
        detection = self.camera.latest_detection()

        print("\n================ 当前状态 ================")
        print(f"会话目录       : {self.session_dir}")
        print(f"手眼模式       : {config.handeye_mode}")
        print(f"相机           : {'已打开' if camera_open else '未打开'} | {camera_status}")
        print(f"相机内参       : {intrinsics.source if intrinsics else '未加载'}")
        if config.board_type == "chessboard":
            print(
                "标定板         : 普通黑白棋盘格，"
                f"内角点={config.chessboard_inner_corners_x}x"
                f"{config.chessboard_inner_corners_y}, "
                f"方格边长={config.chessboard_square_length_m:.6f}m"
            )
        else:
            print(
                "标定板         : ChArUco，"
                f"方格={config.charuco_squares_x}x{config.charuco_squares_y}, "
                f"方格边长={config.charuco_square_length_m:.6f}m, "
                f"Marker边长={config.charuco_marker_length_m:.6f}m"
            )
        print(f"机械臂位姿来源 : {config.arm_pose_source}")
        if robot_state is None:
            print(f"机械臂状态     : 无效 | {robot_error}")
        else:
            xyz = robot_state.base_to_gripper[:3, 3]
            rpy = matrix_to_rpy(robot_state.base_to_gripper[:3, :3])
            print(
                f"机械臂状态     : 有效 ({robot_state.source}), "
                f"xyz=[{xyz[0]:.4f}, {xyz[1]:.4f}, {xyz[2]:.4f}] m, "
                f"rpy=[{math.degrees(rpy[0]):.2f}, {math.degrees(rpy[1]):.2f}, "
                f"{math.degrees(rpy[2]):.2f}] deg"
            )
        if detection is None:
            print("图案识别       : 尚无相机结果")
            print("PnP 位姿       : 尚无相机结果")
        else:
            pattern_text = "已识别" if detection.pattern_detected else "未识别"
            print(
                f"图案识别       : {pattern_text}, "
                f"角点={detection.corner_count}/{detection.expected_corner_count}"
            )
            print(
                f"PnP 位姿       : {'有效' if detection.success else '未就绪'}, "
                f"{detection.message}"
            )
        print(f"已采集样本     : {len(self.samples)}")
        print("==========================================")

    def configure_camera(self) -> None:
        config = self.get_config()
        try:
            camera_index = self._ask("camera index", config.camera_index, int)
            backend = self._ask("camera backend (v4l2/any)", config.camera_backend, str).lower()
            width = self._ask("width", config.camera_width, int)
            height = self._ask("height", config.camera_height, int)
            fps = self._ask("fps", config.camera_fps, float)
            fourcc = self._ask("FOURCC (例如 MJPG，留空沿用)", config.camera_fourcc, str).upper()
            self.update_config(
                camera_index=camera_index,
                camera_backend=backend,
                camera_width=width,
                camera_height=height,
                camera_fps=fps,
                camera_fourcc=fourcc,
            )
            self.camera.restart()
            print("[OK] 相机参数已更新，正在重新打开")
        except (ValueError, TypeError) as exc:
            print(f"[输入错误] {exc}")

    def configure_board(self) -> None:
        config = self.get_config()
        print("\n========== 配置标定板 ==========")
        print("你的标定板如果只有黑白方格、没有 ArUco 编码图案，应选择 1")
        print("注意：棋盘格尺寸里的横向/纵向数量是‘内角点数’，不是黑白方格数")
        print("例如 10×7 个黑白方格，对应 9×6 个内角点")

        current_choice = "1" if config.board_type == "chessboard" else "2"
        try:
            choice = self._guided_choice(
                "请选择标定板类型",
                {
                    "1": "普通黑白棋盘格（无 ArUco 编码）",
                    "2": "ChArUco 标定板（棋盘格中带 ArUco 编码）",
                    "0": "返回主菜单，不修改",
                },
                current_choice,
            )
            if choice == "0":
                print("已返回，未修改标定板参数")
                return

            if choice == "1":
                corners_x = self._guided_value(
                    "[1/4] 横向内角点数量",
                    config.chessboard_inner_corners_x,
                    int,
                    "沿棋盘格水平方向数内部交点，不计算外边框若横向有 10 格，则输入 9",
                    "9",
                )
                corners_y = self._guided_value(
                    "[2/4] 纵向内角点数量",
                    config.chessboard_inner_corners_y,
                    int,
                    "沿棋盘格竖直方向数内部交点若纵向有 7 格，则输入 6",
                    "6",
                )
                square_length = self._guided_value(
                    "[3/4] 单个黑白方格的实际边长，单位 m",
                    config.chessboard_square_length_m,
                    float,
                    "用尺或游标卡尺测量相邻两个内角点之间的实际距离25 mm 应输入 0.025",
                    "0.025",
                )
                max_error = self._guided_value(
                    "[4/4] 最大 PnP 重投影误差，单位像素",
                    config.max_reprojection_error_px,
                    float,
                    "误差超过该值时不允许采样初次建议 1.5，排查时可临时设为 2.5",
                    "1.5",
                )
                if corners_x < 3 or corners_y < 3:
                    raise ValueError("横向和纵向内角点数量都必须至少为 3")
                if square_length <= 0:
                    raise ValueError("方格边长必须大于 0")
                if max_error <= 0:
                    raise ValueError("重投影误差阈值必须大于 0")

                self.update_config(
                    board_type="chessboard",
                    chessboard_inner_corners_x=corners_x,
                    chessboard_inner_corners_y=corners_y,
                    chessboard_square_length_m=square_length,
                    max_reprojection_error_px=max_error,
                )
                self.camera.restart()
                print("\n[OK] 已切换为普通黑白棋盘格检测")
                print(
                    f"请将完整棋盘格放入画面，窗口应显示 pattern: FOUND "
                    f"{corners_x * corners_y}/{corners_x * corners_y}"
                )
                print("若角点已找到但 pose 显示 NOT READY，请在菜单 4 加载相机内参")
                print("警告：普通棋盘格存在 180° 对称性，采样时不要让角点编号方向发生翻转")
                return

            dictionary_map = {
                "1": "DICT_4X4_50",
                "2": "DICT_5X5_100",
                "3": "DICT_6X6_250",
                "4": "DICT_7X7_250",
                "5": "自定义",
            }
            current_dictionary_choice = next(
                (key for key, value in dictionary_map.items() if value == config.charuco_dictionary),
                "5",
            )
            dictionary_choice = self._guided_choice(
                "[1/7] 选择 ArUco 字典",
                dictionary_map,
                current_dictionary_choice,
            )
            dictionary = dictionary_map[dictionary_choice]
            if dictionary == "自定义":
                dictionary = self._guided_value(
                    "输入 OpenCV ArUco 字典名称",
                    config.charuco_dictionary,
                    str,
                    "必须是 OpenCV 支持的常量名称",
                    "DICT_4X4_50",
                )
            squares_x = self._guided_value(
                "[2/7] 横向方格数 squares_x",
                config.charuco_squares_x,
                int,
                "这里数的是完整方格数量，不是内角点",
                "7",
            )
            squares_y = self._guided_value(
                "[3/7] 纵向方格数 squares_y",
                config.charuco_squares_y,
                int,
                "这里数的是完整方格数量，不是内角点",
                "5",
            )
            square_length = self._guided_value(
                "[4/7] 单个方格边长，单位 m",
                config.charuco_square_length_m,
                float,
                "例如 30 mm 输入 0.030",
                "0.030",
            )
            marker_length = self._guided_value(
                "[5/7] ArUco 黑色编码区域边长，单位 m",
                config.charuco_marker_length_m,
                float,
                "必须小于方格边长，例如 22 mm 输入 0.022",
                "0.022",
            )
            min_corners = self._guided_value(
                "[6/7] 最少 ChArUco 角点数",
                config.min_charuco_corners,
                int,
                "低于该数量不进行 PnP，建议至少 8",
                "8",
            )
            max_error = self._guided_value(
                "[7/7] 最大 PnP 重投影误差，单位像素",
                config.max_reprojection_error_px,
                float,
                "初次建议 1.5",
                "1.5",
            )
            if squares_x < 3 or squares_y < 3:
                raise ValueError("ChArUco 横向和纵向方格数都必须至少为 3")
            if marker_length <= 0 or square_length <= 0 or marker_length >= square_length:
                raise ValueError("尺寸必须满足 0 < ArUco边长 < 方格边长")
            if min_corners < 4:
                raise ValueError("最少 ChArUco 角点数不能小于 4")

            self.update_config(
                board_type="charuco",
                charuco_dictionary=dictionary,
                charuco_squares_x=squares_x,
                charuco_squares_y=squares_y,
                charuco_square_length_m=square_length,
                charuco_marker_length_m=marker_length,
                min_charuco_corners=min_corners,
                max_reprojection_error_px=max_error,
            )
            self.camera.restart()
            print("\n[OK] 已切换为 ChArUco 检测")
        except MenuCancelled:
            print("已取消，未修改标定板参数")
        except (ValueError, TypeError) as exc:
            print(f"[输入错误] {exc}")

    def configure_intrinsics(self) -> None:
        print("\n========== 相机内参 ==========")
        print("手眼标定必须先有相机内参没有内参时，程序只能识别角点，不能计算标定板位姿，也不能采样")
        try:
            choice = self._guided_choice(
                "请选择内参来源",
                {
                    "1": "从 ROS camera_info.yaml 或 OpenCV YAML/XML 文件加载",
                    "2": "手动输入 fx、fy、cx、cy 和畸变参数",
                    "0": "返回主菜单",
                },
            )
            if choice == "0":
                return
            if choice == "1":
                print("\n请输入内参文件的完整路径")
                print("示例：/home/wheeltec/calibration/camera_info.yaml")
                print("输入 q 返回上一级")
                path = self._input(">>> ").strip()
                if path.lower() in ("q", "quit", "cancel", "取消"):
                    return
                if not path:
                    raise ValueError("文件路径不能为空")
                intrinsics = self.load_intrinsics(path)
                print("[OK] 相机内参已加载")
                print("K =")
                print(intrinsics.matrix)
                print("D =", intrinsics.distortion.reshape(-1))
                print("现在相机窗口中的 pose 应从 NOT READY 变为 VALID（前提是棋盘格已识别）")
                return

            fx, fy, cx, cy = parse_float_list(
                self._guided_value(
                    "输入 fx,fy,cx,cy",
                    "fx,fy,cx,cy",
                    str,
                    "四个值使用英文逗号分隔fx/fy 是焦距像素值，cx/cy 是主点坐标",
                    "615.2,614.8,640.0,360.0",
                ),
                4,
            )
            distortion_text = self._guided_value(
                "输入畸变参数",
                "k1,k2,p1,p2,k3",
                str,
                "至少输入 k1,k2,p1,p2；常用五参数为 k1,k2,p1,p2,k3",
                "-0.12,0.08,0.0,0.0,0.0",
            )
            distortion = [
                float(item.strip())
                for item in distortion_text.split(",")
                if item.strip()
            ]
            if len(distortion) < 4:
                raise ValueError("畸变参数至少需要 4 个")
            self.set_manual_intrinsics(
                [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0], distortion
            )
            print("[OK] 已设置并保存手动内参")
        except MenuCancelled:
            print("已取消设置相机内参")
        except Exception as exc:  # noqa: BLE001
            print(f"[失败] {exc}")

    def configure_robot(self) -> None:
        config = self.get_config()
        try:
            mode = self._ask("手眼模式 (eye_in_hand/eye_to_hand)", config.handeye_mode, str).lower()
            if mode not in ("eye_in_hand", "eye_to_hand"):
                raise ValueError("手眼模式只能是 eye_in_hand 或 eye_to_hand")
            source = self._ask("位姿来源 (auto/pose_topic/joint_fk)", config.arm_pose_source, str).lower()
            if source not in ("auto", "pose_topic", "joint_fk"):
                raise ValueError("位姿来源只能是 auto、pose_topic 或 joint_fk")
            base_frame = self._ask("base frame", config.base_frame, str)
            gripper_frame = self._ask("gripper frame", config.gripper_frame, str)
            camera_frame = self._ask("camera frame", config.camera_frame, str)
            target_frame = self._ask("target frame", config.target_frame, str)
            require_stable = self._ask_bool("采样前要求机械臂稳定", config.require_stable_before_capture)
            stable_duration = self._ask("稳定时间 [s]", config.stable_duration_s, float)
            stable_range = self._ask("稳定窗口最大关节变化 [rad]", config.stable_joint_range_rad, float)

            coefficients = config.orientation_rpy_joint_coeffs
            offsets = config.orientation_rpy_offset_rad
            if source in ("auto", "joint_fk"):
                print("当前关节到 RPY 系数（按 roll 5个、pitch 5个、yaw 5个）:")
                print(np.asarray(coefficients).reshape(3, 5))
                change = self._input("是否修改姿态映射? [y/N]: ").strip().lower()
                if change in ("y", "yes", "1"):
                    coefficients = parse_float_list(
                        self._input("输入 15 个系数，以逗号分隔: ").strip(), 15
                    )
                    offsets = parse_float_list(
                        self._input("输入 roll,pitch,yaw 偏置 [rad]: ").strip(), 3
                    )

            self.update_config(
                handeye_mode=mode,
                arm_pose_source=source,
                base_frame=base_frame,
                gripper_frame=gripper_frame,
                camera_frame=camera_frame,
                target_frame=target_frame,
                require_stable_before_capture=require_stable,
                stable_duration_s=stable_duration,
                stable_joint_range_rad=stable_range,
                orientation_rpy_joint_coeffs=coefficients,
                orientation_rpy_offset_rad=offsets,
            )
            print("[OK] 机械臂与手眼参数已更新")
        except (ValueError, TypeError) as exc:
            print(f"[输入错误] {exc}")

    def command_arm_menu(self) -> None:
        config = self.get_config()
        print("\n1. 发送 5 关节目标（角度制输入）")
        print("2. 发送末端 5D 位姿 x,y,z,pitch,yaw")
        choice = self._input("选择: ").strip()
        try:
            if choice == "1":
                joints_deg = parse_float_list(self._input("q0,q1,q2,q3,q4 [deg]: ").strip(), 5)
                speed = self._ask("速度 [rad/s]", config.default_arm_speed_rad_s, float)
                joints_rad = [math.radians(value) for value in joints_deg]
                ok, message = self.node.command_joints(joints_rad, speed)
                print(f"[{'OK' if ok else '失败'}] {message}")
            elif choice == "2":
                values = parse_float_list(self._input("x,y,z [m], pitch,yaw [deg]: ").strip(), 5)
                speed = self._ask("速度 [rad/s]", config.default_arm_speed_rad_s, float)
                ok, message = self.node.command_pose(
                    values[0], values[1], values[2],
                    math.radians(values[3]), math.radians(values[4]), speed,
                )
                print(f"[{'OK' if ok else '失败'}] {message}")
            else:
                print("已取消")
        except (ValueError, TypeError) as exc:
            print(f"[输入错误] {exc}")

    def move_and_capture(self) -> None:
        config = self.get_config()
        try:
            joints_deg = parse_float_list(self._input("q0,q1,q2,q3,q4 [deg]: ").strip(), 5)
            speed = self._ask("速度 [rad/s]", config.default_arm_speed_rad_s, float)
            ok, message = self.node.command_joints([math.radians(v) for v in joints_deg], speed)
            print(f"[{'OK' if ok else '失败'}] {message}")
            if ok:
                self.capture_sample()
        except (ValueError, TypeError) as exc:
            print(f"[输入错误] {exc}")

    def manage_samples(self) -> None:
        print(f"\n当前样本数：{len(self.samples)}")
        for sample in self.samples:
            print(
                f"#{sample['id']:>3} | board={sample.get('board_type', 'unknown'):<10} | "
                f"corners={sample.get('board_corner_count', sample.get('charuco_corner_count', 0)):>3} | "
                f"reproj={sample['reprojection_error_px']:.3f}px | "
                f"source={sample['robot_pose_source']}"
            )
        print("\n1. 删除指定样本")
        print("2. 清空全部样本")
        print("0. 返回")
        choice = self._input("选择: ").strip()
        if choice == "1":
            try:
                sample_id = int(self._input("样本 ID: ").strip())
                print("[OK] 已删除" if self.delete_sample(sample_id) else "[失败] 未找到该样本")
            except ValueError:
                print("[输入错误] ID 必须是整数")
        elif choice == "2":
            confirm = self._input("确认清空全部样本? 输入 CLEAR: ").strip()
            if confirm == "CLEAR":
                self.clear_samples()
                print("[OK] 已清空")

    def print_menu(self) -> None:
        print("\n\n========== UVC 单目手眼标定工具 ==========")
        print("1. 查看当前状态")
        print("2. 配置 UVC 相机")
        print("3. 配置标定板（普通黑白棋盘格 / ChArUco）")
        print("4. 加载/设置相机内参（采样前必须完成）")
        print("5. 配置机械臂与手眼模式")
        print("6. 控制机械臂")
        print("7. 采集当前样本")
        print("8. 发送关节目标并采集")
        print("9. 查看/删除采集样本")
        print("10. 求解手眼标定")
        print("11. 保存当前配置")
        print("12. 重启摄像头")
        print("0. 退出")
        print("操作说明：输入菜单编号并按 Enter；直接输入 c 可采样，输入 q 可退出")
        print("摄像头窗口获得焦点时，也可按 C 采样、Q 退出")
        print("==========================================")

    def run_menu(self) -> None:
        self.camera.start()
        try:
            while rclpy.ok():
                if not self.camera.is_alive():  # camera Q closes the application
                    break
                self.print_menu()
                try:
                    print("请输入菜单编号（例如 3），然后按 Enter：")
                    choice = self._input(">>> ").strip().lower()
                except (EOFError, KeyboardInterrupt):
                    print("\n正在退出...")
                    break

                if choice == "c":
                    self.capture_sample()
                elif choice == "q":
                    break
                elif choice == "1":
                    self.print_status()
                elif choice == "2":
                    self.configure_camera()
                elif choice == "3":
                    self.configure_board()
                elif choice == "4":
                    self.configure_intrinsics()
                elif choice == "5":
                    self.configure_robot()
                elif choice == "6":
                    self.command_arm_menu()
                elif choice == "7":
                    self.capture_sample()
                elif choice == "8":
                    self.move_and_capture()
                elif choice == "9":
                    self.manage_samples()
                elif choice == "10":
                    self.solve()
                elif choice == "11":
                    print(f"[OK] 配置已保存：{self.save_runtime_config()}")
                elif choice == "12":
                    self.camera.restart()
                    print("[OK] 已请求重启摄像头")
                elif choice == "0":
                    break
                elif choice:
                    print("[输入错误] 未知菜单编号")
        finally:
            self.save_runtime_config()
            self.camera.stop()


# ------------------------------- 启动入口 -------------------------------


def declare_and_read_config(node: Node) -> AppConfig:
    defaults = AppConfig()

    def read(name: str, default: Any) -> Any:
        return node.declare_parameter(name, default).value

    return AppConfig(
        camera_index=int(read("camera_index", defaults.camera_index)),
        camera_backend=str(read("camera_backend", defaults.camera_backend)),
        camera_width=int(read("camera_width", defaults.camera_width)),
        camera_height=int(read("camera_height", defaults.camera_height)),
        camera_fps=float(read("camera_fps", defaults.camera_fps)),
        camera_fourcc=str(read("camera_fourcc", defaults.camera_fourcc)),
        camera_window_name=str(read("camera_window_name", defaults.camera_window_name)),
        intrinsics_file=str(read("intrinsics_file", defaults.intrinsics_file)),
        board_type=str(read("board_type", defaults.board_type)).lower(),
        chessboard_inner_corners_x=int(
            read("chessboard_inner_corners_x", defaults.chessboard_inner_corners_x)
        ),
        chessboard_inner_corners_y=int(
            read("chessboard_inner_corners_y", defaults.chessboard_inner_corners_y)
        ),
        chessboard_square_length_m=float(
            read("chessboard_square_length_m", defaults.chessboard_square_length_m)
        ),
        chessboard_use_sb=bool(read("chessboard_use_sb", defaults.chessboard_use_sb)),
        charuco_dictionary=str(read("charuco_dictionary", defaults.charuco_dictionary)),
        charuco_squares_x=int(read("charuco_squares_x", defaults.charuco_squares_x)),
        charuco_squares_y=int(read("charuco_squares_y", defaults.charuco_squares_y)),
        charuco_square_length_m=float(read("charuco_square_length_m", defaults.charuco_square_length_m)),
        charuco_marker_length_m=float(read("charuco_marker_length_m", defaults.charuco_marker_length_m)),
        min_charuco_corners=int(read("min_charuco_corners", defaults.min_charuco_corners)),
        max_reprojection_error_px=float(
            read("max_reprojection_error_px", defaults.max_reprojection_error_px)
        ),
        handeye_mode=str(read("handeye_mode", defaults.handeye_mode)),
        base_frame=str(read("base_frame", defaults.base_frame)),
        gripper_frame=str(read("gripper_frame", defaults.gripper_frame)),
        camera_frame=str(read("camera_frame", defaults.camera_frame)),
        target_frame=str(read("target_frame", defaults.target_frame)),
        arm_pose_source=str(read("arm_pose_source", defaults.arm_pose_source)),
        arm_pose_topic=str(read("arm_pose_topic", defaults.arm_pose_topic)),
        arm_joint_state_topic=str(read("arm_joint_state_topic", defaults.arm_joint_state_topic)),
        arm_fk_position_topic=str(read("arm_fk_position_topic", defaults.arm_fk_position_topic)),
        orientation_rpy_joint_coeffs=[
            float(v) for v in read(
                "orientation_rpy_joint_coeffs", defaults.orientation_rpy_joint_coeffs
            )
        ],
        orientation_rpy_offset_rad=[
            float(v) for v in read("orientation_rpy_offset_rad", defaults.orientation_rpy_offset_rad)
        ],
        max_robot_state_age_s=float(read("max_robot_state_age_s", defaults.max_robot_state_age_s)),
        robot_sync_tolerance_s=float(
            read("robot_sync_tolerance_s", defaults.robot_sync_tolerance_s)
        ),
        require_stable_before_capture=bool(
            read("require_stable_before_capture", defaults.require_stable_before_capture)
        ),
        stable_duration_s=float(read("stable_duration_s", defaults.stable_duration_s)),
        stable_joint_range_rad=float(
            read("stable_joint_range_rad", defaults.stable_joint_range_rad)
        ),
        stable_wait_timeout_s=float(
            read("stable_wait_timeout_s", defaults.stable_wait_timeout_s)
        ),
        arm_joints_service=str(read("arm_joints_service", defaults.arm_joints_service)),
        arm_pose_service=str(read("arm_pose_service", defaults.arm_pose_service)),
        default_arm_speed_rad_s=float(
            read("default_arm_speed_rad_s", defaults.default_arm_speed_rad_s)
        ),
        output_directory=str(read("output_directory", defaults.output_directory)),
    )


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)
    bootstrap = Node("handeye_calibration_tool")
    config = declare_and_read_config(bootstrap)
    bootstrap.destroy_node()

    node = ArmStateNode(config)
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, name="handeye_ros_spin", daemon=True)
    spin_thread.start()

    application = HandEyeApplication(node, config)
    print(f"[INFO] 标定会话目录：{application.session_dir}")
    print("[INFO] UVC 摄像头窗口将独立显示；终端用于配置、控制与采集")
    print(f"[INFO] 终端输入源：{application.console.source}")
    try:
        application.run_menu()
    finally:
        application.console.close()
        executor.shutdown(timeout_sec=2.0)
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
