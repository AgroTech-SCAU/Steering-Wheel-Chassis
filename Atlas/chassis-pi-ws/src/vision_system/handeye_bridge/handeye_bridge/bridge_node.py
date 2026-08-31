#!/usr/bin/env python3
"""
handeye_bridge — 相机像素 → 手眼转换 → 机械臂控制
==================================================
独立 ROS2 节点。

深度:  射线-平面相交法。像素反投影为空间射线，
       与已知高度平面求交点得 (x,y)，Z 取配置高度值。

输入:  /detection_centers (DetectionCenterArray) — 按像素坐标排序的检测中心点+角标签
        /pick_target (PickTarget)                 — 选择角(0=TL/1=TR/2=BR/3=BL)+层(1/2/3)
        /arm/pose (PoseStamped)                   — 末端位姿 (MCU 的 FK，与标定一致)

输出:  /mcu/set_arm_pose                          — 控制机械臂
"""

from __future__ import annotations

import os
import sys
from collections import OrderedDict, deque
from typing import Callable, Optional, Tuple

import cv2
import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from vison_topic_interfaces.msg import DetectionCenterArray, PickTarget

# 延迟导入: mcu_comm_bridge 可能未安装 (仅 handeye_bridge 需要)
_SetArmPose = None


def _get_set_arm_pose():
    global _SetArmPose
    if _SetArmPose is None:
        from mcu_comm_bridge.srv import SetArmPose as _SAP
        _SetArmPose = _SAP
    return _SetArmPose

# calib 不是 ROS 包，通过路径引入 fk_utils 避免重复实现四元数/旋转公式
_calib_path = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "calib"))
if os.path.isdir(_calib_path):
    if _calib_path not in sys.path:
        sys.path.insert(0, _calib_path)
    try:
        from fk_utils import quaternion_to_matrix as _r_from_quat, make_transform
    except ImportError:
        _r_from_quat = None
        make_transform = None
else:
    _r_from_quat = None
    make_transform = None


# ── 四元数 → 4x4 齐次矩阵 ──

def _quat_to_matrix(x: float, y: float, z: float, w: float, t: np.ndarray) -> np.ndarray:
    """四元数 (x,y,z,w) + 平移 → 4x4 齐次矩阵.

    优先委托 calib/fk_utils.quaternion_to_matrix + make_transform 避免公式重复;
    回退到本地实现以保证独立运行。
    """
    if _r_from_quat is not None and make_transform is not None:
        return make_transform(_r_from_quat(x, y, z, w), t)

    # ── 回退 (与 fk_utils.quaternion_to_matrix 完全一致) ──
    n = np.sqrt(x*x + y*y + z*z + w*w)
    if n < 1e-10:
        return np.eye(4, dtype=np.float64)
    x, y, z, w = x/n, y/n, z/n, w/n
    R = np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ], dtype=np.float64)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


# ── 节点 ──

class HandEyeBridgeNode(Node):
    """接收像素检测 → 射线平面求交 → 控制机械臂"""

    def __init__(self) -> None:
        super().__init__("handeye_bridge")

        # ── 参数声明 ──
        self.declare_parameter("intrinsics_file", "")
        self.declare_parameter("handeye_result_file", "")
        self.declare_parameter("detection_centers_topic", "/detection_centers")
        self.declare_parameter("pick_target_topic", "/pick_target")
        self.declare_parameter("pose_topic", "/arm/pose")
        self.declare_parameter("arm_pose_service", "/mcu/set_arm_pose")
        self.declare_parameter("initial_pose_service", "/move_to_initial_pose")
        self.declare_parameter("initial_pose_ready_topic", "/initial_pose_ready")
        self.declare_parameter("plane1_z_m", 0.05)
        self.declare_parameter("plane2_z_m", 0.12)
        self.declare_parameter("plane3_z_m", 0.19)
        self.declare_parameter("camera_to_plane1_distance_m", 0.35)
        self.declare_parameter("camera_to_plane2_distance_m", 0.28)
        self.declare_parameter("camera_to_plane3_distance_m", 0.21)
        self.declare_parameter("default_plane", 1)
        self.declare_parameter("default_speed_rad_s", 0.8)
        self.declare_parameter("target_z_offset_m", 0.03)
        self.declare_parameter("auto_send", False)
        self.declare_parameter("plane_heights_configured", False)
        self.declare_parameter("expected_image_width", 640)
        self.declare_parameter("expected_image_height", 480)
        self.declare_parameter("max_detection_age_s", 0.5)
        self.declare_parameter("final_best_valid_s", 30.0)
        self.declare_parameter("max_pose_sync_dt_ms", 80.0)
        self.declare_parameter("pose_history_size", 200)
        self.declare_parameter("detection_pose_cache_size", 2000)
        self.declare_parameter("min_handeye_inliers", 6)
        self.declare_parameter("max_handeye_translation_m", 0.50)
        self.declare_parameter("manual_offset_x_m", 0.0)
        self.declare_parameter("manual_offset_y_m", 0.0)
        self.declare_parameter("manual_offset_z_m", 0.0)
        self.declare_parameter("initial_pose_configured", False)
        self.declare_parameter("auto_move_to_initial_on_start", True)
        self.declare_parameter("initial_move_delay_s", 2.0)
        self.declare_parameter("initial_x_m", 0.0)
        self.declare_parameter("initial_y_m", 0.0)
        self.declare_parameter("initial_z_m", 0.0)
        self.declare_parameter("initial_pitch_rad", 0.0)
        self.declare_parameter("initial_yaw_rad", 0.0)
        self.declare_parameter("initial_speed_rad_s", 0.3)
        self.declare_parameter("initial_position_tolerance_m", 0.005)
        self.declare_parameter("initial_stable_samples", 5)
        self.declare_parameter("initial_move_timeout_s", 15.0)
        self.declare_parameter("workspace_max_xy_m", 0.80)
        self.declare_parameter("workspace_z_min_m", -0.20)
        self.declare_parameter("workspace_z_max_m", 0.50)
        self.declare_parameter("initial_pose_departure_tolerance_m", 0.03)

        # ── 深度模式 ──
        # "manual": 使用 planeX_z_m 固定高度 (默认, 兼容旧配置)
        # "pnp":    用 4 个检测角点 + 已知物理间距做 PnP 求解深度
        self.declare_parameter("depth_mode", "manual")
        # PnP 模式参数: 4 个螺丝/角点围成的矩形的物理尺寸 (单位米)
        self.declare_parameter("pnp_target_width_m", 0.05)   # TL→TR 的物理宽度 (X)
        self.declare_parameter("pnp_target_height_m", 0.05)  # TL→BL 的物理高度 (Y)
        self.declare_parameter("pnp_max_reprojection_px", 2.0)  # PnP 最大允许重投影误差
        self.declare_parameter("pnp_min_depth_m", 0.05)      # 最小合理深度 (太近 = 异常)
        self.declare_parameter("pnp_max_depth_m", 0.80)      # 最大合理深度 (太远 = 异常)
        self.declare_parameter("require_intrinsics_binding", True)

        # ── 加载标定文件 ──
        self.camera_mtx: Optional[np.ndarray] = None
        self.camera_dist: Optional[np.ndarray] = None
        self.camera_image_size: Optional[Tuple[int, int]] = None
        self.T_gripper_camera: np.ndarray = np.eye(4)
        self._intrinsics_valid = False
        self._handeye_valid = False

        intrinsics_file = str(self.get_parameter("intrinsics_file").value)
        handeye_file = str(self.get_parameter("handeye_result_file").value)

        # 相对路径 → 相对于 package share/config 解析
        intrinsics_file = self._resolve_path(intrinsics_file)
        handeye_file = self._resolve_path(handeye_file)

        if intrinsics_file and os.path.exists(intrinsics_file):
            try:
                self._intrinsics_valid = self._load_intrinsics(intrinsics_file)
            except Exception as exc:
                self.get_logger().error(f"相机内参加载失败: {exc}")
        else:
            self.get_logger().warn(
                f"未配置 camera_intrinsics.yaml (path={intrinsics_file})")

        if handeye_file and os.path.exists(handeye_file):
            try:
                self._handeye_valid = self._load_handeye(handeye_file)
            except Exception as exc:
                self.get_logger().error(f"手眼结果加载失败: {exc}")
        else:
            self.get_logger().error(
                f"未配置 handeye_result.yaml！请先运行 solve.py (path={handeye_file})")

        # ── 状态 ──
        history_size = max(20, int(self.get_parameter("pose_history_size").value))
        self._pose_history = deque(maxlen=history_size)
        self._detection_pose_cache = OrderedDict()
        self._detection_pose_cache_size = max(
            100, int(self.get_parameter("detection_pose_cache_size").value))
        self._latest_detections: Optional[DetectionCenterArray] = None
        self._latest_detection_stamp_ns: Optional[int] = None
        self._latest_detection_received_ns: Optional[int] = None
        self._latest_detection_is_final_best = False
        self._initial_move_pending = False
        self._initial_command_accepted = False
        self._initial_pose_ready = False
        self._initial_stable_count = 0
        self._initial_target_xyz: Optional[np.ndarray] = None
        self._initial_deadline_ns = 0
        self._initial_pose_received_count = 0     # 诊断: 初始移动期间收到的pose数
        self._initial_last_distance_m = float("inf")  # 诊断: 最近一次距离
        self._initial_last_diag_ns = 0            # 诊断: 上次打印距离日志的时间
        self._pending_pose_futures: list = []  # 防止 future GC 导致回调丢失
        self._calibration_ready = self._intrinsics_valid and self._handeye_valid
        if not self._calibration_ready:
            self.get_logger().error("标定未通过有效性检查：坐标计算和自动发送均已禁用")

        # ── 订阅 ──
        centers_topic = str(self.get_parameter("detection_centers_topic").value)
        pick_topic = str(self.get_parameter("pick_target_topic").value)
        pose_topic = str(self.get_parameter("pose_topic").value)

        self.det_sub = self.create_subscription(
            DetectionCenterArray, centers_topic, self._on_detections, 10)
        self.pick_sub = self.create_subscription(
            PickTarget, pick_topic, self._on_pick_target, 10)
        self.pose_sub = self.create_subscription(
            PoseStamped, pose_topic, self._on_pose, 20)

        # ── 服务客户端（持久化，避免回调内 create_client 的 DDS 竞态）──
        self._arm_pose_cli = None
        try:
            arm_pose_svc = str(self.get_parameter("arm_pose_service").value)
            self._arm_pose_cli = self.create_client(
                _get_set_arm_pose(), arm_pose_svc)
        except Exception as exc:
            self.get_logger().warn(
                f"无法创建 SetArmPose 客户端 (mcu_comm_bridge 未安装?): {exc}")

        # ── 初始位置服务与状态 ──
        ready_qos = QoSProfile(depth=1)
        ready_qos.reliability = ReliabilityPolicy.RELIABLE
        ready_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        ready_topic = str(self.get_parameter("initial_pose_ready_topic").value)
        self.initial_ready_pub = self.create_publisher(Bool, ready_topic, ready_qos)
        initial_srv = str(self.get_parameter("initial_pose_service").value)
        self.initial_pose_srv = self.create_service(
            Trigger, initial_srv, self._on_move_to_initial_pose)
        self._publish_initial_ready(False)

        self._initial_watchdog_timer = self.create_timer(
            0.1, self._check_initial_move_timeout)
        self._initial_auto_timer = None
        if bool(self.get_parameter("auto_move_to_initial_on_start").value):
            delay_s = max(0.1, float(
                self.get_parameter("initial_move_delay_s").value))
            self._initial_auto_timer = self.create_timer(
                delay_s, self._auto_move_to_initial_once)

        self.get_logger().info("HandEye Bridge 就绪  (射线-平面求交, 使用 /arm/pose FK)")
        self.get_logger().info(
            f"初始位置服务: {initial_srv}，就绪状态: {ready_topic}")

    # ── 路径解析 ──

    @staticmethod
    def _resolve_path(path: str) -> str:
        """相对路径 → 基于 package share 目录解析；绝对路径原样返回."""
        if not path:
            return ""
        if os.path.isabs(path):
            return path
        try:
            from ament_index_python.packages import get_package_share_directory
            pkg_share = get_package_share_directory("handeye_bridge")
            return os.path.join(pkg_share, "config", path)
        except Exception:
            return path

    # ── 加载 ──

    def _load_intrinsics(self, path: str) -> bool:
        with open(path, encoding="utf-8") as f:
            d = yaml.safe_load(f)
        camera_mtx = np.array(d["camera_matrix"]["data"], dtype=np.float64).reshape(3, 3)
        camera_dist = np.array(
            d["distortion_coefficients"]["data"], dtype=np.float64).reshape(-1, 1)
        if not np.isfinite(camera_mtx).all() or not np.isfinite(camera_dist).all():
            raise ValueError("内参包含 NaN/inf")
        if camera_mtx[0, 0] <= 0 or camera_mtx[1, 1] <= 0:
            raise ValueError("fx/fy 必须大于 0")

        expected_w = int(self.get_parameter("expected_image_width").value)
        expected_h = int(self.get_parameter("expected_image_height").value)
        image_w = int(d.get("image_width", 0))
        image_h = int(d.get("image_height", 0))
        if (image_w, image_h) != (expected_w, expected_h):
            raise ValueError(
                f"内参分辨率 {image_w}x{image_h} 与应用分辨率 "
                f"{expected_w}x{expected_h} 不一致")

        self.camera_mtx = camera_mtx
        self.camera_dist = camera_dist
        self.camera_image_size = (image_w, image_h)
        fx, fy = self.camera_mtx[0,0], self.camera_mtx[1,1]
        cx, cy = self.camera_mtx[0,2], self.camera_mtx[1,2]
        self.get_logger().info(
            f"相机内参: {image_w}x{image_h} fx={fx:.1f} fy={fy:.1f} "
            f"cx={cx:.1f} cy={cy:.1f}")
        return True

    def _load_handeye(self, path: str) -> bool:
        with open(path, encoding="utf-8") as f:
            d = yaml.safe_load(f)
        mode = str(d.get("handeye_mode", "eye_in_hand"))
        if mode != "eye_in_hand":
            raise ValueError(
                f"handeye_bridge 当前变换链仅支持 eye_in_hand，收到 {mode}")
        stored = np.array(d["transform_matrix"], dtype=np.float64)
        if stored.shape != (4, 4) or not np.isfinite(stored).all():
            raise ValueError("transform_matrix 必须是有限的 4x4 矩阵")
        if not np.allclose(stored[3], [0.0, 0.0, 0.0, 1.0], atol=1e-6):
            raise ValueError("transform_matrix 最后一行无效")
        convention = str(d.get("stored_transform_convention", "legacy"))
        if convention.startswith("camera_to_gripper"):
            self.T_gripper_camera = stored
        else:
            # 兼容旧版 solve.py 保存的 ^camera T_gripper。
            self.T_gripper_camera = np.linalg.inv(stored)

        binding = d.get("intrinsics_binding")
        require_binding = bool(
            self.get_parameter("require_intrinsics_binding").value)
        if binding is None:
            if require_binding:
                raise ValueError(
                    "手眼结果未绑定采集时内参；请用新版 solve.py 重新求解，"
                    "禁止与未知内参混用")
            self.get_logger().warn("手眼结果没有内参绑定，兼容模式继续运行")
        else:
            if self.camera_mtx is None or self.camera_dist is None:
                raise ValueError("必须先成功加载相机内参，才能验证手眼结果")
            bound_k = np.asarray(
                binding.get("camera_matrix_data", []), dtype=np.float64)
            bound_d = np.asarray(
                binding.get("distortion_data", []), dtype=np.float64).reshape(-1)
            bound_size = (
                int(binding.get("image_width", 0)),
                int(binding.get("image_height", 0)))
            if bound_k.size != 9:
                raise ValueError("手眼结果中的内参绑定格式无效")
            bound_k = bound_k.reshape(3, 3)
            same = (
                bound_size == self.camera_image_size
                and bound_d.shape == self.camera_dist.reshape(-1).shape
                and np.allclose(bound_k, self.camera_mtx, rtol=0.0, atol=1e-9)
                and np.allclose(bound_d, self.camera_dist.reshape(-1),
                                rtol=0.0, atol=1e-9))
            if not same:
                raise ValueError(
                    "运行时 camera_intrinsics.yaml 与手眼标定采集内参不一致；"
                    "请用当前内参重新计算 target_to_camera 并重新求解手眼")
        rotation = self.T_gripper_camera[:3, :3]
        if (np.linalg.norm(rotation.T @ rotation - np.eye(3)) > 1e-4
                or abs(np.linalg.det(rotation) - 1.0) > 1e-4):
            raise ValueError("手眼旋转矩阵不是有效旋转")

        inlier_count = int(d.get("inlier_count", 0))
        min_inliers = int(self.get_parameter("min_handeye_inliers").value)
        if inlier_count < min_inliers:
            raise ValueError(
                f"手眼结果只有 {inlier_count} 个内点，至少需要 {min_inliers} 个")

        t = self.T_gripper_camera[:3, 3]
        t_norm = float(np.linalg.norm(t))
        max_translation = float(
            self.get_parameter("max_handeye_translation_m").value)
        if t_norm > max_translation:
            raise ValueError(
                f"相机安装平移 {t_norm:.3f}m 超过上限 {max_translation:.3f}m")
        self.get_logger().info(
            f"手眼矩阵: x={t[0]:.4f} y={t[1]:.4f} z={t[2]:.4f} m  "
            f"内点={inlier_count}  (^gripper T_camera)")
        return True

    # ── 回调 ──

    def _publish_initial_ready(self, ready: bool) -> None:
        self._initial_pose_ready = bool(ready)
        msg = Bool()
        msg.data = self._initial_pose_ready
        self.initial_ready_pub.publish(msg)

    def _initial_pose_command(self) -> Tuple[float, float, float, float, float, float]:
        values = (
            float(self.get_parameter("initial_x_m").value),
            float(self.get_parameter("initial_y_m").value),
            float(self.get_parameter("initial_z_m").value),
            float(self.get_parameter("initial_pitch_rad").value),
            float(self.get_parameter("initial_yaw_rad").value),
            float(self.get_parameter("initial_speed_rad_s").value),
        )
        if not np.isfinite(values).all():
            raise ValueError("初始位置参数包含 NaN/inf")
        if values[5] <= 0.0:
            raise ValueError("initial_speed_rad_s 必须大于 0")
        tolerance = float(
            self.get_parameter("initial_position_tolerance_m").value)
        if not np.isfinite(tolerance) or tolerance <= 0.0:
            raise ValueError("initial_position_tolerance_m 必须大于 0")
        return values

    def _request_initial_pose(self, source: str) -> Tuple[bool, str]:
        if not bool(self.get_parameter("initial_pose_configured").value):
            message = (
                "初始位置尚未配置：请填写 bridge_node.yaml 的 initial_x/y/z_m "
                "等参数，并把 initial_pose_configured 改为 true")
            self.get_logger().error(message)
            self._publish_initial_ready(False)
            return False, message
        if self._initial_move_pending:
            return False, "初始位置运动正在进行中，请勿重复发送"

        try:
            x, y, z, pitch, yaw, speed = self._initial_pose_command()
        except ValueError as exc:
            self.get_logger().error(str(exc))
            self._publish_initial_ready(False)
            return False, str(exc)

        self._initial_move_pending = True
        self._initial_command_accepted = False
        self._initial_stable_count = 0
        self._initial_pose_received_count = 0
        self._initial_last_distance_m = float("inf")
        self._initial_last_diag_ns = 0
        self._initial_target_xyz = np.array([x, y, z], dtype=np.float64)
        timeout_s = max(1.0, float(
            self.get_parameter("initial_move_timeout_s").value))
        self._initial_deadline_ns = (
            int(self.get_clock().now().nanoseconds) + int(timeout_s * 1e9))
        self._publish_initial_ready(False)

        sent = self._send_pose(
            x, y, z, speed,
            pitch=pitch,
            yaw=yaw,
            label="初始位置",
            on_complete=self._on_initial_command_result,
        )
        if not sent:
            self._initial_move_pending = False
            self._initial_target_xyz = None
            return False, "MCU位姿服务不可用，初始位置命令未发送"

        message = (
            f"已由{source}发送初始位置命令: "
            f"x={x:.4f} y={y:.4f} z={z:.4f} m；等待 /arm/pose 到位")
        self.get_logger().info(message)
        return True, message

    def _on_move_to_initial_pose(self, _request, response):
        """手动触发回初始位置；响应成功表示命令已提交，不代表已经到位。"""
        response.success, response.message = self._request_initial_pose("服务")
        return response

    def _auto_move_to_initial_once(self) -> None:
        if self._initial_auto_timer is not None:
            self._initial_auto_timer.cancel()
            self._initial_auto_timer = None
        self._request_initial_pose("启动自动流程")

    def _on_initial_command_result(self, success: bool) -> None:
        if not self._initial_move_pending:
            return
        self._initial_command_accepted = bool(success)
        if not success:
            self._initial_move_pending = False
            self._initial_target_xyz = None
            self._publish_initial_ready(False)
            self.get_logger().error("MCU拒绝初始位置命令，视觉检测保持禁用")

    def _check_initial_move_timeout(self) -> None:
        if not self._initial_move_pending:
            return
        if int(self.get_clock().now().nanoseconds) <= self._initial_deadline_ns:
            return
        # 超时：打印诊断信息帮助排查
        if self._initial_pose_received_count == 0:
            hint = (
                "未收到任何 /arm/pose 消息！"
                "请检查 MCU 是否在发布 /arm/pose（ros2 topic echo /arm/pose）")
        else:
            hint = (
                f"共收到 {self._initial_pose_received_count} 帧 /arm/pose，"
                f"最近距离 {self._initial_last_distance_m * 1000.0:.1f} mm")
        self._initial_move_pending = False
        self._initial_command_accepted = False
        self._initial_target_xyz = None
        self._initial_stable_count = 0
        self._initial_pose_received_count = 0
        self._publish_initial_ready(False)
        self.get_logger().error(
            f"机械臂回初始位置超时，视觉检测保持禁用。{hint}")

    def _update_initial_pose_state(self, position: np.ndarray) -> None:
        # ── 初始移动到位检测 ──
        if (self._initial_move_pending
                and self._initial_command_accepted
                and self._initial_target_xyz is not None):
            distance_m = float(np.linalg.norm(position - self._initial_target_xyz))
            self._initial_pose_received_count += 1
            self._initial_last_distance_m = distance_m
            tolerance_m = float(
                self.get_parameter("initial_position_tolerance_m").value)
            if distance_m <= tolerance_m:
                self._initial_stable_count += 1
            else:
                self._initial_stable_count = 0

            # 每秒打印一次诊断信息
            now_ns = int(self.get_clock().now().nanoseconds)
            if now_ns - self._initial_last_diag_ns >= 1_000_000_000:
                self._initial_last_diag_ns = now_ns
                required = max(1, int(
                    self.get_parameter("initial_stable_samples").value))
                self.get_logger().info(
                    f"[初始位置诊断] 距目标 {distance_m * 1000.0:.1f} mm  "
                    f"(容差 {tolerance_m * 1000.0:.1f} mm)  "
                    f"已稳定 {self._initial_stable_count}/{required} 帧  "
                    f"共收到 {self._initial_pose_received_count} 帧pose  "
                    f"当前位置 ({position[0]:.4f}, {position[1]:.4f}, {position[2]:.4f})",
                    throttle_duration_sec=1.0)

            required = max(1, int(
                self.get_parameter("initial_stable_samples").value))
            if self._initial_stable_count >= required:
                self._initial_move_pending = False
                self._initial_target_xyz = None
                self._publish_initial_ready(True)
                self.get_logger().info(
                    f"机械臂已到达初始位置并稳定 {required} 帧 "
                    f"(位置误差 {distance_m * 1000.0:.1f} mm)，允许启动视觉检测")
            return

        # ── 持续监控：非移动期间检测机械臂是否偏离初始位 ──
        if (not self._initial_move_pending
                and self._initial_pose_ready
                and self._initial_pose_configured()):
            try:
                init_x, init_y, init_z, _pitch, _yaw, _speed = self._initial_pose_command()
                init_xyz = np.array([init_x, init_y, init_z], dtype=np.float64)
            except ValueError:
                return
            departure_tol = float(
                self.get_parameter("initial_pose_departure_tolerance_m").value)
            dist_from_init = float(np.linalg.norm(position - init_xyz))
            if dist_from_init > departure_tol:
                self.get_logger().warn(
                    f"机械臂偏离初始观察位 {dist_from_init*1000:.0f}mm "
                    f"(容差{departure_tol*1000:.0f}mm)，"
                    f"当前位置({position[0]:.3f},{position[1]:.3f},{position[2]:.3f}) "
                    f"≠ 初始位({init_x:.3f},{init_y:.3f},{init_z:.3f})，"
                    f"视觉检测门禁已关闭")
                self._publish_initial_ready(False)

    def _initial_pose_configured(self) -> bool:
        """初始位置参数是否已配置且有效."""
        return bool(self.get_parameter("initial_pose_configured").value)

    def _on_pose(self, msg: PoseStamped) -> None:
        """接收 MCU 的末端位姿 (与标定时用的同一套 FK)"""
        p = msg.pose.position
        q = msg.pose.orientation
        stamp_ns = self._stamp_to_ns(msg.header.stamp)
        if stamp_ns <= 0:
            self.get_logger().warn("/arm/pose 没有有效 header.stamp，忽略该位姿")
            return
        if np.linalg.norm([q.x, q.y, q.z, q.w]) < 1e-8:
            self.get_logger().warn("/arm/pose 四元数无效，忽略该位姿")
            return
        T = _quat_to_matrix(q.x, q.y, q.z, q.w,
                            np.array([p.x, p.y, p.z], dtype=np.float64))
        if np.isfinite(T).all():
            self._pose_history.append((stamp_ns, T))
            self._update_initial_pose_state(T[:3, 3])

    def _on_detections(self, msg: DetectionCenterArray) -> None:
        """缓存检测结果，并保存该帧在基座系计算所需的原始机械臂位姿。"""
        stamp_ns = self._stamp_to_ns(msg.header.stamp)
        if stamp_ns <= 0:
            self.get_logger().warn("/detection_centers 没有有效 header.stamp，忽略该帧")
            return

        # 实时帧到达时立即缓存它对应的机械臂位姿。停止时重发的最佳帧仍使用
        # 同一个原始stamp，因此即使机械臂后来移动，也不会改用停止时位姿。
        if stamp_ns not in self._detection_pose_cache:
            transform, sync_dt_ms = self._pose_at(stamp_ns)
            max_sync_dt_ms = float(self.get_parameter("max_pose_sync_dt_ms").value)
            if transform is not None and sync_dt_ms <= max_sync_dt_ms:
                self._detection_pose_cache[stamp_ns] = (transform.copy(), sync_dt_ms)
                while len(self._detection_pose_cache) > self._detection_pose_cache_size:
                    self._detection_pose_cache.popitem(last=False)

        self._latest_detections = msg
        self._latest_detection_stamp_ns = stamp_ns
        self._latest_detection_received_ns = int(self.get_clock().now().nanoseconds)
        self._latest_detection_is_final_best = bool(msg.is_final_best)
        if self._latest_detection_is_final_best:
            if stamp_ns in self._detection_pose_cache:
                self.get_logger().info(
                    "已接收停止检测时的最终最佳帧，并找到其原始机械臂位姿")
            else:
                self.get_logger().warn(
                    "已接收最终最佳帧，但未缓存到其原始机械臂位姿；该帧将不能抓取")

    @staticmethod
    def _stamp_to_ns(stamp) -> int:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def _pose_at(self, stamp_ns: int) -> Tuple[Optional[np.ndarray], float]:
        """返回离检测帧时间最近的 ^base T_gripper 和时间差(ms)。"""
        if not self._pose_history:
            return None, float("inf")
        pose_stamp_ns, transform = min(
            self._pose_history, key=lambda item: abs(item[0] - stamp_ns))
        return transform, abs(pose_stamp_ns - stamp_ns) / 1e6

    def _on_pick_target(self, msg: PickTarget) -> None:
        """
        收到选择指令: msg.corner_index (0=TL/1=TR/2=BR/3=BL), msg.layer (1/2/3)
        匹配对应角的检测目标 → 射线求交得(x,y) → Z 取 yaml 高度 → setpose
        """
        corner = msg.corner_index
        layer = msg.layer

        speed = float(self.get_parameter("default_speed_rad_s").value)
        z_offset = float(self.get_parameter("target_z_offset_m").value)
        auto_send = bool(self.get_parameter("auto_send").value)

        if not self._calibration_ready:
            self.get_logger().error("标定未就绪，拒绝计算和发送目标")
            return

        # 校验层号
        if layer not in (1, 2, 3):
            layer = int(self.get_parameter("default_plane").value)
            if layer not in (1, 2, 3):
                layer = 1
            self.get_logger().warn(f"无效 layer={msg.layer}, 回退到 layer={layer}")

        plane_z = float(self.get_parameter(f"plane{layer}_z_m").value)

        if self._latest_detections is None or len(self._latest_detections.detections) == 0:
            self.get_logger().warn("尚无检测数据 (/detection_centers)")
            return

        detection_stamp_ns = self._latest_detection_stamp_ns
        if detection_stamp_ns is None:
            self.get_logger().warn("检测帧没有可用时间戳")
            return
        now_ns = int(self.get_clock().now().nanoseconds)
        if self._latest_detection_is_final_best:
            received_ns = self._latest_detection_received_ns
            if received_ns is None:
                self.get_logger().warn("最终最佳帧缺少本地接收时间")
                return
            detection_age_s = (now_ns - received_ns) / 1e9
            max_age_s = float(self.get_parameter("final_best_valid_s").value)
            age_name = "最终最佳帧接收后"
        else:
            detection_age_s = (now_ns - detection_stamp_ns) / 1e9
            max_age_s = float(self.get_parameter("max_detection_age_s").value)
            age_name = "实时检测帧"
        if detection_age_s < -0.05 or detection_age_s > max_age_s:
            self.get_logger().warn(
                f"{age_name}已过期或时钟异常: age={detection_age_s:.3f}s, "
                f"允许 0~{max_age_s:.3f}s")
            return

        cached_pose = self._detection_pose_cache.get(detection_stamp_ns)
        if cached_pose is not None:
            T_base_gripper, sync_dt_ms = cached_pose
        else:
            T_base_gripper, sync_dt_ms = self._pose_at(detection_stamp_ns)
        max_sync_dt_ms = float(self.get_parameter("max_pose_sync_dt_ms").value)
        if T_base_gripper is None or sync_dt_ms > max_sync_dt_ms:
            self.get_logger().warn(
                f"检测帧无法匹配机械臂位姿: dt={sync_dt_ms:.1f}ms > "
                f"{max_sync_dt_ms:.1f}ms")
            return

        # 按 corner_index 匹配
        match = None
        for d in self._latest_detections.detections:
            if d.corner_index == corner:
                match = d
                break

        if match is None:
            available = [d.corner_index for d in self._latest_detections.detections]
            self.get_logger().warn(
                f"未找到 corner_index={corner} 的检测目标, "
                f"可用角: {available}"
            )
            return

        px_x, px_y = match.u, match.v

        # ── 深度模式选择 ──
        depth_mode = str(self.get_parameter("depth_mode").value).strip().lower()
        pnp_diag = None  # PnP 诊断信息

        if depth_mode == "pnp":
            # ═══ PnP 深度模式: 用 4 角点 + 已知物理间距求解 ═══
            pnp_result = self._pnp_solve_depth(self._latest_detections)
            if pnp_result is None:
                return
            T_camera_target, depth_m, pnp_diag = pnp_result

            # ── PnP 质量校验 ──
            max_reproj = float(self.get_parameter("pnp_max_reprojection_px").value)
            min_d = float(self.get_parameter("pnp_min_depth_m").value)
            max_d = float(self.get_parameter("pnp_max_depth_m").value)

            if pnp_diag["reproj_px"] > max_reproj:
                self.get_logger().warn(
                    f"PnP 重投影误差过大: {pnp_diag['reproj_px']:.3f}px > "
                    f"{max_reproj}px, 拒绝 (可能检测框不准或间距参数填错)")
                return
            if not (min_d <= depth_m <= max_d):
                self.get_logger().warn(
                    f"PnP 深度不合理: {depth_m*1000:.0f}mm, "
                    f"允许范围 [{min_d*1000:.0f}, {max_d*1000:.0f}]mm, 拒绝")
                return

            # 目标角点在目标坐标系中的 3D 位置
            obj_pts = self._pnp_object_points()
            corner_3d_target = obj_pts[corner]  # corner: 0=TL 1=TR 2=BR 3=BL

            # 转到相机坐标系
            p_target_cam = (T_camera_target[:3, :3] @ corner_3d_target
                            + T_camera_target[:3, 3])

            # 转到 base 坐标系
            T_base_camera = T_base_gripper @ self.T_gripper_camera
            p_target_base = (T_base_camera[:3, :3] @ p_target_cam
                             + T_base_camera[:3, 3])

            x, y, z_pnp = (float(p_target_base[0]),
                           float(p_target_base[1]),
                           float(p_target_base[2]))
            z = z_pnp + z_offset  # 加 Z 偏移避免碰撞

        else:
            # ═══ 手动深度模式 (默认): 射线-平面求交 ═══
            P_base = self._ray_plane_intersect(
                px_x, px_y, plane_z, T_base_gripper)
            if P_base is None:
                return

            x, y = float(P_base[0]), float(P_base[1])
            z = float(plane_z + z_offset)  # Z 直接用 yaml 高度 + 偏移

        # 手动偏置 (补偿系统误差，通常标定后微调用)
        x += float(self.get_parameter("manual_offset_x_m").value)
        y += float(self.get_parameter("manual_offset_y_m").value)
        z += float(self.get_parameter("manual_offset_z_m").value)

        corner_names = {0: "左上", 1: "右上", 2: "右下", 3: "左下"}
        cn = corner_names.get(corner, f"角{corner}")
        if depth_mode == "pnp" and pnp_diag is not None:
            self.get_logger().info(
                f"🎯 [PnP] {'最终最佳帧 ' if self._latest_detection_is_final_best else ''}"
                f"{cn} 层{layer} "
                f"像素({px_x:.0f},{px_y:.0f}) cls={match.cls_name} "
                f"同步差={sync_dt_ms:.1f}ms "
                f"PnP深度={pnp_diag['depth_mm']:.0f}mm "
                f"重投影={pnp_diag['reproj_px']:.3f}px "
                f"→ 基座({x:.4f},{y:.4f},{z:.4f})m"
            )
        else:
            self.get_logger().info(
                f"🎯 [手动] {'最终最佳帧 ' if self._latest_detection_is_final_best else ''}"
                f"{cn} 层{layer} "
                f"像素({px_x:.0f},{px_y:.0f}) cls={match.cls_name} "
                f"同步差={sync_dt_ms:.1f}ms Z={plane_z:.3f}m "
                f"→ 基座({x:.4f},{y:.4f},{z:.4f})m"
            )

        # ── workspace 校验：拒绝明显超出机械臂可达范围的位置 ──
        max_xy = float(self.get_parameter("workspace_max_xy_m").value)
        z_min = float(self.get_parameter("workspace_z_min_m").value)
        z_max = float(self.get_parameter("workspace_z_max_m").value)
        dist_xy = float(np.sqrt(x * x + y * y))
        if dist_xy > max_xy or not (z_min <= z <= z_max):
            self.get_logger().error(
                f"计算位置 ({x:.3f},{y:.3f},{z:.3f}) 超出 workspace: "
                f"XY距离={dist_xy:.2f}m (上限{max_xy:.2f}m), "
                f"Z范围=[{z_min:.2f},{z_max:.2f}]m — "
                f"可能是检测框在图像边缘导致坐标偏移，已拒绝发送")
            return

        if auto_send:
            if depth_mode == "manual" and not bool(self.get_parameter("plane_heights_configured").value):
                self.get_logger().error(
                    "plane_heights_configured=false，平面高度尚未确认，拒绝自动发送")
            else:
                pitch = float(self.get_parameter("initial_pitch_rad").value)
                yaw = float(self.get_parameter("initial_yaw_rad").value)
                self._send_pose(x, y, z, speed, pitch=pitch, yaw=yaw)

    # ── 射线-平面求交 ──

    def _ray_plane_intersect(self, px_x: float, px_y: float,
                              plane_z: float,
                              T_base_gripper: np.ndarray) -> Optional[np.ndarray]:
        """
        像素 → 畸变矫正 → 射线方向 (相机系)
            → 转到 base 系 (手眼矩阵 + MCU FK)
            → 与 Z=plane_z 求交
        """
        if self.camera_mtx is None:
            return None

        fx, fy = self.camera_mtx[0, 0], self.camera_mtx[1, 1]
        cx, cy = self.camera_mtx[0, 2], self.camera_mtx[1, 2]

        # 1. 畸变矫正 + 归一化射线方向 (相机坐标系)
        pixel = np.array([[[px_x, px_y]]], dtype=np.float32)
        undistorted = cv2.undistortPoints(pixel, self.camera_mtx, self.camera_dist, P=self.camera_mtx)
        ux, uy = undistorted[0, 0]
        dx = (ux - cx) / fx
        dy = (uy - cy) / fy
        direction_cam = np.array([dx, dy, 1.0], dtype=np.float64)
        direction_cam /= np.linalg.norm(direction_cam)

        # 2. 相机原点 + 方向 → base 坐标系
        # chain: camera → gripper → base
        # ^base T_camera = ^base T_gripper @ ^gripper T_camera
        T_base_camera = T_base_gripper @ self.T_gripper_camera
        P_cam_base = T_base_camera[:3, 3]
        R_cam_base = T_base_camera[:3, :3]
        direction_base = R_cam_base @ direction_cam

        # 3. 射线 ∩ Z=plane_z 平面
        if abs(direction_base[2]) < 1e-12:
            self.get_logger().warn(f"射线平行于平面: dir_z={direction_base[2]:.8f}")
            return None

        t = (plane_z - P_cam_base[2]) / direction_base[2]
        if t <= 0:
            self.get_logger().warn(f"平面在相机后方: t={t:.3f}")
            return None

        return P_cam_base + t * direction_base

    # ── PnP 深度求解 ──

    def _pnp_object_points(self) -> np.ndarray:
        """返回 4 个角点在目标坐标系中的 3D 坐标 (Z=0 平面).

        角点顺序: TL(0), TR(1), BR(2), BL(3)
        坐标系: 目标中心为原点, X 向右, Y 向下
        """
        w = float(self.get_parameter("pnp_target_width_m").value)
        h = float(self.get_parameter("pnp_target_height_m").value)
        return np.array([
            [-w/2, -h/2, 0.0],   # TL
            [ w/2, -h/2, 0.0],   # TR
            [ w/2,  h/2, 0.0],   # BR
            [-w/2,  h/2, 0.0],   # BL
        ], dtype=np.float64)

    def _pnp_solve_depth(self, detections_msg) -> Optional[Tuple[np.ndarray, float, dict]]:
        """PnP 求解目标在相机坐标系下的位姿和深度.

        使用 4 个检测角点的像素坐标 + 已知物理间距 → solvePnP → T_camera_target.

        Args:
            detections_msg: DetectionCenterArray (需含完整 4 角)

        Returns:
            (T_camera_target, depth_m, diagnostics) or None
            diagnostics: {"reproj_px": float, "n_points": int, "depth_mm": float}
        """
        if self.camera_mtx is None:
            self.get_logger().error("PnP 深度模式需要相机内参")
            return None

        # 收集 4 个角的像素坐标
        corner_px = {}  # {corner_index: (u, v)}
        for d in detections_msg.detections:
            if 0 <= d.corner_index <= 3:
                corner_px[d.corner_index] = (d.u, d.v)

        if len(corner_px) < 4:
            missing = sorted(set(range(4)) - set(corner_px.keys()))
            self.get_logger().warn(
                f"PnP 需要完整 4 角, 当前只有 {len(corner_px)} 个, "
                f"缺少: {missing}")
            return None

        # 按 TL/TR/BR/BL 顺序提取图像点
        img_pts = np.array([
            corner_px[0],  # TL
            corner_px[1],  # TR
            corner_px[2],  # BR
            corner_px[3],  # BL
        ], dtype=np.float64).reshape(-1, 1, 2)

        # 3D 目标点
        obj_pts = self._pnp_object_points().reshape(-1, 1, 3).astype(np.float64)

        # IPPE: 专为共面点设计, 最适合平面矩形目标
        try:
            result = cv2.solvePnPGeneric(
                obj_pts, img_pts,
                self.camera_mtx, self.camera_dist,
                flags=cv2.SOLVEPNP_IPPE)
            retval, rvecs, tvecs = result[0], result[1], result[2]
        except cv2.error as exc:
            self.get_logger().error(f"PnP 求解异常: {exc}")
            return None

        if retval is None or retval == 0:
            self.get_logger().warn("PnP (IPPE) 求解失败")
            return None

        # 选重投影误差最小的解
        if isinstance(rvecs, np.ndarray):
            n_solutions = rvecs.shape[0] if rvecs.ndim >= 2 else 0
        elif isinstance(rvecs, (list, tuple)):
            n_solutions = len(rvecs)
        else:
            n_solutions = 0

        best = None
        for i in range(n_solutions):
            if isinstance(rvecs, np.ndarray):
                rv = rvecs[i].reshape(3, 1).astype(np.float64)
                tv = tvecs[i].reshape(3, 1).astype(np.float64)
            else:
                rv = np.array(rvecs[i], dtype=np.float64).reshape(3, 1)
                tv = np.array(tvecs[i], dtype=np.float64).reshape(3, 1)

            # 物理合理性: 目标必须在相机前方
            if tv[2, 0] <= 0:
                continue

            proj, _ = cv2.projectPoints(obj_pts, rv, tv,
                                        self.camera_mtx, self.camera_dist)
            reproj = float(np.sqrt(np.mean(
                np.sum((proj.reshape(-1, 2) - img_pts.reshape(-1, 2))**2, axis=1)
            )))

            if best is None or reproj < best[1]:
                R, _ = cv2.Rodrigues(rv)
                T = np.eye(4, dtype=np.float64)
                T[:3, :3] = R
                T[:3, 3] = tv.ravel()
                best = (T, reproj)

        if best is None:
            self.get_logger().warn("PnP 所有解均在相机后方或无效")
            return None

        T_camera_target, reproj_px = best
        depth_m = float(T_camera_target[2, 3])
        diag = {
            "reproj_px": reproj_px,
            "n_points": 4,
            "depth_mm": depth_m * 1000.0,
        }
        return T_camera_target, depth_m, diag

    # ── 机械臂控制 ──

    def _send_pose(
        self,
        x: float,
        y: float,
        z: float,
        speed: float,
        *,
        pitch: float = 0.0,
        yaw: float = 0.0,
        label: str = "目标位置",
        on_complete: Optional[Callable[[bool], None]] = None,
    ) -> bool:
        if self._arm_pose_cli is None:
            self.get_logger().error(
                "SetArmPose 客户端未初始化 (mcu_comm_bridge 未安装?)")
            return False
        svc_name = str(self.get_parameter("arm_pose_service").value)
        if not self._arm_pose_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().error(f"MCU {svc_name} 不可用")
            return False

        req = _get_set_arm_pose().Request()
        req.x_m = float(x)
        req.y_m = float(y)
        req.z_m = float(z)
        req.pitch_rad = float(pitch)
        req.yaw_rad = float(yaw)
        req.speed_rad_s = float(speed)
        req.suction_valid = False
        req.suction_enable = False

        self.get_logger().info(
            f"→ 发送 SetArmPose({label}): "
            f"x_m={req.x_m} y_m={req.y_m} z_m={req.z_m} "
            f"pitch_rad={req.pitch_rad} yaw_rad={req.yaw_rad} "
            f"speed_rad_s={req.speed_rad_s}")

        future = self._arm_pose_cli.call_async(req)
        self._pending_pose_futures.append(future)
        future.add_done_callback(
            lambda f, _x=x, _y=y, _z=z, _label=label, _done=on_complete:
            self._on_pose_result(f, _x, _y, _z, _label, _done))
        self._pending_pose_futures[:] = [
            pf for pf in self._pending_pose_futures if not pf.done()]
        return True

    def _on_pose_result(
        self,
        future,
        x: float,
        y: float,
        z: float,
        label: str = "目标位置",
        on_complete: Optional[Callable[[bool], None]] = None,
    ) -> None:
        success = False
        try:
            result = future.result()
            if result is not None and result.success:
                success = True
                self.get_logger().info(
                    f"→ MCU OK ({label}): x={x:.4f} y={y:.4f} z={z:.4f} m  "
                    f"seq={result.command_seq}")
            else:
                msg = result.message if result else "无响应"
                self.get_logger().error(
                    f"→ MCU FAIL ({label}): {msg}  "
                    f"(x={x:.4f} y={y:.4f} z={z:.4f})")
        except Exception as e:
            self.get_logger().error(
                f"→ MCU 调用异常 ({label}): {e}  "
                f"(x={x:.4f} y={y:.4f} z={z:.4f})")
        finally:
            if on_complete is not None:
                on_complete(success)


def main(args=None):
    rclpy.init(args=args)
    node = HandEyeBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
