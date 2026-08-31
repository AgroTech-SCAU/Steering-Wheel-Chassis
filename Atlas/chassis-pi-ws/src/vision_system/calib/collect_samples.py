#!/usr/bin/env python3
"""
============================================================
  手眼标定数据采集
============================================================
用法:
  1. 先完成相机标定（camera_calib.py），得到 camera_intrinsics.yaml
  2. 填好下面的棋盘格参数
  3. 运行: python collect_samples.py
  4. 每次移动机械臂后输入位姿，三种方式任选：
     - ros 模式: 自动订阅 /arm/pose，按回车即可 (推荐!)
     - pose 模式: x y z qx qy qz qw (7个数, xyz 米 + 四元数 xyzw)
     - joints 模式: q0 q1 q2 q3 q4 (5个数, 关节角 弧度)
  5. 确认棋盘格角点检测成功（画面显示绿色），回车采集
  6. 采集 15~20 组后，按 Q 退出
  7. 数据保存在 samples.yaml
============================================================
"""

import sys
import os
import argparse
import select
import time
import threading

import cv2
import numpy as np
import yaml
from datetime import datetime

# ROS2
import rclpy
from rclpy.node import Node

# 把当前目录加到路径
_here = os.path.dirname(os.path.abspath(__file__))
if _here not in sys.path:
    sys.path.insert(0, _here)

_vision_source = os.path.normpath(os.path.join(_here, "..", "vison_topic"))
if _vision_source not in sys.path:
    sys.path.insert(0, _vision_source)

from fk_utils import quaternion_to_matrix, make_transform, fk_gripper_in_base, rotation_angle_deg
from calib_utils import (find_chessboard, refine_chessboard_corners,
                          validate_chessboard_geometry, make_chessboard_objp,
                          compute_laplacian_variance, compute_corner_quality)
from vison_topic.camera_utils import DEFAULT_CAMERA_SETTINGS, open_project_camera

# ── ROS2 可选导入 (仅 ros 模式需要) ──
_ROS_AVAILABLE = False
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from geometry_msgs.msg import PoseStamped
    _ROS_AVAILABLE = True
except ImportError:
    pass


# ╔══════════════════════════════════════════════════════════╗
# ║           ★ 在这里填写你的参数 ★                         ║
# ╚══════════════════════════════════════════════════════════╝

# --- 棋盘格 ---
# 和相机标定时用的同一张棋盘格！
CHESSBOARD_COLS = 11   # 横向内角点数，例如 9
CHESSBOARD_ROWS = 8   # 纵向内角点数，例如 6
SQUARE_SIZE_MM  = 15   # 每个方格边长 (毫米)，例如 25

# --- 相机 ---
CAMERA_INDEX = 0


# --- 相机内参文件 (相机标定步骤的输出) ---
INTRINSICS_FILE = os.path.join(_here, "camera_intrinsics.yaml")
DEBUG_IMAGE_FILE = os.path.join(_here, "debug", "calib_debug.jpg")

# --- 手眼模式 ---
HAND_EYE_MODE = "eye_in_hand"   # "eye_in_hand" (相机在机械臂上) 或 "eye_to_hand"

# --- PnP 质量控制 ---
MAX_REPROJ_ERROR_PX = 0.40

# --- 图像清晰度检测 (Laplacian 方差) ---
# 运动模糊 / 失焦会导致 PnP 精度严重下降，采集前自动检测
MIN_LAPLACIAN_GOOD  = 120   # >= 此值: 清晰，放心采集
MIN_LAPLACIAN_OK    = 80    # >= 此值: 可接受但偏模糊；< 此值直接拒绝

# --- 亚像素角点质量 ---
# cornerSubPix 前后角点移动的 RMS，用于判断角点检测是否稳定
MAX_CORNER_RMS_PX = 0.2    # RMS > 此值说明角点不稳定，拒绝采集

# --- 输入模式 (必须选一个，采集过程中不要混用！) ---
# "ros"    → 自动订阅 ROS2 话题 /arm/pose (PoseStamped)，按回车即采集 (推荐!)
#            数据来源: 和 solve.py 使用相同的 MCU FK
# "pose"   → 每次输入 7 个数字: x y z qx qy qz qw
#            xyz 单位米，四元数 xyzw
#            数据来源: ros2 topic echo /arm/pose --once
# "joints" → 每次输入 5 个数字: q0 q1 q2 q3 q4
#            关节角单位弧度
#            数据来源: ros2 topic echo /arm/joint_states --once
# ⚠️  三种模式得到的是同一物理量(base→gripper)，但来源不同:
#     "ros" 和 "pose" = MCU 内部 FK 算出的末端位姿 (一致)
#     "joints" = Python MDH 正运动学算出的末端位姿
#     如果 MCU 和 Python 的 DH 参数不完全一致，两种结果会不同！
#     因此请选定一种模式，整个采集过程不要切换。
INPUT_MODE = "ros"             # "ros" / "pose" / "joints"

# --- ROS2 话题 (仅 INPUT_MODE="ros" 时生效) ---
POSE_TOPIC = "/arm/pose"       # geometry_msgs/PoseStamped

# --- 输出 ---
OUTPUT_FILE = os.path.join(_here, "samples.yaml")


# ╔══════════════════════════════════════════════════════════╗
# ║            下面不用改                                   ║
# ╚══════════════════════════════════════════════════════════╝

# ── ROS2 位姿订阅 ──

# 同步参数
MAX_FRAME_POSE_DT_MS = 80      # 帧与位姿时间差超过此值给出警告
POSE_HISTORY_SIZE = 100         # 保留最近 N 条位姿用于时间戳匹配
STABILITY_WINDOW_MS = 500     # 稳定性检查窗口
STABILITY_POS_MM = 3.0        # 稳定窗口内位置波动阈值 (mm)
STABILITY_ROT_DEG = 1.5      # 稳定窗口内旋转波动阈值 (度)

class LatestPose:
    """线程安全的位姿缓存，带 ROS 时间戳和稳定性检查.

    时间戳统一使用 ROS 时钟 (PoseStamped.header.stamp), 避免 time.time()
    与 ROS 时间之间的漂移导致帧-位姿同步误差。
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._position = None      # (x, y, z)
        self._quaternion = None    # (qx, qy, qz, qw)
        self._count = 0
        self._timestamp = 0.0      # 最新位姿的 ROS 时间戳 (float 秒)
        self._history = []         # [(ros_timestamp_s, x, y, z, qx, qy, qz, qw), ...]

    def update(self, x, y, z, qx, qy, qz, qw, stamp=None):
        """更新最新位姿.

        Args:
            stamp: ROS 时间戳 (float 秒), 来自 PoseStamped.header.stamp.
                   若为 None 则回退到 time.time() (非 ROS 模式).
        """
        ts = stamp if stamp is not None else time.time()
        with self._lock:
            self._position = (x, y, z)
            self._quaternion = (qx, qy, qz, qw)
            self._timestamp = ts
            self._count += 1
            self._history.append((ts, x, y, z, qx, qy, qz, qw))
            if len(self._history) > POSE_HISTORY_SIZE:
                self._history = self._history[-POSE_HISTORY_SIZE:]

    def get(self):
        with self._lock:
            if self._position is None:
                return None, None, 0
            return self._position, self._quaternion, self._count

    def get_with_ts(self):
        """返回 (pos, quat, count, ros_timestamp_s)"""
        with self._lock:
            if self._position is None:
                return None, None, 0, 0.0
            return self._position, self._quaternion, self._count, self._timestamp

    def get_closest(self, target_ts):
        """根据时间戳找到最接近的位姿，返回 (pos, quat, dt_ms) 或 (None, None, inf)

        target_ts 必须与历史中的时间戳使用同一时钟源 (ROS 时钟).
        """
        with self._lock:
            if not self._history:
                return None, None, float("inf")
            best = None
            best_dt = float("inf")
            for entry in self._history:
                dt = abs(entry[0] - target_ts)
                if dt < best_dt:
                    best_dt = dt
                    best = entry
            if best is None:
                return None, None, float("inf")
            return (best[1], best[2], best[3]), (best[4], best[5], best[6], best[7]), best_dt * 1000

    def is_stable(self):
        """检查最近 STABILITY_WINDOW_MS 内位姿是否稳定.

        使用历史中最新的 ROS 时间戳作为时间基准, 确保与 pose 消息时钟一致.
        """
        with self._lock:
            if len(self._history) < 3:
                return False, 0.0, 0.0
            # 使用最新位姿的时间戳 (ROS 时钟) 作为当前时间基准
            now = self._history[-1][0]
            window = [(t, x, y, z, qx, qy, qz, qw)
                      for t, x, y, z, qx, qy, qz, qw in self._history
                      if now - t <= STABILITY_WINDOW_MS / 1000.0]
            if len(window) < 2:
                return False, 0.0, 0.0
            positions = np.array([(x, y, z) for _, x, y, z, _, _, _, _ in window])
            max_disp = np.max(np.linalg.norm(positions - positions[0], axis=1)) * 1000  # mm
            # 旋转稳定性: 用第一个位姿为参考
            from fk_utils import quaternion_to_matrix, rotation_angle_deg
            R0 = quaternion_to_matrix(window[0][4], window[0][5], window[0][6], window[0][7])
            max_rot = 0.0
            for _, _, _, _, qx, qy, qz, qw in window[1:]:
                R = quaternion_to_matrix(qx, qy, qz, qw)
                ang = rotation_angle_deg(R0.T @ R)
                if ang > max_rot:
                    max_rot = ang
            stable = (max_disp <= STABILITY_POS_MM) and (max_rot <= STABILITY_ROT_DEG)
            return stable, max_disp, max_rot

    def is_ready(self):
        with self._lock:
            return self._position is not None


class PoseSubscriber(Node):
    """订阅 /arm/pose 的 ROS2 节点"""
    def __init__(self, topic, latest: LatestPose):
        super().__init__("handeye_collect_samples")
        self._latest = latest
        self.create_subscription(PoseStamped, topic, self._on_pose, 10)
        self.get_logger().info(f"手眼标定采集节点启动, 订阅 {topic}")

    def _on_pose(self, msg: PoseStamped):
        # 使用 ROS Header Stamp (而非 time.time()) — 与相机帧时间戳同一时钟源
        ros_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        self._latest.update(
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
            msg.pose.orientation.x, msg.pose.orientation.y,
            msg.pose.orientation.z, msg.pose.orientation.w,
            stamp=ros_sec)


def start_ros_spin(node):
    """在后台线程中运行 rclpy spin"""
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    except Exception as e:
        print(f"\n  ⚠ ROS2 spin 异常: {e}")


def _ros_now_sec(node):
    """获取当前 ROS 时间 (float 秒), 与 PoseStamped.header.stamp 同一时钟源.

    若 ROS 不可用则回退到 time.time().
    """
    if node is not None:
        try:
            t = node.get_clock().now()
            return float(t.nanoseconds) * 1e-9
        except Exception:
            pass
    return time.time()


def validate_params():
    missing = []
    if CHESSBOARD_COLS is None:
        missing.append("CHESSBOARD_COLS (横向内角点数)")
    if CHESSBOARD_ROWS is None:
        missing.append("CHESSBOARD_ROWS (纵向内角点数)")
    if SQUARE_SIZE_MM is None:
        missing.append("SQUARE_SIZE_MM (方格边长 mm)")
    if INPUT_MODE not in ("pose", "joints", "ros"):
        missing.append("INPUT_MODE (必须是 'ros' / 'pose' / 'joints')")
    if INPUT_MODE == "ros" and not _ROS_AVAILABLE:
        missing.append("INPUT_MODE='ros' 但 ROS2 (rclpy) 未安装，请 pip install rclpy 或改用 'pose'/'joints'")
    if missing:
        print("\n❌ 以下参数还没填写或填写错误：")
        for m in missing:
            print(f"   - {m}")
        return False
    if not os.path.exists(INTRINSICS_FILE):
        print(f"\n❌ 找不到相机内参文件: {INTRINSICS_FILE}")
        print(f"   请先运行 camera_calib.py 完成相机标定")
        return False
    return True


def load_intrinsics(path):
    """加载相机内参，并返回标定图像尺寸。"""
    with open(path, encoding="utf-8") as f:
        data = yaml.safe_load(f)

    mtx = np.array(
        data["camera_matrix"]["data"],
        dtype=np.float64,
    ).reshape(3, 3)

    dist = np.array(
        data["distortion_coefficients"]["data"],
        dtype=np.float64,
    ).reshape(-1, 1)

    calib_board = data.get("chessboard", "")
    calib_sq = data.get("square_size_mm", None)
    current_board = f"{CHESSBOARD_COLS}x{CHESSBOARD_ROWS}"

    if calib_board and calib_board != current_board:
        raise ValueError(
            f"棋盘格参数不一致: "
            f"内参文件={calib_board}, 当前={current_board}"
        )

    if calib_sq is not None and abs(
        float(calib_sq) - float(SQUARE_SIZE_MM)
    ) > 1e-6:
        raise ValueError(
            f"方格尺寸不一致: "
            f"内参文件={calib_sq}mm, 当前={SQUARE_SIZE_MM}mm"
        )

    image_width = data.get("image_width")
    image_height = data.get("image_height")

    calib_size = None
    if image_width is not None and image_height is not None:
        calib_size = (
            int(image_width),
            int(image_height),
        )

    return mtx, dist, calib_size

def solve_pnp(obj_points_3d, img_points_2d, mtx, dist):
    """PnP 求解标定板在相机坐标系下的位姿, 返回 (4x4变换矩阵, 重投影误差, rvec, tvec)

    使用 solvePnPGeneric 获取多个候选解 → 比较重投影误差 → 取最优。
    方法级联: IPPE (平面最优, 2解) → SQPNP (全局搜索) → ITERATIVE (经典迭代)。
    平面棋盘格特别适合 IPPE，因为 IPPE 专为共面点设计。
    """
    # ── 方法级联 ──
    method_chain = []
    # IPPE: Infinitesimal Plane-Based Pose Estimation, 专为平面标定板设计, 返回 2 个解
    method_chain.append((cv2.SOLVEPNP_IPPE, "IPPE"))
    # SQPNP: 全局最优解, Softposit 的改进版, 对噪声鲁棒
    if hasattr(cv2, 'SOLVEPNP_SQPNP'):
        method_chain.append((cv2.SOLVEPNP_SQPNP, "SQPNP"))
    # ITERATIVE: 经典 Levenberg-Marquardt 迭代, 永远作为最终保底
    method_chain.append((cv2.SOLVEPNP_ITERATIVE, "ITERATIVE"))

    best_error = float("inf")
    best_result = None    # (rvec, tvec)
    best_method = None

    for method_flag, method_name in method_chain:
        try:
            # solvePnPGeneric 在不同 OpenCV 版本中返回 3 或 4 个值
            result = cv2.solvePnPGeneric(
                obj_points_3d, img_points_2d, mtx, dist,
                flags=method_flag)
            retval, rvecs, tvecs = result[0], result[1], result[2]
        except cv2.error:
            continue

        if retval is None or retval == 0:
            continue

        # rvecs/tvecs 在不同 OpenCV 版本中可能是 ndarray (N,3,1) 或 list
        if isinstance(rvecs, np.ndarray):
            n_solutions = rvecs.shape[0] if rvecs.ndim >= 2 else 0
        elif isinstance(rvecs, (list, tuple)):
            n_solutions = len(rvecs)
        else:
            n_solutions = 0

        for i in range(n_solutions):
            # 提取单个解
            if isinstance(rvecs, np.ndarray):
                rv = rvecs[i].reshape(3, 1).astype(np.float64)
                tv = tvecs[i].reshape(3, 1).astype(np.float64)
            else:
                rv = np.array(rvecs[i], dtype=np.float64).reshape(3, 1)
                tv = np.array(tvecs[i], dtype=np.float64).reshape(3, 1)

            # 物理合理性: 标定板必须在相机前方 (z > 0)
            if tv[2, 0] <= 0:
                continue

            # 重投影误差
            projected, _ = cv2.projectPoints(obj_points_3d, rv, tv, mtx, dist)
            error = float(np.sqrt(np.mean(
                np.sum((projected.reshape(-1, 2) - img_points_2d.reshape(-1, 2))**2, axis=1)
            )))

            if error < best_error:
                best_error = error
                best_result = (rv, tv)
                best_method = method_name

    # ── 如果 solvePnPGeneric 全部失败，回退到标准 solvePnP ──
    if best_result is None:
        try:
            ok, rv, tv = cv2.solvePnP(
                obj_points_3d, img_points_2d, mtx, dist,
                flags=cv2.SOLVEPNP_ITERATIVE)
            if not ok:
                return None, None, None, None
            best_result = (rv, tv)
            best_method = "ITERATIVE(fallback)"
            projected, _ = cv2.projectPoints(obj_points_3d, rv, tv, mtx, dist)
            best_error = float(np.sqrt(np.mean(
                np.sum((projected.reshape(-1, 2) - img_points_2d.reshape(-1, 2))**2, axis=1)
            )))
        except cv2.error:
            return None, None, None, None
    else:
        rv, tv = best_result

    # 用最佳候选做一次统一的像素级 LM 精化；IPPE/SQPNP 负责避开坏初值，
    # LM 负责在当前内参与畸变模型下进一步降低角点残差。
    if hasattr(cv2, "solvePnPRefineLM"):
        try:
            rv_refined, tv_refined = cv2.solvePnPRefineLM(
                obj_points_3d, img_points_2d, mtx, dist, rv, tv)
            if tv_refined[2, 0] > 0:
                projected, _ = cv2.projectPoints(
                    obj_points_3d, rv_refined, tv_refined, mtx, dist)
                refined_error = float(np.sqrt(np.mean(np.sum(
                    (projected.reshape(-1, 2)
                     - img_points_2d.reshape(-1, 2)) ** 2,
                    axis=1))))
                if np.isfinite(refined_error) and refined_error <= best_error:
                    rv, tv = rv_refined, tv_refined
                    best_error = refined_error
        except cv2.error:
            pass

    R, _ = cv2.Rodrigues(rv)
    T = make_transform(R, tv.reshape(3))
    return T, best_error, rv, tv


def _input_reader(hint, result, done_event):
    """在后台线程中读取用户输入，避免阻塞主线程的画面刷新"""
    try:
        result[0] = input(hint).strip()
    except (EOFError, KeyboardInterrupt):
        result[0] = 'q'
    done_event.set()


def _sample_motion_coverage(sample_entries):
    """返回当前采样集合的旋转激励覆盖度。"""
    poses = [np.asarray(s["gripper_in_base"], dtype=np.float64)
             for s in sample_entries]
    rvecs, angles = [], []
    for i in range(len(poses)):
        for j in range(i + 1, len(poses)):
            R_rel = poses[j][:3, :3].T @ poses[i][:3, :3]
            rv = cv2.Rodrigues(R_rel)[0].ravel()
            angle = float(np.linalg.norm(rv) * 180.0 / np.pi)
            if angle > 0.05:
                rvecs.append(rv)
                angles.append(angle)
    if not angles:
        return 0.0, 0.0, 0.0
    s = np.linalg.svd(np.asarray(rvecs), compute_uv=False)
    axis_ratio = float(s[1] / s[0]) if len(s) >= 2 and s[0] > 1e-12 else 0.0
    return float(np.median(angles)), float(np.max(angles)), axis_ratio


def _nearest_pose_distance(samples, candidate):
    """候选位姿到已有样本的最近 (平移mm, 旋转deg)。"""
    best = None
    for sample in samples:
        old = np.asarray(sample["gripper_in_base"], dtype=np.float64)
        trans_mm = float(np.linalg.norm(
            old[:3, 3] - candidate[:3, 3]) * 1000.0)
        rot_deg = rotation_angle_deg(
            old[:3, :3].T @ candidate[:3, :3])
        score = trans_mm / 3.0 + rot_deg / 0.5
        if best is None or score < best[0]:
            best = (score, trans_mm, rot_deg)
    return (best[1], best[2]) if best is not None else (float("inf"), float("inf"))


def main():
    parser = argparse.ArgumentParser(description="手眼标定数据采集")
    parser.add_argument("--headless", action="store_true",
                        help="无头模式: 不弹 OpenCV 窗口，截图保存到 calib/debug/calib_debug.jpg")
    parser.add_argument("--minimal", action="store_true",
                        help="极简模式: 跳过所有质量检查，只要棋盘格检测成功就保存")
    args = parser.parse_args()

    if not validate_params():
        return

    square_size_m = SQUARE_SIZE_MM / 1000.0
    pattern_size = (CHESSBOARD_COLS, CHESSBOARD_ROWS)
    total_corners = CHESSBOARD_COLS * CHESSBOARD_ROWS

    # 生成标定板角点的 3D 坐标
    objp = make_chessboard_objp(CHESSBOARD_COLS, CHESSBOARD_ROWS, square_size_m)

    # 加载相机内参
    mtx, dist, calib_image_size = load_intrinsics(INTRINSICS_FILE)

    # ── ROS2 初始化 (ros 模式) ──
    ros_node = None
    ros_spin_thread = None
    latest_pose = None
    if INPUT_MODE == "ros":
        rclpy.init(args=[sys.argv[0]])  # 只传程序名，避免 --headless 等自定义参数被 rclpy 误解析
        latest_pose = LatestPose()
        ros_node = PoseSubscriber(POSE_TOPIC, latest_pose)
        ros_spin_thread = threading.Thread(
            target=start_ros_spin, args=(ros_node,), daemon=True)
        ros_spin_thread.start()
        # 等待第一条消息
        print("  等待 /arm/pose 消息...")
        deadline = datetime.now().timestamp() + 5.0
        while not latest_pose.is_ready():
            if datetime.now().timestamp() > deadline:
                print("  ⚠ 5 秒内未收到 /arm/pose 消息，继续等待 (请确认 ROS2 正在发布位姿)")
                deadline = float("inf")
            if args.headless:
                time.sleep(0.1)
            else:
                cv2.waitKey(100)

    # 打开相机
    # 直接使用内参文件记录的分辨率，禁止手眼采集与相机标定使用不同尺寸。
    default_size = (
        int(DEFAULT_CAMERA_SETTINGS["width"]),
        int(DEFAULT_CAMERA_SETTINGS["height"]),
    )
    request_w, request_h = calib_image_size or default_size
    cap, camera_info = open_project_camera(
        CAMERA_INDEX,
        width=int(request_w),
        height=int(request_h),
        fps=float(DEFAULT_CAMERA_SETTINGS["fps"]),
        strict_resolution=True,
        log=print,
    )
    actual_size = (camera_info.width, camera_info.height)
    actual_w, actual_h = actual_size

    print(f"  当前相机分辨率: {actual_w}*{actual_h}")

    if calib_image_size is not None:
        print(
            f"  内参标定分辨率: "
            f"{calib_image_size[0]}*{calib_image_size[1]}"
        )

        if actual_size != calib_image_size:
            cap.release()
            raise RuntimeError(
                f"当前分辨率 {actual_w}*{actual_h} "
                f"与内参标定分辨率 "
                f"{calib_image_size[0]}*"
                f"{calib_image_size[1]} 不一致"
            )
    else:
        print("  ⚠ 内参文件未记录图像分辨率，无法自动校验")

    samples = []
    last_input_parts = None   # 上次输入的数字 (用于回车沿用)
    prev_target_R = None      # 上一个样本的 target_to_camera 旋转矩阵 (翻转检测)

    # 非阻塞输入: 后台线程调用 input()，主线程持续刷新画面
    input_value = [None]
    input_ready = threading.Event()
    waiting_for_input = False

    print(f"\n{'='*55}")
    print(f"  手眼标定数据采集")
    print(f"  模式: {HAND_EYE_MODE}")
    print(f"  棋盘格: {CHESSBOARD_COLS}×{CHESSBOARD_ROWS} 内角点, {SQUARE_SIZE_MM}mm")
    print(f"  内参: {INTRINSICS_FILE}")
    if INPUT_MODE == "ros":
        print(f"  位姿: 自动订阅 {POSE_TOPIC}")
    print(f"{'='*55}")
    print(f"\n  操作说明:")
    print(f"    1. 移动机械臂到新位置")
    if INPUT_MODE == "ros":
        print(f"    2. 位姿自动从 {POSE_TOPIC} 获取 (画面右下角显示实时位姿)")
    elif INPUT_MODE == "joints":
        print(f"    2. 终端输入: q0 q1 q2 q3 q4")
        print(f"       关节角单位弧度，自动正运动学算末端位姿")
        print(f"       示例: 0.17 -0.35 0.52 0.70 -0.87")
    else:
        print(f"    2. 终端输入: x y z qx qy qz qw")
        print(f"       xyz 单位米, 四元数 xyzw")
        print(f"       示例: 0.35 0.15 0.25 0.0 0.0 0.707 0.707")
    print(f"    3. 确认画面显示绿色角点 + 棋盘质量 OK")
    print(f"    4. 按 回车 采集")
    print(f"    5. 重复 15~20 次")
    print(f"    6. 按 Q 退出并保存")
    if INPUT_MODE == "ros":
        print(f"  ✨ ROS 模式: 每次只需按回车，无需手动输入位姿！")
    print(f"  (直接按回车沿用上一次的位姿)\n")

    while True:
        pnp_corners_used = None  # 每帧初始化，确保不会残留旧值

        ok, frame = cap.read()
        # 使用 ROS 时钟 (而非 time.time()) 打时间戳, 与 PoseStamped.header.stamp 同一时钟源
        frame_ts = _ros_now_sec(ros_node)
        if not ok:
            print("读取摄像头失败")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        display = frame.copy()

        # ── 图像清晰度检测 (Laplacian 方差) ──
        laplacian_var = compute_laplacian_variance(gray)

        # 检测棋盘格
        found, corners = find_chessboard(gray, pattern_size)

        target_to_camera = None
        reproj_error = None
        pnp_ok = False
        flip_warning = False    # 棋盘格 180° 翻转检测
        flip_angle = 0
        corner_rms = None       # 亚像素角点 RMS
        corner_mean_move = None # 亚像素角点平均移动

        if found and corners is not None:
            cv2.drawChessboardCorners(
                display,
                pattern_size,
                corners,
                True,
            )

            corners_before, corners_sub, _ = refine_chessboard_corners(
                gray, corners, pattern_size)

            # 亚像素角点质量
            corner_mean_move, corner_rms = (
                compute_corner_quality(
                    corners_before,
                    corners_sub,
                )
            )

            grid_ok, grid_diag = validate_chessboard_geometry(
                corners_sub, pattern_size)

            if grid_ok:
                (
                    target_to_camera,
                    reproj_error,
                    rvec,
                    tvec,
                ) = solve_pnp(
                    objp,
                    corners_sub,
                    mtx,
                    dist,
                )
            else:
                target_to_camera = None
                reproj_error = None
                rvec = tvec = None
                board_status = (
                    f"⚠ 角点网格异常: min="
                    f"{grid_diag.get('min_spacing_px', 0):.1f}px")
                board_color = (0, 0, 255)

            if (
                target_to_camera is not None
                and rvec is not None
            ):
                pnp_corners_used = corners_sub  # 非 ROS 模式的 PnP 角点
                cv2.drawFrameAxes(
                    display,
                    mtx,
                    dist,
                    rvec,
                    tvec,
                    square_size_m * 3,
                    3,
                )

                pnp_ok = True

                # 只检测姿态跳变，不自动修改 PnP 结果
                current_R = target_to_camera[:3, :3]

                if prev_target_R is not None:
                    R_diff = (
                        prev_target_R.T
                        @ current_R
                    )

                    flip_angle = rotation_angle_deg(
                        R_diff
                    )

                    if flip_angle > 120.0:
                        flip_warning = True

                board_status = (
                    f"✓ 标定板 OK | "
                    f"重投影={reproj_error:.3f}px"
                )
                board_color = (0, 255, 0)

            else:
                board_status = "⚠ PnP 失败"
                board_color = (0, 165, 255)
        else:
            board_status = "✗ 未检测到棋盘格"
            board_color = (0, 0, 255)

        # ── 实时质量指标 ──
        fx_approx = mtx[0, 0]  # 相机内参 fx
        if pnp_ok and target_to_camera is not None:
            dist_mm = target_to_camera[2, 3] * 1000  # 相机到棋盘距离
            px_per_sq = fx_approx * SQUARE_SIZE_MM / dist_mm if dist_mm > 0 else 0

            if px_per_sq >= 20:
                dist_color = (0, 255, 0)       # 绿: 优
                dist_label = "优"
            elif px_per_sq >= 15:
                dist_color = (0, 215, 255)     # 黄: 可接受
                dist_label = "可"
            else:
                dist_color = (0, 0, 255)       # 红: 太远
                dist_label = "差"

        # 显示信息
        y = 30
        if flip_warning:
            cv2.putText(display, "⚠ 姿态跳变过大，当前帧不会保存", (12, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
            y += 28
        cv2.putText(display, board_status, (12, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, board_color, 2)
        y += 30
        if pnp_ok and target_to_camera is not None:
            qual_txt = (f"距离={dist_mm:.0f}mm | {px_per_sq:.0f}px/格({dist_label}) | "
                        f"重投影={reproj_error:.3f}px")
            cv2.putText(display, qual_txt, (12, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, dist_color, 2)
            y += 30

        # ── 清晰度 + 角点质量 ──
        y_qual = y
        # Laplacian 清晰度
        if laplacian_var >= MIN_LAPLACIAN_GOOD:
            lap_color = (0, 255, 0)      # 绿: 清晰
            lap_label = "清晰"
        elif laplacian_var >= MIN_LAPLACIAN_OK:
            lap_color = (0, 215, 255)    # 黄: 可接受
            lap_label = "偏模糊"
        else:
            lap_color = (0, 0, 255)      # 红: 拒绝
            lap_label = "模糊!不合格"
        cv2.putText(display, f"清晰度(Laplacian): {laplacian_var:.0f} ({lap_label})",
                    (12, y_qual), cv2.FONT_HERSHEY_SIMPLEX, 0.5, lap_color, 1)
        y_qual += 22
        # 角点 RMS
        if corner_rms is not None:
            if corner_rms <= MAX_CORNER_RMS_PX:
                crms_color = (0, 255, 0)
                crms_label = "OK"
            else:
                crms_color = (0, 0, 255)
                crms_label = "NG!"
            cv2.putText(display, f"角点RMS: {corner_rms:.3f}px ({crms_label}) | 移动={corner_mean_move:.3f}px",
                        (12, y_qual), cv2.FONT_HERSHEY_SIMPLEX, 0.5, crms_color, 1)
            y_qual += 22
        y = y_qual

        # ── ROS 模式: 显示实时位姿 + 同步状态 ──
        if INPUT_MODE == "ros" and latest_pose is not None:
            pos, quat, count, ts = latest_pose.get_with_ts()
            h, w = display.shape[:2]
            if pos is not None:
                # 位姿行
                ros_txt = (f"ROS: x={pos[0]:.4f} y={pos[1]:.4f} z={pos[2]:.4f} | "
                           f"q=({quat[0]:.3f},{quat[1]:.3f},{quat[2]:.3f},{quat[3]:.3f}) | "
                           f"收到{count}条")
                cv2.putText(display, ros_txt, (12, h - 28),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 255), 1)
                # 稳定性状态行
                is_stable, disp, rot = latest_pose.is_stable()
                # 帧-位姿时间差 (显示帧 vs 最新位姿), 均使用 ROS 时钟
                dt_frame_pose_ms = abs(_ros_now_sec(ros_node) - ts) * 1000
                if is_stable:
                    sync_txt = (f"✓ 静止 | 波动: {disp:.1f}mm {rot:.2f}° | "
                               f"帧-位姿 ≈{dt_frame_pose_ms:.0f}ms")
                    sync_color = (0, 255, 0)  # 绿色
                else:
                    if disp > STABILITY_POS_MM:
                        sync_txt = (f"⚠ 运动中! 位移{disp:.1f}mm | "
                                   f"等待静止后采集")
                    else:
                        sync_txt = (f"⚠ 旋转中! {rot:.2f}° | 等待静止后采集")
                    sync_color = (0, 0, 255)  # 红色
                cv2.putText(display, sync_txt, (12, h - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.42, sync_color, 1)
            else:
                ros_txt = "ROS: 等待 /arm/pose ..."
                cv2.putText(display, ros_txt, (12, h - 12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)

        if last_input_parts is not None:
            if INPUT_MODE == 'joints':
                txt = (f"上次关节角: q0={last_input_parts[0]:.4f} q1={last_input_parts[1]:.4f} "
                       f"q2={last_input_parts[2]:.4f} q3={last_input_parts[3]:.4f} "
                       f"q4={last_input_parts[4]:.4f} rad")
            else:
                txt = (f"上次位姿: x={last_input_parts[0]:.3f} y={last_input_parts[1]:.3f} "
                       f"z={last_input_parts[2]:.3f} "
                       f"qx={last_input_parts[3]:.4f} qy={last_input_parts[4]:.4f} "
                       f"qz={last_input_parts[5]:.4f} qw={last_input_parts[6]:.4f}")
            cv2.putText(display, txt, (12, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
            y += 25

        if INPUT_MODE == "ros":
            hint_txt = f"已采集: {len(samples)} 组 | 按 回车 采集当前位姿 | Q 退出"
        else:
            hint_txt = f"已采集: {len(samples)} 组 | 在终端输入位姿后按回车"
        cv2.putText(display, hint_txt, (12, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        if args.headless:
            # ── 无头模式: 保存截图 + 终端输出 ──
            os.makedirs(os.path.dirname(DEBUG_IMAGE_FILE), exist_ok=True)
            cv2.imwrite(DEBUG_IMAGE_FILE, display,
                        [cv2.IMWRITE_JPEG_QUALITY, 85])
            # 终端状态行 (仅在非输入状态时刷新，避免覆盖用户输入)
            if not waiting_for_input:
                if pnp_ok and target_to_camera is not None:
                    dd = target_to_camera[2, 3] * 1000
                    pps = mtx[0, 0] * SQUARE_SIZE_MM / dd if dd > 0 else 0
                    if pps >= 20:
                        qlabel = "优"
                    elif pps >= 15:
                        qlabel = "可"
                    else:
                        qlabel = "差"
                    # ROS 模式: 附加稳定性信息
                    sync_extra = ""
                    if INPUT_MODE == "ros" and latest_pose is not None:
                        is_stable, disp, rot = latest_pose.is_stable()
                        if is_stable:
                            sync_extra = f" | 静止✓"
                        else:
                            sync_extra = f" | 运动中!({disp:.1f}mm)"
                    stat = (f"\r  {'⚠ 姿态跳变' if flip_warning else '✓'}  "
                            f"距离={dd:.0f}mm | {pps:.0f}px/格({qlabel}) | "
                            f"重投影={reproj_error:.3f}px | "
                            f"清晰度={laplacian_var:.0f} | "
                            f"角点RMS={corner_rms:.3f}px | "
                            f"已采集:{len(samples)}组{sync_extra} | "
                            f"[回车采集 / Q退出]   ")
                else:
                    stat = (f"\r  ✗ 未检测到棋盘格 | "
                            f"已采集:{len(samples)}组 | "
                            f"[回车采集 / Q退出]   ")
                sys.stdout.write(stat)
                sys.stdout.flush()
            time.sleep(0.05)
        else:
            cv2.imshow("Hand-Eye Data Collection", display)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == ord('Q') or key == 27:
                break

        # ── 非阻塞输入：input() 放在后台线程，画面不会冻结 ──
        if not waiting_for_input:
            if args.headless:
                # ── 无头模式: 使用 select 检测的空行/输入 ──
                user_input = None
                # 已经在 select 中读取过，这里触发采集流程
                if select.select([sys.stdin], [], [], 0)[0]:
                    line = sys.stdin.readline().strip()
                    if line.lower() in ('q', 'quit', 'exit'):
                        print("\n  收到退出信号")
                        break
                    user_input = line
                else:
                    continue  # 没有输入，继续循环

                # 有输入 → 模拟 input_ready
                input_value[0] = user_input
                input_ready.set()
                waiting_for_input = True
            else:
                print(f"\n--- 样本 #{len(samples)+1} ---")
                if INPUT_MODE == "ros":
                    if latest_pose.is_ready():
                        pos, quat, count = latest_pose.get()
                        print(f"  ROS 实时位姿: x={pos[0]:.4f} y={pos[1]:.4f} z={pos[2]:.4f} "
                              f"qx={quat[0]:.4f} qy={quat[1]:.4f} qz={quat[2]:.4f} qw={quat[3]:.4f}")
                        print(f"  (已收到 {count} 条消息)")
                    else:
                        print(f"  ⚠ 尚未收到 /arm/pose 消息")
                    hint = "[回车采集] / Q 退出: "
                elif last_input_parts is not None:
                    if INPUT_MODE == 'joints':
                        hint = (f"q0~q4 [回车沿用上次: "
                                f"{last_input_parts[0]:.4f} {last_input_parts[1]:.4f} "
                                f"{last_input_parts[2]:.4f} {last_input_parts[3]:.4f} "
                                f"{last_input_parts[4]:.4f}]: ")
                    else:
                        hint = (f"x y z qx qy qz qw [回车沿用上次: "
                                f"{last_input_parts[0]:.4f} {last_input_parts[1]:.4f} "
                                f"{last_input_parts[2]:.4f} {last_input_parts[3]:.4f} "
                                f"{last_input_parts[4]:.4f} {last_input_parts[5]:.4f} "
                                f"{last_input_parts[6]:.4f}]: ")
                else:
                    if INPUT_MODE == 'joints':
                        hint = "q0 q1 q2 q3 q4 (关节角 rad): "
                    else:
                        hint = "x y z qx qy qz qw (xyz米 + 四元数): "
                input_ready.clear()
                input_value[0] = None
                threading.Thread(target=_input_reader, args=(hint, input_value, input_ready),
                                 daemon=True).start()
                waiting_for_input = True

        if input_ready.is_set():
            waiting_for_input = False
            user_input = input_value[0]

            if user_input is None:
                continue

            # ── 处理输入 ──
            if user_input is not None and user_input.lower() in ('q', 'quit', 'exit'):
                break

            # ── ROS 模式: 帧-位姿时间戳匹配 ──
            if INPUT_MODE == "ros":
                if user_input != '':
                    print(f"  ⚠ ROS 模式下输入数字将被忽略，始终使用 {POSE_TOPIC} 位姿")
                    print(f"     如需手动输入，请修改 INPUT_MODE = 'pose' 后重新运行")
                if not latest_pose.is_ready():
                    print("  ❌ 尚未收到 /arm/pose 消息，请等待后重试")
                    continue

                # ── ① 稳定性检查: 确保机械臂已静止 ──
                stable, max_disp, max_rot = latest_pose.is_stable()
                if not stable:
                    print(
                    f"  ❌ 机械臂未稳定，本次不采集: "
                    f"位移={max_disp:.2f} mm, 旋转={max_rot:.3f}°"
                    )
                    print(" 请等待机械臂完全停止后重新按回车")
                    continue
                # ── ② 清空相机缓冲区，获取最新帧 ──
                # grab() 只取帧不解码，比 read() 更快地排空缓冲区
                flush_count = 6  # USB 相机缓冲可能更深 (V4L2 默认 ~4~8)
                for _ in range(flush_count):
                    cap.grab()
                ok_fresh, frame_fresh = cap.retrieve()
                fresh_ts = _ros_now_sec(ros_node)  # ROS 时钟 — 与位姿时间戳同一时钟源
                if not ok_fresh:
                    print("  ❌ 读取摄像头失败")
                    continue

                # ── ③ 时间戳匹配: 找与帧时间最接近的 ROS 位姿 ──
                pos_matched, quat_matched, dt_ms = latest_pose.get_closest(fresh_ts)
                if pos_matched is None:
                    print("  ❌ 无法匹配位姿 (历史为空)")
                    continue

                if dt_ms > MAX_FRAME_POSE_DT_MS:
                    print(
                    f"  ❌ 帧-位姿时间差过大: "
                    f"{dt_ms:.0f} ms > {MAX_FRAME_POSE_DT_MS} ms"
                    )
                    print("     本次不保存，请确认位姿话题发布频率和机械臂静止状态")
                    continue

                # ── ④ 在刷新帧上重新检测棋盘格和 PnP ──
                # 在刷新帧上重新完成检测和 PnP，禁止使用旧显示帧结果
                gray_fresh = cv2.cvtColor(frame_fresh, cv2.COLOR_BGR2GRAY)
                laplacian_var_fresh = compute_laplacian_variance(gray_fresh)
                corner_rms_fresh = None
                found_fresh, corners_fresh = find_chessboard(gray_fresh, pattern_size)
                if not found_fresh or corners_fresh is None:
                    print("  ❌ 刷新帧未检测到棋盘格，本次不保存")
                    print("     请保持机械臂静止，并调整棋盘位置后重新采集")
                    continue
                else:
                    corners_before_fresh, corners_sub_fresh, _ = refine_chessboard_corners(
                        gray_fresh, corners_fresh, pattern_size)
                    # 亚像素角点质量
                    _, corner_rms_fresh = compute_corner_quality(
                        corners_before_fresh, corners_sub_fresh)
                    grid_ok, grid_diag = validate_chessboard_geometry(
                        corners_sub_fresh, pattern_size)
                    if not grid_ok:
                        print(
                            f"  ❌ 刷新帧角点网格异常: 最小间距="
                            f"{grid_diag.get('min_spacing_px', 0):.1f}px, "
                            f"中位间距="
                            f"{grid_diag.get('median_spacing_px', 0):.1f}px")
                        continue
                    t2c_fresh, err_fresh, rvec_fresh, tvec_fresh = solve_pnp(objp, corners_sub_fresh, mtx, dist)
                    if t2c_fresh is not None:
                        pnp_corners_used = corners_sub_fresh  # 保存原始角点便于重算
                        flip_warning = False
                        flip_angle = 0.0

                        if prev_target_R is not None:
                            current_R = t2c_fresh[:3, :3]
                            R_diff = prev_target_R.T @ current_R
                            flip_angle = rotation_angle_deg(R_diff)

                            if flip_angle > 120.0:
                                print(
                                    f"  ❌ 当前棋盘姿态与上一有效样本相差 "
                                    f"{flip_angle:.1f}°"
                                )
                                print(
                                    "     可能是棋盘角点编号翻转，"
                                    "也可能是机械臂姿态变化过大。"
                                )
                                print(
                                    "     为避免保存错误位姿，本次不保存。"
                                )
                                continue

                        target_to_camera = t2c_fresh
                        reproj_error = err_fresh
                        pnp_ok = True
                        corner_rms = corner_rms_fresh
                        laplacian_var = laplacian_var_fresh
                        
                        print(
                            f"  ✓ 刷新帧 PnP OK | "
                            f"重投影={err_fresh:.3f}px | "
                            f"清晰度={laplacian_var_fresh:.0f} | "
                            f"角点RMS={corner_rms_fresh:.3f}px | "
                            f"帧-位姿时间差={dt_ms:.0f}ms"
                        )
                    else:
                        print("  ❌ 刷新帧 PnP 失败，本次不保存")
                        print("     请调整光照、距离或棋盘姿态后重新采集")
                        continue

                x, y, z = pos_matched
                qx, qy, qz, qw = quat_matched
                R = quaternion_to_matrix(qx, qy, qz, qw)
                gripper_in_base = make_transform(R, [x, y, z])
                parts = [x, y, z, qx, qy, qz, qw]
                last_input_parts = parts

            elif user_input == '':
                # 空回车 = 复用上次位姿 (仅 pose / joints 模式; ros 模式已在上方拦截)
                if last_input_parts is None:
                    print("  第一次必须输入位姿")
                    continue
                print("  ⚠ 复用上次位姿，请确认机械臂未移动")
                parts = last_input_parts
            else:
                try:
                    parts = [float(x) for x in user_input.replace(',', ' ').split()]
                except ValueError:
                    print("  ❌ 格式错误，请输入数字，空格分隔")
                    continue

            # 按 INPUT_MODE 校验 + 构造 gripper_in_base
            # ros 模式已在 ros 分支中构造好 gripper_in_base / parts / last_input_parts
            if INPUT_MODE == "ros":
                pass  # 跳过, 已在 ros 分支中完成
            elif INPUT_MODE == 'pose':
                n = len(parts)
                if n != 7:
                    print(f"  ❌ pose 模式需要 7 个数字 (xyz+四元数)，实际 {n} 个")
                    continue
                x, y, z, qx, qy, qz, qw = parts
                R = quaternion_to_matrix(qx, qy, qz, qw)
                gripper_in_base = make_transform(R, [x, y, z])
                last_input_parts = parts
            elif INPUT_MODE == 'joints':
                n = len(parts)
                if n != 5:
                    print(f"  ❌ joints 模式需要 5 个数字 (关节角 rad)，实际 {n} 个")
                    continue
                gripper_in_base = fk_gripper_in_base(parts)
                last_input_parts = parts

            if not pnp_ok:
                print("  ❌ 标定板检测不成功（画面没有绿色标记），请调整后重试")
                continue

            if not args.minimal:
                if (
                    reproj_error is not None
                    and reproj_error > MAX_REPROJ_ERROR_PX
                ):
                    print(
                        f"  ❌ 重投影误差过大: "
                        f"{reproj_error:.3f}px > "
                        f"{MAX_REPROJ_ERROR_PX:.3f}px"
                    )
                    print(
                        "     本次不保存，请调整光照、对焦、距离或棋盘姿态"
                    )
                    continue

                # ── 图像清晰度检查 ──
                if laplacian_var is not None and laplacian_var < MIN_LAPLACIAN_OK:
                    print(f"  ❌ 图像模糊! Laplacian方差={laplacian_var:.0f} < {MIN_LAPLACIAN_OK}")
                    print(f"     可能原因: 运动模糊 / 失焦 / 光照不足")
                    print(f"     请确保机械臂完全静止后再采集，或调整对焦/补光")
                    continue
                if laplacian_var is not None and laplacian_var < MIN_LAPLACIAN_GOOD:
                    print(f"  ⚠ 图像略模糊 (Laplacian={laplacian_var:.0f} < {MIN_LAPLACIAN_GOOD})，"
                          f"建议重新采集")
                    continue

                # ── 亚像素角点质量检查 ──
                if corner_rms is not None and corner_rms > MAX_CORNER_RMS_PX:
                    print(f"  ❌ 角点质量差! RMS={corner_rms:.3f}px > {MAX_CORNER_RMS_PX}px")
                    print(f"     角点检测不稳定，可能是图像模糊或棋盘格反光")
                    print(f"     请调整姿态/光照后重试")
                    continue

                # ── 距离 / 方格像素检查 ──
                # 手眼标定中 PnP 是关键环节，棋盘格每格至少 20px 才能保证 PnP 精度
                # 低于此值深度估计误差会被放大，导致手眼标定失败
                dist_mm = target_to_camera[2, 3] * 1000
                px_per_sq = mtx[0, 0] * SQUARE_SIZE_MM / dist_mm if dist_mm > 0 else 0
                if px_per_sq < 20:
                    ideal_dist = mtx[0,0] * SQUARE_SIZE_MM / 25  # 目标 25px/格
                    print(f"  ❌ 方格太小 ({px_per_sq:.0f} px/格 < 20)，PnP 精度严重不足!")
                    print(f"     当前距离: {dist_mm:.0f}mm, 建议靠近到 < {ideal_dist:.0f}mm")
                    print(f"     请靠近标定板后重新采集")
                    continue

                # 近重复姿态不会增加可观测信息，反而会让一小片姿态被重复加权。
                nearest_t_mm, nearest_r_deg = _nearest_pose_distance(
                    samples, gripper_in_base)
                if nearest_t_mm < 3.0 and nearest_r_deg < 0.5:
                    print(f"  ❌ 与已有样本过于接近: Δt={nearest_t_mm:.1f}mm, "
                          f"ΔR={nearest_r_deg:.2f}°")
                    print("     请换一个位置，或尽量绕另一个方向做小幅倾斜")
                    continue

            # 保存
            sample_entry = {
                "id": len(samples) + 1,
                "captured_at": datetime.now().isoformat(timespec="seconds"),
                "gripper_in_base": gripper_in_base.tolist(),
                "target_to_camera": target_to_camera.tolist(),
                "robot_pose_input": last_input_parts,
                "input_mode": INPUT_MODE,
                "reprojection_error_px": float(reproj_error),
                "laplacian_variance": float(laplacian_var) if laplacian_var is not None else None,
                "corner_rms_px": float(corner_rms) if corner_rms is not None else None,
                "corners_px": pnp_corners_used.reshape(-1).tolist() if pnp_corners_used is not None else None,
            }
            # ── ROS 模式: 附加同步诊断信息 ──
            if INPUT_MODE == "ros" and latest_pose is not None:
                sample_entry["sync_frame_pose_dt_ms"] = float(
                    dt_ms
                )
                sample_entry["sync_robot_stable"] = bool(
                    stable
                )
                sample_entry["sync_robot_disp_mm"] = float(
                    max_disp
                )
                sample_entry["sync_robot_rot_deg"] = float(
                    max_rot
                )
            samples.append(sample_entry)
            prev_target_R = target_to_camera[:3, :3].copy()  # 更新翻转检测参考

            median_ang, max_ang, axis_ratio = _sample_motion_coverage(samples)
            print(f"     姿态覆盖: 中位相对旋转={median_ang:.1f}°  "
                  f"最大={max_ang:.1f}°  双轴比={axis_ratio:.3f}")
            if len(samples) >= 6 and (max_ang < 8.0 or axis_ratio < 0.05):
                print("     ⚠ 旋转激励仍偏弱：下一组优先绕不同方向倾斜，不必大幅平移")

            if INPUT_MODE == 'joints':
                print(f"  ✅ 样本 #{len(samples)} 已保存")
                print(f"     关节角: q0={last_input_parts[0]:.4f} q1={last_input_parts[1]:.4f} "
                      f"q2={last_input_parts[2]:.4f} q3={last_input_parts[3]:.4f} "
                      f"q4={last_input_parts[4]:.4f} rad")
                t = gripper_in_base[:3, 3]
                print(f"     FK末端: x={t[0]:.4f} y={t[1]:.4f} z={t[2]:.4f} m")
            else:
                print(f"  ✅ 样本 #{len(samples)} 已保存")
                print(f"     末端位姿: x={last_input_parts[0]:.4f} y={last_input_parts[1]:.4f} "
                      f"z={last_input_parts[2]:.4f} "
                      f"qx={last_input_parts[3]:.4f} qy={last_input_parts[4]:.4f} "
                      f"qz={last_input_parts[5]:.4f} qw={last_input_parts[6]:.4f}")
            print(f"     重投影误差: {reproj_error:.3f} px")
            print(f"     清晰度: {laplacian_var:.0f} | 角点RMS: {corner_rms:.3f}px")
            print(f"     (继续输入下一个位姿，或按 Q 退出)")

    cap.release()
    if not args.headless:
        cv2.destroyAllWindows()

    # ── ROS2 清理 ──
    if ros_node is not None:
        ros_node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

    # 保存
    if samples:
        # ── 备份旧文件 ──
        backup = None
        if os.path.exists(OUTPUT_FILE):
            backup_dir = os.path.normpath(
                os.path.join(_here, "..", "backups", "calib"))
            os.makedirs(backup_dir, exist_ok=True)
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            backup = os.path.join(backup_dir, f"samples_{stamp}.yaml")
            try:
                os.replace(OUTPUT_FILE, backup)
                print(f"  📁 旧数据已备份: {backup}")
            except OSError as exc:
                print(f"  ❌ 旧数据备份失败，拒绝覆盖 {OUTPUT_FILE}: {exc}")
                return

        payload = {
            "handeye_mode": HAND_EYE_MODE,
            "sample_count": len(samples),
            "created_at": datetime.now().isoformat(timespec="seconds"),
            "chessboard": f"{CHESSBOARD_COLS}x{CHESSBOARD_ROWS}",
            "square_size_mm": SQUARE_SIZE_MM,
            "intrinsics_file": os.path.relpath(
                INTRINSICS_FILE, start=os.path.dirname(OUTPUT_FILE)),
            "samples": samples,
        }
        try:
            with open(INTRINSICS_FILE, encoding="utf-8") as f:
                intrinsics_snapshot = yaml.safe_load(f)
            payload["intrinsics_created_at"] = intrinsics_snapshot.get("created_at")
            payload["camera_matrix_at_collection"] = intrinsics_snapshot.get("camera_matrix")
            payload["distortion_at_collection"] = intrinsics_snapshot.get(
                "distortion_coefficients")
            payload["image_size_at_collection"] = [
                intrinsics_snapshot.get("image_width"),
                intrinsics_snapshot.get("image_height"),
            ]
        except Exception as e:
            print(f"  ⚠ 无法保存内参快照: {e}")
        with open(OUTPUT_FILE, 'w', encoding="utf-8") as f:
            yaml.safe_dump(payload, f, sort_keys=False, allow_unicode=True)

        print(f"\n{'='*55}")
        print(f"  ✅ 共采集 {len(samples)} 组数据")
        print(f"  📁 已保存到: {OUTPUT_FILE}")
        median_ang, max_ang, axis_ratio = _sample_motion_coverage(samples)
        print(f"  📐 最终姿态覆盖: median={median_ang:.1f}°  "
              f"max={max_ang:.1f}°  双轴比={axis_ratio:.3f}")
        if len(samples) < 15 or max_ang < 8.0 or axis_ratio < 0.05:
            print("  ⚠ 数据已保存，但运动激励偏弱；建议继续补采不同倾斜方向")
        if backup and os.path.exists(backup):
            print(f"  💾 旧数据备份: {backup}")
        print("  下一步: python solve.py samples.yaml")
        print(f"{'='*55}")
        print(f"\n  ⚠️  注意: 普通黑白棋盘格(非 ChArUco)存在 180° 旋转对称性。")
        print(f"     如果采集过程中角点编号方向发生了翻转,")
        print(f"     会导致该样本的 target_to_camera 方向错误。")
        print(f"     建议: 运行 verify.py 检查可疑样本, 剔除后重新求解。")
    else:
        print("\n  没有采集到任何数据")


if __name__ == "__main__":
    main()
