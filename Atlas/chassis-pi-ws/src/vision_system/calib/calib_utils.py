#!/usr/bin/env python3
"""
手眼标定共享工具
===============
camera_calib.py / collect_samples.py / solve.py / verify.py 的公共函数.
"""

import cv2
import numpy as np
import yaml


# ── 棋盘格检测 ──

def find_chessboard(gray, pattern_size):
    """检测棋盘格角点, 兼容新旧 OpenCV"""
    if hasattr(cv2, 'findChessboardCornersSB'):
        found, corners = cv2.findChessboardCornersSB(
            gray, pattern_size,
            flags=cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)
        if found and corners is not None:
            return True, corners
    found, corners = cv2.findChessboardCorners(
        gray, pattern_size,
        flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
    return found, corners


def corner_subpix_win(gray, pattern_size, corners=None):
    """返回不会跨到相邻角点的 ``cornerSubPix`` 半窗口。

    ``winSize=(w, w)`` 表示角点两侧各 ``w`` 像素。旧实现对
    640x480 / 11x8 棋盘返回 11，实际搜索区域达 23x23 px；当格宽只有
    14~20 px 时会把角点吸到相邻交点。
    """
    if corners is not None:
        pts = np.asarray(corners, dtype=np.float64).reshape(
            int(pattern_size[1]), int(pattern_size[0]), 2)
        spacings = []
        if pts.shape[1] > 1:
            spacings.append(np.linalg.norm(
                pts[:, 1:] - pts[:, :-1], axis=2).ravel())
        if pts.shape[0] > 1:
            spacings.append(np.linalg.norm(
                pts[1:] - pts[:-1], axis=2).ravel())
        if spacings:
            spacing = float(np.median(np.concatenate(spacings)))
            # 总窗口 2*w+1 保持小于约 60% 的相邻角点距离。
            w = max(2, min(5, int(np.floor(spacing * 0.28))))
            return (w, w)

    # 没有角点间距可用时采用保守上限。
    w = max(2, min(5, int(min(gray.shape) / max(pattern_size) / 8)))
    return (w, w)


def refine_chessboard_corners(gray, corners, pattern_size):
    """安全地精化角点，并保留精化前的独立副本供质量评估。"""
    before = np.asarray(corners, dtype=np.float32).copy()
    refined = before.copy()
    criteria = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
        40,
        1e-4,
    )
    win = corner_subpix_win(gray, pattern_size, before)
    cv2.cornerSubPix(gray, refined, win, (-1, -1), criteria)
    return before, refined, win


def validate_chessboard_geometry(corners, pattern_size,
                                 min_spacing_ratio=0.45):
    """检查角点网格是否发生重合、吸附或顺序破坏。"""
    cols, rows = int(pattern_size[0]), int(pattern_size[1])
    pts = np.asarray(corners, dtype=np.float64).reshape(-1, 2)
    if pts.shape[0] != cols * rows or not np.isfinite(pts).all():
        return False, {"reason": "invalid_shape_or_nonfinite"}

    grid = pts.reshape(rows, cols, 2)
    neighbor = []
    if cols > 1:
        neighbor.append(np.linalg.norm(
            grid[:, 1:] - grid[:, :-1], axis=2).ravel())
    if rows > 1:
        neighbor.append(np.linalg.norm(
            grid[1:] - grid[:-1], axis=2).ravel())
    distances = np.concatenate(neighbor)
    median_spacing = float(np.median(distances))
    min_spacing = float(np.min(distances))
    ratio = min_spacing / median_spacing if median_spacing > 1e-9 else 0.0
    ok = median_spacing >= 3.0 and ratio >= float(min_spacing_ratio)
    return ok, {
        "median_spacing_px": median_spacing,
        "min_spacing_px": min_spacing,
        "min_spacing_ratio": ratio,
        "reason": "ok" if ok else "collapsed_or_irregular_grid",
    }


# ── 图像质量评估 ──

def compute_laplacian_variance(gray):
    """计算图像的 Laplacian 方差，用于评估清晰度 / 模糊程度.

    返回值越大图像越清晰。典型阈值:
      > 120 : 清晰，适合标定
      80-120: 可接受但偏模糊，建议重新采集
      < 80  : 严重模糊 (运动模糊 / 失焦)，应当舍弃

    Args:
        gray: 灰度图像 (uint8)
    Returns:
        float: Laplacian 方差
    """
    lap = cv2.Laplacian(gray, cv2.CV_64F)
    return float(lap.var())


def compute_corner_quality(corners_before, corners_after):
    """计算亚像素角点精化质量.

    通过比较 cornerSubPix 前后角点位置的变化来评估角点稳定性。
    如果 RMS > 0.2 pixel，说明原始角点检测不稳定或图像质量差。

    Args:
        corners_before: 精化前的角点 (N, 1, 2) 或 (N, 2)
        corners_after:  精化后的角点 (N, 1, 2) 或 (N, 2)
    Returns:
        (mean_move_px, rms_px):
            mean_move_px - 平均移动距离 (像素)
            rms_px       - 移动距离 RMS (像素)
    """
    before = corners_before.reshape(-1, 2).astype(np.float64)
    after = corners_after.reshape(-1, 2).astype(np.float64)
    diffs = after - before
    distances = np.linalg.norm(diffs, axis=1)
    mean_move = float(np.mean(distances))
    rms = float(np.sqrt(np.mean(distances ** 2)))
    return mean_move, rms


def make_chessboard_objp(cols, rows, square_size_m):
    """生成棋盘格角点的 3D 世界坐标 (Z=0 平面).

    角点顺序: row-major (C-order), 与 OpenCV findChessboardCorners 一致.
    np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) 隐式依赖 C-order 展开,
    即 (row=0,col=0), (0,1), ...(0,cols-1), (1,0), ... 的排列.
    """
    n = cols * rows
    objp = np.zeros((n, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= square_size_m
    return objp


# ── 数据加载 ──

def load_samples(path):
    """从 samples.yaml 加载手眼标定样本"""
    with open(path, encoding="utf-8") as f:
        data = yaml.safe_load(f)
    samples_raw = data.get("samples", [])
    b2g_list = [np.array(s["gripper_in_base"], dtype=np.float64) for s in samples_raw]
    t2c_list = [np.array(s["target_to_camera"], dtype=np.float64) for s in samples_raw]
    return b2g_list, t2c_list, data.get("handeye_mode", "eye_in_hand")


def load_sample_metadata(path):
    """从 samples.yaml 加载每个样本的质量元数据.

    Returns:
        reproj_errors: list of float or None (PnP 重投影误差, px)
        laplacian_vars: list of float or None (Laplacian 清晰度方差)
        corner_rms: list of float or None (角点 RMS, px)
    """
    with open(path, encoding="utf-8") as f:
        data = yaml.safe_load(f)
    samples_raw = data.get("samples", [])
    reproj = [s.get("reprojection_error_px") for s in samples_raw]
    laplacian = [s.get("laplacian_variance") for s in samples_raw]
    crms = [s.get("corner_rms_px") for s in samples_raw]
    return reproj, laplacian, crms


def load_sample_validity(path, max_reprojection_px=0.40,
                         max_corner_rms_px=0.20,
                         max_sync_dt_ms=80.0):
    """在求解前执行不可被权重掩盖的硬质量门禁。

    返回 ``(valid_indices, rejected)``；``rejected`` 的元素为
    ``{"index": zero_based_index, "reasons": [...]}``。
    """
    with open(path, encoding="utf-8") as f:
        data = yaml.safe_load(f)
    samples = data.get("samples", [])
    board = str(data.get("chessboard", ""))
    pattern_size = None
    try:
        cols, rows = (int(v) for v in board.lower().split("x", 1))
        pattern_size = (cols, rows)
    except (TypeError, ValueError):
        pass

    valid, rejected = [], []
    for i, sample in enumerate(samples):
        reasons = []
        reproj = sample.get("reprojection_error_px")
        if reproj is not None:
            if not np.isfinite(float(reproj)):
                reasons.append("reprojection_nonfinite")
            elif float(reproj) > float(max_reprojection_px):
                reasons.append(
                    f"reprojection={float(reproj):.3f}px>{max_reprojection_px:.3f}px")

        corner_rms = sample.get("corner_rms_px")
        if (corner_rms is not None and np.isfinite(float(corner_rms))
                and float(corner_rms) > float(max_corner_rms_px)):
            reasons.append(
                f"corner_rms={float(corner_rms):.3f}px>{max_corner_rms_px:.3f}px")

        sync_dt = sample.get("sync_frame_pose_dt_ms")
        if (sync_dt is not None and np.isfinite(float(sync_dt))
                and float(sync_dt) > float(max_sync_dt_ms)):
            reasons.append(
                f"sync_dt={float(sync_dt):.1f}ms>{max_sync_dt_ms:.1f}ms")
        if sample.get("sync_robot_stable") is False:
            reasons.append("robot_not_stable")

        corners = sample.get("corners_px")
        if corners is not None and pattern_size is not None:
            try:
                grid_ok, grid_diag = validate_chessboard_geometry(
                    np.asarray(corners, dtype=np.float64).reshape(-1, 1, 2),
                    pattern_size)
                if not grid_ok:
                    reasons.append(
                        "corner_grid_collapsed"
                        f"(ratio={grid_diag.get('min_spacing_ratio', 0):.3f})")
            except (TypeError, ValueError):
                reasons.append("corner_grid_invalid")

        if reasons:
            rejected.append({"index": i, "reasons": reasons})
        else:
            valid.append(i)
    return valid, rejected


# ── calibrateHandEye 输入准备 ──

def prepare_inputs(b2g_list, t2c_list, mode):
    """构造 calibrateHandEye 的绝对位姿输入.
    返回 (R_gripper2base, t_gripper2base, R_target2cam, t_target2cam).
    """
    R_gb, t_gb, R_tc, t_tc = [], [], [], []
    for b2g, t2c in zip(b2g_list, t2c_list):
        # eye-in-hand: OpenCV 需要 ^base T_gripper。
        # eye-to-hand: 将基座视为算法中的“gripper”，需输入其逆变换
        # ^gripper T_base，输出才是 ^base T_camera。
        robot_pose = (b2g if mode == "eye_in_hand"
                      else np.linalg.inv(b2g))
        R_gb.append(robot_pose[:3, :3])
        t_gb.append(robot_pose[:3, 3].reshape(3, 1))
        R_tc.append(t2c[:3, :3])
        t_tc.append(t2c[:3, 3].reshape(3, 1))
    return R_gb, t_gb, R_tc, t_tc
