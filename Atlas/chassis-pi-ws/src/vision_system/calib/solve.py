#!/usr/bin/env python3
"""
手眼标定求解
============
用法: python solve.py samples.yaml
"""

import sys
import os
import math
import numpy as np
import cv2
import yaml

# LM 优化依赖 scipy
_SCIPY_OK = False
try:
    from scipy.optimize import least_squares
    _SCIPY_OK = True
except ImportError:
    pass

_here = os.path.dirname(os.path.abspath(__file__))
if _here not in sys.path:
    sys.path.insert(0, _here)
from fk_utils import (matrix_to_rpy, matrix_to_quaternion,
                       make_transform, invert_transform,
                       rotation_angle_deg, mean_rotation)
from calib_utils import (load_samples, prepare_inputs, load_sample_metadata,
                         load_sample_validity)


MAX_SOLVABLE_TRANS_RMS_MM = 50.0
MAX_SOLVABLE_ROT_RMS_DEG = 10.0


def _load_intrinsics_binding(samples_path):
    """提取采集时冻结的内参，随手眼结果一起部署。"""
    with open(samples_path, encoding="utf-8") as f:
        data = yaml.safe_load(f)
    camera = data.get("camera_matrix_at_collection")
    distortion = data.get("distortion_at_collection")
    image_size = data.get("image_size_at_collection")
    if not camera or not distortion or not image_size or len(image_size) != 2:
        return None
    return {
        "camera_matrix_data": [float(v) for v in camera.get("data", [])],
        "distortion_data": [float(v) for v in distortion.get("data", [])],
        "image_width": int(image_size[0]),
        "image_height": int(image_size[1]),
        "created_at": data.get("intrinsics_created_at"),
    }


def _intrinsics_binding_matches_file(binding, intrinsics_path):
    if binding is None or not os.path.exists(intrinsics_path):
        return False
    with open(intrinsics_path, encoding="utf-8") as f:
        data = yaml.safe_load(f)
    try:
        file_k = np.asarray(data["camera_matrix"]["data"], dtype=np.float64)
        file_d = np.asarray(
            data["distortion_coefficients"]["data"], dtype=np.float64)
        bound_k = np.asarray(binding["camera_matrix_data"], dtype=np.float64)
        bound_d = np.asarray(binding["distortion_data"], dtype=np.float64)
        return bool(
            (int(data.get("image_width", 0)), int(data.get("image_height", 0)))
            == (binding["image_width"], binding["image_height"])
            and file_k.shape == bound_k.shape and file_d.shape == bound_d.shape
            and np.allclose(file_k, bound_k, rtol=0.0, atol=1e-9)
            and np.allclose(file_d, bound_d, rtol=0.0, atol=1e-9))
    except (KeyError, TypeError, ValueError):
        return False

# ── 一致性评估 ──

def evaluate(X, b2g_list, t2c_list, mode):
    """计算手眼矩阵 X 在各样本上的一致性误差"""
    constants = []
    for b2g, t2c in zip(b2g_list, t2c_list):
        if mode == "eye_in_hand":
            c = b2g @ X @ t2c       # base_to_target
        else:
            c = invert_transform(b2g) @ X @ t2c   # gripper_to_camera_to_target → 常量
        constants.append(c)

    trans = np.array([c[:3, 3] for c in constants])
    mean_t = np.mean(trans, axis=0)
    t_err = np.linalg.norm(trans - mean_t, axis=1)

    mean_R = mean_rotation([c[:3, :3] for c in constants])
    r_err = np.array([rotation_angle_deg(mean_R.T @ c[:3, :3]) for c in constants])

    return {
        "trans_rms_mm": float(np.sqrt(np.mean(t_err**2)) * 1000),
        "trans_max_mm": float(np.max(t_err) * 1000),
        "rot_rms_deg": float(np.sqrt(np.mean(r_err**2))),
        "rot_max_deg": float(np.max(r_err)),
    }

# ── 加权 ──

def compute_weights(reproj_errors, laplacian_vars=None, corner_rms=None,
                    floor_px=0.03, ref_px=0.8):
    """基于多维质量指标计算样本权重.

    综合以下因素:
      - PnP 重投影误差 (主要): 低误差 → 高权重
      - Laplacian 清晰度 (次要):  低清晰度 → 降权 (运动模糊/失焦)
      - 角点 RMS (次要):         高 RMS → 降权 (角点不稳定)

    例如:
      重投影 0.08px, 清晰度 200, 角点RMS 0.05 → weight ≈ 1.0
      重投影 1.2px,  清晰度 60,  角点RMS 0.3  → weight ≈ 0.15

    Args:
        reproj_errors: list of float or None
        laplacian_vars: list of float or None
        corner_rms: list of float or None
    Returns:
        np.ndarray: weights in [0.01, 1.0], 或 None (若无有效数据)
    """
    valid = [e for e in reproj_errors if e is not None]
    if len(valid) == 0:
        return None
    n = len(reproj_errors)
    med_reproj = np.median(valid)

    errors = np.array([e if e is not None else med_reproj for e in reproj_errors],
                      dtype=np.float64)
    errors = np.maximum(errors, floor_px)
    # Quadratic falloff for reprojection
    w_reproj = np.where(errors <= ref_px, 1.0, (ref_px / errors) ** 2)

    # Laplacian 清晰度因子: < 80 严重降权, 80-120 部分降权, >120 满权
    w_lap = np.ones(n)
    if laplacian_vars is not None and any(v is not None for v in laplacian_vars):
        laps = np.array([v if v is not None else 120.0 for v in laplacian_vars],
                        dtype=np.float64)
        w_lap = np.clip(laps / 120.0, 0.1, 1.0)  # 120 = good baseline

    # 角点 RMS 因子: > 0.2 严重降权, < 0.1 满权
    w_crms = np.ones(n)
    if corner_rms is not None and any(v is not None for v in corner_rms):
        crms = np.array([v if v is not None else 0.1 for v in corner_rms],
                        dtype=np.float64)
        w_crms = np.clip(0.2 / np.maximum(crms, 0.02), 0.1, 1.0)

    # 综合权重: 重投影占主导, 清晰度和角点质量作为惩罚因子
    weights = w_reproj * np.sqrt(w_lap * w_crms)
    weights = np.clip(weights, 0.01, 1.0)
    return weights


# ── LM 非线性优化 ──

def handeye_residual(params, A_rel_list, B_rel_list, weights):
    """相对运动残差: 最小化 Σ w_ij * ||A_ij X - X B_ij||.

    将旋转和平移残差分别压缩为 3 维向量 (axis-angle + 欧氏距离),
    总计 6 维/样本, 避免 9 维旋转残差主导优化方向。

    Args:
        params: (6,) ndarray [rx, ry, rz, tx, ty, tz]
        A_rel_list, B_rel_list: 满足 A_ij X = X B_ij 的相对运动
        weights: (n,) ndarray or None
    Returns:
        (6*n,) ndarray: 堆叠的加权残差
    """
    R = cv2.Rodrigues(params[:3])[0]
    t = params[3:].reshape(3, 1)

    residuals = []
    # 1° 旋转误差和 5mm 平移误差具有相同量级，避免某一部分支配优化。
    rot_scale = np.deg2rad(1.0)
    trans_scale = 0.005

    for i, (A, B) in enumerate(zip(A_rel_list, B_rel_list)):
        R_A = np.asarray(A[:3, :3], dtype=np.float64)
        t_A = A[:3, 3:4].copy()
        R_B = np.asarray(B[:3, :3], dtype=np.float64)
        t_B = B[:3, 3:4].copy()

        # 左右两侧应相等: A_ij @ X == X @ B_ij
        R_left = R_A @ R
        R_right = R @ R_B
        R_diff = R_left.T @ R_right
        r_res = cv2.Rodrigues(R_diff)[0].ravel() / rot_scale

        # 平移残差 → 3 维 (meters)
        t_res = (R_A @ t + t_A - R @ t_B - t).ravel() / trans_scale

        res = np.concatenate([r_res, t_res])
        w = np.sqrt(weights[i]) if weights is not None else 1.0
        residuals.append(res * w)

    return np.concatenate(residuals)


def lm_refine(X_init, A_list, B_list, b2g_list, t2c_list, mode, weights=None):
    """用鲁棒非线性最小二乘精化手眼矩阵 X.

    先由绝对位姿构造全部相对运动，再优化
    min Σ w_ij * ||A_ij X - X B_ij||²。

    从 OpenCV 代数解出发, 迭代优化到局部最优。
    内置几何一致性验证: 如果 LM 降低了 hand-eye 一致性, 自动回退。

    Args:
        X_init: 4x4 初始手眼矩阵 (来自最佳 OpenCV 方法)
        A_list, B_list: list of 4x4
        b2g_list, t2c_list: 原始样本 (用于几何验证)
        mode: 'eye_in_hand' or 'eye_to_hand'
        weights: (n,) ndarray or None
    Returns:
        4x4 精化后的 X
    """
    if not _SCIPY_OK:
        return X_init

    A_rel, B_rel, pair_weights = _compute_all_pair_motions(
        A_list, B_list, min_rot_deg=0.1,
        sample_weights=weights, mode=mode)
    if len(A_rel) < 3:
        return X_init

    # 计算初值的几何一致性 (用于验证)
    m0 = evaluate(X_init, b2g_list, t2c_list, mode)
    score0 = m0["trans_rms_mm"] / 5.0 + m0["rot_rms_deg"] / 1.0

    rvec_init, _ = cv2.Rodrigues(X_init[:3, :3])
    params_init = np.concatenate([rvec_init.ravel(), X_init[:3, 3]])

    result = least_squares(
        handeye_residual, params_init,
        args=(A_rel, B_rel, pair_weights),
        method='trf',
        loss='soft_l1',
        f_scale=1.0,
        x_scale='jac',
        ftol=1e-12,
        xtol=1e-12,
        gtol=1e-12,
        max_nfev=500,
    )

    R_opt, _ = cv2.Rodrigues(result.x[:3])
    X_opt = make_transform(R_opt, result.x[3:])

    # 安全检查 1: 偏离初值过大 → 发散
    R_diff = X_init[:3, :3].T @ R_opt
    ang_diff = rotation_angle_deg(R_diff)
    t_diff = np.linalg.norm(X_opt[:3, 3] - X_init[:3, 3]) * 1000.0
    if ang_diff > 90.0 or t_diff > 1000.0:
        print(f"  ⚠ LM 优化发散 (ΔR={ang_diff:.1f}°, Δt={t_diff:.0f}mm)，回退到初值")
        return X_init

    # 安全检查 2: 几何一致性退化 → 回退 (这才是真正重要的!)
    m_opt = evaluate(X_opt, b2g_list, t2c_list, mode)
    score_opt = m_opt["trans_rms_mm"] / 5.0 + m_opt["rot_rms_deg"] / 1.0
    if score_opt > score0 * 1.05:
        print(f"  ⚠ LM 导致几何一致性退化 "
              f"({m0['trans_rms_mm']:.1f}mm/{m0['rot_rms_deg']:.2f}° → "
              f"{m_opt['trans_rms_mm']:.1f}mm/{m_opt['rot_rms_deg']:.2f}°)，回退到初值")
        return X_init

    return X_opt


# ── RANSAC 手眼标定 ──

def _constant_poses(X, A_list, B_list, mode="eye_in_hand"):
    """由每组绝对位姿反算理论固定不动的参考位姿。"""
    if mode == "eye_in_hand":
        return [A @ X @ B for A, B in zip(A_list, B_list)]
    return [invert_transform(A) @ X @ B for A, B in zip(A_list, B_list)]


def _compute_constant_errors(X, A_list, B_list, mode="eye_in_hand"):
    """相对鲁棒中心计算每个样本的平移/旋转一致性误差。"""
    constants = _constant_poses(X, A_list, B_list, mode)
    trans = np.array([c[:3, 3] for c in constants])
    median_t = np.median(trans, axis=0)
    t_err = np.linalg.norm(trans - median_t, axis=1) * 1000.0

    # 旋转采用 medoid，避免少量大角度异常值把均值拉偏。
    rotations = [c[:3, :3] for c in constants]
    dist = np.array([
        [rotation_angle_deg(Ri.T @ Rj) for Rj in rotations]
        for Ri in rotations
    ])
    center_idx = int(np.argmin(np.median(dist, axis=1)))
    center_R = rotations[center_idx]
    r_err = np.array([rotation_angle_deg(center_R.T @ R) for R in rotations])
    return t_err, r_err


def _best_constant_consensus(X, A_list, B_list, mode,
                             t_thresh_mm, r_thresh_deg, weights=None):
    """寻找最大的固定参考位姿簇，避免全局均值被多数异常值污染。"""
    constants = _constant_poses(X, A_list, B_list, mode)
    n = len(constants)
    w = np.asarray(weights if weights is not None else np.ones(n), dtype=np.float64)
    best = ([], -1.0, float("inf"), None, None)

    for center in constants:
        t_err = np.array([
            np.linalg.norm(c[:3, 3] - center[:3, 3]) * 1000.0
            for c in constants
        ])
        r_err = np.array([
            rotation_angle_deg(center[:3, :3].T @ c[:3, :3])
            for c in constants
        ])
        inliers = np.flatnonzero(
            (t_err <= t_thresh_mm) & (r_err <= r_thresh_deg)).tolist()
        score = float(np.sum(w[inliers]))
        quality = (float(np.median(t_err[inliers]) / t_thresh_mm
                         + np.median(r_err[inliers]) / r_thresh_deg)
                   if inliers else float("inf"))
        if score > best[1] or (abs(score - best[1]) < 1e-12 and quality < best[2]):
            best = (inliers, score, quality, t_err, r_err)
    return best


def _solve_ax_xb_linear(A_rel_list, B_rel_list, weights=None):
    """用全部相对运动直接解 A_ij X = X B_ij。

    旋转使用 Kronecker/SVD，平移使用加权最小二乘。与
    calibrateHandEye 不同，本函数的输入就是相对运动，不会再二次做差。
    """
    n = len(A_rel_list)
    if n < 2:
        return None
    w = np.asarray(weights if weights is not None else np.ones(n), dtype=np.float64)
    w = np.clip(w, 1e-6, None)

    blocks = []
    for A, B, wi in zip(A_rel_list, B_rel_list, w):
        Ra = np.asarray(A[:3, :3], dtype=np.float64)
        Rb = np.asarray(B[:3, :3], dtype=np.float64)
        blocks.append(np.sqrt(wi) * (
            np.kron(np.eye(3), Ra) - np.kron(Rb.T, np.eye(3))))
    M = np.vstack(blocks)
    try:
        _, singular_values, vt = np.linalg.svd(M)
    except np.linalg.LinAlgError:
        return None

    best = None
    for sign in (1.0, -1.0):
        R_raw = (sign * vt[-1]).reshape((3, 3), order="F")
        u, _, v = np.linalg.svd(R_raw)
        corr = np.eye(3)
        corr[2, 2] = np.linalg.det(u @ v)
        R = u @ corr @ v

        lhs, rhs = [], []
        rot_cost = 0.0
        for A, B, wi in zip(A_rel_list, B_rel_list, w):
            Ra, ta = A[:3, :3], A[:3, 3]
            Rb, tb = B[:3, :3], B[:3, 3]
            sw = np.sqrt(wi)
            lhs.append(sw * (Ra - np.eye(3)))
            rhs.append(sw * (R @ tb - ta))
            rot_cost += wi * rotation_angle_deg(
                (Ra @ R).T @ (R @ Rb)) ** 2
        L = np.vstack(lhs)
        y = np.hstack(rhs)
        try:
            t, _, rank, _ = np.linalg.lstsq(L, y, rcond=None)
        except np.linalg.LinAlgError:
            continue
        if rank < 3 or not np.all(np.isfinite(t)):
            continue
        if best is None or rot_cost < best[0]:
            best = (rot_cost, make_transform(R, t))

    if best is None:
        return None
    return best[1]


def _relative_pair_residuals(X, A_rel_list, B_rel_list):
    t_err, r_err = [], []
    for A, B in zip(A_rel_list, B_rel_list):
        left, right = A @ X, X @ B
        t_err.append(np.linalg.norm(left[:3, 3] - right[:3, 3]) * 1000.0)
        r_err.append(rotation_angle_deg(left[:3, :3].T @ right[:3, :3]))
    return np.asarray(t_err), np.asarray(r_err)


def _robust_all_pairs_solve(A_rel_list, B_rel_list, pair_weights=None):
    """全相对运动 IRLS；小角度运动也能累积使用，而非只看相邻样本。"""
    base_w = np.asarray(
        pair_weights if pair_weights is not None else np.ones(len(A_rel_list)),
        dtype=np.float64)
    work_w = np.clip(base_w, 1e-6, None)
    X = None
    for _ in range(6):
        X = _solve_ax_xb_linear(A_rel_list, B_rel_list, work_w)
        if X is None:
            return None
        t_err, r_err = _relative_pair_residuals(X, A_rel_list, B_rel_list)
        t_scale = max(2.0, 1.4826 * np.median(np.abs(t_err - np.median(t_err))))
        r_scale = max(0.3, 1.4826 * np.median(np.abs(r_err - np.median(r_err))))
        normalized = np.sqrt((t_err / (2.5 * t_scale)) ** 2
                             + (r_err / (2.5 * r_scale)) ** 2)
        robust_w = 1.0 / (1.0 + normalized ** 2)
        new_w = np.clip(base_w * robust_w, 1e-6, None)
        if np.max(np.abs(new_w - work_w)) < 1e-4:
            break
        work_w = new_w
    return X


def ransac_handeye(A_list, B_list, weights=None, mode="eye_in_hand",
                   n_iter=600, inlier_t_mm=5.0, inlier_r_deg=2.0):
    """RANSAC 手眼标定: 随机采样 → 子集求解 → 投票 → 最优模型.

    每个随机子集使用全部相对运动直接线性求解，再在绝对位姿层面寻找
    最大固定参考位姿簇。阈值不随坏数据自动放宽，避免把错误数据包装成成功。

    Args:
        A_list, B_list: 成对的绝对机器人位姿与标定板相机位姿
        weights: (n,) ndarray or None (用于加权投票)
        n_iter: RANSAC 迭代次数
        inlier_t_mm: 固定平移内点阈值 (mm)，不会随坏数据自动放宽
        inlier_r_deg: 旋转内点阈值 (deg)

    Returns:
        (X_best, inlier_indices, info_dict)
    """
    n = len(A_list)
    subset_size = min(n, max(5, int(np.ceil(n * 0.30))))

    best_score = -1.0
    best_X = None
    best_inliers = []
    best_quality = float("inf")

    if n < 6:
        return _fallback_solve(A_list, B_list, weights, mode)

    w = weights if weights is not None else np.ones(n)

    rng = np.random.default_rng(20260803)
    for _ in range(n_iter):
        idx = np.sort(rng.choice(n, subset_size, replace=False)).tolist()
        A_sub = [A_list[i] for i in idx]
        B_sub = [B_list[i] for i in idx]
        w_sub = [w[i] for i in idx]
        A_rel, B_rel, pair_w = _compute_all_pair_motions(
            A_sub, B_sub, min_rot_deg=0.1,
            sample_weights=w_sub, mode=mode)
        X = _robust_all_pairs_solve(A_rel, B_rel, pair_w)
        if X is None or not np.all(np.isfinite(X)):
            continue

        inliers, score, quality, _, _ = _best_constant_consensus(
            X, A_list, B_list, mode, inlier_t_mm, inlier_r_deg, w)
        if score > best_score or (abs(score - best_score) < 1e-12
                                  and quality < best_quality):
            best_score = score
            best_quality = quality
            best_X = X
            best_inliers = inliers

    if best_X is None:
        # 所有子集都无法求解 — 回退到全量求解
        return _fallback_solve(A_list, B_list, weights, mode)

    if len(best_inliers) < 4:
        return best_X, best_inliers, {
            "removed": sorted(set(range(n)) - set(best_inliers)),
            "iterations": n_iter, "inlier_count": len(best_inliers),
            "score": float(best_score), "quality": float(best_quality)}

    removed = sorted(set(range(n)) - set(best_inliers))
    info = {
        "removed": removed,
        "iterations": n_iter,
        "inlier_count": len(best_inliers),
        "score": float(best_score),
        "quality": float(best_quality),
    }
    return best_X, best_inliers, info


def _fallback_solve(A_list, B_list, weights=None, mode="eye_in_hand"):
    """全量相对运动回退；不再无条件剔除固定比例样本。"""
    n = len(A_list)
    A_rel, B_rel, pair_w = _compute_all_pair_motions(
        A_list, B_list, min_rot_deg=0.1, sample_weights=weights,
        mode=mode)
    X = _robust_all_pairs_solve(A_rel, B_rel, pair_w)
    if X is None:
        return None, [], {"removed": list(range(n)), "iterations": 0}
    inliers, _, quality, _, _ = _best_constant_consensus(
        X, A_list, B_list, mode, 10.0, 3.0, weights)
    removed = sorted(set(range(n)) - set(inliers))
    info = {"removed": removed, "iterations": 0, "inlier_count": len(inliers),
            "fallback": True, "quality": float(quality)}
    return X, inliers, info


def prepare_inputs_for_lists(A_list, B_list):
    """将 A_i, B_i 列表转为 calibrateHandEye 所需格式."""
    R_a, t_a, R_b, t_b = [], [], [], []
    for A, B in zip(A_list, B_list):
        R_a.append(A[:3, :3])
        t_a.append(A[:3, 3].reshape(3, 1))
        R_b.append(B[:3, :3])
        t_b.append(B[:3, 3].reshape(3, 1))
    return R_a, t_a, R_b, t_b


# ── 小角度运动优化 ──

def _rotation_angle_between(R1, R2):
    """两个旋转矩阵之间的旋转角度 (deg)."""
    return rotation_angle_deg(R1.T @ R2)


def _reorder_by_rotation_diversity(A_list, B_list):
    """贪心最远点排序: 让相邻样本在旋转空间里距离最大化.

    calibrateHandEye 内部用相邻样本对 (i, i+1) 计算相对运动.
    如果采集顺序下相邻样本旋转差异极小(小角度转动), 相对运动信号被噪声淹没.
    重排序后同样的数据能提供更大的有效旋转量.

    返回 (reordered_A, reordered_B, order_indices).
    """
    n = len(A_list)
    if n <= 2:
        return list(A_list), list(B_list), list(range(n))

    Rs = [np.asarray(A[:3, :3], dtype=np.float64) for A in A_list]

    # 从中心开始 (离均值最近的样本)
    rvecs = np.array([cv2.Rodrigues(R)[0].ravel() for R in Rs])
    center = rvecs.mean(axis=0)
    dists = np.linalg.norm(rvecs - center, axis=1)
    start = int(np.argmin(dists))

    visited = {start}
    order = [start]

    # 贪心: 每次选离上一个最远的未访问点
    for _ in range(n - 1):
        last_R = Rs[order[-1]]
        best_i, best_ang = -1, -1.0
        for i in range(n):
            if i in visited:
                continue
            ang = _rotation_angle_between(last_R, Rs[i])
            if ang > best_ang:
                best_ang = ang
                best_i = i
        visited.add(best_i)
        order.append(best_i)

    A_sorted = [A_list[i] for i in order]
    B_sorted = [B_list[i] for i in order]
    return A_sorted, B_sorted, order


def _compute_all_pair_motions(A_list, B_list, min_rot_deg=0.1,
                              sample_weights=None, mode="eye_in_hand"):
    """计算所有样本对的相对运动, 按旋转量加权.

    对于小角度场景, 相邻样本之间相对旋转极小.
    使用全部 N*(N-1)/2 对可以显著增加有效运动信息.
    每对相对运动被当做一个"虚拟样本"输入 handeye 求解.

    Returns:
        A_pairs, B_pairs: 所有满足最小旋转阈值的相对运动对
        pair_weights: 每对的权重 (与旋转角度成正比)
    """
    n = len(A_list)
    sw = np.asarray(sample_weights if sample_weights is not None else np.ones(n),
                    dtype=np.float64)
    A_pairs, B_pairs, pair_weights = [], [], []
    for i in range(n):
        Bi_inv = invert_transform(B_list[i])
        for j in range(i + 1, n):
            # 由固定参考位姿关系推导相对运动：
            # eye-in-hand: inv(A_j) A_i X = X B_j inv(B_i)
            # eye-to-hand: A_j inv(A_i) X = X B_j inv(B_i)
            if mode == "eye_in_hand":
                A_rel = invert_transform(A_list[j]) @ A_list[i]
            else:
                A_rel = A_list[j] @ invert_transform(A_list[i])
            B_rel = B_list[j] @ Bi_inv
            ang_a = rotation_angle_deg(A_rel[:3, :3])
            ang_b = rotation_angle_deg(B_rel[:3, :3])
            excitation = 0.5 * (ang_a + ang_b)
            if max(ang_a, ang_b) >= min_rot_deg:
                A_pairs.append(A_rel)
                B_pairs.append(B_rel)
                # 小角度对仍保留；较大旋转对有更高信息量，但权重封顶。
                motion_w = np.clip(excitation / 15.0, 0.10, 1.0)
                pair_weights.append(np.sqrt(sw[i] * sw[j]) * motion_w)
    return A_pairs, B_pairs, np.array(pair_weights, dtype=np.float64)


def _solve_with_all_pairs(A_pairs, B_pairs, pair_weights, b2g_list, t2c_list, mode,
                           verbose=False):
    """使用全量相对运动对求解手眼矩阵.

    当 calibrateHandEye 因为单对旋转过小而失败时, 用所有对的加权组合求解.
    采用 ANDREFF (对偶四元数) 方法, 对小旋转最鲁棒.
    """
    if len(A_pairs) < 3:
        return None

    X = _robust_all_pairs_solve(A_pairs, B_pairs, pair_weights)

    if X is None:
        return None

    # 用原始绝对位姿做一致性评估
    ev = evaluate(X, b2g_list, t2c_list, mode)
    if verbose:
        print(f"  [全量对] {len(A_pairs)} 对相对运动 → "
              f"平移RMS={ev['trans_rms_mm']:.1f}mm 旋转RMS={ev['rot_rms_deg']:.2f}°")
    return X, ev


# ── 核心求解 API ──

def _solve_handeye_core_legacy(b2g_list, t2c_list, mode,
                               reproj_errors=None, laplacian_vars=None, corner_rms=None,
                               verbose=False):
    """手眼标定核心求解: RANSAC + LM 优化.

    供 solve.py 和 verify.py 共用, 保证求解逻辑一致。

    Args:
        b2g_list, t2c_list: 样本列表
        mode: 'eye_in_hand' or 'eye_to_hand'
        reproj_errors, laplacian_vars, corner_rms: 质量元数据 (可选)
        verbose: 是否打印进度信息

    Returns:
        (X_cam2gripper, inlier_indices, removed_indices, metrics) or None
        X_cam2gripper = calibrateHandEye 原始输出 (cam→gripper)
    """
    n = len(b2g_list)

    # 预处理: 统一为 A_i X = X B_i
    # 两种模式下 A 都是 gripper→base (即 b2g), OpenCV 的 R_gripper2base
    # 参数名相同，内部根据 mode 自动选择相对运动计算方式
    A_list, B_list = [], []
    for b2g, t2c in zip(b2g_list, t2c_list):
        A_list.append(b2g)
        B_list.append(t2c)

    # ── 小角度场景检测 ──
    R_gb_all = [np.asarray(A[:3, :3], dtype=np.float64) for A in A_list]
    pairwise_angles = []
    for i in range(n):
        for j in range(i + 1, n):
            pairwise_angles.append(_rotation_angle_between(R_gb_all[i], R_gb_all[j]))
    med_pair_angle = float(np.median(pairwise_angles)) if pairwise_angles else 0.0

    small_motion = med_pair_angle < 15.0  # 中位旋转差 < 15° → 小角度场景
    if verbose:
        if small_motion:
            print(f"  🔍 检测到小角度场景 (中位旋转差={med_pair_angle:.1f}°)，"
                  f"启用旋转重排序 + 全量对回退")
        else:
            print(f"  中位旋转差={med_pair_angle:.1f}°")

    # 旋转多样性检查
    if len(R_gb_all) >= 4:
        rvecs_all = np.array([cv2.Rodrigues(R)[0].ravel() for R in R_gb_all])
        _, s, _ = np.linalg.svd(rvecs_all - rvecs_all.mean(axis=0))
        if s[-1] < 0.05 and verbose:
            print(f"  ⚠ 旋转轴缺乏多样性 (最小奇异值={s[-1]:.4f})")

    # ── 重排序: 让小角度场景的相邻样本旋转差异最大化 ──
    if small_motion:
        A_ordered, B_ordered, _order = _reorder_by_rotation_diversity(A_list, B_list)
    else:
        A_ordered, B_ordered = A_list, B_list

    # 权重
    weights = compute_weights(reproj_errors, laplacian_vars, corner_rms)

    # ── RANSAC (小角度场景放宽阈值) ──
    inlier_t_mm = 12.0 if small_motion else 8.0
    inlier_r_deg = 4.0 if small_motion else 3.0
    X_ransac, inlier_indices, ransac_info = ransac_handeye(
        A_ordered, B_ordered, weights=weights,
        n_iter=500 if small_motion else 300,
        inlier_t_mm=inlier_t_mm, inlier_r_deg=inlier_r_deg)

    # ── 全量对回退 ──
    if (X_ransac is None or len(inlier_indices) < 4) and small_motion:
        if verbose:
            print(f"  🔄 RANSAC 内点不足 ({len(inlier_indices) if inlier_indices else 0})，"
                  f"回退到全量相对运动对求解...")
        A_pairs, B_pairs, pair_w = _compute_all_pair_motions(
            A_list, B_list, min_rot_deg=0.3)
        if verbose:
            print(f"  📐 全量对: {len(A_pairs)} 对 (共 {n} 样本, {n*(n-1)//2} 种组合)")
        result = _solve_with_all_pairs(A_pairs, B_pairs, pair_w,
                                        b2g_list, t2c_list, mode, verbose=verbose)
        if result is None:
            return None
        X_allpairs, ev_all = result

        # 基于一致性剔除最差的 20%
        t_err, r_err = _compute_constant_errors(X_allpairs, A_list, B_list)
        combined = t_err / max(np.median(t_err), 1e-5) + r_err / max(np.median(r_err), 1e-5)
        threshold = np.percentile(combined, 80)
        inlier_indices = [i for i in range(n) if combined[i] <= threshold]
        removed = sorted(set(range(n)) - set(inlier_indices))

        if len(inlier_indices) < 3:
            inlier_indices = list(range(n))
            removed = []

        # 用内点 + 重排序重新精化
        A_in = [A_list[i] for i in inlier_indices]
        B_in = [B_list[i] for i in inlier_indices]
        b2g_in = [b2g_list[i] for i in inlier_indices]
        t2c_in = [t2c_list[i] for i in inlier_indices]
        w_in = [weights[i] for i in inlier_indices] if weights is not None else None

        if small_motion and len(inlier_indices) >= 4:
            A_in, B_in, _ = _reorder_by_rotation_diversity(A_in, B_in)

        methods = {
            "ANDREFF": cv2.CALIB_HAND_EYE_ANDREFF,
            "HORAUD": cv2.CALIB_HAND_EYE_HORAUD,
            "PARK": cv2.CALIB_HAND_EYE_PARK,
            "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS,
        }

        X_best = X_allpairs
        best_score = float("inf")
        for method_id in methods.values():
            try:
                R_gb_in = [A_in[i][:3, :3] for i in range(len(A_in))]
                t_gb_in = [A_in[i][:3, 3:4] for i in range(len(A_in))]
                R_tc_in = [B_in[i][:3, :3] for i in range(len(B_in))]
                t_tc_in = [B_in[i][:3, 3:4] for i in range(len(B_in))]
                R_x, t_x = cv2.calibrateHandEye(R_gb_in, t_gb_in, R_tc_in, t_tc_in,
                                                 method=method_id)
                X_m = make_transform(R_x, t_x.reshape(3))
                ev = evaluate(X_m, b2g_in, t2c_in, mode)
                score = ev["trans_rms_mm"] / 5.0 + ev["rot_rms_deg"] / 1.0
                if score < best_score:
                    best_score = score
                    X_best = X_m
            except Exception:
                pass

        if _SCIPY_OK:
            X_final = lm_refine(X_best, A_in, B_in, b2g_in, t2c_in, mode, w_in)
        else:
            X_final = X_best

        metrics = evaluate(X_final, b2g_in, t2c_in, mode)
        metrics["inlier_count"] = len(inlier_indices)
        metrics["total_samples"] = n
        return X_final, inlier_indices, removed, metrics

    if X_ransac is None or len(inlier_indices) < 3:
        return None

    if verbose and ransac_info.get("fallback"):
        print(f"  ⚠ RANSAC 未找到可靠内点，回退到全量求解")

    removed = sorted(set(range(n)) - set(inlier_indices))

    # 在内点中找最佳 OpenCV 方法作为 LM 初值
    A_in = [A_list[i] for i in inlier_indices]
    B_in = [B_list[i] for i in inlier_indices]
    w_in = [weights[i] for i in inlier_indices] if weights is not None else None
    b2g_in = [b2g_list[i] for i in inlier_indices]
    t2c_in = [t2c_list[i] for i in inlier_indices]

    methods = {
        "TSAI": cv2.CALIB_HAND_EYE_TSAI, "PARK": cv2.CALIB_HAND_EYE_PARK,
        "HORAUD": cv2.CALIB_HAND_EYE_HORAUD, "ANDREFF": cv2.CALIB_HAND_EYE_ANDREFF,
        "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS,
    }

    X_best_cv = X_ransac
    best_cv_score = float("inf")
    for method_id in methods.values():
        try:
            R_gb_in = [A_in[i][:3, :3] for i in range(len(A_in))]
            t_gb_in = [A_in[i][:3, 3:4] for i in range(len(A_in))]
            R_tc_in = [B_in[i][:3, :3] for i in range(len(B_in))]
            t_tc_in = [B_in[i][:3, 3:4] for i in range(len(B_in))]
            R_x, t_x = cv2.calibrateHandEye(R_gb_in, t_gb_in, R_tc_in, t_tc_in,
                                             method=method_id)
            X_m = make_transform(R_x, t_x.reshape(3))
            ev = evaluate(X_m, b2g_in, t2c_in, mode)
            score = ev["trans_rms_mm"] / 5.0 + ev["rot_rms_deg"] / 1.0
            if score < best_cv_score:
                best_cv_score = score
                X_best_cv = X_m
        except Exception:
            pass

    # LM 精化
    if _SCIPY_OK:
        X_final = lm_refine(X_best_cv, A_in, B_in, b2g_in, t2c_in, mode, w_in)
    else:
        X_final = X_best_cv

    metrics = evaluate(X_final, b2g_in, t2c_in, mode)
    metrics["inlier_count"] = len(inlier_indices)
    metrics["total_samples"] = n

    return X_final, inlier_indices, removed, metrics


# ── 有限运动鲁棒核心 ──

def _motion_observability(A_list, mode="eye_in_hand"):
    """评估旋转激励；手眼标定至少需要两个不平行的旋转方向。"""
    rvecs, angles = [], []
    for i in range(len(A_list)):
        for j in range(i + 1, len(A_list)):
            if mode == "eye_in_hand":
                rel = invert_transform(A_list[j]) @ A_list[i]
            else:
                rel = A_list[j] @ invert_transform(A_list[i])
            rv = cv2.Rodrigues(rel[:3, :3])[0].ravel()
            angle = float(np.linalg.norm(rv) * 180.0 / np.pi)
            if angle > 0.05:
                rvecs.append(rv)
                angles.append(angle)
    if not angles:
        return {"median_angle_deg": 0.0, "max_angle_deg": 0.0,
                "axis_ratio": 0.0, "pair_count": 0}
    s = np.linalg.svd(np.asarray(rvecs), compute_uv=False)
    axis_ratio = float(s[1] / s[0]) if len(s) >= 2 and s[0] > 1e-12 else 0.0
    return {
        "median_angle_deg": float(np.median(angles)),
        "max_angle_deg": float(np.max(angles)),
        "axis_ratio": axis_ratio,
        "pair_count": len(angles),
    }


def solve_handeye_core(b2g_list, t2c_list, mode,
                       reproj_errors=None, laplacian_vars=None, corner_rms=None,
                       verbose=False):
    """有限运动友好的全相对运动 + 样本级 RANSAC 求解。"""
    n = len(b2g_list)
    if n < 6 or mode not in ("eye_in_hand", "eye_to_hand"):
        return None

    A_list = [np.asarray(T, dtype=np.float64) for T in b2g_list]
    B_list = [np.asarray(T, dtype=np.float64) for T in t2c_list]
    if reproj_errors is None:
        reproj_errors = [None] * n
    if laplacian_vars is None:
        laplacian_vars = [None] * n
    if corner_rms is None:
        corner_rms = [None] * n
    weights = compute_weights(reproj_errors, laplacian_vars, corner_rms)
    obs = _motion_observability(A_list, mode)
    small_motion = obs["median_angle_deg"] < 15.0

    if verbose:
        print(f"  运动激励: median={obs['median_angle_deg']:.2f}°  "
              f"max={obs['max_angle_deg']:.2f}°  "
              f"双轴比={obs['axis_ratio']:.3f}")
        if small_motion:
            print("  🔍 有限运动模式：累计使用全部样本对，并按信息量加权")

    if obs["max_angle_deg"] < 1.0 or obs["axis_ratio"] < 0.005:
        if verbose:
            print("  ❌ 旋转激励不可观测：至少绕两个不同方向做小幅旋转")
        return None
    if verbose and (obs["max_angle_deg"] < 8.0 or obs["axis_ratio"] < 0.05):
        print("  ⚠ 旋转激励偏弱，优先增加另一个方向的倾斜")

    t_thresh = 30.0 if small_motion else 20.0
    r_thresh = 8.0 if small_motion else 6.0
    min_inliers = max(5, int(np.ceil(n * 0.30)))
    X, inlier_indices, _ = ransac_handeye(
        A_list, B_list, weights=weights, mode=mode,
        n_iter=900 if small_motion else 600,
        inlier_t_mm=t_thresh, inlier_r_deg=r_thresh)

    if X is None or len(inlier_indices) < min_inliers:
        if verbose:
            got = len(inlier_indices) if inlier_indices is not None else 0
            print(f"  ❌ 最大一致簇只有 {got}/{n}，要求至少 {min_inliers}/{n}")
            print("     不再用少量样本强行输出变化矩阵")
        return None

    # 交替执行“内点全相对运动求解 → 全样本重新投票”。
    for _ in range(4):
        A_in = [A_list[i] for i in inlier_indices]
        B_in = [B_list[i] for i in inlier_indices]
        w_in = [weights[i] for i in inlier_indices]
        A_rel, B_rel, pair_w = _compute_all_pair_motions(
            A_in, B_in, min_rot_deg=0.1,
            sample_weights=w_in, mode=mode)
        X_new = _robust_all_pairs_solve(A_rel, B_rel, pair_w)
        if X_new is None:
            return None
        new_inliers, _, _, _, _ = _best_constant_consensus(
            X_new, A_list, B_list, mode, t_thresh, r_thresh, weights)
        X = X_new
        if new_inliers == inlier_indices:
            break
        if len(new_inliers) < min_inliers:
            break
        inlier_indices = new_inliers

    A_in = [A_list[i] for i in inlier_indices]
    B_in = [B_list[i] for i in inlier_indices]
    w_in = [weights[i] for i in inlier_indices]
    if _SCIPY_OK:
        X = lm_refine(X, A_in, B_in, A_in, B_in, mode, w_in)

    metrics = evaluate(X, A_in, B_in, mode)
    metrics.update(obs)
    metrics["inlier_count"] = len(inlier_indices)
    metrics["total_samples"] = n
    metrics["inlier_ratio"] = len(inlier_indices) / n
    metrics["solver"] = "all-pairs linear + sample RANSAC + robust nonlinear"

    if mode == "eye_in_hand" and np.linalg.norm(X[:3, 3]) > 0.80:
        if verbose:
            print(f"  ❌ 相机距末端 {np.linalg.norm(X[:3, 3]):.3f}m，疑似退化解")
        return None
    if (metrics["trans_rms_mm"] > MAX_SOLVABLE_TRANS_RMS_MM
            or metrics["rot_rms_deg"] > MAX_SOLVABLE_ROT_RMS_DEG):
        if verbose:
            print(
                "  ❌ 内点残差严重超限，拒绝继续优化: "
                f"{metrics['trans_rms_mm']:.2f}mm / "
                f"{metrics['rot_rms_deg']:.2f}°")
        return None

    removed = sorted(set(range(n)) - set(inlier_indices))
    return X, inlier_indices, removed, metrics


# ── 简易求解：模仿 easy_handeye ──

def _solve_simple(samples_path, b2g_list, t2c_list, mode, n,
                  total_samples=None, pre_rejected=None):
    """直接调用 OpenCV calibrateHandEye，不剔除任何样本。

    复现 easy_handeye 的核心逻辑：所有样本参与，Tsai-Lenz/Park 等方法
    分别求解，选一致性最好的输出。无 RANSAC、无 LM 精化。
    """
    R_gb, t_gb, R_tc, t_tc = prepare_inputs(
        b2g_list, t2c_list, mode)

    methods = {
        "TSAI":        cv2.CALIB_HAND_EYE_TSAI,
        "PARK":        cv2.CALIB_HAND_EYE_PARK,
        "HORAUD":      cv2.CALIB_HAND_EYE_HORAUD,
        "ANDREFF":     cv2.CALIB_HAND_EYE_ANDREFF,
        "DANIILIDIS":  cv2.CALIB_HAND_EYE_DANIILIDIS,
    }

    best_X = None
    best_score = float("inf")
    best_method = ""
    results = {}

    print(f"\n  {'Method':<16} {'Trans RMS':>10} {'Rot RMS':>10}")
    print(f"  {'─'*16} {'─'*10} {'─'*10}")

    for name, method_id in methods.items():
        try:
            R_x, t_x = cv2.calibrateHandEye(
                R_gb, t_gb, R_tc, t_tc, method=method_id)
            X = make_transform(R_x, t_x.reshape(3))

            # 用全部样本评估一致性
            t_err, r_err = _compute_constant_errors(X, b2g_list, t2c_list, mode)
            trans_rms = float(np.sqrt(np.mean(t_err**2)))
            rot_rms = float(np.sqrt(np.mean(r_err**2)))
            score = trans_rms / 5.0 + rot_rms / 1.0

            results[name] = {
                "X": X, "trans_rms": trans_rms, "rot_rms": rot_rms,
                "t_err": t_err, "r_err": r_err}

            print(f"  {name:<16} {trans_rms:8.2f} mm {rot_rms:8.2f}°")

            if score < best_score:
                best_score = score
                best_X = X
                best_method = name
        except Exception as e:
            print(f"  {name:<16} ❌ {e}")

    if best_X is None:
        print("\n❌ 所有方法均失败")
        return None

    best_result = results[best_method]

    print(f"\n  ✅ 最优: {best_method}")

    # 保存结果
    X_display = best_X
    t = X_display[:3, 3]
    q = matrix_to_quaternion(X_display[:3, :3])
    r, p, y = matrix_to_rpy(X_display[:3, :3])

    parent = "tool0" if mode == "eye_in_hand" else "arm_base_link"
    child = "camera_optical_frame"
    label = "相机在末端坐标系中的位置" if mode == "eye_in_hand" else "相机在基座坐标系中的位置"

    print(f"\n{'='*55}")
    print(f"  标定结果 (极简模式: {best_method}, 全部 {n} 样本)")
    print(f"{'='*55}")
    print(f"  📐 {label} ({parent} → {child}):")
    print(f"     X = {t[0]:.6f} m")
    print(f"     Y = {t[1]:.6f} m")
    print(f"     Z = {t[2]:.6f} m")
    print(f"  🧭 四元数 xyzw: [{q[0]:.6f}, {q[1]:.6f}, {q[2]:.6f}, {q[3]:.6f}]")
    print(f"  📏 RPY: Roll={r:.6f}  Pitch={p:.6f}  Yaw={y:.6f} rad")
    print(f"  ── 精度 (全部 {n} 样本) ──")
    print(f"  平移 RMS: {best_result['trans_rms']:.3f} mm")
    print(f"  旋转 RMS: {best_result['rot_rms']:.3f}°")
    print(f"  平移 Max: {np.max(best_result['t_err']):.2f} mm")
    print(f"  旋转 Max: {np.max(best_result['r_err']):.2f}°")

    # 保存
    import os as _os
    result_path = _os.path.splitext(samples_path)[0] + "_result.yaml"
    total_samples = int(total_samples if total_samples is not None else n)
    pre_rejected = list(pre_rejected or [])
    payload = {
        "handeye_mode": mode,
        "method": f"EasyHandEye-simple/{best_method}",
        "parent_frame": parent,
        "child_frame": child,
        "transform_matrix": X_display.tolist(),
        "translation_m": t.tolist() if hasattr(t, 'tolist') else list(t),
        "quaternion_xyzw": list(q),
        "rpy_rad": [float(r), float(p), float(y)],
        "translation_rms_mm": best_result["trans_rms"],
        "rotation_rms_deg": best_result["rot_rms"],
        "translation_max_mm": float(np.max(best_result["t_err"])),
        "rotation_max_deg": float(np.max(best_result["r_err"])),
        "inlier_count": n,
        "total_samples": total_samples,
        "inlier_ratio": n / total_samples,
        "stored_transform_convention": (
            "camera_to_gripper (^gripper T_camera)" if mode == "eye_in_hand"
            else "camera_to_base (^base T_camera)"),
        "removed_samples": [i + 1 for i in pre_rejected],
    }
    intrinsics_binding = _load_intrinsics_binding(samples_path)
    if intrinsics_binding is not None:
        payload["intrinsics_binding"] = intrinsics_binding
    with open(result_path, 'w', encoding="utf-8") as f:
        yaml.safe_dump(payload, f, sort_keys=False, allow_unicode=True)
    print(f"\n  📁 结果已保存: {result_path}")

    # 自动部署
    project_root = _os.path.dirname(_here)
    deploy_dir = _os.path.join(project_root, 'handeye_bridge', 'config')
    deploy_path = _os.path.join(deploy_dir, 'samples_result.yaml')
    deploy_intrinsics = _os.path.join(deploy_dir, 'camera_intrinsics.yaml')
    if (_os.path.isdir(deploy_dir)
            and _intrinsics_binding_matches_file(
                intrinsics_binding, deploy_intrinsics)):
        with open(deploy_path, 'w', encoding="utf-8") as f:
            yaml.safe_dump(payload, f, sort_keys=False, allow_unicode=True)
        print(f"  📁 已自动部署到: {deploy_path}")
    elif _os.path.isdir(deploy_dir):
        print("  ❌ 未部署手眼结果：运行配置内参与采集内参不一致")

    return X_display


# ── 主逻辑 ──

def solve(samples_path, simple=False, use_ba=False):
    b2g_list, t2c_list, mode = load_samples(samples_path)
    reproj_errors, laplacian_vars, corner_rms = load_sample_metadata(samples_path)
    total_n = len(b2g_list)

    # 极简模式保持为纯 OpenCV 求解：不做质量筛选、RANSAC 或 LM。
    if simple:
        print(f"\n{'='*55}")
        print("  手眼标定求解 (极简模式 — 全部样本直接 OpenCV)")
        print(f"  模式: {mode}  |  样本: {total_n} 组")
        print(f"{'='*55}")
        if total_n < 3:
            print("\n❌ 极简模式至少需要 3 组样本")
            return
        return _solve_simple(
            samples_path, b2g_list, t2c_list, mode, total_n,
            total_samples=total_n, pre_rejected=[])

    valid_indices, quality_rejected = load_sample_validity(samples_path)
    if quality_rejected:
        print("\n  🧹 求解前质量门禁拒绝以下样本:")
        for item in quality_rejected:
            print(f"     #{item['index'] + 1}: {', '.join(item['reasons'])}")

    b2g_list = [b2g_list[i] for i in valid_indices]
    t2c_list = [t2c_list[i] for i in valid_indices]
    reproj_errors = [reproj_errors[i] for i in valid_indices]
    laplacian_vars = [laplacian_vars[i] for i in valid_indices]
    corner_rms = [corner_rms[i] for i in valid_indices]
    n = len(b2g_list)

    print(f"\n{'='*55}")
    print("  手眼标定求解 (全相对运动 + 鲁棒 RANSAC)")
    print(f"  模式: {mode}  |  样本: {n} 组")
    print(f"{'='*55}")

    if n < 6:
        print("\n❌ 质量筛选后至少需要 6 组有效样本")
        return

    # 调用共享核心求解逻辑
    result = solve_handeye_core(b2g_list, t2c_list, mode,
                                reproj_errors, laplacian_vars, corner_rms,
                                verbose=True)
    if result is None:
        print("\n❌ 求解失败 — 样本一致性过差，无法获得有效标定结果")
        print("   建议: 检查采集过程，确保棋盘格方向一致，机械臂位姿准确")
        print("   可能原因:")
        print("     - 机械臂运动量不足 (旋转多样性不够)")
        print("     - 棋盘格角点方向在采集中发生了 180° 翻转")
        print("     - gripper_in_base 和 target_to_camera 未正确配对")
        stale_result = os.path.splitext(samples_path)[0] + "_result.yaml"
        if os.path.exists(stale_result):
            print(f"   ⚠ 旧结果仍在 {stale_result}，本次并未更新，禁止继续部署旧文件")
        return

    X_final, inlier_indices_local, removed_local, metrics = result
    inlier_indices = [valid_indices[i] for i in inlier_indices_local]
    removed = sorted(set(range(total_n)) - set(inlier_indices))
    metrics["total_samples"] = total_n
    metrics["inlier_count"] = len(inlier_indices)
    metrics["inlier_ratio"] = len(inlier_indices) / total_n

    removed_orig = [i + 1 for i in removed]
    if removed_orig:
        print(f"\n  🧹 剔除 {len(removed_orig)} 个异常样本: #{removed_orig}")
        print(f"     内点: {len(inlier_indices)}/{total_n} 组")
    else:
        print(f"\n  ✅ 无异常值, 全部 {n} 组参与求解")

    # ── 显示结果 ──
    # OpenCV/本求解器的 X = ^gripper T_camera：相机在末端中的位姿，
    # 可直接作为 ROS parent=tool0, child=camera_optical_frame 的静态 TF。
    X_display = X_final
    t = X_display[:3, 3]
    r, p, y = matrix_to_rpy(X_display[:3, :3])
    q = matrix_to_quaternion(X_display[:3, :3])

    parent = "tool0" if mode == "eye_in_hand" else "arm_base_link"
    child = "camera_optical_frame"
    label = "相机在末端坐标系中的位置" if mode == "eye_in_hand" else "相机在基座坐标系中的位置"

    print(f"\n{'='*55}")
    print(f"  标定结果 (全相对运动 + 样本级 RANSAC + 鲁棒精化)")
    print(f"{'='*55}")
    print(f"  📐 {label} ({parent} → {child}):")
    print(f"     X = {t[0]:.6f} m")
    print(f"     Y = {t[1]:.6f} m")
    print(f"     Z = {t[2]:.6f} m")
    print(f"  🧭 四元数 xyzw: [{q[0]:.6f}, {q[1]:.6f}, {q[2]:.6f}, {q[3]:.6f}]")
    print(f"  📏 RPY: Roll={r:.6f}  Pitch={p:.6f}  Yaw={y:.6f} rad")
    print(f"  ── 精度 ──")
    print(f"  平移 RMS: {metrics['trans_rms_mm']:.3f} mm  ( <5 优, <10 可接受)")
    print(f"  旋转 RMS: {metrics['rot_rms_deg']:.3f}°  ( <1° 优, <3° 可接受)")
    print(f"  平移 Max: {metrics['trans_max_mm']:.3f} mm")
    print(f"  旋转 Max: {metrics['rot_max_deg']:.2f}°")
    print(f"\n  📡 ROS2 TF: ros2 run tf2_ros static_transform_publisher \\")
    print(f"    --x {t[0]:.6f} --y {t[1]:.6f} --z {t[2]:.6f} \\")
    print(f"    --qx {q[0]:.6f} --qy {q[1]:.6f} --qz {q[2]:.6f} --qw {q[3]:.6f} \\")
    print(f"    --frame-id {parent} --child-frame-id {child}")
    print(f"\n  4x4: {np.array2string(X_display, precision=6, suppress_small=True)}")

    # ── Bundle Adjustment 精化 ──
    ba_metrics = None
    if use_ba:
        print(f"\n{'='*55}")
        print(f"  Bundle Adjustment 精化 (重投影误差最小化)")
        print(f"{'='*55}")
        try:
            from bundle_adjust import run_bundle_adjustment
            ba_result = run_bundle_adjustment(
                samples_path, X_init=X_final,
                sample_indices=inlier_indices, verbose=True)
            if ba_result is not None:
                X_ba, Y_ba, ba_metrics = ba_result
                ba_t_rms = float(ba_metrics["translation_rms_mm"])
                ba_r_rms = float(ba_metrics["rotation_rms_deg"])
                X_final = X_ba
                X_display = X_final
                t = X_display[:3, 3]
                q = matrix_to_quaternion(X_display[:3, :3])
                r, p, y = matrix_to_rpy(X_display[:3, :3])
                metrics.update({
                    "trans_rms_mm": ba_t_rms,
                    "trans_max_mm": float(ba_metrics["translation_max_mm"]),
                    "rot_rms_deg": ba_r_rms,
                    "rot_max_deg": float(ba_metrics["rotation_max_deg"]),
                })
                print(f"\n  ✅ 采用 BA 精化结果")
                print(f"     X = {t[0]:.6f} m")
                print(f"     Y = {t[1]:.6f} m")
                print(f"     Z = {t[2]:.6f} m")
                print(f"  🧭 四元数 xyzw: [{q[0]:.6f}, {q[1]:.6f}, {q[2]:.6f}, {q[3]:.6f}]")
                print(f"  📏 RPY: Roll={r:.6f}  Pitch={p:.6f}  Yaw={y:.6f} rad")
        except ImportError:
            print(f"  ⚠ bundle_adjust 模块加载失败，跳过 BA 精化")
        except Exception as e:
            print(f"  ⚠ BA 精化失败: {e}")

    # ── 保存结果 ──
    result_path = os.path.splitext(samples_path)[0] + "_result.yaml"
    payload = {
        "handeye_mode": mode,
        "method": metrics.get("solver", "all-pairs robust hand-eye"),
        "parent_frame": parent,
        "child_frame": child,
        "transform_matrix": X_display.tolist(),
        "translation_m": t.tolist() if hasattr(t, 'tolist') else list(t),
        "quaternion_xyzw": list(q),
        "rpy_rad": [float(r), float(p), float(y)],
        "translation_rms_mm": metrics["trans_rms_mm"],
        "rotation_rms_deg": metrics["rot_rms_deg"],
        "translation_max_mm": metrics["trans_max_mm"],
        "rotation_max_deg": metrics["rot_max_deg"],
        "inlier_count": len(inlier_indices),
        "total_samples": total_n,
        "inlier_ratio": metrics.get("inlier_ratio", len(inlier_indices) / n),
        "motion_median_angle_deg": metrics.get("median_angle_deg"),
        "motion_max_angle_deg": metrics.get("max_angle_deg"),
        "motion_axis_ratio": metrics.get("axis_ratio"),
        "stored_transform_convention": (
            "camera_to_gripper (^gripper T_camera)" if mode == "eye_in_hand"
            else "camera_to_base (^base T_camera)"),
        "removed_samples": removed_orig,
        "ba_refined": use_ba and ba_metrics is not None,
    }
    intrinsics_binding = _load_intrinsics_binding(samples_path)
    if intrinsics_binding is not None:
        payload["intrinsics_binding"] = intrinsics_binding
    if ba_metrics:
        payload["ba_reprojection_rms_px"] = ba_metrics.get("ba_reprojection_rms_px")
        payload["method"] = ba_metrics.get("optimization_method",
                                            payload.get("method", ""))

    with open(result_path, 'w', encoding="utf-8") as f:
        yaml.safe_dump(payload, f, sort_keys=False, allow_unicode=True)
    print(f"\n  📁 结果已保存: {result_path}")

    # ── 自动部署到 handeye_bridge/config/ ──
    project_root = os.path.dirname(_here)  # screw_pick/
    deploy_dir = os.path.join(project_root, 'handeye_bridge', 'config')
    deploy_path = os.path.join(deploy_dir, 'samples_result.yaml')
    deploy_intrinsics = os.path.join(deploy_dir, 'camera_intrinsics.yaml')
    if (os.path.isdir(deploy_dir)
            and _intrinsics_binding_matches_file(
                intrinsics_binding, deploy_intrinsics)):
        with open(deploy_path, 'w', encoding="utf-8") as f:
            yaml.safe_dump(payload, f, sort_keys=False, allow_unicode=True)
        print(f"  📁 已自动部署到: {deploy_path}")
    elif os.path.isdir(deploy_dir):
        print("  ❌ 未部署手眼结果：运行配置内参与采集内参不一致")
    else:
        print(f"  ⚠ 部署目录不存在: {deploy_dir}")

    return X_display

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("用法: python solve.py samples.yaml [--simple|--minimal] [--ba]")
        print("  --simple   极简模式: 全部样本直接调用 OpenCV")
        print("  --minimal  兼容别名: 同 --simple")
        print("  --ba       Bundle Adjustment 精化 (需要 samples.yaml 含 corners_px)")
        sys.exit(1)
    simple = "--simple" in sys.argv or "--minimal" in sys.argv
    use_ba = "--ba" in sys.argv
    samples_path = sys.argv[1]
    solve(samples_path, simple=simple, use_ba=use_ba)
