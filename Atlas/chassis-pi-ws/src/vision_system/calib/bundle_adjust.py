#!/usr/bin/env python3
"""
手眼标定 Bundle Adjustment
==========================
直接最小化像素重投影误差，联合优化手眼矩阵 X 和标定板基座位姿 Y。

核心思想:
  传统方法: 图像 → PnP(易受距离/噪声影响) → target_to_camera → 手眼求解
  BA 方法:  图像 ──────────────────────────────→ 直接优化 X 使重投影误差最小

对于 eye-in-hand:
  A_i = T_base_gripper_i   (机器人正运动学)
  X   = T_gripper_camera   (待求手眼矩阵)
  Y   = T_base_target      (标定板在基座坐标系中的固定位姿)

约束: A_i * X * B_i = Y  →  B_i = inv(X) * inv(A_i) * Y

BA: min_{X,Y} Σ_i Σ_j || project(K, B_i, P_j) - p_ij_obs ||²
    其中 B_i = inv(X) * inv(A_i) * Y

优势:
  - 直接优化像素级误差，而非 PnP 导出的 6DOF 位姿
  - 对远距离/低分辨率棋盘格更鲁棒 (PnP 在低 px/格时不准)
  - 自然处理所有样本的全局一致性
"""

import sys
import os
import numpy as np
import cv2

_here = os.path.dirname(os.path.abspath(__file__))
if _here not in sys.path:
    sys.path.insert(0, _here)
from fk_utils import make_transform, invert_transform, rotation_angle_deg
from calib_utils import make_chessboard_objp

_SCIPY_OK = False
try:
    from scipy.optimize import least_squares
    _SCIPY_OK = True
except ImportError:
    pass


def load_intrinsics(path):
    """加载相机内参."""
    import yaml
    with open(path, encoding="utf-8") as f:
        cam = yaml.safe_load(f)
    mtx = np.array(cam["camera_matrix"]["data"], dtype=np.float64).reshape(3, 3)
    dist = np.array(cam["distortion_coefficients"]["data"], dtype=np.float64).reshape(-1)
    return mtx, dist


def project_points(mtx, dist, T_camera_target, obj_points):
    """将 3D 点投影到像素坐标.

    Args:
        mtx: 3x3 相机内参矩阵
        dist: 畸变系数 (4, 5, 8, 12 或 14 个元素)
        T_camera_target: 4x4 target→camera 变换
        obj_points: (M, 3) 标定板 3D 角点

    Returns:
        (M, 2) 投影像素坐标
    """
    R = T_camera_target[:3, :3]
    t = T_camera_target[:3, 3].reshape(3, 1)
    rvec, _ = cv2.Rodrigues(R)
    projected, _ = cv2.projectPoints(obj_points, rvec, t, mtx, dist)
    return projected.reshape(-1, 2)


def ba_residual(params, A_list, obj_points, all_corners_px, mtx, dist,
                sample_mask=None):
    """Bundle Adjustment 残差函数.

    Args:
        params: (12,) [rx, ry, rz, tx, ty, tz,  rYx, rYy, rYz, tYx, tYy, tYz]
                X(rvec, t) 手眼矩阵 + Y(rvec, t) 标定板基座位姿
        A_list: list of 4x4, 机器人位姿 A_i = gripper_in_base
        obj_points: (M, 3) 标定板角点 3D 坐标
        all_corners_px: list of (M, 2) 每个样本的角点像素坐标
        mtx, dist: 相机内参
        sample_mask: 可选 bool 数组，只优化指定样本

    Returns:
        (Σ 2*M,) 堆叠的像素残差
    """
    R_X = cv2.Rodrigues(params[:3])[0]
    t_X = params[3:6].reshape(3, 1)
    X = make_transform(R_X, t_X)

    R_Y = cv2.Rodrigues(params[6:9])[0]
    t_Y = params[9:12].reshape(3, 1)
    Y = make_transform(R_Y, t_Y)

    residuals = []
    n_samples = len(A_list)

    for i in range(n_samples):
        if sample_mask is not None and not sample_mask[i]:
            continue
        if all_corners_px[i] is None:
            continue

        A_i = A_list[i]
        # B_i = inv(X) * inv(A_i) * Y = target_to_camera for this sample
        B_i = invert_transform(X) @ invert_transform(A_i) @ Y

        # 投影
        predicted = project_points(mtx, dist, B_i, obj_points)
        observed = np.asarray(all_corners_px[i], dtype=np.float64).reshape(-1, 2)

        # 像素残差 (在 640x480 图像上 1px 是显著的)
        residuals.append((predicted - observed).ravel())

    if not residuals:
        return np.array([])
    return np.concatenate(residuals)


def run_bundle_adjustment(samples_path, X_init=None, intrinsics_path=None,
                          max_samples=None, sample_indices=None, verbose=True):
    """运行 Bundle Adjustment 精化手眼矩阵.

    Args:
        samples_path: samples.yaml 路径
        X_init: 4x4 初始手眼矩阵 (若为 None 则用 OpenCV TSAI 初始化)
        intrinsics_path: camera_intrinsics.yaml 路径
        max_samples: 最多使用的样本数 (None=全部)
        sample_indices: 允许参与优化的原始样本索引；用于复用求解质量门禁
        verbose: 是否打印信息

    Returns:
        (X_opt, Y_opt, metrics) or None
    """
    if not _SCIPY_OK:
        if verbose:
            print("  ❌ Bundle Adjustment 需要 scipy.optimize.least_squares")
        return None

    import yaml
    from calib_utils import load_samples

    # ── 加载数据 ──
    b2g_list, t2c_list, mode = load_samples(samples_path)
    if mode != "eye_in_hand":
        if verbose:
            print(f"  ❌ BA 目前仅支持 eye_in_hand, 当前模式: {mode}")
        return None

    with open(samples_path, encoding="utf-8") as f:
        data = yaml.safe_load(f)

    samples_raw = data.get("samples", [])
    sq_mm = data.get("square_size_mm", 15)
    chessboard = data.get("chessboard", "11x8")
    parts = chessboard.split("x")
    cols, rows = int(parts[0]), int(parts[1])
    sq_m = sq_mm / 1000.0
    obj_points = make_chessboard_objp(cols, rows, sq_m)

    # ── 加载内参 ──
    if intrinsics_path is None:
        intrinsics_path = os.path.join(_here, "camera_intrinsics.yaml")
    mtx, dist = load_intrinsics(intrinsics_path)

    # ── 准备角点数据 ──
    A_list = [np.asarray(s["gripper_in_base"], dtype=np.float64)
              for s in samples_raw]
    all_corners = []
    sample_mask = []

    allowed = (set(int(i) for i in sample_indices)
               if sample_indices is not None else None)
    for i, s in enumerate(samples_raw):
        corners_flat = s.get("corners_px")
        if (corners_flat is not None and len(corners_flat) == cols * rows * 2
                and (allowed is None or i in allowed)):
            corners = np.array(corners_flat, dtype=np.float64).reshape(-1, 2)
            all_corners.append(corners)
            sample_mask.append(True)
        else:
            all_corners.append(None)
            sample_mask.append(False)

    n_valid = sum(sample_mask)
    if n_valid < 4:
        if verbose:
            print(f"  ❌ 只有 {n_valid} 个样本有角点数据，需要 ≥4")
        return None

    # 可选：限制样本数
    if max_samples is not None and n_valid > max_samples:
        # 选重投影误差最小的样本
        reproj = [s.get("reprojection_error_px", 999) for s in samples_raw]
        valid_idx = [i for i, m in enumerate(sample_mask) if m]
        valid_idx.sort(key=lambda i: reproj[i])
        keep = set(valid_idx[:max_samples])
        sample_mask = [(i in keep) for i in range(len(sample_mask))]
        n_valid = sum(sample_mask)
        if verbose:
            print(f"  📊 选取 {n_valid} 个最佳样本 (按重投影误差)")

    if verbose:
        print(f"  📐 使用 {n_valid}/{len(samples_raw)} 个样本 (有角点数据)")
        print(f"  📷 棋盘格: {cols}x{rows}, 方格={sq_mm}mm")
        print(f"  📷 内参: fx={mtx[0,0]:.1f}, fy={mtx[1,1]:.1f}")

    # ── 初始值 ──
    if X_init is None:
        # 用 OpenCV TSAI 初始化
        from solve import prepare_inputs_for_lists
        R_gb, t_gb, R_tc, t_tc = prepare_inputs_for_lists(b2g_list, t2c_list)
        try:
            R_x, t_x = cv2.calibrateHandEye(R_gb, t_gb, R_tc, t_tc,
                                            method=cv2.CALIB_HAND_EYE_TSAI)
            X_init = make_transform(R_x, t_x.reshape(3))
        except Exception:
            if verbose:
                print("  ❌ OpenCV TSAI 初始化失败")
            return None

    # Y_init: 从 X_init 反算 board_in_base 的中位数
    # Y = A_i * X * B_i  (原始 PnP 的 B_i)
    Y_candidates = []
    for i in range(len(A_list)):
        if not sample_mask[i]:
            continue
        B_i = t2c_list[i]
        Y_candidates.append(A_list[i] @ X_init @ B_i)

    # 用中位数作为 Y_init (鲁棒)
    positions = np.array([Y[:3, 3] for Y in Y_candidates])
    median_pos = np.median(positions, axis=0)

    # 旋转: 取离中位平移最近的那个旋转
    dists = np.linalg.norm(positions - median_pos, axis=1)
    best_idx = int(np.argmin(dists))
    R_Y_init = Y_candidates[best_idx][:3, :3]

    Y_init = make_transform(R_Y_init, median_pos)

    # ── 初始残差 ──
    rvec_X, _ = cv2.Rodrigues(X_init[:3, :3])
    rvec_Y, _ = cv2.Rodrigues(Y_init[:3, :3])
    params_init = np.concatenate([
        rvec_X.ravel(), X_init[:3, 3],
        rvec_Y.ravel(), Y_init[:3, 3],
    ])

    res0 = ba_residual(params_init, A_list, obj_points, all_corners, mtx, dist,
                       sample_mask)
    rms0_px = float(np.sqrt(np.mean(res0 ** 2))) if len(res0) > 0 else float("inf")
    if verbose:
        print(f"  🔍 初始重投影 RMS: {rms0_px:.4f} px")

    # 初始几何一致性 (用旧的 evaluate)
    from solve import evaluate
    ev0 = evaluate(X_init, [b2g_list[i] for i in range(len(A_list)) if sample_mask[i]],
                   [t2c_list[i] for i in range(len(A_list)) if sample_mask[i]], mode)
    if verbose:
        print(f"  🔍 初始几何一致性: 平移RMS={ev0['trans_rms_mm']:.1f}mm  "
              f"旋转RMS={ev0['rot_rms_deg']:.2f}°")

    # ── 优化 ──
    if verbose:
        print(f"  🔄 Bundle Adjustment 优化中...")

    result = least_squares(
        ba_residual,
        params_init,
        args=(A_list, obj_points, all_corners, mtx, dist, sample_mask),
        method='trf',
        loss='soft_l1',
        f_scale=2.0,          # ~2px 内用 L2, 外用 L1 (鲁棒)
        x_scale='jac',
        ftol=1e-12,
        xtol=1e-12,
        gtol=1e-12,
        max_nfev=300,
        verbose=0,
    )

    # ── 提取结果 ──
    R_X_opt = cv2.Rodrigues(result.x[:3])[0]
    X_opt = make_transform(R_X_opt, result.x[3:6])

    R_Y_opt = cv2.Rodrigues(result.x[6:9])[0]
    Y_opt = make_transform(R_Y_opt, result.x[9:12])

    # ── 安全检查 ──
    # 1. 偏离初值过大
    R_diff = X_init[:3, :3].T @ R_X_opt
    ang_diff = rotation_angle_deg(R_diff)
    t_diff = np.linalg.norm(X_opt[:3, 3] - X_init[:3, 3]) * 1000.0
    if ang_diff > 45.0 or t_diff > 500.0:
        if verbose:
            print(f"  ⚠ BA 偏离初值过大 (ΔR={ang_diff:.1f}°, Δt={t_diff:.0f}mm)，"
                  f"回退到初值")
        return None

    # 2. 最终重投影误差
    res_final = ba_residual(result.x, A_list, obj_points, all_corners, mtx, dist,
                            sample_mask)
    rms_final_px = float(np.sqrt(np.mean(res_final ** 2)))
    if rms_final_px > max(rms0_px * 1.5, 1.0):
        if verbose:
            print(f"  ⚠ BA 重投影误差增大 ({rms0_px:.4f} → {rms_final_px:.4f} px)，"
                  f"回退到初值")
        return None

    # 3. 几何一致性
    b2g_used = [b2g_list[i] for i in range(len(A_list)) if sample_mask[i]]
    t2c_used_fallback = [t2c_list[i] for i in range(len(A_list)) if sample_mask[i]]
    ev_opt = evaluate(X_opt, b2g_used, t2c_used_fallback, mode)

    # 4. 检查 X 平移量级 (eye_in_hand 相机距末端一般 < 0.5m)
    if np.linalg.norm(X_opt[:3, 3]) > 0.60:
        if verbose:
            print(f"  ❌ BA 优化后相机距末端 {np.linalg.norm(X_opt[:3,3]):.3f}m，"
                  f"疑似退化解")
        return None

    if verbose:
        print(f"  ✅ BA 完成")
        print(f"     最终重投影 RMS: {rms_final_px:.4f} px "
              f"({'✅' if rms_final_px < 0.3 else '⚠' if rms_final_px < 0.5 else '❌'})")
        print(f"     几何一致性:")
        print(f"       平移 RMS: {ev_opt['trans_rms_mm']:.2f} mm "
              f"({'✅' if ev_opt['trans_rms_mm'] < 5 else '⚠' if ev_opt['trans_rms_mm'] < 10 else '❌'})")
        print(f"       旋转 RMS: {ev_opt['rot_rms_deg']:.2f}° "
              f"({'✅' if ev_opt['rot_rms_deg'] < 1 else '⚠' if ev_opt['rot_rms_deg'] < 2 else '❌'})")
        print(f"     手眼矩阵 X (相机在末端中):")
        print(f"       t=[{X_opt[0,3]:.6f} {X_opt[1,3]:.6f} {X_opt[2,3]:.6f}] m")
        print(f"     标定板基座位姿 Y:")
        print(f"       t=[{Y_opt[0,3]:.6f} {Y_opt[1,3]:.6f} {Y_opt[2,3]:.6f}] m")

    metrics = {
        "ba_reprojection_rms_px": rms_final_px,
        "ba_reprojection_rms_initial_px": rms0_px,
        "translation_rms_mm": ev_opt["trans_rms_mm"],
        "translation_max_mm": ev_opt["trans_max_mm"],
        "rotation_rms_deg": ev_opt["rot_rms_deg"],
        "rotation_max_deg": ev_opt["rot_max_deg"],
        "samples_used": n_valid,
        "optimization_method": "Bundle Adjustment (reprojection error)",
    }

    return X_opt, Y_opt, metrics


if __name__ == "__main__":
    samples_path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(_here, "samples.yaml")
    result = run_bundle_adjustment(samples_path, verbose=True)
    if result:
        X, Y, metrics = result
        print(f"\n  最终 X:\n{np.array2string(X, precision=6, suppress_small=True)}")
