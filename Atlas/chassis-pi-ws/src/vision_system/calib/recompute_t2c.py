#!/usr/bin/env python3
"""
重新计算 samples.yaml 中的 target_to_camera
==========================================
用途: 当相机内参更新后，用存储的原始角点 + 新内参重新做 PnP，
     无需重新采集所有机械臂位姿。

用法:
    python recompute_t2c.py samples.yaml [camera_intrinsics.yaml]

如果不指定内参文件，默认读取同目录下的 camera_intrinsics.yaml。
"""

import sys
import os
import shutil
import numpy as np
import cv2
import yaml

_here = os.path.dirname(os.path.abspath(__file__))
if _here not in sys.path:
    sys.path.insert(0, _here)

from fk_utils import make_transform
from calib_utils import make_chessboard_objp


def load_intrinsics(path):
    with open(path, encoding="utf-8") as f:
        cam = yaml.safe_load(f)
    mtx = np.array(cam["camera_matrix"]["data"]).reshape(3, 3)
    dist = np.array(cam["distortion_coefficients"]["data"])
    return mtx, dist


def solve_pnp(obj_points_3d, img_points_2d, mtx, dist):
    """PnP 求解 (与 collect_samples.py 保持一致)"""
    method_chain = [(cv2.SOLVEPNP_IPPE, "IPPE")]
    if hasattr(cv2, "SOLVEPNP_SQPNP"):
        method_chain.append((cv2.SOLVEPNP_SQPNP, "SQPNP"))
    method_chain.append((cv2.SOLVEPNP_ITERATIVE, "ITERATIVE"))

    best_error = float("inf")
    best_result = None

    for method_flag, _ in method_chain:
        try:
            result = cv2.solvePnPGeneric(
                obj_points_3d, img_points_2d, mtx, dist, flags=method_flag
            )
            retval, rvecs, tvecs = result[0], result[1], result[2]
        except cv2.error:
            continue
        if retval is None or retval == 0:
            continue

        if isinstance(rvecs, np.ndarray):
            n_solutions = rvecs.shape[0] if rvecs.ndim >= 2 else 0
        elif isinstance(rvecs, (list, tuple)):
            n_solutions = len(rvecs)
        else:
            n_solutions = 0

        for i in range(n_solutions):
            rv = (rvecs[i].reshape(3, 1).astype(np.float64)
                  if isinstance(rvecs, np.ndarray)
                  else np.array(rvecs[i], dtype=np.float64).reshape(3, 1))
            tv = (tvecs[i].reshape(3, 1).astype(np.float64)
                  if isinstance(tvecs, np.ndarray)
                  else np.array(tvecs[i], dtype=np.float64).reshape(3, 1))
            if tv[2, 0] <= 0:
                continue
            projected, _ = cv2.projectPoints(obj_points_3d, rv, tv, mtx, dist)
            error = float(np.sqrt(np.mean(
                np.sum((projected.reshape(-1, 2) - img_points_2d.reshape(-1, 2)) ** 2,
                       axis=1)
            )))
            if error < best_error:
                best_error = error
                best_result = (rv, tv)

    if best_result is None:
        try:
            ok, rv, tv = cv2.solvePnP(
                obj_points_3d, img_points_2d, mtx, dist,
                flags=cv2.SOLVEPNP_ITERATIVE,
            )
            if not ok:
                return None, None
            best_result = (rv, tv)
        except cv2.error:
            return None, None
    else:
        rv, tv = best_result

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
    err = None
    try:
        projected, _ = cv2.projectPoints(obj_points_3d, rv, tv, mtx, dist)
        err = float(np.sqrt(np.mean(
            np.sum((projected.reshape(-1, 2) - img_points_2d.reshape(-1, 2)) ** 2,
                   axis=1)
        )))
    except cv2.error:
        pass
    return T, err


def main():
    samples_path = sys.argv[1] if len(sys.argv) >= 2 else None
    if samples_path is None:
        print("用法: python recompute_t2c.py samples.yaml [camera_intrinsics.yaml]")
        sys.exit(1)

    intrinsics_path = (sys.argv[2] if len(sys.argv) >= 3
                       else os.path.join(os.path.dirname(os.path.abspath(samples_path)),
                                         "camera_intrinsics.yaml"))
    if not os.path.exists(intrinsics_path):
        intrinsics_path = os.path.join(_here, "camera_intrinsics.yaml")
    if not os.path.exists(intrinsics_path):
        print(f"❌ 找不到相机内参文件: {intrinsics_path}")
        sys.exit(1)

    print(f"📷 相机内参: {intrinsics_path}")
    mtx, dist = load_intrinsics(intrinsics_path)
    print(f"   fx={mtx[0,0]:.2f} fy={mtx[1,1]:.2f} cx={mtx[0,2]:.2f} cy={mtx[1,2]:.2f}")

    with open(samples_path, encoding="utf-8") as f:
        data = yaml.safe_load(f)
    samples = data.get("samples", [])

    # 从旧内参推断棋盘格尺寸
    old_intrinsics = os.path.join(_here, "camera_intrinsics.yaml")
    chess = None
    if os.path.exists(old_intrinsics):
        with open(old_intrinsics, encoding="utf-8") as f:
            old_cam = yaml.safe_load(f)
        chess = old_cam.get("chessboard", None)
    if chess is None:
        print("⚠ 无法推断棋盘格尺寸，默认使用 11x8 15mm")
        cols, rows, sq_mm = 11, 8, 15
    else:
        parts = chess.split("x")
        cols, rows = int(parts[0]), int(parts[1])
        sq_mm = old_cam.get("square_size_mm", 15)

    sq_m = sq_mm / 1000.0
    objp = make_chessboard_objp(cols, rows, sq_m)
    print(f"📐 棋盘格: {cols}x{rows} 内角点, 方格={sq_mm}mm")

    updated = 0
    skipped_no_corners = 0
    skipped_pnp_fail = 0

    for s in samples:
        corners_flat = s.get("corners_px")
        if corners_flat is None:
            skipped_no_corners += 1
            continue

        corners = np.array(corners_flat, dtype=np.float32).reshape(-1, 1, 2)
        if corners.shape[0] != cols * rows:
            print(f"  ⚠ 样本 #{s['id']}: 角点数 {corners.shape[0]} != {cols*rows}，跳过")
            skipped_no_corners += 1
            continue

        t2c_new, err_new = solve_pnp(objp, corners, mtx, dist)
        if t2c_new is None:
            print(f"  ❌ 样本 #{s['id']}: PnP 失败")
            skipped_pnp_fail += 1
            continue

        t2c_old = np.array(s["target_to_camera"])
        delta_t = np.linalg.norm(t2c_new[:3, 3] - t2c_old[:3, 3]) * 1000

        s["target_to_camera"] = t2c_new.tolist()
        s["reprojection_error_px"] = float(err_new) if err_new is not None else None
        updated += 1

        print(f"  ✓ 样本 #{s['id']}: 重投影={err_new:.4f}px  "
              f"Δ平移={delta_t:.1f}mm")

    # 保存
    backup = samples_path + ".bak"
    import shutil
    shutil.copy2(samples_path, backup)

    with open(samples_path, "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, sort_keys=False, allow_unicode=True)

    print(f"\n{'='*50}")
    print(f"  更新: {updated}  跳过(无角点): {skipped_no_corners}  "
          f"跳过(PnP失败): {skipped_pnp_fail}")
    print(f"  原始文件备份: {backup}")
    print(f"  已保存: {samples_path}")
    print(f"{'='*50}")
    print(f"\n  下一步: python solve.py {samples_path}")


if __name__ == "__main__":
    main()
