#!/usr/bin/env python3
"""
手眼标定精度验证
================
用法:
  # 从 samples.yaml 求解并验证
  python verify.py samples.yaml

  # 用已有的标定结果验证
  python verify.py samples.yaml --result handeye_result.yaml

输出:
  - 每个样本反算的一致性误差
  - X/Y/Z 各方向的偏差
  - 综合精度评级
  - 标记可疑样本
"""

import sys
import os
import math
import numpy as np
import yaml
import cv2

_here = os.path.dirname(os.path.abspath(__file__))
if _here not in sys.path:
    sys.path.insert(0, _here)
from fk_utils import (make_transform, invert_transform,
                       matrix_to_rpy, rotation_angle_deg, mean_rotation)
from calib_utils import load_samples, prepare_inputs, load_sample_metadata
from solve import solve_handeye_core  # 统一求解逻辑


def load_result(path):
    """加载 samples_result.yaml 中的手眼矩阵.

    新版 solve.py 直接输出 ^gripper T_camera。
    旧结果没有 stored_transform_convention 字段，按历史格式取逆兼容。
    """
    with open(path, encoding="utf-8") as f:
        data = yaml.safe_load(f)
    stored = np.array(data["transform_matrix"], dtype=np.float64)
    convention = str(data.get("stored_transform_convention", "legacy"))
    if convention.startswith("camera_to_gripper"):
        return stored
    return invert_transform(stored)


def main():
    if len(sys.argv) < 2:
        print("用法: python verify.py samples.yaml [--result handeye_result.yaml]")
        sys.exit(1)

    samples_path = sys.argv[1]
    result_path = None
    if "--result" in sys.argv:
        idx = sys.argv.index("--result")
        if idx + 1 >= len(sys.argv):
            print("❌ --result 后面需要跟文件名，例如: python verify.py samples.yaml --result samples_result.yaml")
            sys.exit(1)
        result_path = sys.argv[idx + 1]

    b2g_list, t2c_list, mode = load_samples(samples_path)

    if result_path:
        X = load_result(result_path)
        print(f"\n  使用已有标定结果: {result_path}")
    else:
        print("\n  正在求解手眼矩阵 (RANSAC + LM)...")
        reproj_errors, laplacian_vars, corner_rms = load_sample_metadata(samples_path)
        result = solve_handeye_core(b2g_list, t2c_list, mode,
                                    reproj_errors, laplacian_vars, corner_rms,
                                    verbose=True)
        if result is None:
            print("  ❌ 求解失败")
            return
        X, inlier_indices, removed, metrics = result
        if removed:
            print(f"  🧹 自动剔除 {len(removed)} 个异常样本: {[i+1 for i in removed]}")
            print(f"     参与求解: {len(inlier_indices)} / {len(b2g_list)} 组")

    n = len(b2g_list)

    # 对每个样本计算一致性参考量
    # eye_in_hand:  base → target (标定板在基座坐标系中的位姿)
    # eye_to_hand:  gripper → target (标定板在末端坐标系中的位姿)
    ref_label = "base→target" if mode == "eye_in_hand" else "gripper→target"
    ref_poses = []
    for b2g, t2c in zip(b2g_list, t2c_list):
        if mode == "eye_in_hand":
            ref = b2g @ X @ t2c       # T_base_target
        else:
            ref = invert_transform(b2g) @ X @ t2c  # T_gripper_target
        ref_poses.append(ref)

    # ── MAD 检测异常样本，基于 inlier 计算参考均值 ──
    positions_all = np.array([t[:3, 3] for t in ref_poses])
    Rs_all = [t[:3, :3] for t in ref_poses]

    # 先以全部样本的 median 作为初始参考，做一轮 MAD
    med_pos = np.median(positions_all, axis=0)
    med_R = mean_rotation(Rs_all)  # mean_rotation 本身对 outlier 有一定鲁棒性
    t_err_all = np.linalg.norm(positions_all - med_pos, axis=1)
    r_err_all = np.array([rotation_angle_deg(med_R.T @ R) for R in Rs_all])

    mad_t = np.median(np.abs(t_err_all - np.median(t_err_all))) * 1.4826
    mad_r = np.median(np.abs(r_err_all - np.median(r_err_all))) * 1.4826
    t_thresh = np.median(t_err_all) + 3.0 * max(mad_t, 1e-5)
    r_thresh = np.median(r_err_all) + 3.0 * max(mad_r, 1e-5)

    inlier_mask = np.array([
        not (t_err_all[i] > t_thresh or r_err_all[i] > r_thresh)
        for i in range(n)
    ])
    outlier_indices = [i + 1 for i in range(n) if not inlier_mask[i]]
    inlier_indices = [i + 1 for i in range(n) if inlier_mask[i]]

    # 用 inlier 计算参考均值
    positions_inlier = positions_all[inlier_mask]
    Rs_inlier = [Rs_all[i] for i in range(n) if inlier_mask[i]]

    if len(positions_inlier) == 0:
        print("  ❌ 没有足够的 inlier 样本")
        return

    mean_pos = np.mean(positions_inlier, axis=0)
    mean_R = mean_rotation(Rs_inlier)

    # 所有样本相对 inlier 均值的误差
    t_errors = np.linalg.norm(positions_all - mean_pos, axis=1) * 1000  # mm
    r_errors = np.array([rotation_angle_deg(mean_R.T @ R) for R in Rs_all])

    # inlier-only RMS
    t_errors_inlier = t_errors[inlier_mask]
    r_errors_inlier = r_errors[inlier_mask]
    t_rms_in = np.sqrt(np.mean(t_errors_inlier**2))
    r_rms_in = np.sqrt(np.mean(r_errors_inlier**2))

    # ── 输出 ──
    print(f"\n{'='*60}")
    print(f"  精度验证报告  |  模式: {mode}  |  样本: {n}")
    print(f"{'='*60}")
    if outlier_indices:
        print(f"  🧹 异常样本: {outlier_indices} (共 {len(outlier_indices)} 组，不参与参考均值计算)")
        print(f"  ✅ Inlier:  {inlier_indices} (共 {len(inlier_indices)} 组)")
    print(f"  参考位姿 ({ref_label}, inlier 反算均值):")
    print(f"    X={mean_pos[0]:.6f}  Y={mean_pos[1]:.6f}  Z={mean_pos[2]:.6f} m")
    r, p, y = matrix_to_rpy(mean_R)
    print(f"    Roll={r:.6f}  Pitch={p:.6f}  Yaw={y:.6f} rad")
    print(f"{'='*60}")

    # Inlier 精度
    print(f"\n  ── Inlier 精度 ({len(inlier_indices)} 组) ──")
    print(f"  平移 RMS:     {t_rms_in:.3f} mm")
    print(f"  平移 Max:     {np.max(t_errors_inlier):.3f} mm")
    print(f"  旋转 RMS:     {r_rms_in:.3f}°")
    print(f"  旋转 Max:     {np.max(r_errors_inlier):.3f}°")

    # 全部样本精度
    t_rms_all = np.sqrt(np.mean(t_errors**2))
    t_max_all = np.max(t_errors)
    r_rms_all = np.sqrt(np.mean(r_errors**2))
    r_max_all = np.max(r_errors)

    print(f"\n  ── 全部样本精度 ({n} 组，含异常值) ──")
    print(f"  平移 RMS:     {t_rms_all:.3f} mm")
    print(f"  平移 Max:     {t_max_all:.3f} mm")
    print(f"  X 方向 ±{np.std(positions_all, axis=0)[0]*1000:.3f}  "
          f"Y 方向 ±{np.std(positions_all, axis=0)[1]*1000:.3f}  "
          f"Z 方向 ±{np.std(positions_all, axis=0)[2]*1000:.3f} mm")
    print(f"  旋转 RMS:     {r_rms_all:.3f}°")
    print(f"  旋转 Max:     {r_max_all:.3f}°")

    # 评级 (基于 inlier)
    print(f"\n  ── 综合评级 (基于 inlier) ──")
    if t_rms_in < 3 and r_rms_in < 0.5:
        print(f"  ✅ 优秀 — 平移 <3mm 且 旋转 <0.5°")
    elif t_rms_in < 8 and r_rms_in < 1.5:
        print(f"  👍 良好 — 平移 <8mm 且 旋转 <1.5°")
    elif t_rms_in < 15 and r_rms_in < 3:
        print(f"  ⚠️  可接受 — 平移 <15mm 且 旋转 <3°，建议增加样本或重采")
    else:
        print(f"  ❌ 精度不足 — 可能原因: 内参不准 / 方格尺寸填错 / 运动范围不够 / 角点翻转")

    # 可疑样本 (基于 inlier RMS)
    bad = [(i, t_errors[i], r_errors[i]) for i in range(n)
           if t_errors[i] > t_rms_in * 2.5 or r_errors[i] > r_rms_in * 2.5]
    if bad:
        print(f"\n  ⚠️  可疑样本 (误差 > 2.5×inlier_RMS):")
        for i, te, re in bad:
            print(f"    样本 #{i+1}: 平移={te:.3f}mm  旋转={re:.3f}°")

    # 逐样本表
    print(f"\n  ── 逐样本误差 (相对 inlier 均值) ──")
    print(f"  {'样本':<6} {'平移(mm)':<12} {'旋转(°)':<10} {'状态'}")
    print(f"  {'-'*40}")
    for i in range(n):
        if inlier_mask[i]:
            s = "✓" if t_errors[i] <= t_rms_in * 1.5 else "⚠"
        else:
            s = "✗ 异常"
        print(f"  #{i+1:<5} {t_errors[i]:<12.3f} {r_errors[i]:<10.3f} {s}")

    print(f"\n{'='*60}")
    print(f"  反算得到的固定参考位姿 ({ref_label}, inlier 均值):")
    print(f"  X={mean_pos[0]:.6f}  Y={mean_pos[1]:.6f}  Z={mean_pos[2]:.6f} m")
    print(f"  此值由 {len(inlier_indices)} 个 inlier 样本反算得出，波动见上表")
    print(f"{'='*60}\n")


if __name__ == "__main__":
    main()
