#!/usr/bin/env python3
"""
标定数据诊断 — 排查 53mm RMS 误差的根因

用法: python calib/diagnose.py calib/samples.yaml
"""

import sys
import os
import numpy as np
import cv2
import yaml

_here = os.path.dirname(os.path.abspath(__file__))
if _here not in sys.path:
    sys.path.insert(0, _here)
from fk_utils import rotation_angle_deg, quaternion_to_matrix, make_transform
from calib_utils import load_samples


def diagnose(samples_path):
    b2g_list, t2c_list, mode = load_samples(samples_path)
    n = len(b2g_list)

    # 加载原始 YAML 数据 (用于访问 square_size_mm 等元数据)
    with open(samples_path) as f:
        data = yaml.safe_load(f)

    print(f"\n{'='*60}")
    print(f"  标定数据诊断 | {mode} | {n} 样本")
    print(f"{'='*60}")

    # ── 1. target_to_camera 旋转一致性 (检测棋盘格翻转) ──
    print(f"\n  ── ① 棋盘格翻转检测 ──")
    print(f"  比较相邻样本 target_to_camera 的旋转变化:")
    flips = []
    for i in range(1, n):
        R_prev = t2c_list[i-1][:3, :3]
        R_curr = t2c_list[i][:3, :3]
        # R_diff = R_prev.T @ R_curr: 如果接近 180°，说明棋盘格翻转了
        angle = rotation_angle_deg(R_prev.T @ R_curr)
        if angle > 120:
            flips.append((i+1, angle))
    if flips:
        print(f"  ⚠️  发现 {len(flips)} 对相邻样本旋转 > 120° (疑似翻转):")
        for idx, ang in flips:
            print(f"     样本 #{idx-1} → #{idx}: 旋转 {ang:.1f}°")
    else:
        print(f"  ✅ 未发现相邻样本翻转")

    # ── 2. target_to_camera 旋转整体分布 ──
    print(f"\n  ── ② target_to_camera 旋转分布 ──")
    angles_from_identity = []
    for i, t2c in enumerate(t2c_list):
        R = t2c[:3, :3]
        ang = rotation_angle_deg(R)
        angles_from_identity.append(ang)
    angles = np.array(angles_from_identity)
    print(f"  旋转角分布: mean={np.mean(angles):.1f}°  std={np.std(angles):.1f}°  "
          f"min={np.min(angles):.1f}°  max={np.max(angles):.1f}°")
    # 如果旋转角分布成两个簇 (0°附近 和 180°附近)，说明有系统性翻转
    hist, edges = np.histogram(angles, bins=[0, 30, 90, 150, 180])
    print(f"  分布: 0-30°:{hist[0]}  30-90°:{hist[1]}  90-150°:{hist[2]}  150-180°:{hist[3]}")
    if hist[3] > 0:
        print(f"  ⚠️  {hist[3]} 个样本的旋转角在 150-180° 范围，疑似棋盘格翻转!")

    # ── 3. 相机→棋盘距离分布 ──
    print(f"\n  ── ③ 相机→棋盘距离 ──")
    dists = np.array([t2c[2, 3] * 1000 for t2c in t2c_list])
    print(f"  mean={np.mean(dists):.0f}mm  std={np.std(dists):.0f}mm  "
          f"min={np.min(dists):.0f}mm  max={np.max(dists):.0f}mm")

    # 检查相机内参 fx，估算 px/格
    # square_size_mm 优先从 samples.yaml 取 (采集时实际使用的值),
    # 其次从 camera_intrinsics.yaml 取 (标定时使用的值)
    sq_mm = data.get("square_size_mm", None)
    intrinsics_file = os.path.join(_here, "camera_intrinsics.yaml")
    with open(intrinsics_file) as f:
        cam_data = yaml.safe_load(f)
    if sq_mm is None:
        sq_mm = cam_data.get("square_size_mm", 15)
    else:
        cam_sq = cam_data.get("square_size_mm", None)
        if cam_sq is not None and cam_sq != sq_mm:
            print(f"  ⚠️  samples.yaml 中 square_size_mm={sq_mm}，但 camera_intrinsics.yaml 中={cam_sq}，不一致!")
            print(f"     使用 samples.yaml 的值 ({sq_mm}mm) 计算 px/格")
    fx = cam_data["camera_matrix"]["data"][0]
    px_per_sq = fx * sq_mm / dists
    print(f"  px/格 (基于 fx={fx:.1f}): mean={np.mean(px_per_sq):.0f}  min={np.min(px_per_sq):.0f}")
    low_res = np.sum(px_per_sq < 15)
    if low_res > 0:
        print(f"  ⚠️  {low_res} 个样本 px/格 < 15 (分辨率不足，PnP 误差大)")
    low_res2 = np.sum(px_per_sq < 20)
    if low_res2 > 0:
        print(f"  ⚠️  {low_res2} 个样本 px/格 < 20 (建议靠近标定板)")

    # ── 4. 机械臂运动范围 ──
    print(f"\n  ── ④ 机械臂末端运动范围 ──")
    b2g_pos = np.array([b2g[:3, 3] for b2g in b2g_list])
    for axis, name in enumerate(['X', 'Y', 'Z']):
        vals = b2g_pos[:, axis]
        print(f"  {name}: span={np.max(vals)-np.min(vals):.3f}m  "
              f"std={np.std(vals):.3f}m  "
              f"[{np.min(vals):.3f}, {np.max(vals):.3f}]")

    # 检查旋转多样性
    b2g_R = [b2g[:3, :3] for b2g in b2g_list]
    axes = np.array([cv2.Rodrigues(R)[0].ravel() for R in b2g_R])
    _, s, _ = np.linalg.svd(axes - axes.mean(axis=0))
    print(f"  旋转轴奇异值: {s}")
    if s[-1] < 0.05:
        print(f"  ⚠️  旋转多样性不足 (最小奇异值={s[-1]:.4f})！机械臂需要不同方向的旋转")

    # ── 5. 一致性预检 (不依赖求解结果) ──
    print(f"\n  ── ⑤ 位姿一致性预检 ──")
    # 对于 eye_in_hand: base_to_target = b2g_i * X * t2c_i = constant
    # 如果 b2g_i^{-1} * b2g_j 和 t2c_i * t2c_j^{-1} 有相同的旋转角，
    # 说明数据内部一致 (Schmidt 1991 定理)
    from fk_utils import invert_transform
    pair_errors = np.array([])  # 默认值, 防止 n < 10 时 NameError
    if n >= 10:
        pair_errors = []
        for _ in range(min(500, n * (n-1) // 2)):
            i, j = np.random.choice(n, 2, replace=False)
            # A = b2g_j^{-1} * b2g_i (relative gripper motion)
            # B = t2c_j * t2c_i^{-1} (relative camera motion, 但由于 eye_in_hand B = target2cam_j * cam2target_i)
            # 实际上: b2g_i * X * t2c_i = b2g_j * X * t2c_j
            # => b2g_j^{-1} * b2g_i * X = X * t2c_j * t2c_i^{-1}
            # A = b2g_j^{-1} * b2g_i, B = t2c_j * t2c_i^{-1}
            # 检查: rotation_angle(A) 和 rotation_angle(B) 应该接近
            A = invert_transform(b2g_list[j]) @ b2g_list[i]
            B = t2c_list[j] @ invert_transform(t2c_list[i])
            ang_A = rotation_angle_deg(A[:3, :3])
            ang_B = rotation_angle_deg(B[:3, :3])
            pair_errors.append(abs(ang_A - ang_B))
        pair_errors = np.array(pair_errors)
        print(f"  随机 {len(pair_errors)} 对样本的 |angle(A)-angle(B)|:")
        print(f"    mean={np.mean(pair_errors):.1f}°  median={np.median(pair_errors):.1f}°  "
              f"max={np.max(pair_errors):.1f}°")
        if np.median(pair_errors) > 5:
            print(f"  ❌ 角度差异大! 数据严重不一致，可能原因:")
            print(f"     1) 相机内参不准确")
            print(f"     2) 机械臂位姿数据有误")
            print(f"     3) 棋盘格 180° 翻转")
        elif np.median(pair_errors) > 2:
            print(f"  ⚠️  角度差异中等，可能存在系统性误差")
        else:
            print(f"  ✅ 角度差异小，数据内部一致性好")
    else:
        print(f"  ⚠️  样本数不足 ({n}<10)，跳过位姿一致性预检 (至少需要 10 组)")

    # ── 6. 快速求解对比 ──
    print(f"\n  ── ⑥ 快速手眼求解 (不剔除异常值) ──")
    from calib_utils import prepare_inputs
    from fk_utils import mean_rotation

    R_gb, t_gb, R_tc, t_tc = prepare_inputs(b2g_list, t2c_list, mode)
    methods = {
        "TSAI": cv2.CALIB_HAND_EYE_TSAI,
        "PARK": cv2.CALIB_HAND_EYE_PARK,
        "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS,
    }
    for name, mid in methods.items():
        try:
            R_x, t_x = cv2.calibrateHandEye(R_gb, t_gb, R_tc, t_tc, method=mid)
            X = make_transform(R_x, t_x.reshape(3))
            # 一致性
            constants = []
            for b2g, t2c in zip(b2g_list, t2c_list):
                if mode == "eye_in_hand":
                    c = b2g @ X @ t2c
                else:
                    c = invert_transform(b2g) @ X @ t2c
                constants.append(c)
            trans = np.array([c[:3, 3] for c in constants])
            t_rms = float(np.sqrt(np.mean(
                np.linalg.norm(trans - np.mean(trans, axis=0), axis=1)**2))) * 1000
            mean_R = mean_rotation([c[:3, :3] for c in constants])
            r_rms = float(np.sqrt(np.mean(
                [rotation_angle_deg(mean_R.T @ c[:3, :3])**2 for c in constants])))
            print(f"  {name:<12} 平移RMS={t_rms:.1f}mm  旋转RMS={r_rms:.1f}°")
        except Exception as e:
            print(f"  {name:<12} 失败: {e}")

    # ── 7. 建议 ──
    print(f"\n{'='*60}")
    print(f"  诊断建议")
    print(f"{'='*60}")

    issues = []
    if flips:
        issues.append(f"发现 {len(flips)} 处相邻样本旋转 >120° → 棋盘格 180° 翻转问题")
    if hist[3] > n * 0.05:
        issues.append(f"大量样本 ({hist[3]}) 中 target_to_camera 旋转 >150° → 系统性翻转")
    if low_res > n * 0.1:
        issues.append(f"大量样本 px/格 <15 → 分辨率不足，请靠近标定板")
    if np.median(pair_errors) > 5:
        issues.append("角度一致性差 → 相机内参或位姿数据有问题")
    if s[-1] < 0.05:
        issues.append("旋转多样性不足 → 机械臂需要更多方向的旋转")

    if not issues:
        print(f"  ⚠️  未发现明显异常，但误差仍大。请检查:")
        print(f"     1. SQUARE_SIZE_MM 是否与实际棋盘格一致? (当前 {sq_mm}mm)")
        print(f"     2. 相机内参标定是否准确? (重新运行 camera_calib.py)")
        print(f"     3. 机械臂位姿 (/arm/pose) 是否准确? (ros2 topic echo /arm/pose)")
        print(f"     4. 采集时机械臂是否静止? (运动中采集会导致模糊)")
        print(f"     5. 如用 ChArUco 标记板替代黑白棋盘格 (推荐)")
    else:
        for i, issue in enumerate(issues):
            print(f"  {i+1}. {issue}")
    print()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("用法: python diagnose.py samples.yaml")
        sys.exit(1)
    diagnose(sys.argv[1])
