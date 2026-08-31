#!/usr/bin/env python3
"""
============================================================
  相机标定 — 简单版
============================================================
用途: 标定单目相机内参 (fx, fy, cx, cy, 畸变参数)

用法:
  1. 把棋盘格参数填好（下面有标记 ★ 的地方）
  2. 运行: python camera_calib.py
  3. 拿棋盘格对着相机，换不同角度
  4. 画面显示角点检测成功后，按 空格键 拍照
  5. 拍 20~30 张后，按 C 键计算
  6. 结果保存为 camera_intrinsics.yaml

提示:
  - 棋盘格尽量占画面不同区域（左上、右下、中间…）
  - 每次倾斜角度不同（正对、左倾、右倾、上倾、下倾）
  - 远近也要变化
============================================================
"""


import numpy as np
import os
import cv2
import sys
import argparse
import select
import time
import yaml
from datetime import datetime
from calib_utils import (find_chessboard, refine_chessboard_corners,
                         validate_chessboard_geometry, make_chessboard_objp)

# 统一使用在线检测包中的相机初始化函数；源码未构建时也可通过相对路径导入。
_here = os.path.dirname(os.path.abspath(__file__))
_vision_source = os.path.normpath(os.path.join(_here, "..", "vison_topic"))
if _vision_source not in sys.path:
    sys.path.insert(0, _vision_source)
from vison_topic.camera_utils import DEFAULT_CAMERA_SETTINGS, open_project_camera

# ╔══════════════════════════════════════════════════════════╗
# ║           ★ 在这里填写你的参数 ★                         ║
# ╚══════════════════════════════════════════════════════════╝

# --- 棋盘格 ---
# 注意：填的是"内角点"数量，不是方格数！
# 例如 10×7 方格 → 内角点 = 9×6
CHESSBOARD_COLS = 11   # 横向内角点数，例如 9
CHESSBOARD_ROWS = 8   # 纵向内角点数，例如 6
SQUARE_SIZE_MM  = 15   # 每个方格边长 (毫米)，例如 25

# --- 相机 ---
CAMERA_INDEX = 0       # USB 相机通常是 0 或 2
CAMERA_WIDTH = int(DEFAULT_CAMERA_SETTINGS["width"])
CAMERA_HEIGHT = int(DEFAULT_CAMERA_SETTINGS["height"])
CAMERA_FPS = float(DEFAULT_CAMERA_SETTINGS["fps"])

# --- 保存路径 ---
OUTPUT_FILE = os.path.join(_here, "camera_intrinsics.yaml")
DEBUG_IMAGE_FILE = os.path.join(_here, "debug", "calib_camera_debug.jpg")


# ╔══════════════════════════════════════════════════════════╗
# ║            下面不用改                                   ║
# ╚══════════════════════════════════════════════════════════╝

def validate_params():
    """检查用户是否填了参数"""
    missing = []
    if CHESSBOARD_COLS is None:
        missing.append("CHESSBOARD_COLS (横向内角点数)")
    if CHESSBOARD_ROWS is None:
        missing.append("CHESSBOARD_ROWS (纵向内角点数)")
    if SQUARE_SIZE_MM is None:
        missing.append("SQUARE_SIZE_MM (方格边长 mm)")
    if missing:
        print("\n❌ 以下参数还没填写，请在脚本顶部填写后重新运行：")
        for m in missing:
            print(f"   - {m}")
        print("\n例如：10×7 方格 → CHESSBOARD_COLS=9, CHESSBOARD_ROWS=6")
        return False
    return True


def main():
    parser = argparse.ArgumentParser(description="相机标定")
    parser.add_argument("--headless", action="store_true",
                        help="无头模式: 截图保存到 calib/debug/calib_camera_debug.jpg")
    parser.add_argument(
        "--minimal", "--simple", dest="minimal", action="store_true",
        help="极简模式: 全部图像直接 OpenCV 标定，跳过质量诊断")
    args = parser.parse_args()

    if not validate_params():
        return

    square_size_m = SQUARE_SIZE_MM / 1000.0
    pattern_size = (CHESSBOARD_COLS, CHESSBOARD_ROWS)
    total_corners = CHESSBOARD_COLS * CHESSBOARD_ROWS

    # 生成棋盘格角点的 3D 世界坐标 (假设棋盘格在 Z=0 平面)
    objp = make_chessboard_objp(CHESSBOARD_COLS, CHESSBOARD_ROWS, square_size_m)

    # 存储
    obj_points = []   # 3D 世界坐标
    img_points = []   # 2D 图像坐标
    captured = 0

    # 打开相机
    cap, camera_info = open_project_camera(
        CAMERA_INDEX,
        width=CAMERA_WIDTH,
        height=CAMERA_HEIGHT,
        fps=CAMERA_FPS,
        strict_resolution=True,
        log=print,
    )
    actual_size = (camera_info.width, camera_info.height)

    print(f"\n{'='*55}")
    print(f"  相机标定{'（极简模式）' if args.minimal else ''}")
    print(f"  棋盘格: {CHESSBOARD_COLS}×{CHESSBOARD_ROWS} 内角点, 方格={SQUARE_SIZE_MM}mm")
    print(f"{'='*55}")
    print(f"\n  操作:")
    print(f"    空格键 = 拍照（画面显示绿色角点时再按）")
    print(f"    C 键   = 开始计算")
    print(f"    Q 键   = 退出")
    if args.minimal:
        print("\n  极简模式: 至少拍 3 张，全部图像直接参与 OpenCV 求解\n")
    else:
        print(f"\n  目标: 拍 20~30 张不同角度、不同位置的照片\n")

    while True:
        ok, frame = cap.read()
        if not ok:
            print("读取摄像头失败")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        display = frame.copy()

        # 检测棋盘格角点
        found, corners = find_chessboard(gray, pattern_size)

        if found and corners is not None:
            cv2.drawChessboardCorners(display, pattern_size, corners, True)
            status = f"✓ 检测成功 ({total_corners} 角点) — 按空格拍照"
            color = (0, 255, 0)
        else:
            status = "✗ 未检测到棋盘格 — 请调整角度/距离"
            color = (0, 0, 255)

        # 显示信息
        cv2.putText(display, status, (12, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        cv2.putText(display, f"已拍: {captured} 张 | SPACE=拍 C=计算 Q=退出",
                    (12, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        if args.headless:
            # ── 无头模式: 保存截图 + 终端状态 ──
            os.makedirs(os.path.dirname(DEBUG_IMAGE_FILE), exist_ok=True)
            cv2.imwrite(DEBUG_IMAGE_FILE, display,
                        [cv2.IMWRITE_JPEG_QUALITY, 85])
            if found and corners is not None:
                stat = (f"\r  ✓ 检测成功 ({total_corners} 角点) | "
                        f"已拍:{captured}张 | [回车拍照 / C计算 / Q退出]   ")
            else:
                stat = (f"\r  ✗ 未检测到棋盘格 | "
                        f"已拍:{captured}张 | [回车拍照 / C计算 / Q退出]   ")
            sys.stdout.write(stat)
            sys.stdout.flush()
            time.sleep(0.05)

            # 非阻塞检查 stdin
            cmd = None
            if select.select([sys.stdin], [], [], 0.05)[0]:
                cmd = sys.stdin.readline().strip()
        else:
            cmd = None
            cv2.imshow("Camera Calibration", display)
            key = cv2.waitKey(1) & 0xFF
            # 映射到 cmd
            if key == ord(' '):
                cmd = ' '
            elif key == ord('c') or key == ord('C'):
                cmd = 'c'
            elif key == ord('q') or key == ord('Q') or key == 27:
                cmd = 'q'

        # ── 统一命令处理 ──
        if cmd in (' ', '') and found:
            _, corners_sub, _ = refine_chessboard_corners(
                gray, corners, pattern_size)
            grid_ok, grid_diag = validate_chessboard_geometry(
                corners_sub, pattern_size)
            if not grid_ok:
                print(f"\n  ⚠ 角点网格异常，跳过: {grid_diag}")
                continue
            obj_points.append(objp)
            img_points.append(corners_sub)
            captured += 1
            print(f"\n  [{captured}] 已保存")

        elif cmd in ('c', 'C'):
            min_images = 3 if args.minimal else 10
            if captured < min_images:
                if args.minimal:
                    print(f"\n  ⚠ 只有 {captured} 张，极简模式至少需要 3 张")
                    continue
                print(f"\n  ⚠️  只有 {captured} 张，建议至少 15 张，20+ 更好")
                print(f"  继续拍照 (回车) 或 再按 C 强制计算")
                continue

            print(f"\n  正在用 {captured} 张照片计算相机内参...")

            if not args.minimal:
                # 检查图像覆盖多样性：角点是否覆盖了画面的各个区域
                h_img, w_img = gray.shape
                grid_h, grid_w = 3, 3
                coverage = np.zeros((grid_h, grid_w), dtype=bool)
                for corner_set in img_points:
                    c = corner_set.reshape(-1, 2)
                    for px, py in c:
                        ci = min(int(py / h_img * grid_h), grid_h - 1)
                        cj = min(int(px / w_img * grid_w), grid_w - 1)
                        coverage[ci, cj] = True
                covered = int(np.sum(coverage))
                total_cells = grid_h * grid_w
                if covered < 6:
                    print(f"  ⚠ 棋盘格仅覆盖画面 {covered}/{total_cells} 区域（建议 ≥6），"
                          f"请补充其他角度的照片")

            # 使用 CALIB_FIX_K3 防止过拟合
            # 普通 USB 相机极少需要 k3，强行拟合 k3 会把噪声也拟合进去
            # 导致畸变参数极端 (如 k3=-0.6) 且重投影误差异常低 (过拟合)
            flags = 0 if args.minimal else cv2.CALIB_FIX_K3

            ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
                obj_points, img_points, gray.shape[::-1], None, None, flags=flags)

            if not ret:
                print("  ❌ 标定失败，请重新拍照")
                continue

            fx, fy = mtx[0, 0], mtx[1, 1]
            cx, cy = mtx[0, 2], mtx[1, 2]
            dist_flat = dist.ravel()

            # 计算每张图的重投影误差
            per_image_errors = []
            for i in range(len(obj_points)):
                projected, _ = cv2.projectPoints(obj_points[i], rvecs[i], tvecs[i], mtx, dist)
                diff = (img_points[i].reshape(-1, 2)
                        - projected.reshape(-1, 2))
                error = float(np.sqrt(np.mean(np.sum(diff * diff, axis=1))))
                per_image_errors.append(error)
            mean_error = float(np.mean(per_image_errors))
            median_error = float(np.median(per_image_errors))

            # 评估方格在图像中的像素尺寸 (影响角点精度上限)
            median_dist_m = float(np.median([np.linalg.norm(tvecs[i]) for i in range(len(tvecs))]))
            px_per_sq = fx * SQUARE_SIZE_MM / 1000.0 / max(median_dist_m, 0.01)
            if px_per_sq < 15:
                grade = f"方格仅 {px_per_sq:.0f} px/格 — 角点精度受限，重投影放宽到 1.0px"
                max_ok = 1.0
            elif px_per_sq < 25:
                grade = f"方格 {px_per_sq:.0f} px/格 — 适中，重投影 < 0.5px 即可"
                max_ok = 0.5
            else:
                grade = f"方格 {px_per_sq:.0f} px/格 — 充裕，重投影应 < 0.3px"
                max_ok = 0.3

            print(f"\n  {'='*50}")
            print(f"  ✅ 标定完成！")
            print(f"  {'='*50}")
            print(f"  fx = {fx:.4f}  (焦距 X, 像素)")
            print(f"  fy = {fy:.4f}  (焦距 Y, 像素)")
            print(f"  cx = {cx:.4f}  (光心 X)")
            print(f"  cy = {cy:.4f}  (光心 Y)")
            print(f"  畸变: k1={dist_flat[0]:.6f}, k2={dist_flat[1]:.6f}"
                  f"{', k3=' + f'{dist_flat[4]:.6f}' if len(dist_flat) >= 5 else ''}")
            print(f"  📐 {grade}")
            print(f"  平均重投影误差: {mean_error:.4f} px  "
                  f"(中位={median_error:.4f}, 最差={max(per_image_errors):.4f})")

            # ── 逐张误差报告 ──
            # 异常阈值: 取 3x 中位误差 或 max_ok，取较大者 (小棋盘宽容)
            outlier_thresh = max(median_error * 3.0, max_ok * 1.5)
            outliers = [i for i, e in enumerate(per_image_errors) if e > outlier_thresh]
            if outliers:
                print(f"\n  ⚠ {len(outliers)} 张偏差较大的图像 (误差 > {outlier_thresh:.3f}px):")
                for idx in outliers:
                    flag = " ← 建议重拍" if per_image_errors[idx] > max(max_ok * 2.0, 1.0) else ""
                    print(f"     第{idx+1}张: {per_image_errors[idx]:.4f} px{flag}")

            # ── 质量诊断 ──
            quality_ok = True
            if not args.minimal:
                # 过拟合检测：中位误差过低 + k2 过大
                if median_error < 0.03 and abs(dist_flat[1]) > 0.15:
                    print(f"\n  ⚠ 疑似过拟合! 中位重投影={median_error:.4f}px + k2={dist_flat[1]:.3f}")
                    print(f"     建议: 增加更多不同距离/角度的照片后重新标定")
                    quality_ok = False
                # 光心偏移检测
                cx_off = cx - actual_size[0] / 2
                cy_off = cy - actual_size[1] / 2
                if abs(cx_off) > 20 or abs(cy_off) > 20:
                    print(f"\n  ⚠ 光心偏离中心较大 (cx={cx_off:+.0f}px, cy={cy_off:+.0f}px)")
                    print(f"     可能是镜头安装偏差，也可能是标定图像分布不均")
                # 焦距异常检测
                expected_f = max(actual_size) * 0.8
                if fx < expected_f * 0.5 or fx > expected_f * 2.0:
                    print(f"\n  ⚠ 焦距异常 (fx={fx:.0f}px, 期望 ~{expected_f:.0f}px)")
                    quality_ok = False

            if not quality_ok:
                print(f"\n  ⚠ 标定质量存疑，手眼标定结果可能不准。建议重新采集。")
            print(f"  {'='*50}")

            # 保存
            payload = {
                "camera_matrix": {
                    "rows": 3, "cols": 3,
                    "data": mtx.reshape(-1).tolist(),
                },
                "distortion_coefficients": {
                    "rows": 1, "cols": int(len(dist_flat)),
                    "data": dist_flat.tolist(),
                },
                "image_width": int(actual_size[0]),
                "image_height": int(actual_size[1]),
                "chessboard": f"{CHESSBOARD_COLS}x{CHESSBOARD_ROWS}",
                "square_size_mm": SQUARE_SIZE_MM,
                "reprojection_error_px": float(mean_error),
                "reprojection_error_median_px": float(median_error),
                "reprojection_error_max_px": float(max(per_image_errors)),
                "reprojection_per_image_px": [float(e) for e in per_image_errors],
                "captured_images": captured,
                "calibration_mode": "minimal" if args.minimal else "standard",
                "created_at": datetime.now().isoformat(timespec="seconds"),
            }
            with open(OUTPUT_FILE, 'w', encoding="utf-8") as f:
                yaml.safe_dump(payload, f, sort_keys=False, allow_unicode=True)

            print(f"\n  📁 已保存到: {OUTPUT_FILE}")
            deploy_dir = os.path.normpath(os.path.join(
                _here, "..", "handeye_bridge", "config"))
            deploy_path = os.path.join(deploy_dir, "camera_intrinsics.yaml")
            if quality_ok and os.path.isdir(deploy_dir):
                with open(deploy_path, "w", encoding="utf-8") as f:
                    yaml.safe_dump(
                        payload, f, sort_keys=False, allow_unicode=True)
                print(f"  📁 已同步部署到: {deploy_path}")
            elif not quality_ok:
                print("  ⚠ 标定质量未通过，未覆盖运行时相机内参")
            print(f"  下一步: 用这个文件做手眼标定\n")
            break

        elif cmd in ('q', 'Q'):
            print("\n  已退出")
            break

    cap.release()
    if not args.headless:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
