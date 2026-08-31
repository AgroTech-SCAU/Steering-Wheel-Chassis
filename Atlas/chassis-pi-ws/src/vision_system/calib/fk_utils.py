#!/usr/bin/env python3
"""
旋转矩阵 / 齐次变换 工具函数
"""

from typing import Sequence, Tuple
import numpy as np


# ── Euler 角 / 旋转矩阵 / 四元数 转换 ──

def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Rz(yaw)·Ry(pitch)·Rx(roll) 固定轴旋转 → 3x3 旋转矩阵"""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def matrix_to_rpy(R: np.ndarray) -> Tuple[float, float, float]:
    """3x3 旋转矩阵 → (roll, pitch, yaw) 固定轴欧拉角 (rad)"""
    sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    if sy > 1e-9:
        roll = np.arctan2(R[2, 1], R[2, 2])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        roll = np.arctan2(-R[1, 2], R[1, 1])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = 0.0
    return float(roll), float(pitch), float(yaw)


def quaternion_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """四元数 (x, y, z, w) → 3x3 旋转矩阵"""
    n = np.sqrt(x*x + y*y + z*z + w*w)
    if n < 1e-10:
        raise ValueError(f"零模长四元数: ({x:.6f}, {y:.6f}, {z:.6f}, {w:.6f})")
    x, y, z, w = x/n, y/n, z/n, w/n
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ], dtype=np.float64)


def matrix_to_quaternion(R: np.ndarray) -> Tuple[float, float, float, float]:
    """3x3 旋转矩阵 → (x, y, z, w) 四元数"""
    trace = float(np.trace(R))
    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w], dtype=np.float64)
    q /= np.linalg.norm(q)
    return tuple(float(v) for v in q)


# ── 齐次变换 ──

def make_transform(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    """3x3 旋转 + 3x1 平移 → 4x4 齐次矩阵"""
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = np.asarray(R, dtype=np.float64).reshape(3, 3)
    T[:3, 3] = np.asarray(t, dtype=np.float64).reshape(3)
    return T


def invert_transform(T: np.ndarray) -> np.ndarray:
    """4x4 齐次矩阵求逆"""
    R = T[:3, :3]
    t = T[:3, 3]
    inv = np.eye(4, dtype=np.float64)
    inv[:3, :3] = R.T
    inv[:3, 3] = -R.T @ t
    return inv


def rotation_angle_deg(R: np.ndarray) -> float:
    """旋转矩阵的旋转角度 (度)"""
    val = (float(np.trace(R)) - 1.0) * 0.5
    return float(np.degrees(np.arccos(max(-1.0, min(1.0, val)))))


def mean_rotation(rotations: Sequence[np.ndarray]) -> np.ndarray:
    """一组旋转矩阵的 Karcher 均值"""
    S = np.sum(np.asarray(rotations), axis=0)
    u, _, vt = np.linalg.svd(S)
    corr = np.eye(3)
    corr[2, 2] = np.linalg.det(u @ vt)
    return u @ corr @ vt


# ── 正运动学 (MDH) ──

def _load_robot_params():
    """加载 MDH 参数和 TOOL 变换.

    优先从 robot_params.yaml 读取, 文件不存在时回退到硬编码默认值.
    这样更换末端工具或维修后只需修改 YAML, 无需改代码.
    """
    import os as _os
    _cfg_path = _os.path.join(_os.path.dirname(_os.path.abspath(__file__)),
                              "robot_params.yaml")

    defaults = {
        "a":      np.array([0.0, 0.0276200491203067, 0.2167241256700170,
                            0.2002827243995208, 0.0451594898594991]),
        "d":      np.array([0.0, -0.0162679040568649, -0.0192068569153542,
                            0.0014389528584892, 0.0]),
        "alpha":  np.array([0.0, np.pi*0.5, np.pi, 0.0, np.pi*0.5]),
        "offset": np.array([-np.pi, -np.pi*0.5, -3.3836013435535577,
                            -2.8616351199480290, -np.pi]),
        "sign":   np.array([-1.0, -1.0, 1.0, 1.0, -1.0]),
        "base_tz": 0.0605,
        "tool_T": np.array([
            [ 0.9999619259637,  0.0,              -0.0087262032439,  0.0],
            [ 0.0000761495224, -0.9999619230642,   0.0087262032439,  0.0],
            [-0.0087258709769, -0.0087265354984,  -0.9999238504776, -0.0184685931641],
            [ 0.0,              0.0,               0.0,               1.0],
        ]),
    }

    if not _os.path.exists(_cfg_path):
        return defaults

    try:
        import yaml
        with open(_cfg_path, encoding="utf-8") as f:
            cfg = yaml.safe_load(f)
        mdh = cfg.get("mdh", {})
        tool = cfg.get("tool_transform", {})

        params = {}
        for key, field in [("a", "a"), ("d", "d"), ("alpha", "alpha"),
                           ("offset", "offset"), ("sign", "sign")]:
            val = mdh.get(field, None)
            params[key] = np.array(val, dtype=np.float64) if val is not None else defaults[key]

        params["base_tz"] = float(mdh.get("base_tz", defaults["base_tz"]))

        tool_data = tool.get("data", None)
        params["tool_T"] = np.array(tool_data, dtype=np.float64) if tool_data is not None else defaults["tool_T"]

        return params
    except Exception as e:
        import sys as _sys
        print(f"  ⚠ 加载 robot_params.yaml 失败 ({e})，回退到默认 DH 参数",
              file=_sys.stderr)
        return defaults


_ROBOT = _load_robot_params()
_MDH_A     = _ROBOT["a"]
_MDH_D     = _ROBOT["d"]
_MDH_ALPHA = _ROBOT["alpha"]
_MDH_OFFSET = _ROBOT["offset"]
_MDH_SIGN  = _ROBOT["sign"]
_BASE_TZ   = _ROBOT["base_tz"]
_TOOL_T    = _ROBOT["tool_T"]


def _rotx(a): ca, sa = np.cos(a), np.sin(a); return np.array([[1,0,0,0],[0,ca,-sa,0],[0,sa,ca,0],[0,0,0,1]])
def _rotz(a): ca, sa = np.cos(a), np.sin(a); return np.array([[ca,-sa,0,0],[sa,ca,0,0],[0,0,1,0],[0,0,0,1]])
def _transx(x): t = np.eye(4); t[0,3] = x; return t
def _transz(z): t = np.eye(4); t[2,3] = z; return t


def fk_gripper_in_base(joints_rad: Sequence[float]) -> np.ndarray:
    """5 关节正运动学 (MDH).
    返回 4x4 齐次矩阵, 表示末端(gripper)在基座(base)坐标系中的位姿.
    将 gripper 坐标系下的点映射到 arm_base_link 坐标系.
    """
    joints = np.asarray(joints_rad, dtype=np.float64)
    theta = _MDH_SIGN * joints + _MDH_OFFSET
    T = np.eye(4)
    T[2, 3] = _BASE_TZ
    for i in range(5):
        T = T @ _transx(_MDH_A[i]) @ _rotx(_MDH_ALPHA[i]) @ _rotz(theta[i]) @ _transz(_MDH_D[i])
    return T @ _TOOL_T
