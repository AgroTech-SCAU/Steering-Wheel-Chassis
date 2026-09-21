from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Optional

import numpy as np


@dataclass(frozen=True)
class VisionPoseTarget:
    name: str
    configured: bool
    xyz: np.ndarray
    tool_axis: Optional[np.ndarray] = None


def vision_pose_for_position(
    position: np.ndarray,
    targets: Iterable[VisionPoseTarget],
    tolerance_m: float,
) -> Optional[str]:
    if not np.isfinite(position).all() or not np.isfinite(tolerance_m) or tolerance_m <= 0.0:
        return None
    for target in targets:
        if not target.configured:
            continue
        if not np.isfinite(target.xyz).all():
            continue
        if float(np.linalg.norm(position - target.xyz)) <= tolerance_m:
            return target.name
    return None


def direction_axis(pitch: float, yaw: float) -> np.ndarray:
    return np.array([np.cos(pitch) * np.cos(yaw),
                     np.cos(pitch) * np.sin(yaw), np.sin(pitch)])


def vision_pose_for_transform(transform, targets, tolerance_m, axis_tolerance_deg):
    if not np.isfinite(transform).all():
        return None
    for target in targets:
        if vision_pose_for_position(transform[:3, 3], [target], tolerance_m) is None:
            continue
        if target.tool_axis is None or not np.isfinite(target.tool_axis).all():
            continue
        norm = float(np.linalg.norm(target.tool_axis) * np.linalg.norm(transform[:3, 2]))
        if norm <= 0.0:
            continue
        cosine = float(np.dot(transform[:3, 2], target.tool_axis)) / norm
        if np.degrees(np.arccos(np.clip(cosine, -1., 1.))) < axis_tolerance_deg:
            return target.name
    return None


def tcp_is_stable(history, now_ns, window_s, displacement_m, max_age_s):
    if len(history) < 2 or window_s <= 0 or displacement_m <= 0:
        return False
    last_ns = history[-1][0]
    if not 0 <= now_ns - last_ns <= max_age_s * 1e9:
        return False
    cutoff = last_ns - int(window_s * 1e9)
    samples = list(history)
    older = [i for i, (stamp, _) in enumerate(samples) if stamp <= cutoff]
    if not older:
        return False
    samples = samples[older[-1]:]
    stamps = np.array([stamp for stamp, _ in samples], dtype=np.int64)
    if np.any(np.diff(stamps) <= 0) or np.any(np.diff(stamps) > max_age_s * 1e9):
        return False
    positions = np.array([transform[:3, 3] for _, transform in samples])
    if not np.isfinite(positions).all():
        return False
    distances = np.linalg.norm(positions[:, None, :] - positions[None, :, :], axis=2)
    return bool(np.max(distances) < displacement_m)
