from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Optional

import numpy as np


@dataclass(frozen=True)
class VisionPoseTarget:
    name: str
    configured: bool
    xyz: np.ndarray


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
