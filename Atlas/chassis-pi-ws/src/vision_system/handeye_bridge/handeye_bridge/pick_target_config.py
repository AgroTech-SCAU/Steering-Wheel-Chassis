from __future__ import annotations


def select_pick_detection(detections, corner_index: int):
    candidates = list(detections)
    for detection in candidates:
        if detection.corner_index == corner_index:
            return detection
    # A slot-specific view may contain just one cargo, labeled corner 0 by
    # the detector regardless of the physical slot being processed.
    return candidates[0] if len(candidates) == 1 else None


def resolve_pick_target_parameters(msg, default_z: float, default_pitch: float, default_yaw: float):
    use_target_z = bool(getattr(msg, "use_target_z", False))
    use_orientation = bool(getattr(msg, "use_orientation", False))
    target_z = float(getattr(msg, "target_z_m", default_z)) if use_target_z else float(default_z)
    pitch = float(getattr(msg, "pitch_rad", default_pitch)) if use_orientation else float(default_pitch)
    yaw = float(getattr(msg, "yaw_rad", default_yaw)) if use_orientation else float(default_yaw)
    return target_z, pitch, yaw, use_target_z


def pick_command_z(msg, plane_z: float, default_offset_m: float) -> float:
    use_approach = bool(getattr(msg, "use_approach", False))
    offset = float(getattr(msg, "approach_m", default_offset_m)) if use_approach else float(default_offset_m)
    if offset < 0.0:
        raise ValueError("approach_m must be >= 0")
    return float(plane_z) + offset


def tool_axis_angles(transform):
    """Encode detection-synchronized tool z in base coordinates"""
    import math
    x, y, z = (float(v) for v in transform[:3, 2])
    if not all(math.isfinite(v) for v in (x, y, z)) or math.hypot(x, y, z) < 1e-10:
        raise ValueError("invalid tool axis")
    horizontal = math.hypot(x, y)
    return math.atan2(z, horizontal), (math.atan2(y, x) if horizontal > 1e-10 else 0.0)
