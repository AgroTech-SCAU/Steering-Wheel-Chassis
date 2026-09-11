from __future__ import annotations


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
