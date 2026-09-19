from __future__ import annotations

import copy
import math
from typing import Any, Mapping

import yaml


def merge_calibration(
    source: Mapping[str, Any], arena: str, calibration: Mapping[str, Any]
) -> dict[str, Any]:
    arena_name = str(arena).strip().upper()
    if arena_name not in {"A", "B"}:
        raise ValueError("arena must be A or B")

    merged = copy.deepcopy(dict(source))
    competition = merged.setdefault("competition", {})
    if not isinstance(competition, dict):
        raise ValueError("competition must be a mapping")

    arm_motion = competition.setdefault("arm_motion", {})
    fixed_poses = arm_motion.setdefault("fixed_poses", {})
    if not isinstance(fixed_poses, dict):
        raise ValueError("competition.arm_motion.fixed_poses must be a mapping")
    for name, pose in dict(calibration["fixed_poses"]).items():
        fixed_poses[str(name)] = copy.deepcopy(dict(pose))
    arenas = arm_motion.setdefault("arenas", {})
    arena_cfg = arenas.setdefault(arena_name, {})
    for area in ("pickup", "park_1", "park_2"):
        arena_cfg[area] = copy.deepcopy(dict(calibration[area]))
    for observation in arena_cfg["pickup"].get("observations", []):
        heights = observation["layer_z_m"]
        if len(heights) != 2:
            raise ValueError("pickup observation must have two layer_z_m values")
        observe_z = float(observation["z_m"])
        for layer, plane_z in enumerate(heights, start=1):
            distance = observe_z - float(plane_z)
            if not math.isfinite(distance) or distance <= 0.0:
                raise ValueError(f"pickup observation layer {layer} distance must be positive")
            observation[f"camera_to_plane{layer}_distance_m"] = round(distance, 6)

    vision = competition.setdefault("vision", {})
    if isinstance(vision, dict):
        vision.pop("sorting_scan_a", None)
        vision.pop("sorting_scan_b", None)

    manipulation = competition.setdefault("manipulation", {})
    placement = manipulation.setdefault("placement", {}) if isinstance(manipulation, dict) else {}
    if isinstance(placement, dict):
        placement.pop("park_1", None)
        placement.pop("park_2", None)
        placement.pop("slot_offsets_xy_m", None)
        placement["enabled"] = True

    return merged


def dump_clean_yaml(data: Mapping[str, Any]) -> str:
    return yaml.safe_dump(
        copy.deepcopy(dict(data)),
        allow_unicode=True,
        sort_keys=False,
        default_flow_style=False,
    )
