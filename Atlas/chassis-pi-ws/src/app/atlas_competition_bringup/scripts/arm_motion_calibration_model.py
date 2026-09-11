from __future__ import annotations

import copy
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
    arm_motion["fixed_poses"] = copy.deepcopy(dict(calibration["fixed_poses"]))
    arenas = arm_motion.setdefault("arenas", {})
    arena_cfg = arenas.setdefault(arena_name, {})
    arena_cfg["pickup"] = copy.deepcopy(dict(calibration["pickup"]))
    arena_cfg["park_1"] = copy.deepcopy(dict(calibration["park_1"]))
    arena_cfg["park_2"] = copy.deepcopy(dict(calibration["park_2"]))

    vision = competition.setdefault("vision", {})
    if isinstance(vision, dict):
        vision.pop("sorting_scan_a", None)
        vision.pop("sorting_scan_b", None)

    manipulation = competition.setdefault("manipulation", {})
    placement = manipulation.setdefault("placement", {}) if isinstance(manipulation, dict) else {}
    if isinstance(placement, dict):
        placement.pop("park_1", None)
        placement.pop("park_2", None)
        placement["enabled"] = True

    return merged


def dump_clean_yaml(data: Mapping[str, Any]) -> str:
    return yaml.safe_dump(
        copy.deepcopy(dict(data)),
        allow_unicode=True,
        sort_keys=False,
        default_flow_style=False,
    )
