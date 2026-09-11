from __future__ import annotations

import copy
import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping

import yaml


class CompetitionConfigError(ValueError):
    """Raised when competition YAML cannot produce a safe runtime command."""


@dataclass(frozen=True)
class NavigationGoal:
    x_m: float
    y_m: float
    yaw_rad: float
    map_path: str
    pbstream_path: str
    arena: str
    waypoint_id: str


@dataclass(frozen=True)
class CompetitionConfig:
    backend_name: str
    vision: dict[str, Any] = field(default_factory=dict)
    navigation: dict[str, Any] = field(default_factory=dict)
    manipulation: dict[str, Any] = field(default_factory=dict)
    arm_motion: dict[str, Any] = field(default_factory=dict)
    source_path: Path | None = None


class ArenaLock:
    """Lock the match to the first arena inferred from the sorting rule."""

    def __init__(self) -> None:
        self._arena = ""

    @property
    def arena(self) -> str:
        return self._arena

    def accept(self, arena: str) -> str:
        normalized = _normalize_arena(arena)
        if not self._arena:
            self._arena = normalized
            return self._arena
        if normalized != self._arena:
            raise CompetitionConfigError(
                "arena mismatch: "
                f"locked arena={self._arena}, requested arena={normalized}"
            )
        return self._arena


def load_competition_config(path: str | os.PathLike[str]) -> CompetitionConfig:
    config_path = Path(path).expanduser()
    with config_path.open(encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    competition = data.get("competition", data)
    if not isinstance(competition, Mapping):
        raise CompetitionConfigError("competition config must be a mapping")

    navigation = dict(competition.get("navigation", {}) or {})
    navigation["_source_path"] = str(config_path)
    arm_motion = dict(competition.get("arm_motion", {}) or {})
    vision = _vision_with_arm_motion_compatibility(
        dict(competition.get("vision", {}) or {}),
        arm_motion,
    )
    return CompetitionConfig(
        backend_name=str(competition.get("backend_name", "nav2_competition")),
        vision=vision,
        navigation=navigation,
        manipulation=dict(competition.get("manipulation", {}) or {}),
        arm_motion=arm_motion,
        source_path=config_path,
    )


def load_optional_competition_config(
    path: str | os.PathLike[str] | None,
) -> CompetitionConfig | None:
    if path is None:
        return None
    text = str(path).strip()
    if not text:
        return None
    return load_competition_config(text)


def resolve_navigation_waypoint(
    navigation: Mapping[str, Any],
    arena: str,
    waypoint_id: str,
    *,
    base_path: str | os.PathLike[str] | None = None,
) -> NavigationGoal:
    normalized_arena = _normalize_arena(arena)
    waypoint = str(waypoint_id or "").strip()
    if not waypoint:
        raise CompetitionConfigError(
            "waypoint_id is required for semantic navigation"
        )

    arena_config = _mapping(navigation.get("arenas", {})).get(
        normalized_arena
    )
    if not isinstance(arena_config, Mapping):
        raise CompetitionConfigError(
            f"arena {normalized_arena} is not configured"
        )

    source_path = navigation.get("_source_path") or base_path
    map_path = _resolve_config_path(
        str(arena_config.get("map", "") or ""),
        source_path,
    )
    pbstream_path = _resolve_config_path(
        str(arena_config.get("pbstream", "") or ""), source_path
    )
    if not map_path:
        raise CompetitionConfigError(
            f"arena {normalized_arena} map is not configured"
        )
    if not pbstream_path:
        raise CompetitionConfigError(
            f"arena {normalized_arena} pbstream is not configured"
        )

    waypoints = _mapping(arena_config.get("waypoints", {}))
    waypoint_config = waypoints.get(waypoint)
    if not isinstance(waypoint_config, Mapping):
        raise CompetitionConfigError(
            f"waypoint {waypoint} is not configured "
            f"for arena {normalized_arena}"
        )
    if not bool(waypoint_config.get("configured", False)):
        raise CompetitionConfigError(
            f"waypoint {waypoint} in arena {normalized_arena} configured=false"
        )

    return NavigationGoal(
        x_m=float(waypoint_config["x"]),
        y_m=float(waypoint_config["y"]),
        yaw_rad=float(waypoint_config["yaw"]),
        map_path=map_path,
        pbstream_path=pbstream_path,
        arena=normalized_arena,
        waypoint_id=waypoint,
    )


def resolve_arm_pose(
    arm_motion: Mapping[str, Any],
    pose_name: str,
    *,
    arena: str | None = None,
    area: str | None = None,
) -> dict[str, Any]:
    name = str(pose_name or "").strip()
    if name in {"zero", "sorting_scan_a", "sorting_scan_b", "navigation_safe"}:
        pose = _mapping(_mapping(arm_motion.get("fixed_poses", {})).get(name))
    elif name == "pickup_observe":
        normalized_arena = _normalize_arena(arena or "")
        arena_cfg = _mapping(_mapping(arm_motion.get("arenas", {})).get(normalized_arena))
        pose = _mapping(_mapping(arena_cfg.get("pickup", {})).get("observe"))
    elif name == "park_prepare":
        normalized_arena = _normalize_arena(arena or "")
        park = str(area or "").strip()
        if park not in {"park_1", "park_2"}:
            raise CompetitionConfigError("park_prepare area must be park_1 or park_2")
        arena_cfg = _mapping(_mapping(arm_motion.get("arenas", {})).get(normalized_arena))
        pose = _mapping(_mapping(arena_cfg.get(park, {})).get("prepare"))
    else:
        raise CompetitionConfigError(f"unknown arm pose: {name}")

    if not pose:
        raise CompetitionConfigError(f"arm pose {name} is not configured")
    if not bool(pose.get("configured", False)):
        raise CompetitionConfigError(f"arm pose {name} configured=false")
    joints = list(pose.get("joints_rad", []) or [])
    if len(joints) != 5:
        raise CompetitionConfigError(f"arm pose {name} joints_rad must contain 5 values")
    required = ("x_m", "y_m", "z_m", "pitch_rad", "yaw_rad", "speed_rad_s")
    missing = [key for key in required if key not in pose]
    if missing:
        raise CompetitionConfigError(
            f"arm pose {name} missing fields: {', '.join(missing)}"
        )
    result = copy.deepcopy(dict(pose))
    result["joints_rad"] = [float(v) for v in joints]
    for key in required:
        result[key] = float(result[key])
    return result


def resolve_pickup_layer_z(
    arm_motion: Mapping[str, Any], arena: str, layer: int
) -> float:
    normalized_arena = _normalize_arena(arena)
    if int(layer) not in (1, 2, 3):
        raise CompetitionConfigError("pickup layer must be 1, 2 or 3")
    arena_cfg = _mapping(_mapping(arm_motion.get("arenas", {})).get(normalized_arena))
    pickup = _mapping(arena_cfg.get("pickup", {}))
    if not bool(pickup.get("layer_z_configured", False)):
        raise CompetitionConfigError(
            f"arena {normalized_arena} pickup.layer_z_configured=false"
        )
    values = list(pickup.get("layer_z_m", []) or [])
    if len(values) != 3:
        raise CompetitionConfigError(
            f"arena {normalized_arena} pickup.layer_z_m must contain 3 values"
        )
    return float(values[int(layer) - 1])


def resolve_placement_reference(
    arm_motion: Mapping[str, Any], arena: str, park: str
) -> dict[str, float]:
    normalized_arena = _normalize_arena(arena)
    park_name = str(park or "").strip()
    if park_name not in {"park_1", "park_2"}:
        raise CompetitionConfigError("park must be park_1 or park_2")
    arena_cfg = _mapping(_mapping(arm_motion.get("arenas", {})).get(normalized_arena))
    reference = _mapping(_mapping(arena_cfg.get(park_name, {})).get("placement_reference"))
    required = ("x_m", "y_m", "first_layer_z_m")
    if not reference or any(key not in reference for key in required):
        raise CompetitionConfigError(
            f"arena {normalized_arena} {park_name}.placement_reference is not configured"
        )
    if not bool(reference.get("configured", False)):
        raise CompetitionConfigError(
            f"arena {normalized_arena} {park_name}.placement_reference configured=false"
        )
    return {key: float(reference[key]) for key in required}


def _vision_with_arm_motion_compatibility(
    vision: Mapping[str, Any], arm_motion: Mapping[str, Any]
) -> dict[str, Any]:
    merged = copy.deepcopy(dict(vision))
    fixed = _mapping(arm_motion.get("fixed_poses", {}))
    for key in ("sorting_scan_a", "sorting_scan_b"):
        pose = fixed.get(key)
        if isinstance(pose, Mapping):
            merged[key] = {
                field: copy.deepcopy(pose[field])
                for field in (
                    "configured", "x_m", "y_m", "z_m",
                    "pitch_rad", "yaw_rad", "speed_rad_s",
                )
                if field in pose
            }
    return merged


def apply_vision_backend_overrides(
    base: Mapping[str, Any],
    vision: Mapping[str, Any],
) -> dict[str, Any]:
    merged = copy.deepcopy(dict(base))
    for key in ("class_aliases", "sorting_rule"):
        if key in vision and isinstance(vision[key], Mapping):
            merged[key] = copy.deepcopy(dict(vision[key]))
    return merged


def apply_handeye_scan_overrides(
    base: Mapping[str, Any],
    vision: Mapping[str, Any],
) -> dict[str, Any]:
    merged = copy.deepcopy(dict(base))
    for key in ("sorting_scan_a", "sorting_scan_b"):
        if key in vision and isinstance(vision[key], Mapping):
            merged[key] = copy.deepcopy(dict(vision[key]))
    return merged


def apply_manipulation_placement_overrides(
    base: Mapping[str, Any],
    manipulation: Mapping[str, Any],
) -> dict[str, Any]:
    merged = copy.deepcopy(dict(base))
    placement = manipulation.get("placement")
    if isinstance(placement, Mapping):
        merged["placement"] = copy.deepcopy(dict(placement))
    return merged


def with_source_path(
    navigation: Mapping[str, Any],
    source_path: str | os.PathLike[str] | None,
) -> dict[str, Any]:
    copied = copy.deepcopy(dict(navigation))
    if source_path:
        copied["_source_path"] = str(source_path)
    return copied


def _mapping(value: Any) -> Mapping[str, Any]:
    return value if isinstance(value, Mapping) else {}


def _normalize_arena(arena: str) -> str:
    normalized = str(arena or "").strip().upper()
    if normalized not in {"A", "B"}:
        raise CompetitionConfigError("arena must be A or B")
    return normalized


def _resolve_config_path(
    path: str,
    source_path: str | os.PathLike[str] | None,
) -> str:
    if not path:
        return ""
    expanded = os.path.expanduser(path)
    if os.path.isabs(expanded):
        return expanded
    if source_path:
        base = Path(source_path)
        if base.suffix:
            base = base.parent
        return str(base / expanded)
    return expanded
