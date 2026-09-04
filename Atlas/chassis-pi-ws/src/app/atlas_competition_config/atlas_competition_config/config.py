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
    return CompetitionConfig(
        backend_name=str(competition.get("backend_name", "nav2_competition")),
        vision=dict(competition.get("vision", {}) or {}),
        navigation=navigation,
        manipulation=dict(competition.get("manipulation", {}) or {}),
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
