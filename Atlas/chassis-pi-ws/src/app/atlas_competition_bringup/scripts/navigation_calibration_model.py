import copy
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

import yaml


@dataclass(frozen=True)
class NavigationMapStatus:
    arena: str
    map_value: str
    pbstream_value: str
    map_path: Path
    image_path: Path
    pbstream_path: Path
    ready: bool
    errors: tuple[str, ...]


def _mapping(value: Any) -> Mapping[str, Any]:
    return value if isinstance(value, Mapping) else {}


def _resolve_asset(config_path: Path, value: str) -> Path:
    raw = str(value or "").strip()
    if not raw:
        return Path()
    path = Path(raw).expanduser()
    if path.is_absolute():
        return path
    return config_path.parent / path


def _map_image_path(map_path: Path) -> tuple[Path, str | None]:
    if not map_path.is_file():
        return Path(), None
    try:
        data = yaml.safe_load(map_path.read_text(encoding="utf-8")) or {}
    except (OSError, yaml.YAMLError) as exc:
        return Path(), f"map YAML 无法读取: {exc}"
    image_value = str(data.get("image", "") or "").strip()
    if not image_value:
        return Path(), "map YAML 缺少 image 字段"
    image = Path(image_value).expanduser()
    if not image.is_absolute():
        image = map_path.parent / image
    return image, None


def inspect_navigation_maps(
    source: Mapping[str, Any], config_path: str | Path
) -> dict[str, NavigationMapStatus]:
    config_path = Path(config_path).expanduser()
    competition = _mapping(source.get("competition", source))
    navigation = _mapping(competition.get("navigation", {}))
    arenas = _mapping(navigation.get("arenas", {}))
    result: dict[str, NavigationMapStatus] = {}

    for arena in ("A", "B"):
        arena_cfg = _mapping(arenas.get(arena, {}))
        map_value = str(arena_cfg.get("map", "") or "").strip()
        pbstream_value = str(arena_cfg.get("pbstream", "") or "").strip()
        map_path = _resolve_asset(config_path, map_value)
        pbstream_path = _resolve_asset(config_path, pbstream_value)
        errors: list[str] = []

        if not map_value:
            errors.append("map 路径未配置")
        elif not map_path.is_file():
            errors.append(f"map YAML 不存在: {map_path}")

        image_path = Path()
        if map_value and map_path.is_file():
            image_path, image_error = _map_image_path(map_path)
            if image_error:
                errors.append(image_error)
            elif not image_path.is_file():
                errors.append(f"map image 不存在: {image_path}")

        if not pbstream_value:
            errors.append("pbstream 路径未配置")
        elif not pbstream_path.is_file():
            errors.append(f"pbstream 不存在: {pbstream_path}")

        result[arena] = NavigationMapStatus(
            arena=arena,
            map_value=map_value,
            pbstream_value=pbstream_value,
            map_path=map_path,
            image_path=image_path,
            pbstream_path=pbstream_path,
            ready=not errors,
            errors=tuple(errors),
        )

    return result


def merge_navigation_calibration(
    source: Mapping[str, Any],
    arena: str,
    waypoints: Mapping[str, Mapping[str, Any]],
) -> dict[str, Any]:
    arena_name = str(arena or "").strip().upper()
    if arena_name not in {"A", "B"}:
        raise ValueError("arena must be A or B")

    required = ("pickup", "park_1", "park_2")
    missing = [name for name in required if name not in waypoints]
    if missing:
        raise ValueError(f"missing waypoints: {', '.join(missing)}")

    merged = copy.deepcopy(dict(source))
    competition = merged.setdefault("competition", {})
    if not isinstance(competition, dict):
        raise ValueError("competition must be a mapping")
    navigation = competition.setdefault("navigation", {})
    if not isinstance(navigation, dict):
        raise ValueError("competition.navigation must be a mapping")
    navigation["coordinate_mode"] = "absolute_map"
    arenas = navigation.setdefault("arenas", {})
    if not isinstance(arenas, dict):
        raise ValueError("competition.navigation.arenas must be a mapping")
    arena_cfg = arenas.setdefault(arena_name, {})
    if not isinstance(arena_cfg, dict):
        raise ValueError(f"arena {arena_name} must be a mapping")
    arena_cfg["waypoints"] = {
        name: copy.deepcopy(dict(waypoints[name])) for name in required
    }
    return merged


def dump_clean_yaml(data: Mapping[str, Any]) -> str:
    return yaml.safe_dump(
        copy.deepcopy(dict(data)),
        allow_unicode=True,
        sort_keys=False,
        default_flow_style=False,
    )
