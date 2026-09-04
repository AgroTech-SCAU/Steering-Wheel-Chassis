from pathlib import Path

import pytest
import yaml

from atlas_competition_config.config import (
    ArenaLock,
    CompetitionConfigError,
    apply_manipulation_placement_overrides,
    apply_vision_backend_overrides,
    load_competition_config,
    resolve_navigation_waypoint,
)


def _write_config(tmp_path: Path, payload: dict) -> Path:
    path = tmp_path / "competition.yaml"
    path.write_text(
        yaml.safe_dump(payload, allow_unicode=True),
        encoding="utf-8",
    )
    return path


def test_navigation_resolves_configured_arena_waypoint_and_paths(
    tmp_path,
):
    """Catches dropping semantic waypoint resolution back to request pose."""
    config_path = _write_config(
        tmp_path,
        {
            "competition": {
                "navigation": {
                    "arenas": {
                        "A": {
                            "map": "maps/arena_a.yaml",
                            "pbstream": "maps/arena_a.pbstream",
                            "waypoints": {
                                "pickup": {
                                    "x": 1.25,
                                    "y": -0.40,
                                    "yaw": 1.57,
                                    "configured": True,
                                }
                            },
                        }
                    }
                }
            }
        },
    )

    config = load_competition_config(config_path)
    goal = resolve_navigation_waypoint(config.navigation, "A", "pickup")

    assert goal.x_m == 1.25
    assert goal.y_m == -0.40
    assert goal.yaw_rad == 1.57
    assert goal.map_path == str(tmp_path / "maps/arena_a.yaml")
    assert goal.pbstream_path == str(tmp_path / "maps/arena_a.pbstream")


def test_navigation_rejects_missing_map_or_unconfigured_waypoint(
    tmp_path,
):
    """Catches accepting zero defaults as real navigation targets."""
    config_path = _write_config(
        tmp_path,
        {
            "competition": {
                "navigation": {
                    "arenas": {
                        "A": {
                            "map": "",
                            "pbstream": "",
                            "waypoints": {
                                "pickup": {
                                    "x": 0.0,
                                    "y": 0.0,
                                    "yaw": 0.0,
                                    "configured": False,
                                }
                            },
                        }
                    }
                }
            }
        },
    )

    config = load_competition_config(config_path)

    with pytest.raises(CompetitionConfigError, match="map"):
        resolve_navigation_waypoint(config.navigation, "A", "pickup")


def test_arena_lock_accepts_first_arena_and_rejects_later_mismatch():
    """Catches silently switching maps after arena inference."""
    lock = ArenaLock()

    assert lock.accept("B") == "B"
    assert lock.accept("B") == "B"
    with pytest.raises(CompetitionConfigError, match="arena"):
        lock.accept("A")


def test_vision_and_manipulation_sections_override_existing_ros_parameters(
    tmp_path,
):
    """Catches consumers ignoring the top-level competition YAML sections."""
    config_path = _write_config(
        tmp_path,
        {
            "competition": {
                "vision": {
                    "class_aliases": {"chilun": "gear", "luosi": "t_bolt"},
                    "sorting_rule": {
                        "enabled": True,
                        "park_1_roi": [10, 20, 110, 120],
                        "park_2_roi": [130, 20, 230, 120],
                    },
                },
                "manipulation": {
                    "placement": {
                        "enabled": True,
                        "approach_m": 0.08,
                        "layer_step_m": 0.045,
                        "park_1": {
                            "x_m": 0.31,
                            "y_m": 0.11,
                            "first_layer_z_m": 0.06,
                        },
                        "park_2": {
                            "x_m": 0.42,
                            "y_m": -0.12,
                            "first_layer_z_m": 0.07,
                        },
                        "slot_offsets_xy_m": [
                            0.0,
                            0.0,
                            0.05,
                            0.0,
                            0.05,
                            0.05,
                            0.0,
                            0.05,
                        ],
                    }
                },
            }
        },
    )

    config = load_competition_config(config_path)

    vision = apply_vision_backend_overrides(
        {"sorting_rule": {"enabled": False}, "class_aliases": {}},
        config.vision,
    )
    placement = apply_manipulation_placement_overrides(
        {"placement": {"enabled": False}},
        config.manipulation,
    )

    assert vision["sorting_rule"]["enabled"] is True
    assert vision["sorting_rule"]["park_1_roi"] == [10, 20, 110, 120]
    assert vision["class_aliases"] == {"chilun": "gear", "luosi": "t_bolt"}
    assert placement["placement"]["enabled"] is True
    assert placement["placement"]["park_1"]["x_m"] == 0.31
    assert placement["placement"]["slot_offsets_xy_m"] == [
        0.0,
        0.0,
        0.05,
        0.0,
        0.05,
        0.05,
        0.0,
        0.05,
    ]
