from pathlib import Path
import importlib.util

import yaml


MODULE_PATH = Path(__file__).resolve().parents[1] / "scripts" / "arm_motion_calibration_model.py"
spec = importlib.util.spec_from_file_location("arm_motion_calibration_model", MODULE_PATH)
model = importlib.util.module_from_spec(spec)
spec.loader.exec_module(model)


def _pose(seed: float):
    return {
        "configured": True,
        "joints_rad": [seed + i for i in range(5)],
        "x_m": seed,
        "y_m": seed + 0.1,
        "z_m": seed + 0.2,
        "pitch_rad": seed + 0.3,
        "yaw_rad": seed + 0.4,
        "speed_rad_s": 0.5,
    }


def test_merge_calibration_updates_only_selected_arena_and_removes_legacy_duplicates():
    source = {
        "competition": {
            "backend_name": "nav2_competition",
            "vision": {
                "sorting_scan_a": {"configured": False},
                "sorting_scan_b": {"configured": False},
                "sorting_rule": {"enabled": False},
            },
            "arm_motion": {
                "fixed_poses": {
                    "sorting_scan_b": _pose(8.0),
                },
                "arenas": {
                    "B": {
                        "pickup": {"observe": {"configured": True, "x_m": 9.0}}
                    }
                }
            },
            "manipulation": {
                "placement": {
                    "enabled": False,
                    "park_1": {"x_m": 9.0},
                    "park_2": {"x_m": 9.0},
                    "approach_m": 0.06,
                    "layer_step_m": 0.05,
                    "slot_offsets_xy_m": [0, 0, 0.05, 0, 0.05, 0.05, 0, 0.05],
                }
            },
        }
    }
    calibration = {
        "fixed_poses": {
            "zero": _pose(0.0),
            "sorting_scan_a": _pose(1.0),
            "navigation_safe": _pose(3.0),
        },
        "pickup": {
            "observe": _pose(4.0),
            "layer_z_configured": True,
            "layer_z_m": [0.03, 0.08, 0.13],
        },
        "park_1": {
            "prepare": _pose(5.0),
            "placement_reference": {"configured": True, "x_m": 0.2, "y_m": 0.1, "first_layer_z_m": 0.05},
        },
        "park_2": {
            "prepare": _pose(6.0),
            "placement_reference": {"configured": True, "x_m": 0.3, "y_m": -0.1, "first_layer_z_m": 0.05},
        },
    }

    merged = model.merge_calibration(source, "A", calibration)
    competition = merged["competition"]

    assert competition["arm_motion"]["fixed_poses"]["navigation_safe"]["joints_rad"][0] == 3.0
    assert competition["arm_motion"]["fixed_poses"]["sorting_scan_a"]["joints_rad"][0] == 1.0
    assert competition["arm_motion"]["fixed_poses"]["sorting_scan_b"]["joints_rad"][0] == 8.0
    assert competition["arm_motion"]["arenas"]["A"]["pickup"]["layer_z_m"] == [0.03, 0.08, 0.13]
    assert competition["arm_motion"]["arenas"]["B"]["pickup"]["observe"]["x_m"] == 9.0
    assert "sorting_scan_a" not in competition["vision"]
    assert "sorting_scan_b" not in competition["vision"]
    assert "park_1" not in competition["manipulation"]["placement"]
    assert "park_2" not in competition["manipulation"]["placement"]
    assert competition["manipulation"]["placement"]["enabled"] is True


def test_dump_clean_yaml_contains_no_comments_and_round_trips():
    data = {"competition": {"arm_motion": {"fixed_poses": {"zero": _pose(0.0)}}}}
    text = model.dump_clean_yaml(data)

    assert "#" not in text
    assert yaml.safe_load(text) == data
