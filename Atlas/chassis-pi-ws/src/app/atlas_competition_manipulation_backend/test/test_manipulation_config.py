import pytest

from atlas_competition_manipulation_backend.backend import (
    load_placement_config,
)


def test_load_placement_config_accepts_top_level_competition_section(tmp_path):
    """Catches the backend ignoring competition.manipulation.placement."""
    config_path = tmp_path / "competition.yaml"
    config_path.write_text(
        """
competition:
  manipulation:
    placement:
      enabled: true
      approach_m: 0.08
      layer_step_m: 0.045
      park_1: {x_m: 0.31, y_m: 0.11, first_layer_z_m: 0.06}
      park_2: {x_m: 0.42, y_m: -0.12, first_layer_z_m: 0.07}
      slot_offsets_xy_m: [0.0, 0.0, 0.05, 0.0, 0.05, 0.05, 0.0, 0.05]
""",
        encoding="utf-8",
    )

    placement = load_placement_config(str(config_path))

    assert placement["enabled"] is True
    assert placement["approach_m"] == 0.08
    assert placement["layer_step_m"] == 0.045
    assert placement["park_1"]["x_m"] == 0.31
    assert placement["park_2"]["y_m"] == -0.12


def _arm_motion_config():
    return {
        "fixed_poses": {
            "navigation_safe": {
                "configured": True,
                "joints_rad": [0, 0, 0, 0, 0],
                "x_m": 0.0,
                "y_m": 0.0,
                "z_m": 0.3,
                "pitch_rad": 0.0,
                "yaw_rad": 0.0,
                "speed_rad_s": 0.5,
            }
        },
        "arenas": {
            "A": {
                "pickup": {
                    "observe": {
                        "configured": True,
                        "joints_rad": [1, 2, 3, 4, 5],
                        "x_m": 0.2,
                        "y_m": 0.0,
                        "z_m": 0.35,
                        "pitch_rad": -1.2,
                        "yaw_rad": 0.7,
                        "speed_rad_s": 0.4,
                    },
                    "layer_z_configured": True,
                    "layer_z_m": [0.03, 0.08, 0.13],
                },
                "park_1": {
                    "prepare": {
                        "configured": True,
                        "joints_rad": [5, 4, 3, 2, 1],
                        "x_m": 0.25,
                        "y_m": 0.05,
                        "z_m": 0.30,
                        "pitch_rad": -1.0,
                        "yaw_rad": 0.1,
                        "speed_rad_s": 0.4,
                    },
                    "placement_reference": {
                        "configured": True,
                        "x_m": 0.30,
                        "y_m": 0.10,
                        "first_layer_z_m": 0.05,
                    },
                },
            }
        },
    }


def test_pickup_target_spec_uses_arena_layer_height_and_observe_orientation():
    from atlas_competition_manipulation_backend.backend import pickup_target_spec

    spec = pickup_target_spec(_arm_motion_config(), "A", 2)

    assert spec["target_z_m"] == 0.08
    assert spec["pitch_rad"] == -1.2
    assert spec["yaw_rad"] == 0.7


def test_compute_placement_target_uses_arena_reference_slot_and_layer():
    from atlas_competition_manipulation_backend.backend import compute_placement_target

    placement = {
        "enabled": True,
        "layer_step_m": 0.05,
        "slot_offsets_xy_m": [0.0, 0.0, 0.05, 0.0, 0.05, 0.05, 0.0, 0.05],
    }
    target = compute_placement_target(
        _arm_motion_config(), placement, "A", "park_1", 2, 1
    )

    assert target.x == pytest.approx(0.35)
    assert target.y == pytest.approx(0.15)
    assert target.z == pytest.approx(0.10)


def test_pre_recognition_reports_status_and_moves_to_pickup_observe(monkeypatch):
    import atlas_competition_manipulation_backend.backend as backend

    class DummyStatus:
        STATE_RUNNING = 1

    monkeypatch.setattr(backend, "ManipulationStatus", DummyStatus)
    node = object.__new__(backend.CompetitionManipulationBackend)
    status_calls = []
    move_calls = []
    node._set_status = lambda state, **kwargs: status_calls.append((state, kwargs))
    node._move_named_pose = lambda name, arena="", area="": move_calls.append(
        (name, arena, area)
    ) or True

    assert node._do_pre_recognition("A", "pickup") is True
    assert status_calls == [
        (1, {"step": "move_to_observe_pose", "message": "移动到 pickup 固定观察或预备位"})
    ]
    assert move_calls == [("pickup_observe", "A", "")]


def test_pick_contact_target_descends_to_calibrated_layer_z_not_relative_magic_distance():
    from atlas_competition_manipulation_backend.backend import XYZ, compute_pick_contact_target

    above = XYZ(0.2, -0.1, 0.18)
    contact = compute_pick_contact_target(above, 0.12)

    assert contact == XYZ(0.2, -0.1, 0.12)
