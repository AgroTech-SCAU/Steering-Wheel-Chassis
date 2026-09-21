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


def test_four_observations_and_four_placement_points_use_selected_slot():
    from atlas_competition_manipulation_backend.backend import compute_placement_target

    motion = _arm_motion_config()
    pickup = motion["arenas"]["A"]["pickup"]
    pickup.pop("observe")
    pickup["observations"] = [
        {**_pose_for_slot(slot), "layer_z_m": [0.01 + slot * 0.01, 0.02 + slot * 0.01]}
        for slot in range(4)
    ]
    park = motion["arenas"]["A"]["park_1"]
    park.pop("placement_reference")
    park["placement_points"] = [
        {"configured": True, "x_m": 0.1 + slot * 0.1, "y_m": 0.2, "first_layer_z_m": 0.03}
        for slot in range(4)
    ]
    target = compute_placement_target(motion, {"enabled": True, "layer_step_m": 0.05}, "A", "park_1", 3, 0)
    assert (target.x, target.y, target.z) == pytest.approx((0.4, 0.2, 0.03))


def _pose_for_slot(slot):
    return {
        "configured": True, "joints_rad": [0.0] * 5,
        "x_m": 0.2, "y_m": 0.0, "z_m": 0.3,
        "pitch_rad": 0.0, "yaw_rad": slot * 0.1, "speed_rad_s": 0.5,
    }


def test_pre_recognition_reports_status_and_moves_to_pickup_observe(monkeypatch):
    import atlas_competition_manipulation_backend.backend as backend

    class DummyStatus:
        STATE_RUNNING = 1

    monkeypatch.setattr(backend, "ManipulationStatus", DummyStatus)
    node = object.__new__(backend.CompetitionManipulationBackend)
    status_calls = []
    move_calls = []
    node._set_status = lambda state, **kwargs: status_calls.append((state, kwargs))
    node._move_named_pose = lambda name, arena="", area="", slot=0: move_calls.append(
        (name, arena, area, slot)
    ) or True
    node.settle_before_observe_s = 0.0

    assert node._do_pre_recognition("A", "pickup", 3) is True
    assert status_calls == [
        (1, {"step": "move_to_observe_pose", "message": "移动到 pickup 固定观察或预备位"})
    ]
    assert move_calls == [("pickup_observe", "A", "", 3)]


def test_pick_reuses_screw_pick_without_overriding_verified_bridge_parameters(monkeypatch):
    import atlas_competition_manipulation_backend.backend as backend

    class DummyPickTarget:
        def __init__(self):
            self.corner_index = 0
            self.layer = 0
            self.use_target_z = True
            self.target_z_m = 123.0
            self.use_orientation = True
            self.pitch_rad = 123.0
            self.yaw_rad = 123.0
            self.use_approach = True
            self.approach_m = 123.0

    class DummyStatus:
        STATE_RUNNING = 1

    class DummyPublisher:
        def __init__(self):
            self.messages = []
        def publish(self, msg):
            self.messages.append(msg)

    monkeypatch.setattr(backend, "PickTarget", DummyPickTarget)
    monkeypatch.setattr(backend, "ManipulationStatus", DummyStatus)

    node = object.__new__(backend.CompetitionManipulationBackend)
    node._next_pick_id = 100
    node._cancelled = lambda: False
    node.pick_target_settle_s = 0.0
    node.pick_suction_hold_s = 0.0
    node.pick_lift_m = 0.05
    node.motion_timeout_s = 30.0
    node.pick_bridge_timeout_s = 10.0
    node.pick_motion_start_timeout_s = 2.0
    node.pick_target_pub = DummyPublisher()
    node._current_pose = lambda: backend.XYZ(0.26, 0.03, 0.21)
    from types import SimpleNamespace
    node._wait_pick_result = lambda *_args: SimpleNamespace(
        x_m=0.31, y_m=-0.04, z_m=0.073, pitch_rad=-1.57, yaw_rad=0.0)
    node._wait_pose_target = lambda *_args, **_kwargs: True
    node._set_status = lambda *_args, **_kwargs: None
    node.get_logger = lambda: type("Logger", (), {"error": lambda self, _msg: None})()
    suction = []
    node._set_suction = lambda enabled: suction.append(enabled) or True
    moves = []
    node._move_pose = lambda target, **kwargs: moves.append((target, kwargs)) or True

    assert node._do_pick("B", 2, 2) is True

    assert len(node.pick_target_pub.messages) == 1
    msg = node.pick_target_pub.messages[0]
    assert msg.corner_index == 2
    assert msg.layer == 2
    assert msg.use_target_z is False
    assert msg.use_orientation is False
    assert msg.use_approach is False
    assert suction == [True]
    assert len(moves) == 1
    lift, params = moves[0]
    assert lift == backend.XYZ(0.31, -0.04, 0.123)
    assert params["direction"] == (-1.57, 0.0)
    assert msg.request_id == 101
    assert params["suction_valid"] is True
    assert params["suction_enable"] is True


def test_pick_bridge_rejection_fails_locally_for_scheduler_defer(monkeypatch):
    import atlas_competition_manipulation_backend.backend as backend

    class DummyPickTarget:
        def __init__(self):
            self.corner_index = 0
            self.layer = 0
            self.use_target_z = False
            self.use_orientation = False
            self.use_approach = False

    class DummyStatus:
        STATE_RUNNING = 1

    class DummyPublisher:
        def publish(self, _msg):
            pass

    monkeypatch.setattr(backend, "PickTarget", DummyPickTarget)
    monkeypatch.setattr(backend, "ManipulationStatus", DummyStatus)
    node = object.__new__(backend.CompetitionManipulationBackend)
    node._next_pick_id = 100
    node._cancelled = lambda: False
    node.pick_target_settle_s = 0.0
    node.motion_timeout_s = 30.0
    node.pick_bridge_timeout_s = 10.0
    node.pick_motion_start_timeout_s = 2.0
    node.pick_target_pub = DummyPublisher()
    node._current_pose = lambda: backend.XYZ(0.2, 0.0, 0.3)
    node._wait_pick_result = lambda *_args: None
    node._set_status = lambda *_args, **_kwargs: None
    node.get_logger = lambda: type("Logger", (), {"error": lambda self, _msg: None})()
    node._set_suction = lambda _enabled: (_ for _ in ()).throw(AssertionError("must not suction"))

    assert node._do_pick("A", 0, 1) is False
