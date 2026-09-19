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


def test_pickup_target_spec_uses_layer_height_and_vertical_orientation():
    from atlas_competition_manipulation_backend.backend import pickup_target_spec

    spec = pickup_target_spec(_arm_motion_config(), "A", 2)

    assert spec["target_z_m"] == 0.08
    assert spec["pitch_rad"] == 0.0
    assert spec["yaw_rad"] == 0.0


def test_pickup_target_spec_uses_configured_pick_orientation():
    from atlas_competition_manipulation_backend.backend import pickup_target_spec

    motion = _arm_motion_config()
    motion["pick_orientation"] = {"pitch_rad": -0.1, "yaw_rad": 0.2}
    spec = pickup_target_spec(motion, "A", 2)

    assert spec["target_z_m"] == 0.08
    assert spec["pitch_rad"] == -0.1
    assert spec["yaw_rad"] == 0.2


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
    from atlas_competition_manipulation_backend.backend import (
        compute_placement_target, pickup_target_spec,
    )

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
    spec = pickup_target_spec(motion, "A", 2, 3)
    assert spec["target_z_m"] == pytest.approx(0.05)
    assert spec["pitch_rad"] == 0.0
    assert spec["yaw_rad"] == 0.0
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


def test_pick_contact_target_descends_to_calibrated_layer_z_not_relative_magic_distance():
    from atlas_competition_manipulation_backend.backend import XYZ, compute_pick_contact_target

    above = XYZ(0.2, -0.1, 0.18)
    contact = compute_pick_contact_target(above, 0.12)

    assert contact == XYZ(0.2, -0.1, 0.12)


def test_move_pose_sends_five_dof_target_and_suction(monkeypatch):
    import atlas_competition_manipulation_backend.backend as backend

    class DummySetArmPose:
        class Request:
            pass

    monkeypatch.setattr(backend, "SetArmPose", DummySetArmPose)
    node = object.__new__(backend.CompetitionManipulationBackend)
    node.default_speed_rad_s = 0.8
    node.motion_timeout_s = 30.0
    node.arm_pose_client = object()
    node._cancelled = lambda: False
    captured = {}

    def call_service(client, req):
        captured["client"] = client
        captured["req"] = req
        return type("Result", (), {"success": True})()

    node._call_service = call_service
    node._wait_pose_target = lambda target, timeout: captured.update(
        target=target, timeout=timeout
    ) or True

    target = backend.XYZ(0.2, -0.1, 0.12)
    assert node._move_pose(
        target,
        pitch_rad=-0.11,
        yaw_rad=0.23,
        suction_valid=True,
        suction_enable=True,
    ) is True

    req = captured["req"]
    assert (req.x_m, req.y_m, req.z_m) == pytest.approx((0.2, -0.1, 0.12))
    assert req.pitch_rad == pytest.approx(-0.11)
    assert req.yaw_rad == pytest.approx(0.23)
    assert req.speed_rad_s == pytest.approx(0.8)
    assert req.suction_valid is True
    assert req.suction_enable is True
    assert captured["target"] == target
    assert captured["timeout"] == pytest.approx(30.0)


def test_pick_descend_and_lift_keep_same_pose_constraint(monkeypatch):
    import atlas_competition_manipulation_backend.backend as backend

    class DummyPickTarget:
        def __init__(self):
            self.corner_index = 0
            self.layer = 0
            self.use_target_z = False
            self.target_z_m = 0.0
            self.use_orientation = False
            self.pitch_rad = 0.0
            self.yaw_rad = 0.0
            self.use_approach = False
            self.approach_m = 0.0

    class DummyStatus:
        STATE_RUNNING = 1

    class DummyPublisher:
        def __init__(self):
            self.messages = []

        def publish(self, msg):
            self.messages.append(msg)

    monkeypatch.setattr(backend, "PickTarget", DummyPickTarget)
    monkeypatch.setattr(backend, "ManipulationStatus", DummyStatus)
    monkeypatch.setattr(
        backend,
        "pickup_target_spec",
        lambda _arm, _arena, _layer, _slot: {
            "target_z_m": 0.12,
            "pitch_rad": -0.10,
            "yaw_rad": 0.20,
        },
    )

    node = object.__new__(backend.CompetitionManipulationBackend)
    node.arm_motion = {}
    node.motion_timeout_s = 30.0
    node.pick_target_settle_s = 0.0
    node.pick_approach_m = 0.05
    node.pick_suction_hold_s = 0.0
    node.pick_approach_timeout_s = 7.0
    node.pick_motion_start_timeout_s = 2.0
    node.pick_target_pub = DummyPublisher()
    node._current_pose = lambda: backend.XYZ(0.10, 0.00, 0.30)
    node._wait_for_motion_then_stable = lambda _start, _timeout, _start_timeout=None: backend.XYZ(
        0.20, -0.10, 0.17
    )
    node._set_status = lambda *_args, **_kwargs: None
    node.get_logger = lambda: type(
        "Logger", (), {"error": lambda self, _msg: None, "warn": lambda self, _msg: None}
    )()
    moves = []

    def move_pose(target, **kwargs):
        moves.append((target, kwargs))
        return True

    node._move_pick_pose_with_fallback = move_pose

    assert node._do_pick("A", 2, 1) is True
    assert len(moves) == 2

    descend_target, descend = moves[0]
    lift_target, lift = moves[1]
    assert descend_target == backend.XYZ(0.20, -0.10, 0.12)
    assert lift_target == backend.XYZ(0.20, -0.10, 0.17)
    for params in (descend, lift):
        assert params["pitch_rad"] == pytest.approx(-0.10)
        assert params["yaw_rad"] == pytest.approx(0.20)
        assert params["suction_valid"] is True
        assert params["suction_enable"] is True
    assert descend["phase"] == "pick_descend"
    assert lift["phase"] == "pick_lift"

    published = node.pick_target_pub.messages[0]
    assert published.use_orientation is True
    assert published.pitch_rad == pytest.approx(-0.10)
    assert published.yaw_rad == pytest.approx(0.20)


def test_pick_pose_falls_back_to_position_when_5d_does_not_move(monkeypatch):
    import atlas_competition_manipulation_backend.backend as backend

    class DummySetArmPose:
        class Request:
            pass

    class DummySetArmPosition:
        class Request:
            pass

    monkeypatch.setattr(backend, "SetArmPose", DummySetArmPose)
    monkeypatch.setattr(backend, "SetArmPosition", DummySetArmPosition)

    node = object.__new__(backend.CompetitionManipulationBackend)
    node.default_speed_rad_s = 0.8
    node.pick_motion_timeout_s = 4.0
    node.pick_motion_start_timeout_s = 2.0
    node.pick_allow_position_fallback = True
    node.arm_pose_client = object()
    node.arm_position_client = object()
    node._cancelled = lambda: False
    node._current_pose = lambda: backend.XYZ(0.1, 0.0, 0.2)
    node.get_logger = lambda: type(
        "Logger", (), {"warn": lambda self, _msg: None, "error": lambda self, _msg: None}
    )()

    calls = []

    def call_service(client, req):
        calls.append((client, req))
        return type("Result", (), {"success": True})()

    node._call_service = call_service
    statuses = iter(["no_motion", "reached"])
    node._wait_pose_target_watchdog = lambda *_args, **_kwargs: next(statuses)

    target = backend.XYZ(0.2, -0.1, 0.12)
    assert node._move_pick_pose_with_fallback(
        target, pitch_rad=0.0, yaw_rad=0.0,
        suction_valid=True, suction_enable=True, phase="pick_descend"
    ) is True

    assert len(calls) == 2
    assert calls[0][0] is node.arm_pose_client
    assert calls[1][0] is node.arm_position_client
    assert isinstance(calls[0][1], DummySetArmPose.Request)
    assert isinstance(calls[1][1], DummySetArmPosition.Request)
    assert calls[1][1].suction_enable is True
