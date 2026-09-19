from handeye_bridge.competition_pose_targets import (
    pickup_vision_pose_targets,
    park_vision_pose_targets,
)


def _pose(x):
    return {
        "configured": True,
        "joints_rad": [0, 0, 0, 0, 0],
        "x_m": x,
        "y_m": 0.1,
        "z_m": 0.3,
        "pitch_rad": -1.0,
        "yaw_rad": 0.2,
        "speed_rad_s": 0.5,
    }


def test_pickup_vision_pose_targets_collects_configured_a_and_b():
    arm_motion = {
        "arenas": {
            "A": {"pickup": {"observe": _pose(0.2)}},
            "B": {"pickup": {"observe": _pose(0.4)}},
        }
    }

    assert pickup_vision_pose_targets(arm_motion) == [
        ("pickup_observe_a", (0.2, 0.1, 0.3)),
        ("pickup_observe_b", (0.4, 0.1, 0.3)),
    ]


def test_pickup_vision_pose_targets_skips_unconfigured_arena():
    pose = _pose(0.2)
    pose["configured"] = False
    arm_motion = {"arenas": {"A": {"pickup": {"observe": pose}}}}

    assert pickup_vision_pose_targets(arm_motion) == []


def test_pickup_vision_pose_targets_include_all_four_observations():
    pose = _pose(0.2)
    arm_motion = {
        "arenas": {
            "A": {
                "pickup": {
                    "observations": [{**pose, "x_m": 0.2 + slot * 0.1} for slot in range(4)]
                }
            }
        }
    }
    targets = pickup_vision_pose_targets(arm_motion)
    assert [name for name, _ in targets] == [
        "pickup_observe_a_slot0", "pickup_observe_a_slot1",
        "pickup_observe_a_slot2", "pickup_observe_a_slot3",
    ]


def test_park_vision_pose_targets_include_only_configured_prepare_poses():
    unconfigured = _pose(0.4)
    unconfigured["configured"] = False
    arm_motion = {
        "arenas": {
            "A": {"park_1": {"prepare": _pose(0.2)}, "park_2": {"prepare": _pose(0.3)}},
            "B": {"park_1": {"prepare": unconfigured}},
        }
    }

    assert park_vision_pose_targets(arm_motion) == [
        ("park_1_prepare_a", (0.2, 0.1, 0.3)),
        ("park_2_prepare_a", (0.3, 0.1, 0.3)),
    ]
