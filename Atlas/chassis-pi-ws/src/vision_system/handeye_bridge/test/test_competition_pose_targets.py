from handeye_bridge.competition_pose_targets import (
    pickup_plane_calibrations,
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


def test_pickup_plane_calibrations_use_slot_heights_and_derive_distances():
    pose = _pose(0.2)
    observations = [
        {**pose, "z_m": 0.30 + slot * 0.01,
         "layer_z_m": [0.02 + slot * 0.001, 0.06 + slot * 0.001]}
        for slot in range(4)
    ]
    arm_motion = {"arenas": {"A": {"pickup": {"observations": observations}}}}

    calibrations = pickup_plane_calibrations(arm_motion)

    slot2 = calibrations["pickup_observe_a_slot2"]
    assert slot2["layer_z_m"] == (0.022, 0.062)
    assert slot2["camera_to_plane_distance_m"] == (0.298, 0.258)


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


def test_observation_axis_comes_from_calibrated_joints_not_legacy_rpy():
    import numpy as np
    from handeye_bridge.competition_pose_targets import calibrated_tool_axis
    pose = _pose(.2)
    arm_motion = {'arenas': {'A': {'pickup': {'observe': pose}}}}
    targets = pickup_vision_pose_targets(arm_motion, include_axis=True)
    np.testing.assert_allclose(targets[0][2], calibrated_tool_axis(pose))
    pose['pitch_rad'] = 2.5
    pose['yaw_rad'] = -2.3
    np.testing.assert_allclose(pickup_vision_pose_targets(arm_motion, include_axis=True)[0][2], targets[0][2])
    pose['joints_rad'][2] = .5
    assert not np.allclose(pickup_vision_pose_targets(arm_motion, include_axis=True)[0][2], targets[0][2])
