import numpy as np

from handeye_bridge.vision_pose_gate import VisionPoseTarget, vision_pose_for_position


def test_vision_pose_ready_accepts_initial_and_sorting_scan_targets():
    poses = [
        VisionPoseTarget("initial", True, np.array([0.20, 0.00, 0.30])),
        VisionPoseTarget("sorting_scan_a", True, np.array([0.30, 0.10, 0.35])),
        VisionPoseTarget("sorting_scan_b", True, np.array([0.30, -0.10, 0.35])),
    ]

    assert vision_pose_for_position(np.array([0.201, 0.0, 0.30]), poses, 0.01) == "initial"
    assert vision_pose_for_position(np.array([0.30, 0.099, 0.35]), poses, 0.01) == "sorting_scan_a"
    assert (
        vision_pose_for_position(np.array([0.30, -0.099, 0.35]), poses, 0.01)
        == "sorting_scan_b"
    )


def test_vision_pose_ready_ignores_unconfigured_targets_and_other_positions():
    poses = [
        VisionPoseTarget("initial", True, np.array([0.20, 0.00, 0.30])),
        VisionPoseTarget("sorting_scan_a", False, np.array([0.30, 0.10, 0.35])),
    ]

    assert vision_pose_for_position(np.array([0.30, 0.10, 0.35]), poses, 0.01) is None
    assert vision_pose_for_position(np.array([0.40, 0.20, 0.10]), poses, 0.01) is None
