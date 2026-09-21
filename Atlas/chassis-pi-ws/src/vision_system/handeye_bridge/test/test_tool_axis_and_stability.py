import math
import numpy as np
from handeye_bridge import vision_pose_gate as gate
from handeye_bridge import pick_target_config as config


def test_direction_uses_tool_z_column_and_vertical_yaw_is_defined():
    assert hasattr(config, 'tool_axis_angles')
    transform = np.eye(4)
    transform[:3, 2] = [0., 0., -1.]
    assert config.tool_axis_angles(transform) == (-math.pi / 2, 0.)
    transform[:3, 2] = [0.3, -0.4, -math.sqrt(0.75)]
    pitch, yaw = config.tool_axis_angles(transform)
    np.testing.assert_allclose(gate.direction_axis(pitch, yaw), transform[:3, 2])


def test_matching_xyz_rejects_wrong_tool_axis_and_accepts_free_spin():
    assert hasattr(gate, 'vision_pose_for_transform')
    t = np.eye(4)
    target = gate.VisionPoseTarget('obs', True, t[:3, 3], np.array([0., 0., -1.]))
    assert gate.vision_pose_for_transform(t, [target], .01, 5.) is None
    t[:3, :3] = np.diag([-1., 1., -1.])
    assert gate.vision_pose_for_transform(t, [target], .01, 5.) == 'obs'


def test_tcp_stability_requires_whole_window_freshness_and_less_than_one_mm():
    assert hasattr(gate, 'tcp_is_stable')
    t = np.eye(4)
    history = [(1_000_000_000, t.copy()), (1_100_000_000, t.copy()), (1_200_000_000, t.copy())]
    assert gate.tcp_is_stable(history, 1_200_000_000, .2, .001, .3)
    assert not gate.tcp_is_stable(history[-2:], 1_200_000_000, .2, .001, .3)
    assert not gate.tcp_is_stable(history, 1_600_000_000, .2, .001, .3)
    history[1][1][0, 3] = .0011
    assert not gate.tcp_is_stable(history, 1_200_000_000, .2, .001, .3)
    # A fresh packet cannot conceal a long gap in telemetry
    assert not gate.tcp_is_stable([history[0], (3_000_000_000, t)], 3_000_000_000, .2, .001, .3)
