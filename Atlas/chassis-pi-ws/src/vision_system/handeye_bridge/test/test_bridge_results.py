"""Exercise actual bridge callbacks without requiring generated ROS modules"""
import ast
from collections import OrderedDict, deque
from pathlib import Path
from types import SimpleNamespace as NS
from typing import Callable, Optional, Tuple
import numpy as np
from handeye_bridge.pick_target_config import pick_command_z, resolve_pick_target_parameters, select_pick_detection, tool_axis_angles
from handeye_bridge.vision_pose_gate import *


def bridge_class():
    source = Path(__file__).parents[1] / 'handeye_bridge' / 'bridge_node.py'
    module = ast.parse(source.read_text())
    node = next(n for n in module.body if isinstance(n, ast.ClassDef) and n.name == 'HandEyeBridgeNode')
    node.bases = []
    namespace = dict(globals(), PickTarget=NS, PoseStamped=NS, DetectionCenterArray=NS, PickResult=NS, Bool=NS)
    exec(compile(ast.Module(body=[node], type_ignores=[]), str(source), 'exec'), namespace)
    return namespace[node.name]


def make_bridge():
    bridge = bridge_class().__new__(bridge_class())
    bridge.clock_ns = 1_000_000_000
    bridge.get_clock = lambda: NS(now=lambda: NS(nanoseconds=bridge.clock_ns))
    bridge.get_logger = lambda: NS(info=lambda *a, **k: None, warn=lambda *a, **k: None, error=lambda *a, **k: None)
    params = dict(initial_pitch_rad=-3.063, initial_yaw_rad=-3.112, default_speed_rad_s=.5, target_z_offset_m=.003, auto_send=True,
                  plane1_z_m=.031, plane2_z_m=.07, plane3_z_m=.109,
                  max_detection_age_s=.5, final_best_valid_s=30., max_pose_sync_dt_ms=80.,
                  depth_mode='manual', manual_offset_x_m=-.055, manual_offset_y_m=0.,
                  manual_offset_z_m=0., workspace_max_xy_m=.5, workspace_z_min_m=-.2,
                  workspace_z_max_m=.5, plane_heights_configured=True,
                  arm_result_timeout_s=1., initial_position_tolerance_m=.005,
                  vision_axis_tolerance_deg=5., vision_pose_max_age_s=.3,
                  pick_approach_timeout_s=10.)
    bridge.get_parameter = lambda key: NS(value=params[key])
    bridge.results = []
    bridge.pick_result_pub = NS(publish=bridge.results.append)
    bridge._arm_pending = {}
    bridge._early_arm_results = OrderedDict()
    bridge._active_pick = None
    bridge._calibration_ready = True
    bridge._vision_pose_ready = True
    bridge._vision_ready_since_ns = 0
    bridge._detection_pose_cache_size = 2000
    bridge._latest_detections = None
    bridge._latest_detection_stamp_ns = 1_000_000_000
    bridge._latest_detection_received_ns = 1_000_000_000
    bridge._latest_detection_is_final_best = False
    t = np.diag([1., -1., -1., 1.])
    bridge._detection_pose_cache = {1_000_000_000: (t, 0.)}
    bridge._pose_history = deque([(1_000_000_000, t)])
    bridge._ray_plane_intersect = lambda *a: np.array([.3, .02, .031])
    bridge.commands = []
    bridge._send_pose = lambda *a, **k: bridge.commands.append((a, k)) or True
    return bridge


def test_no_detection_and_workspace_rejection_publish_request_correlated_failure():
    b = make_bridge()
    b._on_pick_target(NS(request_id=41, corner_index=0, layer=1))
    assert len(b.results) == 1, 'missing explicit vision rejection'
    assert (b.results[0].request_id, b.results[0].reason, b.results[0].success) == (41, 'NO_DETECTION', False)
    b._latest_detections = NS(detections=[NS(corner_index=0, u=10., v=20., cls_name='x')])
    b._ray_plane_intersect = lambda *a: np.array([2., 0., .03])
    b._on_pick_target(NS(request_id=42, corner_index=0, layer=1))
    assert b.results[-1].reason == 'WORKSPACE_REJECTED'
    assert not b.commands


def test_contact_axis_uses_detection_pose_and_success_waits_for_mcu():
    b = make_bridge()
    b._latest_detections = NS(detections=[NS(corner_index=0, u=10., v=20., cls_name='x')])
    b._on_pick_target(NS(request_id=9, corner_index=0, layer=1))
    assert not b.results
    args, kwargs = b.commands[0]
    assert np.allclose(args[:3], [.245, .02, .034])
    assert kwargs['pitch'] == -np.pi / 2
    kwargs['on_result'](True, 'ACCEPTED', 27)
    assert b.results[-1].success and b.results[-1].command_seq == 27
    assert b.results[-1].z_m == .034


def test_queue_response_is_not_mcu_acceptance_and_wrong_seq_is_ignored():
    b = make_bridge()
    assert hasattr(b, '_on_arm_command_result')
    observed = []
    token = object()
    b._arm_pending[token] = dict(seq=None, sent_ns=b.clock_ns, deadline_ns=b.clock_ns + 1_000_000_000,
                                 done=None, result=lambda *a: observed.append(a))
    b._on_pose_result(NS(result=lambda: NS(success=True, command_seq=22)), 0, 0, 0, token=token)
    assert observed == []
    b._on_arm_command_result(NS(command_seq=23, result=1, arm_status=-1))
    assert observed == []
    b._on_arm_command_result(NS(command_seq=22, result=1, arm_status=-1))
    assert observed == [(False, 'MCU_REJECTED', 22)]


def test_early_mcu_result_and_timeout_do_not_complete_next_request():
    b = make_bridge()
    assert hasattr(b, '_on_arm_command_result')
    observed = []
    token = object()
    b._arm_pending[token] = dict(seq=None, sent_ns=b.clock_ns, deadline_ns=b.clock_ns + 1_000_000_000,
                                 done=None, result=lambda *a: observed.append(a))
    b._on_arm_command_result(NS(command_seq=22, result=0, arm_status=0))
    b._on_pose_result(NS(result=lambda: NS(success=True, command_seq=22)), 0, 0, 0, token=token)
    assert observed == [(True, 'ACCEPTED', 22)]
    token2 = object()
    b._arm_pending[token2] = dict(seq=None, sent_ns=b.clock_ns, deadline_ns=b.clock_ns + 1,
                                  done=None, result=lambda *a: observed.append(a))
    b.clock_ns += 2
    b._check_arm_results()
    assert observed[-1] == (False, 'MCU_TIMEOUT', 0)
    b._on_pose_result(NS(result=lambda: NS(success=True, command_seq=24)), 0, 0, 0, token=token2)
    b._on_arm_command_result(NS(command_seq=24, result=0, arm_status=0))
    assert len(observed) == 2


def test_approach_waits_for_arrival_then_final_contact_acceptance():
    b = make_bridge()
    b._latest_detections = NS(detections=[NS(corner_index=0, u=10., v=20., cls_name='x')])
    b._on_pick_target(NS(request_id=7, corner_index=0, layer=1, use_target_z=True,
                         target_z_m=.04, use_approach=True, approach_m=.05))
    assert not b.results
    args, kwargs = b.commands[0]
    assert abs(args[2] - .09) < 1e-9
    kwargs['on_result'](True, 'ACCEPTED', 31)
    assert not b.results and len(b.commands) == 1
    t = np.diag([1., -1., -1., 1.]); t[:3, 3] = args[:3]
    b._check_pick_approach(t, b.clock_ns)
    assert len(b.commands) == 2
    args, kwargs = b.commands[-1]
    assert abs(args[2] - .04) < 1e-9
    kwargs['on_result'](True, 'ACCEPTED', 32)
    assert b.results[-1].success and b.results[-1].command_seq == 32


def test_invalid_approach_and_nonfinite_projection_report_failures():
    b = make_bridge()
    b._latest_detections = NS(detections=[NS(corner_index=0, u=10., v=20., cls_name='x')])
    b._on_pick_target(NS(request_id=11, corner_index=0, layer=1, use_approach=True, approach_m=-.05))
    assert b.results[-1].reason == 'INVALID_PARAM'
    b._ray_plane_intersect = lambda *a: np.array([np.nan, .02, .031])
    b._on_pick_target(NS(request_id=12, corner_index=0, layer=1))
    assert b.results[-1].reason == 'WORKSPACE_REJECTED'
    assert not b.commands


def test_pose_gate_requires_axis_stability_freshness_and_clears_old_detection():
    b = make_bridge()
    params = dict(vision_stable_window_s=.2, vision_stable_displacement_m=.001,
                  vision_pose_max_age_s=.3, initial_pose_departure_tolerance_m=.05,
                  vision_axis_tolerance_deg=5.)
    b.get_parameter = lambda key: NS(value=params[key])
    b._vision_pose_ready = False
    b._current_vision_pose_name = ''
    b.ready = []
    b.vision_ready_pub = NS(publish=lambda msg: b.ready.append(msg.data))
    b._vision_pose_targets = lambda: [VisionPoseTarget('obs1', True, np.zeros(3), np.array([0., 0., -1.]))]
    t = np.diag([1., -1., -1., 1.])
    b._pose_history = deque([(b.clock_ns-200_000_000, t.copy()), (b.clock_ns-100_000_000, t.copy()), (b.clock_ns, t.copy())])
    b._refresh_vision_pose_ready(t)
    assert b.ready[-1]
    b._latest_detections = NS(detections=['old'])
    b._refresh_vision_pose_ready(np.eye(4))
    assert not b.ready[-1]
    assert b._latest_detections is None
    b._refresh_vision_pose_ready(t)
    assert b.ready[-1]
    b._vision_pose_targets = lambda: [VisionPoseTarget('obs2', True, np.zeros(3), np.array([0., 0., -1.]))]
    b._refresh_vision_pose_ready(t)
    assert b.ready[-2:] == [False, True]
    b.clock_ns += 400_000_000
    b._refresh_vision_pose_ready(t)
    assert not b.ready[-1]


def test_initial_arrival_does_not_open_gate_with_wrong_axis_or_jitter():
    b = make_bridge()
    params = dict(initial_position_tolerance_m=.005, initial_stable_samples=1,
                  vision_stable_window_s=.2, vision_stable_displacement_m=.001,
                  vision_pose_max_age_s=.3, vision_axis_tolerance_deg=5.)
    b.get_parameter = lambda key: NS(value=params[key])
    b._initial_move_pending = True
    b._initial_command_accepted = True
    b._initial_target_xyz = np.zeros(3)
    b._initial_stable_count = 0
    b._initial_pose_received_count = 0
    b._initial_last_diag_ns = b.clock_ns
    b._initial_pose_ready = False
    b._initial_pose_command = lambda: (0., 0., 0., -np.pi/2, 0., .5)
    b.initial_ready_pub = NS(publish=lambda msg: None)
    wrong = np.eye(4)
    b._pose_history = deque([(b.clock_ns-200_000_000, wrong.copy()), (b.clock_ns, wrong.copy())])
    b._update_initial_pose_state(np.zeros(3))
    assert not b._initial_pose_ready
    correct = np.diag([1., -1., -1., 1.])
    jitter = correct.copy(); jitter[0, 3] = .002
    b._pose_history = deque([(b.clock_ns-200_000_000, jitter), (b.clock_ns, correct)])
    b._update_initial_pose_state(np.zeros(3))
    assert not b._initial_pose_ready


def test_cancel_after_approach_acceptance_prevents_delayed_contact():
    b = make_bridge()
    b._latest_detections = NS(detections=[NS(corner_index=0, u=10., v=20., cls_name='x')])
    b._on_pick_target(NS(request_id=7, corner_index=0, layer=1, use_target_z=True,
                         target_z_m=.04, use_approach=True, approach_m=.05))
    args, kwargs = b.commands[0]
    kwargs['on_result'](True, 'ACCEPTED', 31)
    b._on_pick_target(NS(request_id=7, cancel=True))
    assert b.results[-1].reason == 'CANCELLED'
    t = np.diag([1., -1., -1., 1.]); t[:3, 3] = args[:3]
    b._check_pick_approach(t, b.clock_ns)
    assert len(b.commands) == 1


def test_pick_rejects_closed_or_stale_pose_gate_before_motion():
    b = make_bridge()
    b._latest_detections = NS(detections=[NS(corner_index=0, u=10., v=20., cls_name='x')])
    b._vision_pose_ready = False
    b._on_pick_target(NS(request_id=7, corner_index=0, layer=1))
    assert b.results and b.results[-1].reason == 'POSE_NOT_READY'
    assert not b.commands
    b._vision_pose_ready = True
    b.clock_ns += 500_000_000
    b._on_pick_target(NS(request_id=8, corner_index=0, layer=1))
    assert b.results[-1].reason == 'POSE_NOT_READY'
    assert not b.commands


def test_late_previous_observation_best_frame_cannot_enter_new_pose_cache():
    b = make_bridge()
    b._vision_pose_ready = True
    b._vision_ready_since_ns = b.clock_ns
    b._latest_detections = None
    old = NS(header=NS(stamp=NS(sec=0, nanosec=950_000_000)), detections=['old'], is_final_best=True)
    b._on_detections(old)
    assert b._latest_detections is None
