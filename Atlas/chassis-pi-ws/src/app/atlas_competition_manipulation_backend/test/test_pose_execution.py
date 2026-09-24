import math
import threading
import time
from types import SimpleNamespace as NS
import pytest
from atlas_competition_manipulation_backend import backend as b


def node():
    n = object.__new__(b.CompetitionManipulationBackend)
    n._result_cv = threading.Condition()
    n._arm_results = {}
    n._pick_results = {}
    n._cancelled = lambda: False
    n.arm_result_timeout_s = 0.02
    n._last_failure = ''
    n.get_logger = lambda: NS(error=lambda msg: None, info=lambda msg: None, warn=lambda msg: None)
    return n


def test_tool_axis_direction_releases_spin():
    for angle in [0, 0.7, -1.7]:
        # Downward tool with arbitrary self spin
        q = NS(x=math.cos(angle / 2), y=math.sin(angle / 2), z=0., w=0.)
        assert b.tool_direction(q)[0] == pytest.approx(-math.pi / 2)
    assert b.tool_direction(NS(x=0., y=math.sin(math.pi/4), z=0., w=math.cos(math.pi/4))) == pytest.approx((0.,0.))


def test_invalid_quaternion_rejected():
    with pytest.raises(ValueError):
        b.tool_direction(NS(x=0.,y=0.,z=0.,w=0.))


def test_result_before_service_reply_is_not_lost():
    n=node(); since=time.monotonic()
    n._on_arm_result(NS(command_seq=9,result=0,arm_status=0))
    assert n._wait_arm_accepted(9,since)


def test_queue_success_is_not_mcu_success():
    n=node(); since=time.monotonic()
    n._on_arm_result(NS(command_seq=9,result=1,arm_status=4))
    assert not n._wait_arm_accepted(9,since)
    assert 'NO_SOLUTION' in n._last_failure


def test_old_or_other_sequence_does_not_satisfy_new_command():
    n=node()
    n._on_arm_result(NS(command_seq=9,result=0,arm_status=0))
    since=time.monotonic()
    n._on_arm_result(NS(command_seq=8,result=0,arm_status=0))
    assert not n._wait_arm_accepted(9,since)
    assert 'TIMEOUT' in n._last_failure


def test_workspace_reject_returns_without_motion_watchdog():
    n=node(); since=time.monotonic()
    n._on_pick_result(NS(request_id=7,success=False,reason='WORKSPACE_REJECTED'))
    assert n._wait_pick_result(7,since,0.5) is None
    assert n._last_failure == 'WORKSPACE_REJECTED'
    assert time.monotonic()-since < 0.1


def test_pose_move_sends_direction_and_waits_accept_then_arrival(monkeypatch):
    n=node(); calls=[]
    monkeypatch.setattr(b,'SetArmPose',NS(Request=lambda:NS()))
    n.arm_pose_client=object(); n.default_speed_rad_s=.5; n.motion_timeout_s=1.
    n._call_service=lambda client,req: calls.append(('send',req)) or NS(success=True,command_seq=4)
    n._wait_arm_accepted=lambda seq,since: calls.append(('accept',seq)) or True
    n._wait_pose_target=lambda *args,**kwargs: calls.append(('arrive',args,kwargs)) or True
    direction=(-1.5,.7)
    assert n._move_pose(b.XYZ(.2,.1,.3),direction=direction,suction_valid=True,suction_enable=True)
    assert [c[0] for c in calls]==['send','accept','arrive']
    assert (calls[0][1].pitch_rad,calls[0][1].yaw_rad)==direction


def test_position_move_uses_3d_service_and_does_not_check_axis(monkeypatch):
    n=node(); calls=[]
    monkeypatch.setattr(b,'SetArmPosition',NS(Request=lambda:NS()))
    n.arm_position_client=object(); n.default_speed_rad_s=.5; n.motion_timeout_s=1.
    n._call_service=lambda client,req: calls.append(('send',req)) or NS(success=True,command_seq=5)
    n._wait_arm_accepted=lambda seq,since: calls.append(('accept',seq)) or True
    n._wait_pose_target=lambda *args,**kwargs: calls.append(('arrive',args,kwargs)) or True
    assert n._move_position(b.XYZ(.2,.1,.3),suction_valid=True,suction_enable=True)
    assert [c[0] for c in calls]==['send','accept','arrive']
    assert calls[-1][2]['direction'] is None


def test_stale_pose_cannot_be_counted_as_multiple_stable_samples():
    n=node(); n._pose_cv=threading.Condition(); n._latest_pose=b.XYZ(.2,.1,.3)
    n._latest_direction=(-math.pi/2,0.)
    n._latest_pose_time=time.monotonic()-2
    n.pose_feedback_timeout_s=.5; n.position_tolerance_m=.015
    n.tool_axis_tolerance_rad=math.radians(3); n.stable_samples=2
    assert not n._wait_pose_target(n._latest_pose,.03,direction=n._latest_direction,since=time.monotonic())


def test_place_legacy_xyz_uses_3d_descent_and_calibrated_joint_retreat(monkeypatch):
    n=node(); positions=[]; poses=[]; named=[]
    monkeypatch.setattr(b,'ManipulationStatus',NS(STATE_RUNNING=1))
    monkeypatch.setattr(b,'calibrated_placement_direction',lambda *args:None)
    n._place_target=lambda *args:b.XYZ(.3,.1,.04)
    n.arm_motion={}
    n.place_approach_m=.06; n.suction_settle_s=0
    n._set_status=lambda *args,**kwargs:None
    n._set_suction=lambda enabled: not enabled
    n._move_position=lambda target,**kwargs:positions.append((target,kwargs)) or True
    n._move_pose=lambda target,**kwargs:poses.append((target,kwargs)) or True
    n._move_named_pose=lambda *args,**kwargs:named.append((args,kwargs)) or True
    assert n._do_place('A','park_1',0,0)
    assert len(positions)==1 and not poses
    assert positions[0][0].z==pytest.approx(.04)
    assert positions[0][1]['suction_enable'] is True
    assert named == [(('park_prepare',), {
        'arena':'A', 'area':'park_1',
        'suction_valid':True, 'suction_enable':False})]


def test_place_new_calibration_uses_exact_5d_release_pose(monkeypatch):
    n=node(); positions=[]; poses=[]
    monkeypatch.setattr(b,'ManipulationStatus',NS(STATE_RUNNING=1))
    monkeypatch.setattr(b,'calibrated_placement_direction',lambda *args:(-.2,.7))
    n._place_target=lambda *args:b.XYZ(.3,.1,.04)
    n.arm_motion={}; n.suction_settle_s=0
    n._set_status=lambda *args,**kwargs:None
    n._set_suction=lambda enabled:not enabled
    n._move_position=lambda target,**kwargs:positions.append((target,kwargs)) or True
    n._move_pose=lambda target,**kwargs:poses.append((target,kwargs)) or True
    n._move_named_pose=lambda *args,**kwargs:True
    assert n._do_place('A','park_1',0,0)
    assert not positions and len(poses)==1
    assert poses[0][1]['direction']==(-.2,.7)


def test_place_failure_forces_suction_off_even_when_cancelled(monkeypatch):
    n=node(); suction=[]
    monkeypatch.setattr(b,'ManipulationStatus',NS(STATE_RUNNING=1))
    monkeypatch.setattr(b,'calibrated_placement_direction',lambda *args:None)
    n._place_target=lambda *args:b.XYZ(.3,.1,.04)
    n.arm_motion={}; n.place_approach_m=.06; n.suction_settle_s=0
    n._set_status=lambda *args,**kwargs:None
    n._move_position=lambda *args,**kwargs:False
    n._set_suction=lambda enabled,force=False:suction.append((enabled,force)) or True
    n._last_failure='POSE_ARRIVAL_TIMEOUT'

    assert not n._do_place('A','park_1',0,0)
    assert suction == [(False, True)]
    assert n._last_failure == 'POSE_ARRIVAL_TIMEOUT'


def test_view_scan_keeps_measured_axis(monkeypatch):
    n=node(); moves=[]
    monkeypatch.setattr(b,'ManipulationStatus',NS(STATE_RUNNING=1))
    n.view_scan_enabled=True; n.settle_before_observe_s=0
    n._current_pose=lambda:b.XYZ(.2,.1,.3)
    n._current_direction=lambda:(-1.48,.4)
    n._view_scan_offset=lambda attempt:(0.,.03,0.)
    n._set_status=lambda *args,**kwargs:None
    n._move_pose=lambda target,**kwargs:moves.append((target,kwargs)) or True
    assert n._do_view_scan('A','pickup',0,1)
    assert moves[0][1]['direction']==(-1.48,.4)
    assert moves[0][0].y==pytest.approx(.13)


def test_axis_wrong_at_same_xyz_does_not_arrive():
    n=node(); n._pose_cv=threading.Condition(); n._latest_pose=b.XYZ(.2,.1,.3)
    n._latest_direction=(math.pi/2,0.)
    n._latest_pose_time=time.monotonic()
    n.pose_feedback_timeout_s=.5; n.position_tolerance_m=.015
    n.tool_axis_tolerance_rad=math.radians(3); n.stable_samples=1
    assert not n._wait_pose_target(n._latest_pose,.03,direction=(-math.pi/2,0.),since=0.)


def test_cancel_propagates_correlated_pick_request(monkeypatch):
    n=node(); sent=[]
    monkeypatch.setattr(b,'PickTarget',lambda:NS())
    n._active_pick_request_id=42
    n.pick_target_pub=NS(publish=lambda msg:sent.append(msg))
    n._cancel_pick_request()
    assert sent[0].request_id==42 and sent[0].cancel is True


def test_restart_rejected_while_cancelled_worker_is_still_alive(monkeypatch):
    n=node()
    monkeypatch.setattr(b,'ManipulationStatus',NS(STATE_RUNNING=1))
    n.backend_name='vision_arm'; n._state_lock=threading.Lock()
    n._status_state=3; n._worker=NS(is_alive=lambda:True)
    result=n._on_start(NS(backend='vision_arm'),NS())
    assert result.success is False


def test_pose_callback_rejects_stale_and_duplicate_source_stamps():
    n=node(); n._pose_cv=threading.Condition()
    n._latest_pose=None; n._latest_pose_time=0.; n._latest_pose_stamp_ns=0
    n.pose_feedback_timeout_s=.5
    n.get_clock=lambda:NS(now=lambda:NS(nanoseconds=10_000_000_000))
    msg=NS(header=NS(stamp=NS(sec=8,nanosec=0)),
           pose=NS(position=NS(x=.2,y=.1,z=.3),orientation=NS(x=1.,y=0.,z=0.,w=0.)))
    n._on_arm_pose(msg)
    assert n._latest_pose is None
    msg.header.stamp.sec=10
    n._on_arm_pose(msg); received=n._latest_pose_time
    assert n._latest_pose is not None
    n._on_arm_pose(msg)
    assert n._latest_pose_time==received


def _joint_node(joints):
    n=node(); n._joint_cv=threading.Condition()
    n._latest_joints=joints
    n._latest_joint_time=time.monotonic()
    n.pose_feedback_timeout_s=.5
    n.joint_tolerance_rad=.06
    n.stable_samples=1
    return n


def test_joint_target_2pi_wrap_is_detected_as_arrival():
    # q2 目标≈2π(6.28)，舵机反馈常回绕到 0 附近；直接相减会永远判不到位。
    target=[3.1217, 1.5723, 6.2801, 3.1631, 3.2183]
    wrapped=[3.1217, 1.5723, 0.0, 3.1631, 3.2183]
    n=_joint_node(wrapped)
    since=time.monotonic()-0.01
    assert n._wait_joint_target(target, 0.5, since)


def test_joint_target_real_error_still_times_out():
    # 2π 归一化不能把真实超差也一起放过。
    target=[3.1217, 1.5723, 6.2801, 3.1631, 3.2183]
    off=[3.1217, 1.5723, 6.0, 3.1631, 3.2183]  # q2 差 0.28rad，远超 0.06rad 容差
    n=_joint_node(off)
    since=time.monotonic()-0.01
    assert not n._wait_joint_target(target, 0.05, since)
    assert 'JOINT_ARRIVAL_TIMEOUT' in n._last_failure


def test_race_stall_watchdog_auto_advances_pose_without_feedback():
    n=node()
    n._pose_cv=threading.Condition()
    n._latest_pose=None
    n._latest_direction=None
    n._latest_pose_time=0.0
    n.pose_feedback_timeout_s=.5
    n.position_tolerance_m=.015
    n.tool_axis_tolerance_rad=math.radians(3)
    n.stable_samples=2
    n.stall_auto_advance_s=.02
    n.stall_position_motion_m=.0005
    n.stall_axis_motion_rad=math.radians(.2)
    start=time.monotonic()
    assert n._wait_pose_target(b.XYZ(.2,.1,.3),.2,direction=None,since=start)
    assert time.monotonic()-start < .12


def test_race_stall_watchdog_auto_advances_joint_without_feedback():
    n=node()
    n._joint_cv=threading.Condition()
    n._latest_joints=None
    n._latest_joint_time=0.0
    n.pose_feedback_timeout_s=.5
    n.joint_tolerance_rad=.1
    n.stable_samples=2
    n.stall_auto_advance_s=.02
    n.stall_joint_motion_rad=.002
    start=time.monotonic()
    assert n._wait_joint_target([0.,0.,0.,0.,0.],.2,start)
    assert time.monotonic()-start < .12
