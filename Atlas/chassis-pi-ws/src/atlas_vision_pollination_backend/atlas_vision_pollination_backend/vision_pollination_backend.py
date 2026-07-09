#!/usr/bin/env python3
import math
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import JointState

from atlas_mission_interfaces.msg import ManipulationStatus
from atlas_mission_interfaces.srv import StartManipulation, CancelManipulation, DetectCameraTarget
from mcu_comm_bridge.srv import SetArmJoints, SetArmPosition


# 手眼和正运动学参数，来源于已验证的手眼脚本，安装后可独立运行
R_TOOL_CAMERA = np.array([
    [-0.007584453, -0.999738675, -0.021565203],
    [ 0.999649717, -0.008127087,  0.025187167],
    [-0.025355847, -0.021366619,  0.999450123],
], dtype=float)
T_TOOL_CAMERA_TRANSLATION = np.array([0.080627234, -0.034334049, 0.024924414], dtype=float)
T_TOOL_CAMERA = np.eye(4, dtype=float)
T_TOOL_CAMERA[:3, :3] = R_TOOL_CAMERA
T_TOOL_CAMERA[:3, 3] = T_TOOL_CAMERA_TRANSLATION

A = np.array([0.0, 0.0276200491203067, 0.2167241256700170, 0.2002827243995208, 0.0451594898594991], dtype=float)
D = np.array([0.0, -0.0162679040568649, -0.0192068569153542, 0.0014389528584892, 0.0], dtype=float)
ALPHA = np.array([0.0, np.pi * 0.5, np.pi, 0.0, np.pi * 0.5], dtype=float)
Q_OFFSET = np.array([-np.pi, -np.pi * 0.5, -3.3836013435535577, -2.8616351199480290, -np.pi], dtype=float)
Q_SIGN = np.array([-1.0, -1.0, 1.0, 1.0, -1.0], dtype=float)
BASE_T = np.eye(4, dtype=float)
BASE_T[:3, 3] = np.array([0.0, 0.0, 0.0605], dtype=float)
TOOL_T = np.array([
    [ 0.9999619259637, -0.0000000000000, -0.0087262032439,  0.0000000000000],
    [ 0.0000761495224, -0.9999619230642,  0.0087262032439,  0.0000000000000],
    [-0.0087258709769, -0.0087265354984, -0.9999238504776, -0.0184685931641],
    [ 0.0000000000000,  0.0000000000000,  0.0000000000000,  1.0000000000000],
], dtype=float)


def rotx(a: float) -> np.ndarray:
    ca, sa = np.cos(a), np.sin(a)
    return np.array([[1,0,0,0],[0,ca,-sa,0],[0,sa,ca,0],[0,0,0,1]], dtype=float)


def rotz(a: float) -> np.ndarray:
    ca, sa = np.cos(a), np.sin(a)
    return np.array([[ca,-sa,0,0],[sa,ca,0,0],[0,0,1,0],[0,0,0,1]], dtype=float)


def transx(x: float) -> np.ndarray:
    t = np.eye(4, dtype=float); t[0, 3] = x; return t


def transz(z: float) -> np.ndarray:
    t = np.eye(4, dtype=float); t[2, 3] = z; return t


def mdh_transform(a: float, d: float, alpha: float, theta: float) -> np.ndarray:
    return transx(a) @ rotx(alpha) @ rotz(theta) @ transz(d)


def fk_base_to_tool0(joints: np.ndarray, include_tool_t: bool = True) -> np.ndarray:
    theta = Q_SIGN * joints + Q_OFFSET
    t = BASE_T.copy()
    for i in range(5):
        t = t @ mdh_transform(A[i], D[i], ALPHA[i], theta[i])
    if include_tool_t:
        t = t @ TOOL_T
    return t


def transform_point(t: np.ndarray, p: np.ndarray) -> np.ndarray:
    ph = np.array([p[0], p[1], p[2], 1.0], dtype=float)
    return (t @ ph)[:3]


@dataclass
class PrepareAction:
    action_type: str = 'noop'
    joints_rad: Optional[List[float]] = None
    speed_rad_s: float = 1.0
    timeout_s: float = 8.0


@dataclass
class ArrivalTask:
    task_type: str = 'noop'
    empty_target_policy: str = 'skip'
    vision_service: str = '/vision/detect_camera_target'
    pre_tool: List[float] = field(default_factory=lambda: [0.05, -0.015, 0.097])
    pollination_tool: List[float] = field(default_factory=lambda: [0.05, -0.015, 0.087])
    speed_rad_s: float = 1.0
    timeout_s: float = 8.0
    dwell_pollination_s: float = 0.3
    sequence: List[dict] = field(default_factory=list)


class VisionPollinationBackend(Node):
    def __init__(self) -> None:
        super().__init__('atlas_vision_pollination_backend')
        self.backend_name = self.declare_parameter('backend_name', 'vision_pollination').value
        self.status_topic = self.declare_parameter('status_topic', '/atlas/manipulation/status').value
        self.start_service = self.declare_parameter('start_service', '/atlas/manipulation/start').value
        self.cancel_service = self.declare_parameter('cancel_service', '/atlas/manipulation/cancel').value
        self.joint_state_topic = self.declare_parameter('joint_state_topic', '/arm/joint_states').value
        self.vision_service_name = self.declare_parameter('vision_service', '/vision/detect_camera_target').value
        self.arm_joints_service = self.declare_parameter('arm_joints_service', '/mcu/set_arm_joints').value
        self.arm_position_service = self.declare_parameter('arm_position_service', '/mcu/set_arm_position').value
        self.config_yaml_path = self.declare_parameter('config_yaml_path', '').value
        self.update_rate_hz = float(self.declare_parameter('update_rate_hz', 20.0).value)
        self.joint_state_timeout_s = float(self.declare_parameter('joint_state_timeout_s', 0.5).value)
        self.joint_tolerance_rad = float(self.declare_parameter('joint_tolerance_rad', 0.05).value)
        self.position_tolerance_m = float(self.declare_parameter('position_tolerance_m', 0.012).value)
        self.default_speed_rad_s = float(self.declare_parameter('default_speed_rad_s', 1.0).value)
        self.vision_timeout_s = float(self.declare_parameter('vision_timeout_s', 3.0).value)
        self.command_timeout_s = float(self.declare_parameter('command_timeout_s', 2.0).value)
        self.motion_timeout_s = float(self.declare_parameter('motion_timeout_s', 8.0).value)
        self.fk_include_tool_t = bool(self.declare_parameter('fk_include_tool_T', True).value)

        self.prepare_actions: Dict[str, PrepareAction] = {'noop': PrepareAction()}
        self.arrival_tasks: Dict[str, ArrivalTask] = {'noop': ArrivalTask()}
        self.load_config(self.config_yaml_path)

        self.status_pub = self.create_publisher(ManipulationStatus, self.status_topic, 10)
        self.joint_sub = self.create_subscription(JointState, self.joint_state_topic, self.on_joint_state, 20)
        self.start_srv = self.create_service(StartManipulation, self.start_service, self.on_start)
        self.cancel_srv = self.create_service(CancelManipulation, self.cancel_service, self.on_cancel)
        self.vision_client = self.create_client(DetectCameraTarget, self.vision_service_name)
        self.joints_client = self.create_client(SetArmJoints, self.arm_joints_service)
        self.position_client = self.create_client(SetArmPosition, self.arm_position_service)
        self.timer = self.create_timer(1.0 / max(1.0, self.update_rate_hz), self.on_timer)

        self.latest_joints: Optional[np.ndarray] = None
        self.latest_joint_stamp = self.get_clock().now()
        self.state = ManipulationStatus.STATE_IDLE
        self.message = '空闲'
        self.error_code = 0
        self.waypoint_id = ''
        self.prepare_action_name = 'noop'
        self.arrival_task_name = 'noop'
        self.step_name = ''
        self.step_index = 0
        self.sequence: List[dict] = []
        self.current_task = ArrivalTask()
        self.current_prepare = PrepareAction()
        self.step_started_at = self.get_clock().now()
        self.command_future = None
        self.vision_future = None
        self.pending_target_joints: Optional[np.ndarray] = None
        self.pending_goal_position: Optional[np.ndarray] = None
        self.target_base: Optional[np.ndarray] = None
        self.rotation_at_detection: Optional[np.ndarray] = None
        self.dwell_until: Optional[rclpy.time.Time] = None
        self.get_logger().info('视觉授粉后端已启动')

    def load_config(self, path: str) -> None:
        if not path:
            return
        try:
            with open(path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
        except Exception as exc:
            self.get_logger().error('授粉配置读取失败 %s: %s', path, exc)
            return
        for name, node in (data.get('prepare_actions') or {}).items():
            action_type = node.get('type', 'noop')
            joints = None
            if 'joints_rad' in node:
                joints = [float(v) for v in node['joints_rad']]
            elif 'joints_deg' in node:
                joints = [math.radians(float(v)) for v in node['joints_deg']]
            self.prepare_actions[name] = PrepareAction(action_type, joints, float(node.get('speed_rad_s', self.default_speed_rad_s)), float(node.get('timeout_s', self.motion_timeout_s)))
        for name, node in (data.get('arrival_tasks') or {}).items():
            task = ArrivalTask()
            task.task_type = node.get('type', 'noop')
            task.empty_target_policy = node.get('empty_target_policy', 'skip')
            task.vision_service = node.get('vision_service', self.vision_service_name)
            task.pre_tool = [float(v) for v in node.get('pre_pollination_tool_point_m', task.pre_tool)]
            task.pollination_tool = [float(v) for v in node.get('pollination_tool_point_m', task.pollination_tool)]
            task.speed_rad_s = float(node.get('speed_rad_s', self.default_speed_rad_s))
            task.timeout_s = float(node.get('timeout_s', self.motion_timeout_s))
            task.dwell_pollination_s = float(node.get('dwell_pollination_s', 0.3))
            task.sequence = list(node.get('sequence', []))
            self.arrival_tasks[name] = task
        self.get_logger().info('授粉配置已读取，预备动作=%d，到位任务=%d', len(self.prepare_actions), len(self.arrival_tasks))

    def on_joint_state(self, msg: JointState) -> None:
        if len(msg.position) < 5:
            return
        joints = [0.0] * 5
        used_names = False
        if len(msg.name) == len(msg.position):
            try:
                for i in range(5):
                    name = f'joint_{i}'
                    idx = msg.name.index(name)
                    joints[i] = float(msg.position[idx])
                used_names = True
            except ValueError:
                used_names = False
        if not used_names:
            joints = [float(v) for v in msg.position[:5]]
        self.latest_joints = np.array(joints, dtype=float)
        self.latest_joint_stamp = rclpy.time.Time.from_msg(msg.header.stamp) if (msg.header.stamp.sec or msg.header.stamp.nanosec) else self.get_clock().now()

    def joints_fresh(self) -> bool:
        if self.latest_joints is None:
            return False
        age = (self.get_clock().now() - self.latest_joint_stamp).nanoseconds * 1e-9
        return 0.0 <= age <= self.joint_state_timeout_s

    def on_start(self, request: StartManipulation.Request, response: StartManipulation.Response):
        if request.backend and request.backend != self.backend_name:
            response.success = False
            response.message = f'后端不匹配，请求 {request.backend}，当前 {self.backend_name}'
            return response
        if self.state == ManipulationStatus.STATE_RUNNING:
            response.success = False
            response.message = '机械臂任务正在执行'
            return response
        if not self.joints_fresh():
            response.success = False
            response.message = '没有新鲜机械臂关节状态'
            self.waypoint_id = request.waypoint_id
            self.arrival_task_name = request.arrival_task or 'noop'
            self.state = ManipulationStatus.STATE_FAILED
            self.error_code = 3014
            self.message = response.message
            self.publish_status()
            return response
        self.waypoint_id = request.waypoint_id
        self.prepare_action_name = request.prepare_action or 'noop'
        self.arrival_task_name = request.arrival_task or 'noop'
        self.current_prepare = self.prepare_actions.get(self.prepare_action_name, PrepareAction())
        self.current_task = self.arrival_tasks.get(self.arrival_task_name, ArrivalTask())
        if self.current_task.task_type == 'noop':
            self.sequence = []
        else:
            self.sequence = self.current_task.sequence or [
                {'type': 'ensure_prepare_pose'},
                {'type': 'visual_position', 'name': 'pre_pollination', 'tool_point_ref': 'pre_pollination_tool_point_m'},
                {'type': 'visual_position', 'name': 'pollination', 'tool_point_ref': 'pollination_tool_point_m'},
                {'type': 'dwell', 'duration_s': self.current_task.dwell_pollination_s},
                {'type': 'visual_position', 'name': 'pre_pollination_return', 'tool_point_ref': 'pre_pollination_tool_point_m'},
                {'type': 'joints_action', 'action_ref': 'prepare_action'},
            ]
        self.state = ManipulationStatus.STATE_RUNNING
        self.message = '运行中'
        self.error_code = 0
        self.step_index = -1
        self.target_base = None
        self.rotation_at_detection = None
        self.command_future = None
        self.vision_future = None
        self.pending_target_joints = None
        self.pending_goal_position = None
        self.dwell_until = None
        response.success = True
        response.message = '机械臂任务已接受'
        self.advance_step('start')
        return response

    def on_cancel(self, request: CancelManipulation.Request, response: CancelManipulation.Response):
        self.state = ManipulationStatus.STATE_CANCELLED
        self.message = request.reason or '已取消'
        self.error_code = 0
        self.command_future = None
        self.vision_future = None
        response.success = True
        response.message = self.message
        self.publish_status()
        return response

    def fail(self, code: int, message: str) -> None:
        self.state = ManipulationStatus.STATE_FAILED
        self.error_code = code
        self.message = message
        self.get_logger().error(message)
        self.publish_status()

    def succeed(self, message: str) -> None:
        self.state = ManipulationStatus.STATE_SUCCEEDED
        self.error_code = 0
        self.message = message
        self.publish_status()
        self.get_logger().info(message)

    def advance_step(self, reason: str) -> None:
        self.step_index += 1
        self.step_started_at = self.get_clock().now()
        self.command_future = None
        self.vision_future = None
        self.pending_target_joints = None
        self.pending_goal_position = None
        self.dwell_until = None
        if not self.sequence:
            self.succeed('空机械臂任务完成')
            return
        if self.step_index >= len(self.sequence):
            self.succeed('授粉序列完成')
            return
        step = self.sequence[self.step_index]
        self.step_name = step.get('name', step.get('type', f'step_{self.step_index}'))
        self.message = f'{self.step_name}: {reason}'
        self.get_logger().info('step %d/%d: %s', self.step_index + 1, len(self.sequence), self.step_name)
        self.start_current_step()

    def start_current_step(self) -> None:
        step = self.sequence[self.step_index]
        kind = step.get('type', 'noop')
        if kind == 'ensure_prepare_pose':
            self.start_joint_action(self.current_prepare)
        elif kind == 'joints_action':
            ref = step.get('action_ref', 'prepare_action')
            action_name = self.prepare_action_name if ref == 'prepare_action' else ref
            self.start_joint_action(self.prepare_actions.get(action_name, PrepareAction()))
        elif kind == 'visual_position':
            if self.target_base is None:
                self.start_vision_request()
            else:
                self.start_visual_position(step)
        elif kind == 'dwell':
            duration = float(step.get('duration_s', self.current_task.dwell_pollination_s))
            self.dwell_until = self.get_clock().now() + Duration(seconds=max(0.0, duration))
            self.message = f'dwell {duration:.2f}s'
        else:
            self.advance_step(f'ignored unsupported step type {kind}')

    def start_joint_action(self, action: PrepareAction) -> None:
        if action.action_type == 'noop' or not action.joints_rad:
            self.advance_step('空关节动作')
            return
        if not self.joints_client.service_is_ready():
            self.fail(3001, '机械臂关节服务未就绪')
            return
        req = SetArmJoints.Request()
        req.joints_rad = [float(v) for v in action.joints_rad]
        req.speed_rad_s = float(action.speed_rad_s)
        self.pending_target_joints = np.array(req.joints_rad, dtype=float)
        self.command_future = self.joints_client.call_async(req)
        self.step_started_at = self.get_clock().now()
        self.message = f'sending joints action {self.prepare_action_name}'

    def start_vision_request(self) -> None:
        if not self.vision_client.service_is_ready():
            self.fail(3002, '视觉服务未就绪')
            return
        req = DetectCameraTarget.Request()
        req.waypoint_id = self.waypoint_id
        req.task_id = self.arrival_task_name
        self.vision_future = self.vision_client.call_async(req)
        self.step_started_at = self.get_clock().now()
        self.message = '请求相机目标'

    def handle_vision_response(self) -> None:
        if self.vision_future is None or not self.vision_future.done():
            if (self.get_clock().now() - self.step_started_at).nanoseconds * 1e-9 > self.vision_timeout_s:
                self.fail(3003, '视觉服务超时')
            return
        try:
            resp = self.vision_future.result()
        except Exception as exc:
            self.fail(3004, f'视觉服务异常: {exc}')
            return
        self.vision_future = None
        if not resp.success:
            if resp.message == 'NO_TARGET' and self.current_task.empty_target_policy == 'skip':
                self.succeed('视觉返回无目标，按策略跳过该点')
            else:
                self.fail(3005, f'视觉失败: {resp.message}')
            return
        camera_point = np.array([resp.target_camera_m.x, resp.target_camera_m.y, resp.target_camera_m.z], dtype=float)
        joints = self.latest_joints.copy()
        t_base_tool0 = fk_base_to_tool0(joints, self.fk_include_tool_t)
        target_tool0 = transform_point(T_TOOL_CAMERA, camera_point)
        target_base = transform_point(t_base_tool0, target_tool0)
        self.target_base = target_base
        self.rotation_at_detection = t_base_tool0[:3, :3].copy()
        self.get_logger().info('vision target camera=[%.4f %.4f %.4f] base=[%.4f %.4f %.4f]', camera_point[0], camera_point[1], camera_point[2], target_base[0], target_base[1], target_base[2])
        self.start_visual_position(self.sequence[self.step_index])

    def start_visual_position(self, step: dict) -> None:
        if self.target_base is None or self.rotation_at_detection is None:
            self.start_vision_request()
            return
        ref = step.get('tool_point_ref', 'pre_pollination_tool_point_m')
        tool = np.array(self.current_task.pre_tool if ref == 'pre_pollination_tool_point_m' else self.current_task.pollination_tool, dtype=float)
        goal = self.target_base - self.rotation_at_detection @ tool
        if not self.position_client.service_is_ready():
            self.fail(3006, '机械臂位置服务未就绪')
            return
        req = SetArmPosition.Request()
        req.x_m = float(goal[0]); req.y_m = float(goal[1]); req.z_m = float(goal[2])
        req.speed_rad_s = float(self.current_task.speed_rad_s)
        self.pending_goal_position = goal
        self.command_future = self.position_client.call_async(req)
        self.step_started_at = self.get_clock().now()
        self.message = f'发送视觉位置 {self.step_name}'
        self.get_logger().info('visual position %s goal=[%.4f %.4f %.4f] tool=[%.4f %.4f %.4f]', self.step_name, goal[0], goal[1], goal[2], tool[0], tool[1], tool[2])

    def on_timer(self) -> None:
        if self.state == ManipulationStatus.STATE_RUNNING:
            self.update_running()
        self.publish_status()

    def update_running(self) -> None:
        if self.vision_future is not None:
            self.handle_vision_response()
            return
        if self.command_future is not None:
            if (self.get_clock().now() - self.step_started_at).nanoseconds * 1e-9 > self.command_timeout_s and not self.command_future.done():
                self.fail(3007, '机械臂命令服务超时')
                return
            if not self.command_future.done():
                return
            try:
                resp = self.command_future.result()
            except Exception as exc:
                self.fail(3008, f'机械臂命令异常: {exc}')
                return
            self.command_future = None
            if not resp.success:
                self.fail(3009, f'机械臂命令被拒绝: {resp.message}')
                return
            self.step_started_at = self.get_clock().now()
            self.message = '等待机械臂到位'
            return
        if self.dwell_until is not None:
            if self.get_clock().now() >= self.dwell_until:
                self.advance_step('dwell completed')
            return
        if self.pending_target_joints is not None:
            if not self.joints_fresh():
                self.fail(3010, '等待关节目标时关节状态超时')
                return
            err = float(np.max(np.abs(self.latest_joints - self.pending_target_joints)))
            if err <= self.joint_tolerance_rad:
                self.advance_step(f'joints reached err={err:.4f}')
                return
            if (self.get_clock().now() - self.step_started_at).nanoseconds * 1e-9 > self.motion_timeout_s:
                self.fail(3011, f'joints motion timeout err={err:.4f}')
            return
        if self.pending_goal_position is not None:
            if not self.joints_fresh():
                self.fail(3012, 'joint state timeout while waiting position target')
                return
            current = transform_point(fk_base_to_tool0(self.latest_joints, self.fk_include_tool_t), np.zeros(3))
            err = float(np.linalg.norm(current - self.pending_goal_position))
            if err <= self.position_tolerance_m:
                self.advance_step(f'position reached err={err:.4f}')
                return
            if (self.get_clock().now() - self.step_started_at).nanoseconds * 1e-9 > self.motion_timeout_s:
                self.fail(3013, f'position motion timeout err={err:.4f}')
            return
        # 如果当前步骤没有产生等待中的工作，则进入下一步
        self.advance_step('step completed')

    def publish_status(self) -> None:
        msg = ManipulationStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = self.state
        msg.backend = self.backend_name
        msg.waypoint_id = self.waypoint_id
        msg.task_id = self.arrival_task_name
        msg.step_name = self.step_name
        msg.error_code = int(self.error_code)
        msg.message = self.message
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VisionPollinationBackend()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
