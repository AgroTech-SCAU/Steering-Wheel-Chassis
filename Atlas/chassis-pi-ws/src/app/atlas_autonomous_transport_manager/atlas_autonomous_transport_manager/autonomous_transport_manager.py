#!/usr/bin/env python3
"""Atlas 全自主运输区任务状态机"""

from __future__ import annotations

import math
import threading
from dataclasses import dataclass, field
from enum import IntEnum
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import rclpy
import yaml
from geometry_msgs.msg import Twist
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import SetBool

from atlas_mission_interfaces.msg import (
    AsrproEvent,
    AsrproStatus,
    AutonomousTransportStatus,
    ManipulationStatus,
    NavigationStatus,
)
from atlas_mission_interfaces.srv import (
    AsrproSpeak,
    CancelManipulation,
    CancelNavigation,
    ClassifySortingRule,
    StartManipulation,
    StartNavigation,
)
from mcu_comm_bridge.msg import AutoTaskEvent, McuStatus
from mcu_comm_bridge.srv import ReportMissionResult


class LifecycleState(IntEnum):
    """整套自动任务的生命周期状态"""

    BOOTSTRAP = AutonomousTransportStatus.STATE_BOOTSTRAP
    WAIT_MCU_STATUS = AutonomousTransportStatus.STATE_WAIT_MCU_STATUS
    WAIT_START = AutonomousTransportStatus.STATE_WAIT_START
    PRECHECK = AutonomousTransportStatus.STATE_PRECHECK
    RUNNING = AutonomousTransportStatus.STATE_RUNNING
    REPORTING_DONE = AutonomousTransportStatus.STATE_REPORTING_DONE
    REPORTING_FAIL = AutonomousTransportStatus.STATE_REPORTING_FAIL
    WAIT_RESET = AutonomousTransportStatus.STATE_WAIT_RESET
    RECOVERY_REQUIRED = AutonomousTransportStatus.STATE_RECOVERY_REQUIRED
    SHUTTING_DOWN = AutonomousTransportStatus.STATE_SHUTTING_DOWN


class TransportStage(IntEnum):
    """全自主运输区内部阶段"""

    IDLE = AutonomousTransportStatus.STAGE_IDLE
    ANNOUNCE_TRANSITION = AutonomousTransportStatus.STAGE_ANNOUNCE_TRANSITION
    ANNOUNCE_VOICE_PROMPT = AutonomousTransportStatus.STAGE_ANNOUNCE_VOICE_PROMPT
    WAIT_VOICE_START = AutonomousTransportStatus.STAGE_WAIT_VOICE_START
    ANNOUNCE_AUTONOMOUS_START = AutonomousTransportStatus.STAGE_ANNOUNCE_AUTONOMOUS_START
    NAVIGATE_SORTING_AREA = AutonomousTransportStatus.STAGE_NAVIGATE_SORTING_AREA
    CLASSIFY_SORTING_RULE = AutonomousTransportStatus.STAGE_CLASSIFY_SORTING_RULE
    NAVIGATE_DISPATCH_AREA = AutonomousTransportStatus.STAGE_NAVIGATE_DISPATCH_AREA
    SELECT_CARGO = AutonomousTransportStatus.STAGE_SELECT_CARGO
    PICK_CARGO = AutonomousTransportStatus.STAGE_PICK_CARGO
    NAVIGATE_TARGET_PARK = AutonomousTransportStatus.STAGE_NAVIGATE_TARGET_PARK
    PLACE_CARGO = AutonomousTransportStatus.STAGE_PLACE_CARGO
    COMPLETE = AutonomousTransportStatus.STAGE_COMPLETE
    FAILED = AutonomousTransportStatus.STAGE_FAILED


@dataclass(frozen=True)
class Waypoint:
    """导航后端所需的单个地图点"""

    waypoint_id: str
    x_m: float
    y_m: float
    yaw_rad: float
    timeout_s: float


@dataclass
class Operation:
    """导航、机械臂或识别服务的一次异步调用上下文"""

    kind: str = ''
    request_id: str = ''
    purpose: str = ''
    future: object = None
    accepted: bool = False
    started_at_ns: int = 0
    timeout_s: float = 0.0
    retry_count: int = 0

    def clear(self) -> None:
        self.kind = ''
        self.request_id = ''
        self.purpose = ''
        self.future = None
        self.accepted = False
        self.started_at_ns = 0
        self.timeout_s = 0.0
        self.retry_count = 0


@dataclass
class TransportPlan:
    """从 YAML 读取的全自主运输任务参数"""

    calibration_confirmed: bool = False
    max_autonomous_duration_s: float = 300.0
    stop_reserve_s: float = 15.0
    manipulation_timeout_s: float = 60.0
    classification_timeout_s: float = 20.0
    classification_min_confidence: float = 0.60
    target_total: int = 8
    expected_counts: Dict[str, int] = field(
        default_factory=lambda: {'gear': 4, 't_bolt': 4}
    )
    cargo_plan: List[str] = field(
        default_factory=lambda: ['gear', 't_bolt'] * 4
    )
    no_target_limit_per_class: int = 2
    navigation_retry_count: int = 1
    classification_retry_count: int = 2
    pick_retry_count: int = 1
    continue_on_pick_failure: bool = True
    voice_start_required: bool = True
    voice_start_timeout_s: float = 0.0
    voice_start_fallback_enabled: bool = False
    asrpro_status_timeout_s: float = 2.0
    speech_timeout_s: float = 8.0
    speech_retry_count: int = 2
    transition_phrase_id: str = 'transition_complete'
    prompt_phrase_id: str = 'voice_prompt'
    autonomous_start_phrase_id: str = 'autonomous_start'
    delivery_complete_phrase_id: str = 'delivery_complete'
    task_complete_phrase_id: str = 'task_complete'
    task_skipped_phrase_id: str = 'task_skipped'
    accepted_voice_phrases: List[str] = field(
        default_factory=lambda: ['atlas_start']
    )
    fallback_park_1_cargo: str = 'gear'
    fallback_park_2_cargo: str = 't_bolt'
    use_fallback_mapping_on_failure: bool = False
    sorting_waypoint: Optional[Waypoint] = None
    dispatch_waypoint: Optional[Waypoint] = None
    park_waypoints: Dict[str, Waypoint] = field(default_factory=dict)
    safe_prepare_action: str = 'transport_arm_safe'
    sorting_prepare_action: str = 'sorting_scan_prepare'
    pick_prepare_actions: Dict[str, str] = field(
        default_factory=lambda: {
            'gear': 'dispatch_pick_prepare',
            't_bolt': 'dispatch_pick_prepare',
        }
    )
    pick_tasks: Dict[str, str] = field(
        default_factory=lambda: {
            'gear': 'pick_one_gear',
            't_bolt': 'pick_one_t_bolt',
        }
    )
    place_prepare_actions: Dict[str, str] = field(
        default_factory=lambda: {
            'park_1': 'transport_arm_safe',
            'park_2': 'transport_arm_safe',
        }
    )
    place_tasks: Dict[str, List[str]] = field(default_factory=dict)


class AutonomousTransportManager(Node):
    """执行中转区、智能分拣、待派送取货和园区投放闭环"""

    READY_CHASSIS = 1 << 0
    READY_ARM = 1 << 1
    READY_ODOM = 1 << 2
    ONLINE_PI = 1 << 2
    ONLINE_HAS_FAULT = 1 << 3
    ONLINE_ESTOP = 1 << 4

    def __init__(self) -> None:
        super().__init__('atlas_autonomous_transport_manager')
        self._lock = threading.RLock()
        self.callback_group = ReentrantCallbackGroup()

        # 接口名称全部参数化，现场集成时不需要修改状态机源码
        self.mcu_status_topic = str(
            self.declare_parameter('mcu_status_topic', '/mcu/status').value
        )
        self.auto_task_event_topic = str(
            self.declare_parameter('auto_task_event_topic', '/mcu/auto_task_event').value
        )
        self.navigation_status_topic = str(
            self.declare_parameter('navigation_status_topic', '/atlas/navigation/status').value
        )
        self.manipulation_status_topic = str(
            self.declare_parameter('manipulation_status_topic', '/atlas/manipulation/status').value
        )
        self.navigation_cmd_vel_topic = str(
            self.declare_parameter('navigation_cmd_vel_topic', '/atlas/navigation/cmd_vel').value
        )
        self.cmd_vel_topic = str(
            self.declare_parameter('cmd_vel_topic', '/motor_cmd_vel').value
        )
        self.status_topic = str(
            self.declare_parameter(
                'status_topic', '/atlas/autonomous_transport/status'
            ).value
        )
        self.asrpro_status_topic = str(
            self.declare_parameter('asrpro_status_topic', '/atlas/asrpro/status').value
        )
        self.asrpro_event_topic = str(
            self.declare_parameter('asrpro_event_topic', '/atlas/asrpro/event').value
        )
        self.voice_command_topic = str(
            self.declare_parameter(
                'voice_command_topic', '/atlas/asrpro/recognized'
            ).value
        )
        self.asrpro_speak_service = str(
            self.declare_parameter('asrpro_speak_service', '/atlas/asrpro/speak').value
        )

        self.navigation_backend = str(
            self.declare_parameter('navigation_backend', 'full').value
        )
        self.manipulation_backend = str(
            self.declare_parameter('manipulation_backend', 'racom_vision').value
        )
        self.navigation_start_service = str(
            self.declare_parameter('navigation_start_service', '/atlas/navigation/start').value
        )
        self.navigation_cancel_service = str(
            self.declare_parameter('navigation_cancel_service', '/atlas/navigation/cancel').value
        )
        self.manipulation_start_service = str(
            self.declare_parameter('manipulation_start_service', '/atlas/manipulation/start').value
        )
        self.manipulation_cancel_service = str(
            self.declare_parameter('manipulation_cancel_service', '/atlas/manipulation/cancel').value
        )
        self.sorting_rule_service = str(
            self.declare_parameter(
                'sorting_rule_service', '/vision/classify_sorting_rule'
            ).value
        )
        self.mission_result_service = str(
            self.declare_parameter(
                'mission_result_service', '/mcu/report_mission_result'
            ).value
        )
        self.brake_service = str(
            self.declare_parameter('brake_service', '/mcu/set_brake').value
        )

        self.update_rate_hz = max(
            1.0, float(self.declare_parameter('update_rate_hz', 20.0).value)
        )
        self.status_publish_rate_hz = max(
            1.0,
            float(self.declare_parameter('status_publish_rate_hz', 5.0).value),
        )
        self.zero_velocity_publish_rate_hz = max(
            1.0,
            float(
                self.declare_parameter('zero_velocity_publish_rate_hz', 10.0).value
            ),
        )
        self.mcu_status_timeout_s = max(
            0.05,
            float(self.declare_parameter('mcu_status_timeout_s', 0.5).value),
        )
        self.service_response_timeout_s = max(
            0.1,
            float(
                self.declare_parameter('service_response_timeout_s', 2.0).value
            ),
        )
        self.result_confirm_timeout_s = max(
            0.5,
            float(
                self.declare_parameter('result_confirm_timeout_s', 3.0).value
            ),
        )
        self.max_linear_speed_mps = max(
            0.0,
            min(
                2.0,
                float(self.declare_parameter('max_linear_speed_mps', 1.0).value),
            ),
        )
        self.max_angular_speed_rps = max(
            0.0,
            float(self.declare_parameter('max_angular_speed_rps', 1.5).value),
        )
        self.require_arm_ready = bool(
            self.declare_parameter('require_arm_ready', True).value
        )
        self.accept_latched_start_without_event = bool(
            self.declare_parameter('accept_latched_start_without_event', True).value
        )

        config_path = str(self.declare_parameter('config_yaml_path', '').value)
        self.plan = self._load_plan(config_path)
        self.config_errors = self._validate_plan(self.plan)

        # 订阅 MCU 生命周期、后端状态和语音识别文本
        # MCU 状态发布端使用 reliable + transient_local
        # 订阅端保持相同耐久性，节点启动后能够立即获得最近一次安全状态
        mcu_status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            McuStatus,
            self.mcu_status_topic,
            self._on_mcu_status,
            mcu_status_qos,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            AutoTaskEvent,
            self.auto_task_event_topic,
            self._on_auto_task_event,
            10,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            NavigationStatus,
            self.navigation_status_topic,
            self._on_navigation_status,
            10,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            ManipulationStatus,
            self.manipulation_status_topic,
            self._on_manipulation_status,
            10,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            Twist,
            self.navigation_cmd_vel_topic,
            self._on_navigation_cmd_vel,
            10,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            AsrproStatus,
            self.asrpro_status_topic,
            self._on_asrpro_status,
            10,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            AsrproEvent,
            self.asrpro_event_topic,
            self._on_asrpro_event,
            20,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            String,
            self.voice_command_topic,
            self._on_voice_command,
            10,
            callback_group=self.callback_group,
        )

        self.status_publisher = self.create_publisher(
            AutonomousTransportStatus, self.status_topic, 10
        )
        self.cmd_vel_publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        # 状态机只调用后端服务；导航、识别、机械臂控制分别由对应功能包实现
        self.navigation_start_client = self.create_client(
            StartNavigation,
            self.navigation_start_service,
            callback_group=self.callback_group,
        )
        self.navigation_cancel_client = self.create_client(
            CancelNavigation,
            self.navigation_cancel_service,
            callback_group=self.callback_group,
        )
        self.manipulation_start_client = self.create_client(
            StartManipulation,
            self.manipulation_start_service,
            callback_group=self.callback_group,
        )
        self.manipulation_cancel_client = self.create_client(
            CancelManipulation,
            self.manipulation_cancel_service,
            callback_group=self.callback_group,
        )
        self.sorting_rule_client = self.create_client(
            ClassifySortingRule,
            self.sorting_rule_service,
            callback_group=self.callback_group,
        )
        self.result_client = self.create_client(
            ReportMissionResult,
            self.mission_result_service,
            callback_group=self.callback_group,
        )
        self.brake_client = self.create_client(
            SetBool, self.brake_service, callback_group=self.callback_group
        )
        self.asrpro_speak_client = self.create_client(
            AsrproSpeak,
            self.asrpro_speak_service,
            callback_group=self.callback_group,
        )

        now = self.get_clock().now()
        self.lifecycle_state = LifecycleState.BOOTSTRAP
        self.transport_stage = TransportStage.IDLE
        self.state_enter_time = now
        self.stage_enter_time = now
        self.run_start_time = now
        self.last_mcu_status_time = now
        self.last_asrpro_status_time = now
        self.last_status_publish_time = now
        self.last_zero_publish_time = now - Duration(seconds=1.0)
        self.last_brake_request_time = now - Duration(seconds=1.0)

        self.local_run_id = 0
        self.active = False
        self.pending_start = False
        self.pending_reset = False
        self.voice_start_received = False
        self.transition_announcement_sent = False
        self.voice_prompt_sent = False
        self.autonomous_start_announcement_sent = False
        self.completion_announcement_done = False
        self.speech_retry_count = 0
        self.speech_done_phrase_id = ''
        self.speech_failed_phrase_id = ''
        self.sorting_rule_confirmed = False
        self.sorting_rule_confidence = 0.0
        self.park_1_cargo = ''
        self.park_2_cargo = ''
        self.current_cargo = ''
        self.current_park = ''
        self.current_waypoint = ''
        self.holding_cargo = False
        # 每次导航前都要确认机械臂处于收拢安全位姿
        # 抓取或投放阶段开始时该标志会清零，导航包装器负责重新执行安全收臂动作
        self.arm_safe_ready = False
        self.sorting_pose_ready = False
        self.hard_time_stop = False
        self.delivered_counts = {'gear': 0, 't_bolt': 0}
        self.delivered_by_park = {'park_1': 0, 'park_2': 0}
        self.no_target_counts = {'gear': 0, 't_bolt': 0}
        self.attempted_pick_count = 0
        self.plan_cursor = 0
        self.pick_retry_count = 0
        self.classification_retry_count = 0
        self.navigation_retry_count = 0
        self.skipped_stage_count = 0
        self.last_skipped_stage = ''
        self.operation = Operation()
        self.latest_mcu_status: Optional[McuStatus] = None
        self.latest_asrpro_status: Optional[AsrproStatus] = None
        self.latest_navigation_status: Optional[NavigationStatus] = None
        self.latest_manipulation_status: Optional[ManipulationStatus] = None
        self.error_code = 0
        self.message = '正在启动全自主运输状态机'
        self.report_future = None
        self.report_started_at = now
        self.report_accepted = False
        self.shutdown_requested = False
        self.safe_stop_active = True

        self.timer = self.create_timer(
            1.0 / self.update_rate_hz,
            self._tick,
            callback_group=self.callback_group,
        )
        self.get_logger().info(
            'Atlas 全自主运输状态机已启动: '
            f'nav={self.navigation_backend} manipulation={self.manipulation_backend} '
            f'config={config_path or "<empty>"}'
        )
        if self.config_errors:
            for item in self.config_errors:
                self.get_logger().error(f'配置检查: {item}')

    # ------------------------------------------------------------------
    # 配置读取与校验
    # ------------------------------------------------------------------
    @staticmethod
    def _cargo_name(value: object) -> str:
        token = str(value or '').strip().lower().replace(' ', '_').replace('-', '_')
        if token in ('gear', 'chilun', '齿轮'):
            return 'gear'
        if token in ('t_bolt', 'tbolt', 'bolt', 'luosi', '螺栓', 't型螺栓'):
            return 't_bolt'
        return token

    @staticmethod
    def _waypoint(node: dict, fallback_id: str) -> Waypoint:
        return Waypoint(
            waypoint_id=str(node.get('id', fallback_id)),
            x_m=float(node.get('x_m', node.get('x', 0.0))),
            y_m=float(node.get('y_m', node.get('y', 0.0))),
            yaw_rad=float(node.get('yaw_rad', node.get('yaw', 0.0))),
            timeout_s=max(0.5, float(node.get('timeout_s', 20.0))),
        )

    def _load_plan(self, path: str) -> TransportPlan:
        plan = TransportPlan()
        if not path:
            return plan
        try:
            data = yaml.safe_load(Path(path).read_text(encoding='utf-8')) or {}
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'全自主运输配置读取失败 {path}: {exc}')
            return plan

        root = data.get('autonomous_transport', data)
        plan.calibration_confirmed = bool(root.get('calibration_confirmed', False))

        timing = root.get('timing', {}) or {}
        plan.max_autonomous_duration_s = max(
            1.0, float(timing.get('max_autonomous_duration_s', 300.0))
        )
        plan.stop_reserve_s = max(0.0, float(timing.get('stop_reserve_s', 15.0)))
        plan.manipulation_timeout_s = max(
            5.0, float(timing.get('manipulation_timeout_s', 60.0))
        )
        plan.classification_timeout_s = max(
            5.0, float(timing.get('classification_timeout_s', 20.0))
        )

        classification = root.get('classification', {}) or {}
        plan.classification_min_confidence = float(
            classification.get('minimum_confidence', 0.60)
        )
        fallback_mapping = classification.get('fallback_mapping', {}) or {}
        plan.fallback_park_1_cargo = self._cargo_name(
            fallback_mapping.get('park_1', 'gear')
        )
        plan.fallback_park_2_cargo = self._cargo_name(
            fallback_mapping.get('park_2', 't_bolt')
        )
        plan.use_fallback_mapping_on_failure = bool(
            classification.get('use_fallback_mapping_on_failure', False)
        )

        voice = root.get('voice', {}) or {}
        plan.voice_start_required = bool(voice.get('start_required', True))
        plan.voice_start_timeout_s = max(
            0.0, float(voice.get('start_timeout_s', 30.0))
        )
        plan.voice_start_fallback_enabled = bool(
            voice.get('fallback_enabled', False)
        )
        plan.asrpro_status_timeout_s = max(
            0.2, float(voice.get('asrpro_status_timeout_s', 2.0))
        )
        plan.speech_timeout_s = max(
            1.0, float(voice.get('speech_timeout_s', 8.0))
        )
        plan.speech_retry_count = max(
            0, int(voice.get('speech_retry_count', 2))
        )
        phrase_ids = voice.get('phrase_ids', {}) or {}
        plan.transition_phrase_id = str(
            phrase_ids.get('transition_complete', plan.transition_phrase_id)
        )
        plan.prompt_phrase_id = str(
            phrase_ids.get('voice_prompt', plan.prompt_phrase_id)
        )
        plan.autonomous_start_phrase_id = str(
            phrase_ids.get('autonomous_start', plan.autonomous_start_phrase_id)
        )
        plan.delivery_complete_phrase_id = str(
            phrase_ids.get('delivery_complete', plan.delivery_complete_phrase_id)
        )
        plan.task_complete_phrase_id = str(
            phrase_ids.get('task_complete', plan.task_complete_phrase_id)
        )
        plan.task_skipped_phrase_id = str(
            phrase_ids.get('task_skipped', plan.task_skipped_phrase_id)
        )
        plan.accepted_voice_phrases = [
            str(item).strip().lower()
            for item in voice.get(
                'accepted_intents', voice.get('accepted_phrases', plan.accepted_voice_phrases)
            )
            if str(item).strip()
        ]

        cargo = root.get('cargo', {}) or {}
        plan.target_total = max(0, int(cargo.get('target_total', 8)))
        expected = cargo.get('expected_counts', {}) or {}
        plan.expected_counts = {
            'gear': max(0, int(expected.get('gear', 4))),
            't_bolt': max(0, int(expected.get('t_bolt', 4))),
        }
        raw_plan = cargo.get('plan', plan.cargo_plan)
        plan.cargo_plan = [
            self._cargo_name(item)
            for item in raw_plan
            if self._cargo_name(item) in ('gear', 't_bolt')
        ]
        plan.no_target_limit_per_class = max(
            1, int(cargo.get('no_target_limit_per_class', 2))
        )

        recovery = root.get('recovery', {}) or {}
        plan.navigation_retry_count = max(
            0, int(recovery.get('navigation_retry_count', 1))
        )
        plan.classification_retry_count = max(
            0, int(recovery.get('classification_retry_count', 2))
        )
        plan.pick_retry_count = max(0, int(recovery.get('pick_retry_count', 1)))
        plan.continue_on_pick_failure = bool(
            recovery.get('continue_on_pick_failure', True)
        )

        waypoints = root.get('waypoints', {}) or {}
        if 'sorting_area' in waypoints:
            plan.sorting_waypoint = self._waypoint(
                waypoints['sorting_area'], 'sorting_area'
            )
        if 'dispatch_area' in waypoints:
            plan.dispatch_waypoint = self._waypoint(
                waypoints['dispatch_area'], 'dispatch_area'
            )
        if 'park_1' in waypoints:
            plan.park_waypoints['park_1'] = self._waypoint(
                waypoints['park_1'], 'park_1'
            )
        if 'park_2' in waypoints:
            plan.park_waypoints['park_2'] = self._waypoint(
                waypoints['park_2'], 'park_2'
            )

        manipulation = root.get('manipulation', {}) or {}
        plan.safe_prepare_action = str(
            manipulation.get('safe_prepare_action', plan.safe_prepare_action)
        )
        plan.sorting_prepare_action = str(
            manipulation.get(
                'sorting_prepare_action', plan.sorting_prepare_action
            )
        )
        for cargo_name in ('gear', 't_bolt'):
            if cargo_name in (manipulation.get('pick_prepare_actions', {}) or {}):
                plan.pick_prepare_actions[cargo_name] = str(
                    manipulation['pick_prepare_actions'][cargo_name]
                )
            if cargo_name in (manipulation.get('pick_tasks', {}) or {}):
                plan.pick_tasks[cargo_name] = str(
                    manipulation['pick_tasks'][cargo_name]
                )
        for park in ('park_1', 'park_2'):
            if park in (manipulation.get('place_prepare_actions', {}) or {}):
                plan.place_prepare_actions[park] = str(
                    manipulation['place_prepare_actions'][park]
                )
            tasks = (manipulation.get('place_tasks', {}) or {}).get(park, [])
            plan.place_tasks[park] = [str(item) for item in tasks if str(item)]
        return plan

    @staticmethod
    def _validate_waypoint(name: str, waypoint: Optional[Waypoint]) -> List[str]:
        errors: List[str] = []
        if waypoint is None:
            return [f'缺少导航点 {name}']
        values = (waypoint.x_m, waypoint.y_m, waypoint.yaw_rad, waypoint.timeout_s)
        if not all(math.isfinite(item) for item in values):
            errors.append(f'导航点 {name} 包含非有限数值')
        if waypoint.timeout_s <= 0.0:
            errors.append(f'导航点 {name} 的 timeout_s 必须大于 0')
        return errors

    def _validate_plan(self, plan: TransportPlan) -> List[str]:
        errors: List[str] = []
        if not plan.calibration_confirmed:
            errors.append('calibration_confirmed=false，状态机将拒绝进入运动阶段')
        errors.extend(self._validate_waypoint('sorting_area', plan.sorting_waypoint))
        errors.extend(self._validate_waypoint('dispatch_area', plan.dispatch_waypoint))
        errors.extend(self._validate_waypoint('park_1', plan.park_waypoints.get('park_1')))
        errors.extend(self._validate_waypoint('park_2', plan.park_waypoints.get('park_2')))

        # 标定确认打开后，四个区域必须对应四个可区分的地图停车点
        # 该检查用于阻止仅修改 calibration_confirmed、却未填写实际场地坐标的配置进入运动阶段
        if plan.calibration_confirmed:
            named_waypoints = {
                'sorting_area': plan.sorting_waypoint,
                'dispatch_area': plan.dispatch_waypoint,
                'park_1': plan.park_waypoints.get('park_1'),
                'park_2': plan.park_waypoints.get('park_2'),
            }
            available_waypoints = {
                name: waypoint
                for name, waypoint in named_waypoints.items()
                if waypoint is not None
            }
            waypoint_ids = [waypoint.waypoint_id for waypoint in available_waypoints.values()]
            if any(not waypoint_id.strip() for waypoint_id in waypoint_ids):
                errors.append('waypoints 中每个导航点都必须配置非空 id')
            if len(set(waypoint_ids)) != len(waypoint_ids):
                errors.append('waypoints 中四个导航点的 id 必须互不相同')

            # 只比较 x/y 停车位置；允许不同区域具有相同朝向
            # 1 mm 量化可过滤 YAML 浮点表示噪声，同时不会掩盖实际停车点重合
            quantized_positions = {
                (round(waypoint.x_m, 3), round(waypoint.y_m, 3))
                for waypoint in available_waypoints.values()
            }
            if len(quantized_positions) != len(available_waypoints):
                errors.append('waypoints 中四个区域的 x_m/y_m 停车位置必须互不重合')
            if available_waypoints and all(
                abs(waypoint.x_m) < 1.0e-6 and abs(waypoint.y_m) < 1.0e-6
                for waypoint in available_waypoints.values()
            ):
                errors.append('waypoints 仍为全零占位坐标，禁止进入运动阶段')

        if not math.isfinite(plan.classification_min_confidence) or not (
            0.0 <= plan.classification_min_confidence <= 1.0
        ):
            errors.append('classification.minimum_confidence 必须位于 0.0 到 1.0')
        if plan.max_autonomous_duration_s <= plan.stop_reserve_s:
            errors.append('timing.max_autonomous_duration_s 必须大于 stop_reserve_s')
        if plan.target_total <= 0:
            errors.append('cargo.target_total 必须大于 0')
        expected_total = sum(plan.expected_counts.values())
        if expected_total != plan.target_total:
            errors.append(
                'cargo.expected_counts 合计必须等于 cargo.target_total: '
                f'{expected_total} != {plan.target_total}'
            )
        if not plan.cargo_plan:
            errors.append('cargo.plan 不能为空')
        if not plan.safe_prepare_action:
            errors.append('缺少 manipulation.safe_prepare_action')
        if not plan.sorting_prepare_action:
            errors.append('缺少 manipulation.sorting_prepare_action')
        for cargo in ('gear', 't_bolt'):
            if not plan.pick_tasks.get(cargo):
                errors.append(f'缺少 {cargo} 的 pick_task')
        required_place_slots = max(plan.expected_counts.values(), default=0)
        for park in ('park_1', 'park_2'):
            tasks = plan.place_tasks.get(park, [])
            if not tasks:
                errors.append(f'缺少 {park} 的 place_tasks')
            elif len(tasks) < required_place_slots:
                errors.append(
                    f'{park} 的 place_tasks 数量不足: '
                    f'{len(tasks)} < {required_place_slots}'
                )
        fallback = (plan.fallback_park_1_cargo, plan.fallback_park_2_cargo)
        if any(item not in ('gear', 't_bolt') for item in fallback):
            errors.append('classification.fallback_mapping 只能使用 gear 和 t_bolt')
        if fallback[0] == fallback[1]:
            errors.append('classification.fallback_mapping 的两个园区类别必须不同')
        if plan.voice_start_required and not plan.accepted_voice_phrases:
            errors.append('voice.accepted_intents 不能为空')
        phrase_ids = (
            plan.transition_phrase_id,
            plan.prompt_phrase_id,
            plan.autonomous_start_phrase_id,
            plan.delivery_complete_phrase_id,
            plan.task_complete_phrase_id,
            plan.task_skipped_phrase_id,
        )
        if any(not item.strip() for item in phrase_ids):
            errors.append('voice.phrase_ids 中的短语标识不能为空')
        return errors

    # ------------------------------------------------------------------
    # ROS 回调
    # ------------------------------------------------------------------
    def _on_mcu_status(self, message: McuStatus) -> None:
        with self._lock:
            previous_latched = (
                bool(self.latest_mcu_status.auto_start_latched)
                if self.latest_mcu_status is not None
                else False
            )
            self.latest_mcu_status = message
            self.last_mcu_status_time = self.get_clock().now()
            if previous_latched and not bool(message.auto_start_latched):
                self.pending_reset = True

    def _on_auto_task_event(self, message: AutoTaskEvent) -> None:
        with self._lock:
            if message.event == AutoTaskEvent.EVENT_START:
                self.pending_start = True
            elif message.event == AutoTaskEvent.EVENT_RESET:
                self.pending_reset = True

    def _on_navigation_status(self, message: NavigationStatus) -> None:
        with self._lock:
            self.latest_navigation_status = message

    def _on_manipulation_status(self, message: ManipulationStatus) -> None:
        with self._lock:
            self.latest_manipulation_status = message

    def _on_asrpro_status(self, message: AsrproStatus) -> None:
        with self._lock:
            self.latest_asrpro_status = message
            self.last_asrpro_status_time = self.get_clock().now()

    def _on_asrpro_event(self, message: AsrproEvent) -> None:
        with self._lock:
            if message.event == AsrproEvent.EVENT_SPEAK_DONE:
                self.speech_done_phrase_id = message.phrase_id.strip()
            elif message.event == AsrproEvent.EVENT_NACK:
                self.speech_failed_phrase_id = message.phrase_id.strip()

    def _on_voice_command(self, message: String) -> None:
        with self._lock:
            if self.transport_stage != TransportStage.WAIT_VOICE_START:
                return
            spoken = self._normalize_phrase(message.data)
            accepted = {
                self._normalize_phrase(item)
                for item in self.plan.accepted_voice_phrases
            }
            if spoken and spoken in accepted:
                self.voice_start_received = True
                self.message = f'已接收语音启动指令: {message.data.strip()}'
                self.get_logger().info(self.message)

    def _on_navigation_cmd_vel(self, message: Twist) -> None:
        with self._lock:
            # 只有运行中的导航操作拥有速度输出权；识别、抓取、投放、上报和等待阶段均拒绝非零速度
            motion_allowed = (
                self.lifecycle_state == LifecycleState.RUNNING
                and self.operation.kind == 'navigation'
                and self.operation.accepted
                and not self.safe_stop_active
            )
            if not motion_allowed:
                return
            self.cmd_vel_publisher.publish(self._limit_twist(message))

    # ------------------------------------------------------------------
    # 状态机主循环与全局安全门控
    # ------------------------------------------------------------------
    def _tick(self) -> None:
        with self._lock:
            now = self.get_clock().now()
            self._safety_tick(now)

            if self.shutdown_requested:
                self._transition_lifecycle(
                    LifecycleState.SHUTTING_DOWN, '节点正在关闭'
                )
                self._cancel_backends('节点关闭')
                self._enter_safe_stop('节点关闭')
                self._publish_status(now)
                return

            if self._handle_global_conditions(now):
                self._publish_status(now)
                return

            if self.lifecycle_state == LifecycleState.BOOTSTRAP:
                self._enter_safe_stop('启动阶段')
                self._transition_lifecycle(
                    LifecycleState.WAIT_MCU_STATUS, '等待 MCU 状态'
                )

            elif self.lifecycle_state == LifecycleState.WAIT_MCU_STATUS:
                if self._mcu_fresh(now):
                    if self._mcu_is_safe_idle():
                        self._transition_lifecycle(
                            LifecycleState.WAIT_START, 'MCU 状态可用'
                        )
                    elif (
                        self._mcu_is_auto_active()
                        and self.accept_latched_start_without_event
                    ):
                        # AutoPi 锁存状态能够补偿启动事件早于状态机订阅建立的情况；确保遥控手势不会因启动顺序而丢失
                        self.pending_start = False
                        self.local_run_id += 1
                        self._transition_lifecycle(
                            LifecycleState.PRECHECK,
                            '启动时检测到有效 AutoPi 锁存；进入任务预检',
                        )
                    elif self._mcu_is_auto_active():
                        self._set_error(4101, '启动时 MCU 已处于自动任务状态；当前配置禁止锁存补偿')
                        self._transition_lifecycle(
                            LifecycleState.RECOVERY_REQUIRED, self.message
                        )
                    else:
                        self._transition_lifecycle(
                            LifecycleState.WAIT_RESET, '等待 MCU 恢复安全状态'
                        )

            elif self.lifecycle_state == LifecycleState.WAIT_START:
                self._enter_safe_stop('等待全自主运输启动')
                if not self._mcu_fresh(now):
                    self._transition_lifecycle(
                        LifecycleState.WAIT_MCU_STATUS, 'MCU 状态不可用'
                    )
                elif self.pending_start and self._mcu_is_auto_active():
                    self.pending_start = False
                    self.local_run_id += 1
                    self._transition_lifecycle(
                        LifecycleState.PRECHECK, '自动任务启动事件已确认'
                    )
                elif (
                    self.accept_latched_start_without_event
                    and self._mcu_is_auto_active()
                ):
                    self.local_run_id += 1
                    self._transition_lifecycle(
                        LifecycleState.PRECHECK, '自动任务锁存状态已确认'
                    )

            elif self.lifecycle_state == LifecycleState.PRECHECK:
                error = self._common_precheck(now)
                if error is not None:
                    self._enter_safe_stop(error[1])
                    if error[0] == 4207:
                        self._set_error(error[0], error[1])
                        self._transition_lifecycle(
                            LifecycleState.RECOVERY_REQUIRED, error[1]
                        )
                    else:
                        self.message = f'预检等待: {error[1]}'
                else:
                    self._initialize_run(now)
                    self._transition_lifecycle(
                        LifecycleState.RUNNING, '全自主运输任务初始化完成'
                    )

            elif self.lifecycle_state == LifecycleState.RUNNING:
                self._run_transport_flow(now)

            elif self.lifecycle_state in (
                LifecycleState.REPORTING_DONE,
                LifecycleState.REPORTING_FAIL,
            ):
                self._handle_result_reporting(now)

            elif self.lifecycle_state == LifecycleState.WAIT_RESET:
                self._enter_safe_stop('等待 MCU 复位')
                if self._mcu_is_safe_idle():
                    self._clear_run_context()
                    self._transition_lifecycle(
                        LifecycleState.WAIT_START, 'MCU 复位已确认'
                    )

            elif self.lifecycle_state == LifecycleState.RECOVERY_REQUIRED:
                self._enter_safe_stop('需要人工确认并复位')
                if self._mcu_is_safe_idle():
                    self._clear_run_context()
                    self._transition_lifecycle(
                        LifecycleState.WAIT_START, '安全基线已恢复'
                    )

            elif self.lifecycle_state == LifecycleState.SHUTTING_DOWN:
                self._enter_safe_stop('节点关闭')

            self._publish_status(now)

    def _handle_global_conditions(self, now) -> bool:
        if self.pending_reset:
            self.pending_reset = False
            self._cancel_backends('MCU RESET')
            self._enter_safe_stop('MCU RESET')
            # RESET 会终止本轮上下文并清除尚未消费的启动事件，防止复位后误用失效事件再次进入自动任务
            self._clear_run_context()
            target = (
                LifecycleState.WAIT_START
                if self._mcu_is_safe_idle()
                else LifecycleState.WAIT_RESET
            )
            self._transition_lifecycle(target, '收到 MCU RESET 事件')
            return True

        active_states = (
            LifecycleState.PRECHECK,
            LifecycleState.RUNNING,
            LifecycleState.REPORTING_DONE,
            LifecycleState.REPORTING_FAIL,
        )
        if self.lifecycle_state in active_states:
            if not self._mcu_fresh(now):
                self._abort_to_recovery(4102, '自动任务期间 MCU 状态超时')
                return True
            if self._mcu_has_estop():
                self._abort_to_wait_reset(4103, 'MCU 进入急停状态')
                return True
            if self._mcu_has_fault():
                self._abort_to_wait_reset(4104, 'MCU 进入故障状态')
                return True
            if self.latest_mcu_status.app_state == McuStatus.STATE_MANUAL:
                self._abort_to_wait_reset(4105, '手动控制接管，自动任务立即停止')
                return True
            if not self._mcu_is_auto_active() and self.lifecycle_state in (
                LifecycleState.PRECHECK,
                LifecycleState.RUNNING,
            ):
                self._abort_to_wait_reset(4106, 'MCU 已离开 AutoPi 或清除任务锁存')
                return True
        return False

    def _common_precheck(self, now) -> Optional[Tuple[int, str]]:
        if not self._mcu_fresh(now):
            return 4201, 'MCU 状态不可用或超时'
        if not self._mcu_is_auto_active():
            return 4202, 'MCU 未处于 AutoPi 且任务锁存未建立'
        status = self.latest_mcu_status
        if (status.online_flags & self.ONLINE_PI) == 0:
            return 4203, 'MCU 未确认 Pi 在线'
        if (status.ready_flags & self.READY_CHASSIS) == 0:
            return 4204, '底盘未就绪'
        if (status.ready_flags & self.READY_ODOM) == 0:
            return 4205, '里程计未就绪'
        if self.require_arm_ready and (status.ready_flags & self.READY_ARM) == 0:
            return 4206, '机械臂未就绪'
        if self.config_errors:
            return 4207, '配置校验未通过: ' + '; '.join(self.config_errors)
        if not self._asrpro_ready(now):
            return 4209, 'ASRPRO 未就绪或状态超时'
        required_clients = (
            (self.navigation_start_client, self.navigation_start_service),
            (self.navigation_cancel_client, self.navigation_cancel_service),
            (self.manipulation_start_client, self.manipulation_start_service),
            (self.manipulation_cancel_client, self.manipulation_cancel_service),
            (self.sorting_rule_client, self.sorting_rule_service),
            (self.result_client, self.mission_result_service),
            # 制动服务属于状态机的安全闭环；服务不可用时不允许启动自主运动
            (self.brake_client, self.brake_service),
            (self.asrpro_speak_client, self.asrpro_speak_service),
        )
        unavailable = [name for client, name in required_clients if not client.service_is_ready()]
        if unavailable:
            return 4208, '依赖服务未就绪: ' + ', '.join(unavailable)
        return None

    # ------------------------------------------------------------------
    # 全自主运输区任务流程
    # ------------------------------------------------------------------
    def _initialize_run(self, now) -> None:
        self.active = True
        self.run_start_time = now
        self.error_code = 0
        self.message = '全自主运输任务运行中'
        self.transport_stage = TransportStage.ANNOUNCE_TRANSITION
        self.stage_enter_time = now
        self.transition_announcement_sent = False
        self.voice_prompt_sent = False
        self.autonomous_start_announcement_sent = False
        self.completion_announcement_done = False
        self.speech_retry_count = 0
        self.speech_done_phrase_id = ''
        self.speech_failed_phrase_id = ''
        self.voice_start_received = False
        self.sorting_rule_confirmed = False
        self.sorting_rule_confidence = 0.0
        self.park_1_cargo = ''
        self.park_2_cargo = ''
        self.current_cargo = ''
        self.current_park = ''
        self.current_waypoint = ''
        self.holding_cargo = False
        self.arm_safe_ready = False
        self.sorting_pose_ready = False
        self.hard_time_stop = False
        self.delivered_counts = {'gear': 0, 't_bolt': 0}
        self.delivered_by_park = {'park_1': 0, 'park_2': 0}
        self.no_target_counts = {'gear': 0, 't_bolt': 0}
        self.attempted_pick_count = 0
        self.plan_cursor = 0
        self.pick_retry_count = 0
        self.classification_retry_count = 0
        self.navigation_retry_count = 0
        self.skipped_stage_count = 0
        self.last_skipped_stage = ''
        self._clear_operation()
        self.report_future = None
        self.report_accepted = False
        self._enter_safe_stop('任务初始化')

    def _run_transport_flow(self, now) -> None:
        elapsed = self._elapsed_s(now)
        remaining = self.plan.max_autonomous_duration_s - elapsed

        # 硬时间上限到达时立即取消后端并停止速度输出；不再开始新的动作
        # 按已经完成的货物数量结束本轮任务
        if remaining <= 0.0:
            self.hard_time_stop = True
            self._cancel_backends('全自主运输时间上限到达')
            self._enter_safe_stop('全自主运输时间上限到达')
            self._clear_operation()
            self._record_skipped_stage(
                self.transport_stage,
                '全自主运输时间上限到达；剩余阶段不再执行',
            )
            self._set_stage(
                TransportStage.COMPLETE,
                '时间上限到达；按已完成货物数量结束任务',
            )

        # 没有携带货物时；预留时间不足则不再发起新一轮抓取
        elif (
            not self.holding_cargo
            and remaining <= self.plan.stop_reserve_s
            and self.transport_stage
            not in (TransportStage.COMPLETE, TransportStage.FAILED)
        ):
            self._cancel_backends('进入安全收尾时间')
            self._enter_safe_stop('进入安全收尾时间')
            self._clear_operation()
            self._record_skipped_stage(
                self.transport_stage,
                '剩余时间进入安全收尾窗口；后续运输阶段跳过',
            )
            self._set_stage(
                TransportStage.COMPLETE,
                '剩余时间进入安全收尾窗口',
            )

        if self.transport_stage == TransportStage.ANNOUNCE_TRANSITION:
            self._enter_safe_stop('中转区播报')
            result = self._run_speech_operation(
                self.plan.transition_phrase_id,
                '播报遥操作区任务已完成',
                now,
            )
            if result is True:
                self.transition_announcement_sent = True
                self.speech_retry_count = 0
                self._set_stage(
                    TransportStage.ANNOUNCE_VOICE_PROMPT,
                    '中转区播报完成；准备提示语音启动',
                )
            elif result is False:
                self._handle_speech_failure(
                    TransportStage.ANNOUNCE_VOICE_PROMPT,
                    '中转区播报失败',
                )

        elif self.transport_stage == TransportStage.ANNOUNCE_VOICE_PROMPT:
            self._enter_safe_stop('提示语音启动')
            result = self._run_speech_operation(
                self.plan.prompt_phrase_id,
                '提示说出 Atlas 启动',
                now,
            )
            if result is True:
                self.voice_prompt_sent = True
                self.speech_retry_count = 0
                if self.plan.voice_start_required:
                    self._set_stage(
                        TransportStage.WAIT_VOICE_START,
                        '等待 ASRPRO 识别 Atlas 启动',
                    )
                else:
                    self.voice_start_received = True
                    self._set_stage(
                        TransportStage.ANNOUNCE_AUTONOMOUS_START,
                        '语音门控已关闭；准备播报全自主任务启动',
                    )
            elif result is False:
                self._handle_speech_failure(
                    TransportStage.WAIT_VOICE_START,
                    '语音启动提示播报失败',
                )

        elif self.transport_stage == TransportStage.WAIT_VOICE_START:
            self._enter_safe_stop('等待语音启动')
            if self.voice_start_received:
                self._set_stage(
                    TransportStage.ANNOUNCE_AUTONOMOUS_START,
                    'ASRPRO 已识别启动指令；准备播报并开始任务',
                )
            elif (
                self.plan.voice_start_fallback_enabled
                and self.plan.voice_start_timeout_s > 0.0
                and self._stage_elapsed_s(now) >= self.plan.voice_start_timeout_s
            ):
                self._record_skipped_stage(
                    TransportStage.WAIT_VOICE_START,
                    '语音等待超时；按配置使用自动启动后备策略',
                )
                self.voice_start_received = True
                self._set_stage(
                    TransportStage.ANNOUNCE_AUTONOMOUS_START,
                    '语音等待超时；准备播报并开始任务',
                )

        elif self.transport_stage == TransportStage.ANNOUNCE_AUTONOMOUS_START:
            self._enter_safe_stop('播报全自主任务启动')
            result = self._run_speech_operation(
                self.plan.autonomous_start_phrase_id,
                '播报开始执行全自主运输任务',
                now,
            )
            if result is True:
                self.autonomous_start_announcement_sent = True
                self.speech_retry_count = 0
                self._set_stage(
                    TransportStage.NAVIGATE_SORTING_AREA,
                    '启动播报完成；前往智能分拣区',
                )
            elif result is False:
                self._handle_speech_failure(
                    TransportStage.NAVIGATE_SORTING_AREA,
                    '全自主任务启动播报失败',
                )

        elif self.transport_stage == TransportStage.NAVIGATE_SORTING_AREA:
            result = self._run_navigation_with_arm_safe(
                self.plan.sorting_waypoint,
                '智能分拣区识别位姿',
                now,
            )
            if result is True:
                self.navigation_retry_count = 0
                self._set_stage(
                    TransportStage.CLASSIFY_SORTING_RULE,
                    '已到达智能分拣区；开始识别园区映射',
                )
            elif result is False:
                self._handle_navigation_failure(
                    TransportStage.NAVIGATE_SORTING_AREA,
                    TransportStage.NAVIGATE_DISPATCH_AREA,
                    '前往智能分拣区失败',
                    apply_fallback_mapping=True,
                )

        elif self.transport_stage == TransportStage.CLASSIFY_SORTING_RULE:
            result = self._run_classification_with_camera_pose(now)
            if result is True:
                self.classification_retry_count = 0
                # 分类识别位姿用于观察标识；不视为导航安全收拢位姿
                self.arm_safe_ready = False
                self._set_stage(
                    TransportStage.NAVIGATE_DISPATCH_AREA,
                    f'分类规则确认: 园区1={self.park_1_cargo}; 园区2={self.park_2_cargo}',
                )
            elif result is False:
                if self.classification_retry_count < self.plan.classification_retry_count:
                    self.classification_retry_count += 1
                    self._clear_operation()
                    self.message = (
                        '分类规则识别失败；准备重试 '
                        f'{self.classification_retry_count}/{self.plan.classification_retry_count}'
                    )
                elif self._apply_fallback_mapping('分类规则识别多次失败'):
                    self._record_skipped_stage(
                        TransportStage.CLASSIFY_SORTING_RULE,
                        '分类规则识别多次失败；已使用 YAML 后备园区映射',
                    )
                    self._set_stage(
                        TransportStage.NAVIGATE_DISPATCH_AREA,
                        f'使用后备映射: 园区1={self.park_1_cargo}; 园区2={self.park_2_cargo}',
                    )
                else:
                    self._record_skipped_stage(
                        TransportStage.CLASSIFY_SORTING_RULE,
                        '分类规则识别失败且未启用后备映射；货物派送阶段跳过',
                    )
                    self._set_stage(
                        TransportStage.COMPLETE,
                        '缺少有效园区映射；按部分完成结束任务',
                    )

        elif self.transport_stage == TransportStage.NAVIGATE_DISPATCH_AREA:
            result = self._run_navigation_with_arm_safe(
                self.plan.dispatch_waypoint,
                '待派送区抓取位姿',
                now,
            )
            if result is True:
                self.navigation_retry_count = 0
                self._set_stage(
                    TransportStage.SELECT_CARGO,
                    '已到达待派送区',
                )
            elif result is False:
                self._handle_navigation_failure(
                    TransportStage.NAVIGATE_DISPATCH_AREA,
                    TransportStage.COMPLETE,
                    '前往待派送区失败；后续货物运输阶段跳过',
                )

        elif self.transport_stage == TransportStage.SELECT_CARGO:
            self._enter_safe_stop('选择下一类货物')
            if self._delivered_total() >= self.plan.target_total:
                self._set_stage(
                    TransportStage.COMPLETE,
                    '计划货物数量已全部完成',
                )
            else:
                candidate = self._select_next_cargo()
                if candidate is None:
                    self._set_stage(
                        TransportStage.COMPLETE,
                        '所有可选货物类别均达到完成或无目标终止条件',
                    )
                else:
                    self.current_cargo = candidate
                    self.current_park = self._park_for_cargo(candidate)
                    if not self.current_park:
                        self._record_skipped_stage(
                            TransportStage.SELECT_CARGO,
                            f'未找到 {candidate} 对应园区；该类别不再选择',
                        )
                        self.no_target_counts[candidate] = self.plan.no_target_limit_per_class
                        self.current_cargo = ''
                        self.current_park = ''
                    else:
                        self.pick_retry_count = 0
                        self._set_stage(
                            TransportStage.PICK_CARGO,
                            f'准备抓取 {candidate}',
                        )

        elif self.transport_stage == TransportStage.PICK_CARGO:
            prepare_action = self.plan.pick_prepare_actions[self.current_cargo]
            task = self.plan.pick_tasks[self.current_cargo]
            result = self._run_manipulation_operation(
                request_id=(
                    f'run_{self.local_run_id}_pick_{self.current_cargo}_'
                    f'{self.attempted_pick_count + 1}'
                ),
                prepare_action=prepare_action,
                arrival_task=task,
                purpose=f'抓取 {self.current_cargo}',
                now=now,
            )
            if result is True:
                status = self.latest_manipulation_status
                target_count = int(getattr(status, 'target_count', 0))
                target_found = bool(getattr(status, 'target_found', target_count > 0))
                self.attempted_pick_count += 1
                self.pick_retry_count = 0
                if not target_found or target_count <= 0:
                    self.no_target_counts[self.current_cargo] += 1
                    self.message = (
                        f'待派送区未发现 {self.current_cargo}; '
                        f'累计 {self.no_target_counts[self.current_cargo]} 次'
                    )
                    self._record_skipped_stage(
                        TransportStage.PICK_CARGO,
                        self.message,
                    )
                    self.current_cargo = ''
                    self.current_park = ''
                    self._set_stage(
                        TransportStage.SELECT_CARGO,
                        self.message,
                    )
                else:
                    self.holding_cargo = True
                    self._set_stage(
                        TransportStage.NAVIGATE_TARGET_PARK,
                        f'已抓取 {self.current_cargo}; 前往 {self.current_park}',
                    )
            elif result is False:
                if self.pick_retry_count < self.plan.pick_retry_count:
                    self.pick_retry_count += 1
                    self._clear_operation()
                    self.message = (
                        f'抓取 {self.current_cargo} 失败；准备重试 '
                        f'{self.pick_retry_count}/{self.plan.pick_retry_count}'
                    )
                else:
                    failed_cargo = self.current_cargo
                    self.attempted_pick_count += 1
                    self.no_target_counts[failed_cargo] += 1
                    self._record_skipped_stage(
                        TransportStage.PICK_CARGO,
                        f'抓取 {failed_cargo} 失败；本次抓取阶段跳过',
                    )
                    self.current_cargo = ''
                    self.current_park = ''
                    self._clear_operation()
                    if self.plan.continue_on_pick_failure:
                        self._set_stage(
                            TransportStage.SELECT_CARGO,
                            '抓取失败；继续选择其他货物',
                        )
                    else:
                        self._set_stage(
                            TransportStage.COMPLETE,
                            '抓取失败；按配置结束后续运输',
                        )

        elif self.transport_stage == TransportStage.NAVIGATE_TARGET_PARK:
            waypoint = self.plan.park_waypoints[self.current_park]
            result = self._run_navigation_with_arm_safe(
                waypoint,
                f'{self.current_park} 投放位姿',
                now,
            )
            if result is True:
                self.navigation_retry_count = 0
                self._set_stage(
                    TransportStage.PLACE_CARGO,
                    f'已到达 {self.current_park}; 准备投放 {self.current_cargo}',
                )
            elif result is False:
                self._handle_navigation_failure(
                    TransportStage.NAVIGATE_TARGET_PARK,
                    TransportStage.COMPLETE,
                    f'前往 {self.current_park} 失败；保留吸盘状态并结束后续移动',
                )

        elif self.transport_stage == TransportStage.PLACE_CARGO:
            task = self._place_task_for_current_delivery()
            prepare = self.plan.place_prepare_actions[self.current_park]
            result = self._run_manipulation_operation(
                request_id=(
                    f'run_{self.local_run_id}_place_{self.current_park}_'
                    f'{self.delivered_by_park[self.current_park] + 1}'
                ),
                prepare_action=prepare,
                arrival_task=task,
                purpose=f'在 {self.current_park} 投放 {self.current_cargo}',
                now=now,
            )
            if result is True:
                cargo = self.current_cargo
                park = self.current_park
                self.delivered_counts[cargo] += 1
                self.delivered_by_park[park] += 1
                self.holding_cargo = False
                self.current_cargo = ''
                self.current_park = ''
                self.navigation_retry_count = 0
                self._request_optional_speech(self.plan.delivery_complete_phrase_id)
                if self._delivered_total() >= self.plan.target_total:
                    self._set_stage(
                        TransportStage.COMPLETE,
                        '全自主运输计划已完成',
                    )
                else:
                    self._set_stage(
                        TransportStage.NAVIGATE_DISPATCH_AREA,
                        '返回待派送区继续执行下一轮运输',
                    )
            elif result is False:
                # 投放失败时机器人可能仍携带货物；继续移动会扩大风险；因此直接按安全失败处理
                # 安全失败通过部分完成收尾实现；保留吸盘状态；避免在未知位置主动释放货物
                self._record_skipped_stage(
                    TransportStage.PLACE_CARGO,
                    f'在 {self.current_park} 投放 {self.current_cargo} 失败；保留吸盘状态并结束后续动作',
                )
                self._set_stage(
                    TransportStage.COMPLETE,
                    '投放阶段失败；按部分完成结束任务',
                )

        elif self.transport_stage == TransportStage.COMPLETE:
            self._enter_safe_stop('全自主运输完成')

            # 硬时间上限到达后不再启动任何机械臂动作；只保持零速度并立即上报结束
            # 播报动作同样不再启动；上报内容保留已经完成的货物结果
            if self.hard_time_stop:
                self._cancel_backends('全自主运输时间上限到达')
                self._clear_operation()
                self.active = False
                self._begin_result_report(done=True, code=0, message=self.message)
                return

            if not self.completion_announcement_done:
                speech_result = self._run_speech_operation(
                    self.plan.task_complete_phrase_id,
                    '播报全自主运输任务结束',
                    now,
                )
                if speech_result is None:
                    return
                if speech_result is False:
                    self._record_skipped_stage(
                        TransportStage.COMPLETE,
                        '任务结束播报失败；继续执行安全收尾',
                    )
                self.completion_announcement_done = True
                self.speech_retry_count = 0
                self._clear_operation()

            # 正常完成、无目标结束或进入收尾窗口时；先把机械臂收回底盘安全包络
            # 该动作不改变吸盘状态；避免在未知位置主动释放仍被吸住的货物
            if not self.arm_safe_ready:
                safe_result = self._run_manipulation_operation(
                    request_id=f'run_{self.local_run_id}_final_arm_safe',
                    prepare_action=self.plan.safe_prepare_action,
                    arrival_task='prepare_only',
                    purpose='任务结束前安全收臂',
                    now=now,
                )
                if safe_result is None:
                    return
                if safe_result is False:
                    self._record_skipped_stage(
                        TransportStage.COMPLETE,
                        '任务结束前安全收臂失败；保持底盘停止并继续上报',
                    )
                else:
                    self.arm_safe_ready = True

            self._cancel_backends('全自主运输完成')
            self._clear_operation()
            self.active = False
            self._begin_result_report(done=True, code=0, message=self.message)

        elif self.transport_stage == TransportStage.FAILED:
            self._enter_safe_stop('全自主运输失败')
            self._cancel_backends('全自主运输失败')
            self._clear_operation()
            self.active = False
            self._begin_result_report(
                done=False,
                code=self.error_code or 4399,
                message=self.message,
            )

    # ------------------------------------------------------------------
    # 后端操作
    # ------------------------------------------------------------------
    def _run_speech_operation(
        self,
        phrase_id: str,
        purpose: str,
        now,
    ) -> Optional[bool]:
        """可靠请求 ASRPRO 播报；等待 ACK 和 SPEAK_DONE 事件"""
        if self.operation.kind == '':
            if not self._asrpro_ready(now) or not self.asrpro_speak_client.service_is_ready():
                if self._stage_elapsed_s(now) >= self.plan.speech_timeout_s:
                    self.message = f'ASRPRO 未就绪: {purpose}'
                    return False
                return None
            request = AsrproSpeak.Request()
            request.request_id = f'run_{self.local_run_id}_{phrase_id}'
            request.phrase_id = phrase_id
            self.speech_done_phrase_id = ''
            self.speech_failed_phrase_id = ''
            self.operation = Operation(
                kind='speech',
                request_id=phrase_id,
                purpose=purpose,
                future=self.asrpro_speak_client.call_async(request),
                accepted=False,
                started_at_ns=now.nanoseconds,
                timeout_s=self.plan.speech_timeout_s,
            )
            self._enter_safe_stop(f'ASRPRO 播报: {purpose}')
            return None

        if self.operation.kind != 'speech':
            self._fail_transport(4404, '状态机内部操作类型冲突')
            return False

        elapsed = (now.nanoseconds - self.operation.started_at_ns) * 1e-9
        phrase_id = self.operation.request_id
        if self.speech_failed_phrase_id == phrase_id:
            self.message = f'ASRPRO 拒绝播报: {self.operation.purpose}'
            self._clear_operation()
            return False
        if self.speech_done_phrase_id == phrase_id:
            self.message = f'ASRPRO 播报完成: {self.operation.purpose}'
            self._clear_operation()
            return True
        if elapsed > self.operation.timeout_s:
            self.message = f'ASRPRO 播报超时: {self.operation.purpose}'
            self._clear_operation()
            return False

        if not self.operation.accepted:
            if not self.operation.future.done():
                return None
            try:
                response = self.operation.future.result()
            except Exception as exc:  # noqa: BLE001
                self.message = f'ASRPRO 播报服务异常: {exc}'
                self._clear_operation()
                return False
            if response is None or not bool(response.success) or not bool(response.accepted):
                self.message = (
                    response.message if response is not None else 'ASRPRO 播报服务无响应'
                )
                self._clear_operation()
                return False
            self.operation.accepted = True
            self.operation.future = None
        return None

    def _request_optional_speech(self, phrase_id: str) -> None:
        """非关键播报只尝试一次；失败不会改变任务阶段"""
        if not phrase_id.strip():
            return
        now = self.get_clock().now()
        if not self._asrpro_ready(now) or not self.asrpro_speak_client.service_is_ready():
            return
        request = AsrproSpeak.Request()
        request.request_id = f'run_{self.local_run_id}_optional_{phrase_id}'
        request.phrase_id = phrase_id
        try:
            self.asrpro_speak_client.call_async(request)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'可选播报请求失败: {exc}')

    def _run_navigation_with_arm_safe(
        self, waypoint: Waypoint, purpose: str, now
    ) -> Optional[bool]:
        """先确认机械臂收拢，再允许导航后端获得速度输出权"""
        if not self.arm_safe_ready:
            safe_result = self._run_manipulation_operation(
                request_id=(
                    f'run_{self.local_run_id}_arm_safe_{waypoint.waypoint_id}'
                ),
                prepare_action=self.plan.safe_prepare_action,
                arrival_task='prepare_only',
                purpose=f'导航前安全收臂: {purpose}',
                now=now,
            )
            if safe_result is True:
                self.arm_safe_ready = True
                self.message = f'机械臂已收拢，准备导航: {purpose}'
                return None
            if safe_result is False:
                self.arm_safe_ready = False
                self.message = f'导航前安全收臂失败: {purpose}'
                return False
            return None
        return self._run_navigation_operation(waypoint, purpose, now)

    def _run_navigation_operation(
        self, waypoint: Waypoint, purpose: str, now
    ) -> Optional[bool]:
        if self.operation.kind == '':
            request = StartNavigation.Request()
            request.backend = self.navigation_backend
            request.waypoint_id = waypoint.waypoint_id
            request.x_m = waypoint.x_m
            request.y_m = waypoint.y_m
            request.yaw_rad = waypoint.yaw_rad
            request.reset_origin = False
            request.timeout_s = waypoint.timeout_s
            self.latest_navigation_status = None
            self.operation = Operation(
                kind='navigation',
                request_id=waypoint.waypoint_id,
                purpose=purpose,
                future=self.navigation_start_client.call_async(request),
                accepted=False,
                started_at_ns=now.nanoseconds,
                timeout_s=waypoint.timeout_s + self.service_response_timeout_s,
            )
            self.current_waypoint = waypoint.waypoint_id
            self._leave_safe_stop(f'导航: {purpose}')
            self.get_logger().info(
                f'开始导航 id={waypoint.waypoint_id} x={waypoint.x_m:.3f} '
                f'y={waypoint.y_m:.3f} yaw={waypoint.yaw_rad:.3f}'
            )
            return None

        if self.operation.kind != 'navigation':
            self._fail_transport(4401, '状态机内部操作类型冲突')
            return False

        elapsed = (now.nanoseconds - self.operation.started_at_ns) * 1e-9
        if elapsed > self.operation.timeout_s:
            self.message = f'导航超时: {self.operation.purpose}'
            self._cancel_navigation(self.message)
            self._enter_safe_stop(self.message)
            self._clear_operation()
            return False

        if not self.operation.accepted:
            if not self.operation.future.done():
                return None
            try:
                response = self.operation.future.result()
            except Exception as exc:  # noqa: BLE001
                self.message = f'导航启动服务异常: {exc}'
                self._enter_safe_stop(self.message)
                self._clear_operation()
                return False
            if response is None or not bool(response.success):
                self.message = (
                    response.message if response is not None else '导航启动服务无响应'
                )
                self._enter_safe_stop(self.message)
                self._clear_operation()
                return False
            self.operation.accepted = True
            self.operation.future = None
            return None

        status = self.latest_navigation_status
        if status is None or status.waypoint_id != self.operation.request_id:
            return None
        if status.state == NavigationStatus.STATE_SUCCEEDED:
            self._enter_safe_stop(f'导航完成: {self.operation.purpose}')
            self._clear_operation()
            return True
        if status.state in (
            NavigationStatus.STATE_FAILED,
            NavigationStatus.STATE_CANCELLED,
        ):
            self.message = f'导航失败: {status.message}'
            self._enter_safe_stop(self.message)
            self._clear_operation()
            return False
        return None

    def _run_manipulation_operation(
        self,
        request_id: str,
        prepare_action: str,
        arrival_task: str,
        purpose: str,
        now,
    ) -> Optional[bool]:
        if self.operation.kind == '':
            request = StartManipulation.Request()
            request.backend = self.manipulation_backend
            request.waypoint_id = request_id
            request.prepare_action = prepare_action
            request.arrival_task = arrival_task
            self.latest_manipulation_status = None
            self.operation = Operation(
                kind='manipulation',
                request_id=request_id,
                purpose=purpose,
                future=self.manipulation_start_client.call_async(request),
                accepted=False,
                started_at_ns=now.nanoseconds,
                timeout_s=self.plan.manipulation_timeout_s,
            )
            self._enter_safe_stop(f'机械臂任务: {purpose}')
            self.get_logger().info(
                f'开始机械臂任务 id={request_id} prepare={prepare_action} '
                f'task={arrival_task}'
            )
            return None

        if self.operation.kind != 'manipulation':
            self._fail_transport(4402, '状态机内部操作类型冲突')
            return False

        elapsed = (now.nanoseconds - self.operation.started_at_ns) * 1e-9
        if elapsed > self.operation.timeout_s:
            self.message = f'机械臂任务超时: {self.operation.purpose}'
            self._cancel_manipulation(self.message)
            self._clear_operation()
            return False

        if not self.operation.accepted:
            if not self.operation.future.done():
                return None
            try:
                response = self.operation.future.result()
            except Exception as exc:  # noqa: BLE001
                self.message = f'机械臂启动服务异常: {exc}'
                self._clear_operation()
                return False
            if response is None or not bool(response.success):
                self.message = (
                    response.message if response is not None else '机械臂启动服务无响应'
                )
                self._clear_operation()
                return False
            self.operation.accepted = True
            self.operation.future = None
            return None

        status = self.latest_manipulation_status
        if status is None or status.waypoint_id != self.operation.request_id:
            return None
        if status.state == ManipulationStatus.STATE_SUCCEEDED:
            self._clear_operation()
            return True
        if status.state in (
            ManipulationStatus.STATE_FAILED,
            ManipulationStatus.STATE_CANCELLED,
        ):
            self.message = f'机械臂任务失败: {status.message}'
            self._clear_operation()
            return False
        return None

    def _run_classification_with_camera_pose(self, now) -> Optional[bool]:
        """先把相机送到分拣标识观察位姿，再调用分类规则服务"""
        if not self.sorting_pose_ready:
            pose_result = self._run_manipulation_operation(
                request_id=f'run_{self.local_run_id}_sorting_camera_pose',
                prepare_action=self.plan.sorting_prepare_action,
                arrival_task='prepare_only',
                purpose='智能分拣区标识观察位姿',
                now=now,
            )
            if pose_result is True:
                self.sorting_pose_ready = True
                self.arm_safe_ready = False
                self.message = '相机已到达分类标识观察位姿'
                return None
            if pose_result is False:
                self.sorting_pose_ready = False
                self.message = '分类标识观察位姿执行失败'
                return False
            return None
        return self._run_classification_operation(now)

    def _run_classification_operation(self, now) -> Optional[bool]:
        if self.operation.kind == '':
            request = ClassifySortingRule.Request()
            request.request_id = (
                f'run_{self.local_run_id}_sorting_rule_'
                f'{self.classification_retry_count + 1}'
            )
            request.expected_classes = ['gear', 't_bolt']
            self.operation = Operation(
                kind='classification',
                request_id=request.request_id,
                purpose='识别智能分拣区分类规则',
                future=self.sorting_rule_client.call_async(request),
                accepted=True,
                started_at_ns=now.nanoseconds,
                timeout_s=self.plan.classification_timeout_s,
            )
            self._enter_safe_stop('识别智能分拣区分类规则')
            return None

        if self.operation.kind != 'classification':
            self._fail_transport(4403, '状态机内部操作类型冲突')
            return False
        elapsed = (now.nanoseconds - self.operation.started_at_ns) * 1e-9
        if elapsed > self.operation.timeout_s:
            self.message = '分类规则识别服务超时'
            self._clear_operation()
            return False
        if not self.operation.future.done():
            return None
        try:
            response = self.operation.future.result()
        except Exception as exc:  # noqa: BLE001
            self.message = f'分类规则识别服务异常: {exc}'
            self._clear_operation()
            return False
        self._clear_operation()
        if response is None or not bool(response.success):
            self.message = (
                response.message if response is not None else '分类规则识别服务无响应'
            )
            return False
        confidence = float(response.confidence)
        if not math.isfinite(confidence):
            self.message = '分类规则返回非有限置信度'
            return False
        if confidence < self.plan.classification_min_confidence:
            self.message = (
                '分类规则置信度不足: '
                f'{confidence:.3f} < {self.plan.classification_min_confidence:.3f}'
            )
            return False

        park_1 = self._cargo_name(response.park_1_cargo)
        park_2 = self._cargo_name(response.park_2_cargo)
        if park_1 not in ('gear', 't_bolt') or park_2 not in ('gear', 't_bolt'):
            self.message = '分类规则返回未知货物类别'
            return False
        if park_1 == park_2:
            self.message = '分类规则返回两个相同园区类别'
            return False
        self.park_1_cargo = park_1
        self.park_2_cargo = park_2
        self.sorting_rule_confidence = confidence
        self.sorting_rule_confirmed = True
        self.message = response.message
        self.get_logger().info(
            f'分类完成 park_1={park_1} park_2={park_2} confidence={confidence:.3f}'
        )
        return True

    # ------------------------------------------------------------------
    # 任务策略与恢复
    # ------------------------------------------------------------------
    def _select_next_cargo(self) -> Optional[str]:
        if not self.plan.cargo_plan:
            return None
        for _ in range(len(self.plan.cargo_plan) * 2):
            cargo = self.plan.cargo_plan[self.plan_cursor % len(self.plan.cargo_plan)]
            self.plan_cursor += 1
            if self.delivered_counts[cargo] >= self.plan.expected_counts[cargo]:
                continue
            if self.no_target_counts[cargo] >= self.plan.no_target_limit_per_class:
                continue
            return cargo
        return None

    def _park_for_cargo(self, cargo: str) -> str:
        if self.park_1_cargo == cargo:
            return 'park_1'
        if self.park_2_cargo == cargo:
            return 'park_2'
        return ''

    def _place_task_for_current_delivery(self) -> str:
        tasks = self.plan.place_tasks[self.current_park]
        index = min(self.delivered_by_park[self.current_park], len(tasks) - 1)
        return tasks[index]

    def _record_skipped_stage(
        self,
        stage: TransportStage,
        message: str,
    ) -> None:
        """记录可恢复阶段错误；不把整套任务转入失败状态"""
        self.skipped_stage_count += 1
        self.last_skipped_stage = self._stage_name(stage)
        self.message = message
        self.get_logger().warn(
            f'跳过阶段 stage={self.last_skipped_stage} reason={message}'
        )

    def _apply_fallback_mapping(self, reason: str) -> bool:
        """分类失败时按 YAML 中的后备映射继续任务"""
        if not self.plan.use_fallback_mapping_on_failure:
            return False
        park_1 = self.plan.fallback_park_1_cargo
        park_2 = self.plan.fallback_park_2_cargo
        if park_1 not in ('gear', 't_bolt') or park_2 not in ('gear', 't_bolt'):
            return False
        if park_1 == park_2:
            return False
        self.park_1_cargo = park_1
        self.park_2_cargo = park_2
        self.sorting_rule_confidence = 0.0
        self.sorting_rule_confirmed = False
        self.message = (
            f'{reason}; 使用后备映射 park_1={park_1} park_2={park_2}'
        )
        return True

    def _handle_speech_failure(
        self,
        next_stage: TransportStage,
        message: str,
    ) -> None:
        """关键播报先重试；达到上限后跳过播报并继续任务"""
        current_stage = self.transport_stage
        if self.speech_retry_count < self.plan.speech_retry_count:
            self.speech_retry_count += 1
            self._clear_operation()
            self.message = (
                f'{message}; 准备重试 '
                f'{self.speech_retry_count}/{self.plan.speech_retry_count}'
            )
            self._set_stage(current_stage, self.message, force=True)
            return
        self._record_skipped_stage(
            current_stage,
            f'{message}; 已达到重试上限',
        )
        self.speech_retry_count = 0
        self._clear_operation()
        self._set_stage(next_stage, self.message)

    def _handle_navigation_failure(
        self,
        retry_stage: TransportStage,
        next_stage: TransportStage,
        message: str,
        apply_fallback_mapping: bool = False,
    ) -> None:
        """导航失败先重试；达到上限后跳过当前阶段并进入安全后继阶段"""
        if self.navigation_retry_count < self.plan.navigation_retry_count:
            self.navigation_retry_count += 1
            self._clear_operation()
            self.message = (
                f'{message}; 准备重试 '
                f'{self.navigation_retry_count}/{self.plan.navigation_retry_count}'
            )
            self._set_stage(retry_stage, self.message, force=True)
            return
        self.navigation_retry_count = 0
        self._record_skipped_stage(retry_stage, message)
        if apply_fallback_mapping and not self._apply_fallback_mapping(message):
            next_stage = TransportStage.COMPLETE
            self.message = f'{message}; 缺少有效园区映射；后续派送阶段跳过'
        self._clear_operation()
        self._set_stage(next_stage, self.message)

    def _fail_transport(self, code: int, message: str) -> None:
        self._set_error(code, message)
        self._cancel_backends(message)
        self._enter_safe_stop(message)
        self._clear_operation()
        self._set_stage(TransportStage.FAILED, message)

    # ------------------------------------------------------------------
    # 结果上报
    # ------------------------------------------------------------------
    def _begin_result_report(self, done: bool, code: int, message: str) -> None:
        target_state = (
            LifecycleState.REPORTING_DONE
            if done
            else LifecycleState.REPORTING_FAIL
        )
        if self.lifecycle_state == target_state:
            return
        self.message = message
        self.report_future = None
        self.report_accepted = False
        self.report_started_at = self.get_clock().now()
        self._transition_lifecycle(target_state, message)

    def _handle_result_reporting(self, now) -> None:
        self._enter_safe_stop('上报任务结果')
        done = self.lifecycle_state == LifecycleState.REPORTING_DONE
        expected_state = McuStatus.STATE_FINISHED if done else McuStatus.STATE_FAULT
        if self.latest_mcu_status is not None and self.latest_mcu_status.app_state == expected_state:
            self._transition_lifecycle(
                LifecycleState.WAIT_RESET, 'MCU 已确认任务结果'
            )
            return

        elapsed = (now - self.report_started_at).nanoseconds * 1e-9
        if elapsed > self.result_confirm_timeout_s and self.report_accepted:
            self._set_error(4501, 'MCU 未在规定时间内确认任务结果')
            self._transition_lifecycle(
                LifecycleState.WAIT_RESET, self.message
            )
            return

        if self.report_future is None:
            request = ReportMissionResult.Request()
            request.result = (
                ReportMissionResult.Request.RESULT_DONE
                if done
                else ReportMissionResult.Request.RESULT_FAIL
            )
            request.code = int(max(-32768, min(32767, self.error_code if not done else 0)))
            self.report_future = self.result_client.call_async(request)
            self.report_started_at = now
            return

        if not self.report_future.done():
            if elapsed > self.service_response_timeout_s and not self.report_accepted:
                self._set_error(4502, '任务结果上报服务超时')
                self._transition_lifecycle(
                    LifecycleState.WAIT_RESET, self.message
                )
            return

        if not self.report_accepted:
            try:
                response = self.report_future.result()
            except Exception as exc:  # noqa: BLE001
                self._set_error(4503, f'任务结果上报服务异常: {exc}')
                self._transition_lifecycle(
                    LifecycleState.WAIT_RESET, self.message
                )
                return
            if response is None or not bool(response.success):
                self._set_error(
                    4504,
                    response.message if response is not None else '任务结果上报服务无响应',
                )
                self._transition_lifecycle(
                    LifecycleState.WAIT_RESET, self.message
                )
                return
            self.report_accepted = True
            self.report_started_at = now
            self.message = '任务结果已写入 MCU 通信桥，等待 MCU 状态确认'

    # ------------------------------------------------------------------
    # 安全控制
    # ------------------------------------------------------------------
    def _limit_twist(self, source: Twist) -> Twist:
        result = Twist()
        linear_norm = math.hypot(source.linear.x, source.linear.y)
        scale = 1.0
        if linear_norm > self.max_linear_speed_mps > 0.0:
            scale = self.max_linear_speed_mps / linear_norm
        result.linear.x = float(source.linear.x * scale)
        result.linear.y = float(source.linear.y * scale)
        result.linear.z = 0.0
        result.angular.x = 0.0
        result.angular.y = 0.0
        result.angular.z = float(
            max(
                -self.max_angular_speed_rps,
                min(self.max_angular_speed_rps, source.angular.z),
            )
        )
        return result

    def _safety_tick(self, now) -> None:
        if not self.safe_stop_active:
            return
        period = 1.0 / self.zero_velocity_publish_rate_hz
        if (now - self.last_zero_publish_time).nanoseconds * 1e-9 >= period:
            self.cmd_vel_publisher.publish(Twist())
            self.last_zero_publish_time = now
        if (
            self.brake_client.service_is_ready()
            and (now - self.last_brake_request_time).nanoseconds * 1e-9 >= 0.5
        ):
            request = SetBool.Request()
            request.data = True
            self.brake_client.call_async(request)
            self.last_brake_request_time = now

    def _enter_safe_stop(self, reason: str) -> None:
        if not self.safe_stop_active:
            self.get_logger().info(f'进入安全停止: {reason}')
        self.safe_stop_active = True

    def _leave_safe_stop(self, reason: str) -> None:
        if self.safe_stop_active and self.brake_client.service_is_ready():
            request = SetBool.Request()
            request.data = False
            self.brake_client.call_async(request)
        self.safe_stop_active = False
        self.get_logger().info(f'允许导航速度输出: {reason}')

    def _cancel_navigation(self, reason: str) -> None:
        if self.navigation_cancel_client.service_is_ready():
            request = CancelNavigation.Request()
            request.reason = reason
            self.navigation_cancel_client.call_async(request)

    def _cancel_manipulation(self, reason: str) -> None:
        if self.manipulation_cancel_client.service_is_ready():
            request = CancelManipulation.Request()
            request.reason = reason
            self.manipulation_cancel_client.call_async(request)

    def _cancel_backends(self, reason: str) -> None:
        self._cancel_navigation(reason)
        self._cancel_manipulation(reason)

    # ------------------------------------------------------------------
    # MCU 状态与上下文工具
    # ------------------------------------------------------------------
    def _mcu_fresh(self, now) -> bool:
        if self.latest_mcu_status is None:
            return False
        age = (now - self.last_mcu_status_time).nanoseconds * 1e-9
        return 0.0 <= age <= self.mcu_status_timeout_s

    def _asrpro_ready(self, now) -> bool:
        if self.latest_asrpro_status is None:
            return False
        age = (now - self.last_asrpro_status_time).nanoseconds * 1e-9
        return (
            0.0 <= age <= self.plan.asrpro_status_timeout_s
            and bool(self.latest_asrpro_status.serial_connected)
            and bool(self.latest_asrpro_status.device_ready)
            and (
                not self.plan.voice_start_required
                or bool(self.latest_asrpro_status.listen_enabled)
            )
        )

    def _mcu_has_fault(self) -> bool:
        status = self.latest_mcu_status
        return status is not None and (
            status.app_state == McuStatus.STATE_FAULT
            or (status.online_flags & self.ONLINE_HAS_FAULT) != 0
        )

    def _mcu_has_estop(self) -> bool:
        status = self.latest_mcu_status
        return status is not None and (
            status.app_state == McuStatus.STATE_ESTOP
            or (status.online_flags & self.ONLINE_ESTOP) != 0
        )

    def _mcu_is_auto_active(self) -> bool:
        status = self.latest_mcu_status
        return status is not None and (
            status.app_state == McuStatus.STATE_AUTO_PI
            and bool(status.auto_start_latched)
        )

    def _mcu_is_safe_idle(self) -> bool:
        status = self.latest_mcu_status
        if status is None:
            return False
        return (
            not bool(status.auto_start_latched)
            and status.app_state
            not in (
                McuStatus.STATE_AUTO_PI,
                McuStatus.STATE_FAULT,
                McuStatus.STATE_ESTOP,
            )
            and not self._mcu_has_fault()
            and not self._mcu_has_estop()
        )

    def _abort_to_recovery(self, code: int, message: str) -> None:
        self._set_error(code, message)
        self._cancel_backends(message)
        self._enter_safe_stop(message)
        self._clear_operation()
        self.active = False
        self._transition_lifecycle(LifecycleState.RECOVERY_REQUIRED, message)

    def _abort_to_wait_reset(self, code: int, message: str) -> None:
        self._set_error(code, message)
        self._cancel_backends(message)
        self._enter_safe_stop(message)
        self._clear_operation()
        self.active = False
        self._transition_lifecycle(LifecycleState.WAIT_RESET, message)

    def _clear_run_context(self) -> None:
        # 复位后清除所有仅属于单轮比赛的数据，WAIT_START 状态只呈现当前可启动基线
        self.active = False
        self.pending_start = False
        self.pending_reset = False
        self.voice_start_received = False
        self.transition_announcement_sent = False
        self.voice_prompt_sent = False
        self.autonomous_start_announcement_sent = False
        self.completion_announcement_done = False
        self.speech_retry_count = 0
        self.speech_done_phrase_id = ''
        self.speech_failed_phrase_id = ''
        self.sorting_rule_confirmed = False
        self.sorting_rule_confidence = 0.0
        self.park_1_cargo = ''
        self.park_2_cargo = ''
        self.current_cargo = ''
        self.current_park = ''
        self.current_waypoint = ''
        self.holding_cargo = False
        self.arm_safe_ready = False
        self.sorting_pose_ready = False
        self.hard_time_stop = False
        self.delivered_counts = {'gear': 0, 't_bolt': 0}
        self.delivered_by_park = {'park_1': 0, 'park_2': 0}
        self.no_target_counts = {'gear': 0, 't_bolt': 0}
        self.attempted_pick_count = 0
        self.plan_cursor = 0
        self.pick_retry_count = 0
        self.classification_retry_count = 0
        self.navigation_retry_count = 0
        self.skipped_stage_count = 0
        self.last_skipped_stage = ''
        self.transport_stage = TransportStage.IDLE
        self.error_code = 0
        self.message = '等待下一轮全自主运输任务'
        self._clear_operation()
        self.report_future = None
        self.report_accepted = False

    def _clear_operation(self) -> None:
        self.operation.clear()
        self.current_waypoint = ''

    # ------------------------------------------------------------------
    # 状态发布与文本工具
    # ------------------------------------------------------------------
    def _transition_lifecycle(self, state: LifecycleState, reason: str) -> None:
        if self.lifecycle_state != state:
            self.get_logger().info(
                f'生命周期 {self._state_name(self.lifecycle_state)} -> '
                f'{self._state_name(state)}: {reason}'
            )
            self.lifecycle_state = state
            self.state_enter_time = self.get_clock().now()
        self.message = reason

    def _set_stage(
        self, stage: TransportStage, reason: str, force: bool = False
    ) -> None:
        if self.transport_stage != stage or force:
            self.get_logger().info(
                f'运输阶段 {self._stage_name(self.transport_stage)} -> '
                f'{self._stage_name(stage)}: {reason}'
            )
            self.transport_stage = stage
            self.stage_enter_time = self.get_clock().now()
            if stage in (TransportStage.PICK_CARGO, TransportStage.PLACE_CARGO):
                # 抓取与投放会改变机械臂位姿；下一次导航必须重新执行安全收臂确认
                self.arm_safe_ready = False
            if stage == TransportStage.CLASSIFY_SORTING_RULE:
                self.sorting_pose_ready = False
            self._clear_operation()
        self.message = reason

    def _set_error(self, code: int, message: str) -> None:
        self.error_code = int(code)
        self.message = message
        self.get_logger().error(f'error={code}: {message}')

    def _publish_status(self, now) -> None:
        period = 1.0 / self.status_publish_rate_hz
        if (now - self.last_status_publish_time).nanoseconds * 1e-9 < period:
            return
        message = AutonomousTransportStatus()
        message.header.stamp = now.to_msg()
        message.state = int(self.lifecycle_state)
        message.stage = int(self.transport_stage)
        message.local_run_id = int(self.local_run_id)
        message.active = bool(self.active)
        message.mcu_status_fresh = self._mcu_fresh(now)
        message.auto_start_latched = bool(
            self.latest_mcu_status.auto_start_latched
            if self.latest_mcu_status is not None
            else False
        )
        message.asrpro_ready = self._asrpro_ready(now)
        message.asrpro_speech_busy = bool(
            self.latest_asrpro_status.speech_busy
            if self.latest_asrpro_status is not None
            else False
        )
        message.voice_start_received = bool(self.voice_start_received)
        message.sorting_rule_confirmed = bool(self.sorting_rule_confirmed)
        message.sorting_rule_confidence = float(self.sorting_rule_confidence)
        message.park_1_cargo = self.park_1_cargo
        message.park_2_cargo = self.park_2_cargo
        message.current_cargo = self.current_cargo
        message.current_waypoint = self.current_waypoint
        message.delivered_total = int(min(255, self._delivered_total()))
        message.delivered_gear = int(min(255, self.delivered_counts['gear']))
        message.delivered_t_bolt = int(min(255, self.delivered_counts['t_bolt']))
        message.attempted_pick_count = int(min(255, self.attempted_pick_count))
        message.skipped_stage_count = int(min(255, self.skipped_stage_count))
        elapsed = self._elapsed_s(now) if self.active else 0.0
        message.elapsed_s = float(max(0.0, elapsed))
        message.remaining_s = float(
            max(0.0, self.plan.max_autonomous_duration_s - elapsed)
        )
        message.error_code = int(self.error_code)
        message.last_skipped_stage = self.last_skipped_stage
        message.state_name = self._state_name(self.lifecycle_state)
        message.stage_name = self._stage_name(self.transport_stage)
        message.message = self.message
        self.status_publisher.publish(message)
        self.last_status_publish_time = now

    @staticmethod
    def _normalize_phrase(value: str) -> str:
        return ''.join(str(value or '').strip().lower().split()).replace('，', '').replace('。', '')

    @staticmethod
    def _cargo_cn(cargo: str) -> str:
        return '齿轮' if cargo == 'gear' else 'T型螺栓'

    @staticmethod
    def _state_name(state: LifecycleState) -> str:
        return state.name

    @staticmethod
    def _stage_name(stage: TransportStage) -> str:
        return stage.name

    def _elapsed_s(self, now) -> float:
        return max(0.0, (now - self.run_start_time).nanoseconds * 1e-9)

    def _stage_elapsed_s(self, now) -> float:
        return max(0.0, (now - self.stage_enter_time).nanoseconds * 1e-9)

    def _delivered_total(self) -> int:
        return self.delivered_counts['gear'] + self.delivered_counts['t_bolt']

    def destroy_node(self):
        with self._lock:
            self.shutdown_requested = True
            self._cancel_backends('节点关闭')
            self._enter_safe_stop('节点关闭')
            self.cmd_vel_publisher.publish(Twist())
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AutonomousTransportManager()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
