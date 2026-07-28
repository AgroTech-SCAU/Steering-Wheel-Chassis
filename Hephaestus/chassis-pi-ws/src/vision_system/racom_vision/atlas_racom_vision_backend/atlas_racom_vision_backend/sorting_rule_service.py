"""智能分拣区分类标识识别服务"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from atlas_mission_interfaces.srv import ClassifySortingRule
from vison_topic_interfaces.srv import VisionDetect


@dataclass(frozen=True)
class RuleDetection:
    """用于规则判断的单个分类标识检测结果"""

    cargo: str
    u_px: float
    v_px: float


class SortingRuleService(Node):
    """根据分类标识的横向位置输出园区 1 与园区 2 的货物映射"""

    def __init__(self) -> None:
        super().__init__('atlas_sorting_rule_service')

        # 对外服务名称；任务状态机只依赖该服务，不直接依赖视觉模型的原始接口
        self.service_name = str(
            self.declare_parameter('service_name', '/vision/classify_sorting_rule').value
        )

        # RACOM 视觉检测服务名称；该服务通过 start=true 开始检测，start=false 返回结果
        self.vision_detect_service = str(
            self.declare_parameter('vision_detect_service', '/vision_detect').value
        )

        # 每次采样让检测节点持续运行的时间；较短时间有利于控制总任务时长，
        # 多次采样由 sample_count 提供稳定性
        self.scan_duration_s = max(
            0.05, float(self.declare_parameter('scan_duration_s', 0.8).value)
        )
        self.sample_count = max(
            1, int(self.declare_parameter('sample_count', 3).value)
        )
        self.service_timeout_s = max(
            0.1, float(self.declare_parameter('service_timeout_s', 3.0).value)
        )

        # 图像左侧标识所对应的园区名称；相机安装方向或停车方向变化时只改配置
        self.left_slot_park = self._normalize_park(
            str(self.declare_parameter('left_slot_park', 'park_1').value)
        )
        self.right_slot_park = self._normalize_park(
            str(self.declare_parameter('right_slot_park', 'park_2').value)
        )

        # 只检测到一种分类标识时，需要用固定图像中心判断该标识位于左槽还是右槽
        # 中心值应与 RACOM 检测输入分辨率一致；640 像素宽图像通常填写 320
        self.image_center_u_px = float(
            self.declare_parameter('image_center_u_px', 320.0).value
        )
        self.center_deadband_px = max(
            0.0, float(self.declare_parameter('center_deadband_px', 20.0).value)
        )

        # 规则中只有齿轮与 T 型螺栓两类；类别别名允许直接适配模型导出的名称
        gear_aliases = self.declare_parameter(
            'gear_aliases', ['gear', 'chilun', '齿轮', '1']
        ).value
        bolt_aliases = self.declare_parameter(
            't_bolt_aliases', ['t_bolt', 't-bolt', 'bolt', 'luosi', '螺栓', '0']
        ).value
        self.alias_to_cargo: Dict[str, str] = {}
        self._register_aliases('gear', gear_aliases)
        self._register_aliases('t_bolt', bolt_aliases)

        # 比赛规则明确分类标识由齿轮和 T 型螺栓构成；正式配置要求同时识别两类
        # allow_complement_inference=true 时；只识别到一个标识可利用互补关系推断另一园区
        # 互补推断仍要求至少一个有效检测结果；正式比赛配置保持 false
        self.allow_complement_inference = bool(
            self.declare_parameter('allow_complement_inference', False).value
        )

        # 服务回调内会等待视觉子服务，因此使用可重入回调组并配合多线程执行器
        self.callback_group = ReentrantCallbackGroup()
        self.vision_client = self.create_client(
            VisionDetect,
            self.vision_detect_service,
            callback_group=self.callback_group,
        )
        self.service = self.create_service(
            ClassifySortingRule,
            self.service_name,
            self.on_classify,
            callback_group=self.callback_group,
        )

        self.get_logger().info(
            '智能分拣区规则识别服务已启动: '
            f'service={self.service_name} vision={self.vision_detect_service} '
            f'left={self.left_slot_park} right={self.right_slot_park}'
        )

    @staticmethod
    def _normalize_token(value: object) -> str:
        """统一类别名称格式，避免大小写、空格和连接符差异影响匹配"""
        return str(value or '').strip().lower().replace(' ', '_').replace('-', '_')

    @staticmethod
    def _normalize_park(value: str) -> str:
        """把园区名称归一为 park_1 或 park_2"""
        token = str(value or '').strip().lower().replace(' ', '_').replace('-', '_')
        if token in ('park1', 'park_1', '1', '园区1', '园区一'):
            return 'park_1'
        if token in ('park2', 'park_2', '2', '园区2', '园区二'):
            return 'park_2'
        return token

    def _register_aliases(self, cargo: str, aliases: Iterable[object]) -> None:
        """登记一个标准货物类别及其模型类别别名"""
        self.alias_to_cargo[self._normalize_token(cargo)] = cargo
        for alias in aliases:
            token = self._normalize_token(alias)
            if token:
                self.alias_to_cargo[token] = cargo

    def _wait_future(self, future, timeout_s: float):
        """在服务回调内等待子服务，并在超时后返回 None"""
        deadline = self.get_clock().now() + Duration(seconds=max(0.05, timeout_s))
        while rclpy.ok() and not future.done():
            time.sleep(0.02)
            if self.get_clock().now() > deadline:
                return None
        return future.result() if future.done() else None

    def _call_vision(self, start: bool):
        """调用 RACOM 检测启停服务"""
        if not self.vision_client.wait_for_service(timeout_sec=self.service_timeout_s):
            return None, f'视觉检测服务未就绪: {self.vision_detect_service}'

        request = VisionDetect.Request()
        request.start = bool(start)
        try:
            response = self._wait_future(
                self.vision_client.call_async(request), self.service_timeout_s
            )
        except Exception as exc:  # noqa: BLE001
            return None, f'视觉检测服务调用异常: {exc}'

        if response is None:
            return None, f'视觉检测服务调用超时 start={start}'
        if not bool(response.success):
            return None, response.message or f'视觉检测服务拒绝 start={start}'
        return response, ''

    def _collect_one_sample(self) -> Tuple[bool, str, List[RuleDetection]]:
        """采集一次分类标识检测结果"""
        start_response, error = self._call_vision(True)
        if start_response is None:
            return False, error, []

        end_time = self.get_clock().now() + Duration(seconds=self.scan_duration_s)
        while rclpy.ok() and self.get_clock().now() < end_time:
            time.sleep(0.02)

        stop_response, error = self._call_vision(False)
        if stop_response is None:
            return False, error, []

        detections: List[RuleDetection] = []
        for index in range(max(0, int(stop_response.count))):
            try:
                class_name = str(stop_response.cls_names[index])
                class_id = int(stop_response.cls_ids[index])
                u_px = float(stop_response.u_px[index])
                v_px = float(stop_response.v_px[index])
            except (IndexError, TypeError, ValueError):
                continue

            if not math.isfinite(u_px) or not math.isfinite(v_px):
                continue

            cargo = self.alias_to_cargo.get(self._normalize_token(class_name))
            if cargo is None:
                cargo = self.alias_to_cargo.get(self._normalize_token(class_id))
            if cargo is None:
                continue

            detections.append(RuleDetection(cargo=cargo, u_px=u_px, v_px=v_px))

        return True, stop_response.message, detections

    @staticmethod
    def _best_per_cargo(detections: Sequence[RuleDetection]) -> Dict[str, RuleDetection]:
        """为每个货物类别保留一个用于规则排序的代表检测"""
        grouped: Dict[str, List[RuleDetection]] = {}
        for item in detections:
            grouped.setdefault(item.cargo, []).append(item)

        result: Dict[str, RuleDetection] = {}
        for cargo, items in grouped.items():
            # 使用中位位置可以降低单次框抖动和重复框对左右排序的影响
            ordered = sorted(items, key=lambda item: item.u_px)
            result[cargo] = ordered[len(ordered) // 2]
        return result

    def _map_slots_to_parks(
        self,
        left_cargo: str,
        right_cargo: str,
    ) -> Tuple[str, str]:
        """把图像左右槽位转换为园区 1 和园区 2 的货物类别"""
        mapping = {
            self.left_slot_park: left_cargo,
            self.right_slot_park: right_cargo,
        }
        return mapping.get('park_1', ''), mapping.get('park_2', '')

    def _resolve_rule(
        self,
        detections: Sequence[RuleDetection],
    ) -> Tuple[bool, str, str, float, str]:
        """根据多次采样结果计算园区映射"""
        representatives = self._best_per_cargo(detections)
        gear = representatives.get('gear')
        t_bolt = representatives.get('t_bolt')

        if gear is not None and t_bolt is not None:
            if abs(gear.u_px - t_bolt.u_px) < 1e-6:
                return False, '', '', 0.0, '两个分类标识的横向位置无法区分'
            left, right = (
                ('gear', 't_bolt')
                if gear.u_px < t_bolt.u_px
                else ('t_bolt', 'gear')
            )
            park_1, park_2 = self._map_slots_to_parks(left, right)
            valid_count = len(detections)
            confidence = min(1.0, 0.55 + 0.10 * valid_count)
            return True, park_1, park_2, confidence, (
                f'完整识别: left={left} right={right} valid={valid_count}'
            )

        if self.allow_complement_inference and (gear is not None or t_bolt is not None):
            detected = gear if gear is not None else t_bolt
            detected_cargo = 'gear' if gear is not None else 't_bolt'
            other_cargo = 't_bolt' if detected_cargo == 'gear' else 'gear'

            # 使用相机标定后的固定图像中心判断左右槽位
            # 检测中心落在死区内时拒绝推断，避免把停车偏差或框抖动解释为有效分类规则
            offset_px = detected.u_px - self.image_center_u_px
            if abs(offset_px) <= self.center_deadband_px:
                return False, '', '', 0.0, (
                    '单类别标识位于图像中心死区，无法可靠判断左右槽位: '
                    f'u={detected.u_px:.1f} center={self.image_center_u_px:.1f}'
                )
            detected_on_left = offset_px < 0.0
            left = detected_cargo if detected_on_left else other_cargo
            right = other_cargo if detected_on_left else detected_cargo
            park_1, park_2 = self._map_slots_to_parks(left, right)
            return True, park_1, park_2, 0.45, (
                f'互补推断: detected={detected_cargo} left={left} right={right}'
            )

        return False, '', '', 0.0, '未识别到 gear 或 t_bolt 分类标识'

    def on_classify(
        self,
        request: ClassifySortingRule.Request,
        response: ClassifySortingRule.Response,
    ) -> ClassifySortingRule.Response:
        """执行多次采样并返回分类规则"""
        expected = {
            self._normalize_token(item)
            for item in request.expected_classes
            if self._normalize_token(item)
        }
        if expected and not {'gear', 't_bolt'}.issubset(
            {self.alias_to_cargo.get(item, item) for item in expected}
        ):
            response.success = False
            response.message = 'expected_classes 必须包含 gear 与 t_bolt'
            return response

        all_detections: List[RuleDetection] = []
        sample_messages: List[str] = []
        for sample_index in range(self.sample_count):
            ok, message, detections = self._collect_one_sample()
            sample_messages.append(
                f'sample={sample_index + 1} ok={ok} count={len(detections)} message={message}'
            )
            if ok:
                all_detections.extend(detections)

        success, park_1, park_2, confidence, result_message = self._resolve_rule(
            all_detections
        )
        if success and (
            park_1 not in ('gear', 't_bolt')
            or park_2 not in ('gear', 't_bolt')
            or park_1 == park_2
        ):
            success = False
            result_message = '规则识别结果不完整或两个园区映射相同'

        response.success = success
        response.park_1_cargo = park_1
        response.park_2_cargo = park_2
        response.confidence = float(max(0.0, min(1.0, confidence)))
        response.message = result_message + '; ' + ' | '.join(sample_messages)

        log_method = self.get_logger().info if success else self.get_logger().warning
        log_method(
            f'智能分拣规则 request={request.request_id} success={success} '
            f'park_1={park_1} park_2={park_2} confidence={confidence:.2f} '
            f'message={result_message}'
        )
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SortingRuleService()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
