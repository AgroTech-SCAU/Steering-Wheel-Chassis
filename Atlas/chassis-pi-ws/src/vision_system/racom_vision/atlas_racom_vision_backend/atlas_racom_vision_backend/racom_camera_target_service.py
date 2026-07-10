"""
racom_vision 到 Atlas DetectCameraTarget 的适配服务

racom_vision 当前输出类别和像素坐标；Atlas 旧任务链路需要相机坐标点
这里使用可配置的像素比例和默认深度完成临时转换，保证 mission_manager、
vision_pollination_backend 与机械臂动作序列不需要大改
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import List

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time

from atlas_mission_interfaces.srv import DetectCameraTarget
from geometry_msgs.msg import Point
from vison_topic_interfaces.srv import VisionDetect


@dataclass
class PixelTarget:
    """racom_vision 输出的单个像素目标"""

    cls_id: int
    cls_name: str
    u_px: float
    v_px: float


class RacomCameraTargetService(Node):
    """把 VisionDetect 服务包装成 DetectCameraTarget 服务"""

    def __init__(self) -> None:
        super().__init__('atlas_racom_camera_target_service')
        self.service_name = str(self.declare_parameter('service_name', '/vision/detect_camera_target').value)
        self.vision_detect_service = str(self.declare_parameter('vision_detect_service', '/vision_detect').value)
        self.scan_duration_s = float(self.declare_parameter('scan_duration_s', 1.5).value)
        self.service_timeout_s = float(self.declare_parameter('service_timeout_s', 3.0).value)
        self.default_depth_m = float(self.declare_parameter('default_depth_m', 0.30).value)
        self.image_center_u_px = float(self.declare_parameter('image_center_u_px', 320.0).value)
        self.image_center_v_px = float(self.declare_parameter('image_center_v_px', 320.0).value)
        self.pixel_to_meter_x = float(self.declare_parameter('pixel_to_meter_x', 0.00050).value)
        self.pixel_to_meter_y = float(self.declare_parameter('pixel_to_meter_y', 0.00050).value)
        self.accept_empty_target_class = bool(self.declare_parameter('accept_empty_target_class', True).value)
        self.target_order = str(self.declare_parameter('target_order', 'center_first').value)

        # 服务回调内需要等待另一个服务返回，使用 ReentrantCallbackGroup + MultiThreadedExecutor 避免死锁
        self.callback_group = ReentrantCallbackGroup()
        self.vision_client = self.create_client(VisionDetect, self.vision_detect_service, callback_group=self.callback_group)
        self.service = self.create_service(DetectCameraTarget, self.service_name, self.on_detect_camera_target, callback_group=self.callback_group)
        self.get_logger().info(
            f'racom 视觉适配服务已启动 service={self.service_name} detect_service={self.vision_detect_service}'
        )

    def wait_future(self, future, timeout_s: float):
        """在服务回调中短时等待子服务返回"""
        deadline = self.get_clock().now() + Duration(seconds=max(0.05, timeout_s))
        while rclpy.ok() and not future.done():
            time.sleep(0.02)
            if self.get_clock().now() > deadline:
                return None
        return future.result() if future.done() else None

    def call_vision_detect(self, start: bool):
        if not self.vision_client.wait_for_service(timeout_sec=max(0.1, self.service_timeout_s)):
            return None, f'视觉检测服务未就绪: {self.vision_detect_service}'
        req = VisionDetect.Request()
        req.start = bool(start)
        future = self.vision_client.call_async(req)
        try:
            resp = self.wait_future(future, self.service_timeout_s)
        except Exception as exc:  # noqa: BLE001
            return None, f'调用视觉检测服务异常: {exc}'
        if resp is None:
            return None, f'调用视觉检测服务超时 start={start}'
        return resp, ''

    def collect_targets(self) -> tuple[bool, str, List[PixelTarget]]:
        """启动 racom 检测窗口，停止时读取最后一帧目标"""
        start_resp, err = self.call_vision_detect(True)
        if start_resp is None or not bool(start_resp.success):
            msg = err or getattr(start_resp, 'message', '启动视觉检测失败')
            return False, msg, []

        end_time: Time = self.get_clock().now() + Duration(seconds=max(0.0, self.scan_duration_s))
        while rclpy.ok() and self.get_clock().now() < end_time:
            time.sleep(0.03)

        stop_resp, err = self.call_vision_detect(False)
        if stop_resp is None:
            return False, err, []
        if not bool(stop_resp.success):
            return False, stop_resp.message or '停止视觉检测失败', []

        count = int(stop_resp.count)
        targets: List[PixelTarget] = []
        for i in range(count):
            try:
                target = PixelTarget(
                    cls_id=int(stop_resp.cls_ids[i]),
                    cls_name=str(stop_resp.cls_names[i]),
                    u_px=float(stop_resp.u_px[i]),
                    v_px=float(stop_resp.v_px[i]),
                )
            except (IndexError, TypeError, ValueError):
                continue
            if math.isfinite(target.u_px) and math.isfinite(target.v_px):
                targets.append(target)
        return True, stop_resp.message, targets

    def target_matches(self, target: PixelTarget, requested_class: str) -> bool:
        cls = str(requested_class or '').strip().lower()
        if not cls:
            return self.accept_empty_target_class
        name = target.cls_name.strip().lower()
        if cls == name:
            return True
        if cls == str(target.cls_id):
            return True

        # 全自主运输任务使用稳定的标准类别名；模型可继续输出训练时使用的类别名称
        # 归一化只用于类别匹配；不改变服务返回的原始 cls_name 与 cls_id
        cargo_aliases = {
            'gear': {'gear', 'chilun', '齿轮', '1'},
            't_bolt': {'t_bolt', 't-bolt', 'tbolt', 'bolt', 'luosi', '螺栓', 't型螺栓', '0'},
        }
        normalized_cls = cls.replace(' ', '_').replace('-', '_')
        normalized_name = name.replace(' ', '_').replace('-', '_')
        for standard_name, aliases in cargo_aliases.items():
            normalized_aliases = {
                item.strip().lower().replace(' ', '_').replace('-', '_')
                for item in aliases
            }
            if normalized_cls == standard_name and normalized_name in normalized_aliases:
                return True

        # 允许旧配置中的 female_flower 临时映射到任意 racom 类别，避免旧任务 YAML 不改就完全无目标
        # 正式实车时建议把 pollination_actions.yaml 中 target_class 改成 racom 模型的真实类别名
        if cls in ('female_flower', 'flower') and name in ('luosi', 'chilun', 'female_flower', 'flower'):
            return True
        return False

    def order_targets(self, targets: List[PixelTarget]) -> List[PixelTarget]:
        if self.target_order == 'center_first':
            return sorted(
                targets,
                key=lambda t: (t.u_px - self.image_center_u_px) ** 2 + (t.v_px - self.image_center_v_px) ** 2,
            )
        if self.target_order == 'left_to_right':
            return sorted(targets, key=lambda t: t.u_px)
        if self.target_order == 'right_to_left':
            return sorted(targets, key=lambda t: t.u_px, reverse=True)
        return targets

    def pixel_to_camera_point(self, target: PixelTarget) -> Point:
        point = Point()
        point.x = (target.u_px - self.image_center_u_px) * self.pixel_to_meter_x
        point.y = (target.v_px - self.image_center_v_px) * self.pixel_to_meter_y
        point.z = self.default_depth_m
        return point

    def on_detect_camera_target(self, request: DetectCameraTarget.Request, response: DetectCameraTarget.Response):
        ok, message, targets = self.collect_targets()
        if not ok:
            response.success = False
            response.message = message or 'RACOM_VISION_FAILED'
            response.target_count = 0
            response.targets_camera_m = []
            return response

        filtered = [t for t in targets if self.target_matches(t, request.target_class)]
        filtered = self.order_targets(filtered)
        max_targets = int(request.max_targets) if int(request.max_targets) > 0 else 1
        filtered = filtered[:max_targets]

        if not filtered:
            response.success = False
            response.message = 'NO_TARGET'
            response.target_count = 0
            response.targets_camera_m = []
            self.get_logger().info(
                f'RACOM 视觉无匹配目标 waypoint={request.waypoint_id} task={request.task_id} '
                f'target_class={request.target_class} raw_count={len(targets)}'
            )
            return response

        points = [self.pixel_to_camera_point(t) for t in filtered]
        response.success = True
        response.message = f'RACOM_TARGETS count={len(points)} raw={len(targets)} {message}'
        response.target_count = len(points)
        response.target_camera_m = points[0]
        response.targets_camera_m = points
        self.get_logger().info(
            f'RACOM 视觉目标 waypoint={request.waypoint_id} task={request.task_id} '
            f'count={len(points)} first_uv=({filtered[0].u_px:.1f},{filtered[0].v_px:.1f}) '
            f'first_camera=({points[0].x:.4f},{points[0].y:.4f},{points[0].z:.4f})'
        )
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RacomCameraTargetService()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
