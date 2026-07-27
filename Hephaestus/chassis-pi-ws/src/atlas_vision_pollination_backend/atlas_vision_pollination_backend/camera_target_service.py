#!/usr/bin/env python3
"""相机目标识别服务节点

本节点提供一次请求一次结果的视觉服务
调用方发送 waypoint_id 和 task_id 后，节点会等待画面稳定，持续扫描直到命中业务逻辑或超时，
然后播报识别结果，并返回一个或多个相机坐标系下的目标点
"""

import json
import os
import re
import subprocess
import threading
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from atlas_mission_interfaces.srv import DetectCameraTarget

# cv2 和 ultralytics 延迟到后台初始化线程里导入
# 这样 launch 启动后 service 能尽快注册，模型加载和 NCNN 预热不会卡住服务创建
cv2 = None
YOLO = None

try:
    from cv_bridge import CvBridge
except Exception:  # pragma: no cover
    CvBridge = None


@dataclass
class CandidateTarget:
    """一次识别得到的候选目标"""

    class_name: str
    x_mm: float
    y_mm: float
    z_mm: float
    score: float
    center_x: int
    center_y: int


class CameraTargetService(Node):
    """视觉识别服务

    该节点不再依赖外部启动话题
    每次服务请求都会独立完成稳定等待，图像处理，语音播报，目标选择和坐标返回
    """

    def __init__(self) -> None:
        super().__init__('atlas_camera_target_service')

        # 服务和话题参数，保持默认接口与任务后端一致
        self.service_name = self.declare_parameter('service_name', '/vision/detect_camera_target').value
        self.debug_image_topic = self.declare_parameter('debug_image_topic', '/vision/debug_image').value
        self.voice_text_topic = self.declare_parameter('voice_text_topic', '/vision/voice_text').value

        # 摄像头和模型参数，实车部署时只需要改配置文件
        self.camera_index = int(self.declare_parameter('camera_index', 0).value)
        self.image_width = int(self.declare_parameter('image_width', 1280).value)
        self.image_height = int(self.declare_parameter('image_height', 720).value)
        self.camera_buffer_size = int(self.declare_parameter('camera_buffer_size', 1).value)
        self.model_path = str(self.declare_parameter('model_path', '').value)
        self.stabilize_delay_s = float(self.declare_parameter('stabilize_delay_s', 1.0).value)
        # max_processing_s 作为旧参数保留，max_scan_s 是当前持续扫描的真实超时时间
        self.max_processing_s = float(self.declare_parameter('max_processing_s', 12.0).value)
        self.max_scan_s = float(self.declare_parameter('max_scan_s', self.max_processing_s).value)
        self.frame_retry_count = int(self.declare_parameter('frame_retry_count', 5).value)
        self.target_real_width_mm = float(self.declare_parameter('target_real_width_mm', 35.0).value)

        # 调试参数：用于判断是 YOLO 没框、业务逻辑不满足，还是 HSV/3D 解算失败
        self.debug_log_interval_s = float(self.declare_parameter('debug_log_interval_s', 0.5).value)
        self.debug_save_on_no_target = bool(self.declare_parameter('debug_save_on_no_target', True).value)
        self.debug_output_dir = str(self.declare_parameter('debug_output_dir', '/tmp/atlas_vision_debug').value)
        self.model_warmup_on_start = bool(self.declare_parameter('model_warmup_on_start', True).value)
        self.model_warmup_frames = int(self.declare_parameter('model_warmup_frames', 1).value)
        # service 会先注册，摄像头/模型/预热在后台线程完成
        # 如果请求到来时资源还没初始化完，最多等待这段时间，避免任务流过早判定视觉失败
        self.service_ready_wait_s = float(self.declare_parameter('service_ready_wait_s', 45.0).value)

        # 语音播报参数，直接模式表示节点内部调用播报命令
        self.voice_mode = str(self.declare_parameter('voice_mode', 'topic').value)
        self.voice_device = str(self.declare_parameter('voice_device', 'plughw:2,0').value)
        self.voice_espeak_voice = str(self.declare_parameter('voice_espeak_voice', 'zh+m1').value)
        self.voice_rate = str(self.declare_parameter('voice_rate', '160').value)
        self.voice_pitch = str(self.declare_parameter('voice_pitch', '70').value)
        self.voice_volume = str(self.declare_parameter('voice_volume', '200').value)
        self.voice_timeout_s = float(self.declare_parameter('voice_timeout_s', 4.0).value)

        # 相机内参，默认使用当前标定结果
        self.camera_matrix = np.array([
            [788.700613, 0.0, 619.294928],
            [0.0, 794.129618, 250.945769],
            [0.0, 0.0, 1.0],
        ], dtype=float)
        self.dist_coeff = np.array([[0.123914, -0.169189, 0.003153, 0.001719, 0.0]], dtype=float)

        # 颜色阈值，后续可以继续改成配置文件读入
        self.lower_yellow_a = np.array([11, 30, 0], dtype=np.uint8)
        self.upper_yellow_a = np.array([67, 255, 255], dtype=np.uint8)
        self.lower_yellow_b = np.array([22, 27, 36], dtype=np.uint8)
        self.upper_yellow_b = np.array([130, 255, 255], dtype=np.uint8)

        self.bridge = CvBridge() if CvBridge is not None else None
        self.model = None
        self.cap = None

        self.debug_pub = self.create_publisher(Image, self.debug_image_topic, 10)
        self.voice_pub = self.create_publisher(String, self.voice_text_topic, 10)
        self.service = self.create_service(DetectCameraTarget, self.service_name, self.on_detect_request)

        self.last_detect_debug: Dict[str, object] = {}
        self.vision_ready_event = threading.Event()
        self.vision_ready_ok = False
        self.vision_ready_message = '视觉资源尚未完成初始化'
        self.init_thread = threading.Thread(target=self.initialize_vision_resources, daemon=True)
        self.init_thread.start()
        self.get_logger().info('视觉服务已注册，摄像头打开、模型加载和预热正在后台进行')

    def initialize_vision_resources(self) -> None:
        """后台初始化视觉资源

        不能在 __init__ 里直接做这些重操作，否则 launch 后 service 注册会被模型加载和 NCNN 预热阻塞
        """
        try:
            self.get_logger().info('视觉资源后台初始化开始: 打开摄像头 -> 加载模型 -> 预热')
            self.open_camera()
            self.load_model()
            if self.model_warmup_on_start:
                self.warmup_model()
            self.vision_ready_ok = self.cap is not None and self.model is not None
            if self.vision_ready_ok:
                self.vision_ready_message = '视觉资源已就绪'
                self.get_logger().info('视觉服务节点已就绪，等待识别请求')
            else:
                self.vision_ready_message = '视觉资源初始化失败，请检查摄像头或模型路径'
                self.get_logger().error(self.vision_ready_message)
        except Exception as exc:
            self.vision_ready_ok = False
            self.vision_ready_message = f'视觉资源初始化异常: {exc}'
            self.get_logger().error(self.vision_ready_message)
        finally:
            self.vision_ready_event.set()

    def import_cv2_if_needed(self) -> bool:
        """延迟导入 OpenCV"""
        global cv2
        if cv2 is not None:
            return True
        try:
            import cv2 as cv2_module
            cv2 = cv2_module
            return True
        except Exception as exc:  # pragma: no cover
            self.get_logger().error(f'无法导入 OpenCV，视觉服务会返回失败: {exc}')
            return False

    def import_yolo_if_needed(self) -> bool:
        """延迟导入 ultralytics.YOLO"""
        global YOLO
        if YOLO is not None:
            return True
        try:
            from ultralytics import YOLO as YOLOClass
            YOLO = YOLOClass
            return True
        except Exception as exc:  # pragma: no cover
            self.get_logger().error(f'无法导入 ultralytics，视觉服务会返回失败: {exc}')
            return False

    def load_model(self) -> None:
        """加载目标检测模型"""
        if not self.import_yolo_if_needed():
            return
        if not self.model_path:
            self.get_logger().error('未配置模型路径，视觉服务会返回失败')
            return
        if not os.path.exists(self.model_path):
            self.get_logger().error(f'模型路径不存在: {self.model_path}')
            return
        try:
            self.model = YOLO(self.model_path)
            self.get_logger().info(f'模型加载完成: {self.model_path}')
        except Exception as exc:
            self.model = None
            self.get_logger().error(f'模型加载失败: {exc}')

    def warmup_model(self) -> None:
        """启动阶段预热一次 NCNN/YOLO，避免第一次任务请求把时间耗在懒加载上"""
        if self.cap is None or self.model is None:
            return
        for _ in range(max(0, self.frame_retry_count)):
            self.cap.read()
        warmed = 0
        for _ in range(max(0, self.model_warmup_frames)):
            frame = self.read_frame()
            if frame is None:
                continue
            try:
                _ = self.model(frame, verbose=False)
                warmed += 1
            except Exception as exc:
                self.get_logger().warn(f'模型预热失败: {exc}')
                break
        if warmed > 0:
            self.get_logger().info(f'模型预热完成，帧数={warmed}')

    def open_camera(self) -> None:
        """打开摄像头"""
        if not self.import_cv2_if_needed():
            return
        self.cap = cv2.VideoCapture(self.camera_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_height)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, self.camera_buffer_size)
        if not self.cap.isOpened():
            self.get_logger().error(f'摄像头打开失败，编号: {self.camera_index}')
            self.cap = None
        else:
            self.get_logger().info(f'摄像头已打开，编号: {self.camera_index}')

    def on_detect_request(self, request: DetectCameraTarget.Request, response: DetectCameraTarget.Response):
        """处理一次视觉识别请求

        与旧的独立脚本保持同一思路：
        不是只识别一两帧，而是在 service 调用期间持续扫描，直到 A/B 业务逻辑命中或超时
        """
        max_targets = int(getattr(request, 'max_targets', 1) or 1)
        max_targets = max(1, min(10, max_targets))
        target_class = str(getattr(request, 'target_class', '') or '')
        self.get_logger().info(
            f'收到视觉识别请求，点位={request.waypoint_id}，任务={request.task_id}，'
            f'最大目标数={max_targets}，目标类别={target_class}，最长扫描={self.max_scan_s:.1f}s'
        )
        if not self.vision_ready_event.is_set():
            self.get_logger().warn(f'视觉资源仍在后台初始化，等待最多 {self.service_ready_wait_s:.1f}s')
            self.vision_ready_event.wait(timeout=max(0.0, self.service_ready_wait_s))
        if not self.vision_ready_ok or self.cap is None or self.model is None:
            response.success = False
            response.message = 'VISION_NOT_READY'
            self.get_logger().error(f'视觉服务未就绪: {self.vision_ready_message}')
            return response

        # 每次请求先清理相机缓存，再等待画面稳定
        self.flush_camera()
        time.sleep(max(0.0, self.stabilize_delay_s))

        start_time = time.time()
        next_log_time = start_time
        frame_count = 0
        last_frame = None
        last_debug_frame = None
        last_voice_text = ''
        self.last_detect_debug = {}

        # 持续扫描直到业务逻辑命中或超时
        while time.time() - start_time <= self.max_scan_s:
            frame = self.read_frame()
            if frame is None:
                time.sleep(0.02)
                continue
            frame_count += 1
            last_frame = frame
            targets, voice_text, debug_frame = self.detect_targets(frame)
            last_debug_frame = debug_frame
            last_voice_text = voice_text or last_voice_text
            self.publish_debug_image(debug_frame)

            now = time.time()
            if self.debug_log_interval_s > 0.0 and now >= next_log_time:
                self.log_scan_debug(request, frame_count, now - start_time)
                next_log_time = now + self.debug_log_interval_s

            if targets:
                if voice_text:
                    self.broadcast_voice(voice_text)
                selected_targets = self.select_targets(targets, max_targets)
                if not selected_targets:
                    break
                response.success = True
                response.message = selected_targets[0].class_name
                response.target_count = len(selected_targets)
                response.targets_camera_m.clear()
                for target in selected_targets:
                    point = response.target_camera_m.__class__()
                    point.x = target.x_mm / 1000.0
                    point.y = target.y_mm / 1000.0
                    point.z = target.z_mm / 1000.0
                    response.targets_camera_m.append(point)
                response.target_camera_m.x = response.targets_camera_m[0].x
                response.target_camera_m.y = response.targets_camera_m[0].y
                response.target_camera_m.z = response.targets_camera_m[0].z
                self.get_logger().info(
                    f'识别成功，扫描帧数={frame_count}，返回目标数={len(selected_targets)}，'
                    f'首个类别={selected_targets[0].class_name}，相机坐标='
                    f'{response.target_camera_m.x:.4f} {response.target_camera_m.y:.4f} '
                    f'{response.target_camera_m.z:.4f} 米'
                )
                return response

            # 命中业务逻辑但没有需要处理的雌花，等价于独立脚本中的“逻辑判定为跳过”
            if voice_text:
                self.broadcast_voice(voice_text)
                response.success = False
                response.message = 'NO_TARGET'
                response.target_count = 0
                self.get_logger().info(
                    f'业务逻辑已命中但无需授粉，扫描帧数={frame_count}，返回 NO_TARGET'
                )
                return response

        # 超时仍未命中业务逻辑，保存最后一帧和统计信息，便于排查类别/数量/HSV 问题
        debug_frame = last_debug_frame if last_debug_frame is not None else last_frame
        if debug_frame is not None:
            self.publish_debug_image(debug_frame)
            self.save_no_target_debug(request, debug_frame, frame_count, time.time() - start_time, last_voice_text)
        response.success = False
        response.message = 'NO_TARGET'
        response.target_count = 0
        self.broadcast_voice('未识别到需要处理的目标')
        self.get_logger().warn(
            f'视觉扫描超时，未命中可处理目标，点位={request.waypoint_id}，任务={request.task_id}，'
            f'扫描帧数={frame_count}，耗时={time.time() - start_time:.2f}s，最后统计={self.last_detect_debug}'
        )
        return response

    def flush_camera(self) -> None:
        """清空摄像头缓存，避免使用缓存画面"""
        if self.cap is None:
            return
        for _ in range(max(1, self.frame_retry_count)):
            self.cap.read()

    def read_frame(self) -> Optional[np.ndarray]:
        """读取一帧图像"""
        if self.cap is None:
            return None
        ok, frame = self.cap.read()
        if not ok:
            return None
        return frame

    def detect_targets(self, frame: np.ndarray) -> Tuple[List[CandidateTarget], str, np.ndarray]:
        """执行检测和业务判断

        返回值里的 voice_text 非空，表示 A/B 业务逻辑已经命中
        targets 非空，表示既命中业务逻辑，又成功解算出需要处理的雌花坐标
        """
        display = frame.copy()
        targets: List[CandidateTarget] = []
        voice_text = ''
        debug: Dict[str, object] = {
            'boxes_total': 0,
            'classes': [],
            'A_count': 0,
            'B_count': 0,
            'logic_hit': False,
            'logic_group': '',
            'female_count': 0,
            'solved_count': 0,
            'reason': 'no_boxes',
        }
        try:
            results = self.model(frame, verbose=False)
            boxes = results[0].boxes
        except Exception as exc:
            debug['reason'] = f'inference_error: {exc}'
            self.last_detect_debug = debug
            self.get_logger().error(f'模型推理失败: {exc}')
            return targets, voice_text, display

        grouped: Dict[str, List[dict]] = {'A': [], 'B': []}
        for box in boxes:
            class_id = int(box.cls[0].item())
            class_name = self.model.names[class_id]
            debug['classes'].append(str(class_name))
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            x1 = max(0, min(x1, frame.shape[1] - 1))
            x2 = max(0, min(x2, frame.shape[1] - 1))
            y1 = max(0, min(y1, frame.shape[0] - 1))
            y2 = max(0, min(y2, frame.shape[0] - 1))
            if x2 <= x1 or y2 <= y1:
                continue
            item = {
                'class': class_name,
                'x1': x1,
                'y1': y1,
                'x2': x2,
                'y2': y2,
                'xc': (x1 + x2) * 0.5,
                'yc': (y1 + y2) * 0.5,
            }
            if class_name in ('A_male', 'A_female'):
                grouped['A'].append(item)
            elif class_name in ('B_male', 'B_female'):
                grouped['B'].append(item)
            color = (0, 0, 255) if 'male' in class_name else (255, 105, 180)
            cv2.rectangle(display, (x1, y1), (x2, y2), color, 2)
            cv2.putText(display, class_name, (x1, max(0, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        debug['boxes_total'] = len(debug['classes'])
        debug['A_count'] = len(grouped['A'])
        debug['B_count'] = len(grouped['B'])
        if debug['boxes_total'] > 0:
            debug['reason'] = 'business_logic_not_hit'

        if len(grouped['A']) == 3:
            grouped['A'].sort(key=lambda item: item['yc'])
            names = ['雌花' if item['class'] == 'A_female' else '雄花' for item in grouped['A']]
            voice_text = '从上到下依次为，' + '，'.join(names)
            females = [item for item in grouped['A'] if item['class'] == 'A_female']
            debug['logic_hit'] = True
            debug['logic_group'] = 'A'
            debug['female_count'] = len(females)
            if 0 < len(females) < 3:
                targets.extend(self.solve_target_list(frame, display, females))
                debug['reason'] = 'A_logic_hit_with_targets' if targets else 'A_hsv_or_3d_solve_failed'
            else:
                debug['reason'] = 'A_logic_hit_skip_all_female_or_no_female'
            debug['solved_count'] = len(targets)
            self.last_detect_debug = debug
            return targets, voice_text, display

        if len(grouped['B']) in (2, 3):
            grouped['B'].sort(key=lambda item: item['xc'])
            names = ['雌花' if item['class'] == 'B_female' else '雄花' for item in grouped['B']]
            voice_text = '从左到右依次为，' + '，'.join(names)
            females = [item for item in grouped['B'] if item['class'] == 'B_female']
            debug['logic_hit'] = True
            debug['logic_group'] = 'B'
            debug['female_count'] = len(females)
            targets.extend(self.solve_target_list(frame, display, females))
            debug['solved_count'] = len(targets)
            debug['reason'] = 'B_logic_hit_with_targets' if targets else 'B_hsv_or_3d_solve_failed_or_no_female'
            self.last_detect_debug = debug
            return targets, voice_text, display

        self.last_detect_debug = debug
        return targets, voice_text, display

    def solve_target_list(self, frame: np.ndarray, display: np.ndarray, items: List[dict]) -> List[CandidateTarget]:
        """计算多个候选目标的相机坐标"""
        solved: List[CandidateTarget] = []
        for item in items:
            result = self.calculate_3d_xyz(frame, item)
            if result is None:
                continue
            x_mm, y_mm, z_mm, cx, cy = result
            solved.append(CandidateTarget(item['class'], x_mm, y_mm, z_mm, float(z_mm), cx, cy))
            cv2.drawMarker(display, (cx, cy), (0, 255, 0), cv2.MARKER_CROSS, 12, 2)
            cv2.putText(display, f'{x_mm:.0f},{y_mm:.0f},{z_mm:.0f}mm', (cx + 5, cy + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
        return solved

    def calculate_3d_xyz(self, frame: np.ndarray, item: dict) -> Optional[Tuple[float, float, float, int, int]]:
        """根据目标框内的颜色轮廓估计三维坐标，单位为毫米"""
        x1, y1, x2, y2 = item['x1'], item['y1'], item['x2'], item['y2']
        roi = frame[y1:y2, x1:x2]
        h_roi, w_roi = roi.shape[:2]
        if w_roi < 10 or h_roi < 10:
            return None

        new_mtx = self.camera_matrix.copy()
        new_mtx[0, 2] -= x1
        new_mtx[1, 2] -= y1
        map1, map2 = cv2.initUndistortRectifyMap(self.camera_matrix, self.dist_coeff, None, new_mtx, (w_roi, h_roi), cv2.CV_32FC1)
        map1 -= x1
        map2 -= y1
        undistorted = cv2.remap(roi, map1, map2, cv2.INTER_LINEAR)
        hsv = cv2.cvtColor(undistorted, cv2.COLOR_BGR2HSV)

        if item['class'] == 'A_female':
            lower, upper = self.lower_yellow_a, self.upper_yellow_a
        else:
            lower, upper = self.lower_yellow_b, self.upper_yellow_b

        mask = cv2.inRange(hsv, lower, upper)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        largest = max(contours, key=cv2.contourArea)
        if len(largest) < 5:
            return None
        ellipse = cv2.fitEllipse(largest)
        (cx_roi, cy_roi), (axis_a, axis_b), _ = ellipse
        major_axis = max(axis_a, axis_b)
        if major_axis <= 1e-6:
            return None

        fx = float(new_mtx[0, 0])
        fy = float(new_mtx[1, 1])
        cx0 = float(new_mtx[0, 2])
        cy0 = float(new_mtx[1, 2])
        z_mm = fx * self.target_real_width_mm / major_axis
        x_mm = (cx_roi - cx0) * z_mm / fx
        y_mm = (cy_roi - cy0) * z_mm / fy
        return float(x_mm), float(y_mm), float(z_mm), int(cx_roi + x1), int(cy_roi + y1)

    def select_targets(self, targets: List[CandidateTarget], max_targets: int) -> List[CandidateTarget]:
        """选择多个目标，当前按距离从近到远排序"""
        ordered = sorted(targets, key=lambda item: item.z_mm)
        return ordered[:max(1, max_targets)]

    def log_scan_debug(self, request: DetectCameraTarget.Request, frame_count: int, elapsed_s: float) -> None:
        """周期性打印扫描统计，区分模型没框、类别数量不满足、HSV 解算失败等情况"""
        info = self.last_detect_debug or {}
        classes = info.get('classes', [])
        if isinstance(classes, list) and len(classes) > 8:
            classes_text = ','.join(str(x) for x in classes[:8]) + ',...'
        else:
            classes_text = ','.join(str(x) for x in classes)
        self.get_logger().info(
            f'视觉扫描中 point={request.waypoint_id} task={request.task_id} '
            f'frame={frame_count} elapsed={elapsed_s:.1f}s '
            f'boxes={info.get("boxes_total", 0)} A={info.get("A_count", 0)} B={info.get("B_count", 0)} '
            f'logic={info.get("logic_hit", False)} group={info.get("logic_group", "")} '
            f'female={info.get("female_count", 0)} solved={info.get("solved_count", 0)} '
            f'reason={info.get("reason", "")} classes=[{classes_text}]'
        )

    def safe_file_token(self, text: str) -> str:
        """把点位名和任务名转换成可安全写入文件名的字符串"""
        token = re.sub(r'[^A-Za-z0-9_.-]+', '_', text or 'unknown')
        return token[:96] if token else 'unknown'

    def save_no_target_debug(
        self,
        request: DetectCameraTarget.Request,
        frame: np.ndarray,
        frame_count: int,
        elapsed_s: float,
        last_voice_text: str,
    ) -> None:
        """NO_TARGET 时保存最后一帧和统计 JSON，便于和独立脚本结果对比"""
        if not self.debug_save_on_no_target or cv2 is None:
            return
        try:
            os.makedirs(self.debug_output_dir, exist_ok=True)
            stamp = time.strftime('%Y%m%d_%H%M%S')
            prefix = f'{stamp}_{self.safe_file_token(request.waypoint_id)}_{self.safe_file_token(request.task_id)}'
            image_path = os.path.join(self.debug_output_dir, prefix + '.jpg')
            json_path = os.path.join(self.debug_output_dir, prefix + '.json')
            cv2.imwrite(image_path, frame)
            payload = {
                'waypoint_id': str(request.waypoint_id),
                'task_id': str(request.task_id),
                'frame_count': int(frame_count),
                'elapsed_s': float(elapsed_s),
                'last_voice_text': str(last_voice_text),
                'last_detect_debug': self.last_detect_debug,
            }
            with open(json_path, 'w', encoding='utf-8') as f:
                json.dump(payload, f, ensure_ascii=False, indent=2)
            self.get_logger().warn(f'已保存 NO_TARGET 调试文件: {image_path} / {json_path}')
        except Exception as exc:
            self.get_logger().warn(f'保存 NO_TARGET 调试文件失败: {exc}')

    def publish_debug_image(self, frame: np.ndarray) -> None:
        """发布调试图像"""
        if self.bridge is None:
            return
        try:
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))
        except Exception:
            pass

    def broadcast_voice(self, text: str) -> None:
        """播报识别结果，默认发布文本，也可直接调用本机语音命令"""
        msg = String()
        msg.data = text
        self.voice_pub.publish(msg)
        self.get_logger().info(f'语音文本: {text}')
        if self.voice_mode != 'direct':
            return
        try:
            espeak = subprocess.Popen(
                ['espeak', '-v', self.voice_espeak_voice, '-a', self.voice_volume, '-p', self.voice_pitch, '-s', self.voice_rate, text, '--stdout'],
                stdout=subprocess.PIPE,
            )
            aplay = subprocess.Popen(
                ['aplay', '-D', self.voice_device],
                stdin=espeak.stdout,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            if espeak.stdout is not None:
                espeak.stdout.close()
            aplay.communicate(timeout=self.voice_timeout_s)
        except Exception as exc:
            self.get_logger().warn(f'语音播报失败: {exc}')

    def destroy_node(self) -> bool:
        """释放摄像头资源"""
        if self.cap is not None:
            self.cap.release()
            self.cap = None
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraTargetService()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
