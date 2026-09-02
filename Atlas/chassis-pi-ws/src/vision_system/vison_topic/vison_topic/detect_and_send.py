"""
视觉检测服务节点 (ONNX + ROS2 Service)

功能:
  1. USB 摄像头采集 → YOLO ONNX 推理 → 像素坐标检测
  2. ROS2 服务 /vision_detect 控制启停:
     - start=true  → 启动持续检测
     - start=false → 停止检测，返回最新结果
  3. 话题 /vision_detections 实时发布检测数据流

用法:
  ros2 run vison_topic vision_detect_server
  ros2 run vison_topic vision_detect_server --camera 2
"""

from __future__ import annotations

import argparse
import array
import logging
import math
import os
import threading
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

try:
    from .camera_utils import DEFAULT_CAMERA_SETTINGS, open_project_camera
except ImportError:  # 兼容直接运行源码文件
    from camera_utils import DEFAULT_CAMERA_SETTINGS, open_project_camera

from vison_topic_interfaces.srv import VisionDetect
from vison_topic_interfaces.msg import DetectionCenter, DetectionCenterArray

# ============================================================
# 硬编码配置（不再依赖 config.yaml）
# ============================================================

CONFIG = {
    # ── 摄像头 ──
    # ⚠️ 分辨率必须与相机标定一致，否则内参不匹配
    "camera": {
        **DEFAULT_CAMERA_SETTINGS,
        # 采集固定为640x480；YOLO推理阶段单独letterbox到640x640。
        "frame_id": "camera_optical_frame",
    },

    # ── YOLO 模型 ──
    # ONNX 模型自动从 resource/best.onnx 加载 (相对路径，可移植)
    "yolo": {
        "conf_threshold": 0.55,                     # 置信度阈值，低于此值丢弃
        "class_names": ["luosi", "chilun"],          # 类别名，与 ONNX 模型输出顺序一致
        "plane_area_threshold": 1500.0,              # 平面分类: 框面积 > 此值 → 平面1 (z=1.0 近), 否则 → 平面2 (z=2.0 远)
    },

    # ── ROS2 服务/话题 ──
    "service": {
        "name": "vision_detect",                     # 启停检测的服务名
        "topic_name": "vision_detections",           # 实时检测结果话题
        "centers_topic": "detection_centers",        # 逆时针排序+角标签的检测中心点话题
        "rate_hz": 15,                               # 检测定时器频率 (Hz)
    },

    # ── 机械臂初始观察位门禁 ──
    "initial_pose_gate": {
        "required": True,
        "ready_topic": "/initial_pose_ready",
        "move_service": "/move_to_initial_pose",
    },

    # ── 性能优化 ──
    "optimization": {
        "onnx_threads": 2,                           # ONNX 推理线程数
        "process_every_n": 2,                        # 每 N 帧推理一次，降低 CPU 负载
        "publish_rate_hz": 0,                        # 发布频率上限 (0=不限制)
    },

    # ── 显示 ──
    "display": {
        "show_preview": True,                        # 显示 OpenCV 预览窗口
        "print_coords": False,                       # 终端打印坐标
    },

    # ── 边缘过滤 ──
    # 检测框中心距离图像边缘小于此值(px)时丢弃，避免裁剪导致的坐标偏移
    "edge_margin_px": 25,
}

# NMS 交并比阈值 (越高保留越多框)
NMS_IOU_THRESHOLD = 0.5


# ============================================================
# 数据类
# ============================================================

@dataclass
class Detection:
    """像素坐标系下的单个检测结果"""
    cls_id: int
    cls_name: str
    u: float            # 检测框中心 u (像素)
    v: float            # 检测框中心 v (像素)
    x1: float
    y1: float
    x2: float
    y2: float
    conf: float


def _is_finite_detection(d: Detection) -> bool:
    """检测 NaN/inf —— 树莓派 ARM ONNX Runtime 可能输出非法浮点数"""
    return (
        math.isfinite(d.u) and math.isfinite(d.v)
        and math.isfinite(d.x1) and math.isfinite(d.y1)
        and math.isfinite(d.x2) and math.isfinite(d.y2)
        and math.isfinite(d.conf)
    )


# ============================================================
# 逆时针排序 + 角标签
# ============================================================

_log = logging.getLogger(__name__)


def sort_ccw_and_label(detections: List[Detection]) -> List[Tuple[Detection, int]]:
    """
    按中心点逆时针排序，按坐标位置分配角标签。

    返回: [(detection, corner_index), ...]
      corner_index: 0=左上  1=右上  2=右下  3=左下

    通用策略: 取置信度 top 4 → 对每个目标，找到离它最近的"概念角位置"
    (min_u/min_v/max_u/max_v 两两组合) → 分配对应标签 → 按标签排序返回。
    多个目标同时最近同一个角时，按距离优先分配，远的顺延到空缺角。
    """

    # 四个角的概念位置定义 (相对质心, 用象限)
    CORNER_QUADS = [
        (0, lambda du, dv: du < 0 and dv < 0),   # 0=TL: u小 v小
        (1, lambda du, dv: du >= 0 and dv < 0),  # 1=TR: u大 v小
        (2, lambda du, dv: du >= 0 and dv >= 0), # 2=BR: u大 v大
        (3, lambda du, dv: du < 0 and dv >= 0),  # 3=BL: u小 v大
    ]

    CORNER_NAMES = {0: "TL", 1: "TR", 2: "BR", 3: "BL"}

    if not detections:
        return []

    # 取置信度最高的 4 个
    top4 = sorted(detections, key=lambda d: d.conf, reverse=True)[:4]
    n = len(top4)

    if n < len(detections):
        _log.warning("检测到 %d 个目标，仅取 top 4 参与角标签分配", len(detections))
    if n < 4:
        _log.warning("仅 %d 个目标参与角标签分配，缺少的角 /pick_target 选中时将失败", n)

    if n == 1:
        return [(top4[0], 0)]

    # 质心
    cx = sum(d.u for d in top4) / n
    cy = sum(d.v for d in top4) / n

    # 四个概念角位置：用所有点的 min_u/max_u/min_v/max_v 构造
    us = [d.u for d in top4]
    vs = [d.v for d in top4]
    min_u, max_u = min(us), max(us)
    min_v, max_v = min(vs), max(vs)

    # 概念角坐标 (需要至少 2 个不同 u 和 v 才能区分; 重叠时扩大)
    if abs(max_u - min_u) < 1.0:
        max_u = min_u + 1.0
    if abs(max_v - min_v) < 1.0:
        max_v = min_v + 1.0

    corner_positions = {
        0: (min_u, min_v),  # TL
        1: (max_u, min_v),  # TR
        2: (max_u, max_v),  # BR
        3: (min_u, max_v),  # BL
    }

    # 对每个目标，找最近的概念角
    assignments = []  # [(det, corner_index, distance)]

    # 先按象限初次分配，有冲突再按距离解决
    used_corners = set()

    for d in top4:
        du = d.u - cx
        dv = d.v - cy

        # 找到该点落入的象限
        matched = None
        for cidx, check in CORNER_QUADS:
            if check(du, dv):
                matched = cidx
                break

        if matched is not None and matched not in used_corners:
            assignments.append((d, matched))
            used_corners.add(matched)
        else:
            # 象限冲突或不在任何象限 → 找最近的剩余概念角
            best_corner = None
            best_dist = float("inf")
            for cidx in range(4):
                cu, cv = corner_positions[cidx]
                dist = (d.u - cu) ** 2 + (d.v - cv) ** 2
                if dist < best_dist:
                    best_dist = dist
                    best_corner = cidx
            assignments.append((d, best_corner))

    # 解决冲突: 同一角多个目标 → 最近的拿, 远的找剩余空角
    corner_to_dets = {}  # {cidx: [(det, dist), ...]}
    for det, cidx in assignments:
        cu, cv = corner_positions[cidx]
        dist = (det.u - cu) ** 2 + (det.v - cv) ** 2
        corner_to_dets.setdefault(cidx, []).append((det, dist))

    resolved = []  # [(det, corner_index)]
    occupied = set()
    unassigned = []

    # 每个角只保留最近的那个
    for cidx in range(4):
        if cidx in corner_to_dets:
            best = min(corner_to_dets[cidx], key=lambda x: x[1])
            resolved.append((best[0], cidx))
            occupied.add(cidx)
            # 该角多余的目标进入未分配池
            for det, dist in corner_to_dets[cidx]:
                if det is not best[0]:
                    unassigned.append(det)

    # 未分配的依次填到空角
    empty = [c for c in range(4) if c not in occupied]
    for det in unassigned:
        if empty:
            cidx = empty.pop(0)
            resolved.append((det, cidx))
            occupied.add(cidx)

    # 按角标签排序返回
    resolved.sort(key=lambda x: x[1])
    return resolved


# ============================================================
# ONNX 模型加载
# ============================================================

def _package_share_dir() -> str:
    """获取包 install 空间 share 目录，回退到源码 resource/"""
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory("vison_topic")
    except Exception:
        return os.path.normpath(
            os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         "..", "..", "resource"))


def _find_onnx_path() -> str:
    """查找 best.onnx"""
    share = _package_share_dir()
    candidates = [
        os.path.join(share, "resource", "best.onnx"),
        os.path.join(share, "best.onnx"),
    ]
    for p in candidates:
        if os.path.isfile(p):
            return p
    return candidates[0]


def _find_labels_path() -> Optional[str]:
    """查找 best.labels.txt"""
    share = _package_share_dir()
    for name in ("best.labels.txt", "labels.txt"):
        p = os.path.join(share, "resource", name)
        if os.path.isfile(p):
            return p
    return None


def load_onnx_model(onnx_path: str, num_threads: int = 2) -> Dict:
    """加载 ONNX 模型"""
    import onnxruntime as ort

    opts = ort.SessionOptions()
    opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
    opts.intra_op_num_threads = num_threads
    opts.inter_op_num_threads = max(1, num_threads // 2)
    opts.enable_mem_pattern = True
    opts.enable_cpu_mem_arena = True

    session = ort.InferenceSession(onnx_path, opts, providers=["CPUExecutionProvider"])
    input_info = session.get_inputs()[0]
    output_info = session.get_outputs()[0]

    return {
        "session": session,
        "input_name": input_info.name,
        "img_size": input_info.shape[2],
        "num_classes": output_info.shape[2] - 5,
    }


def load_class_names() -> List[str]:
    """加载类别名：优先 labels.txt，回退硬编码"""
    path = _find_labels_path()
    if path:
        with open(path, encoding="utf-8") as f:
            names = [line.strip() for line in f if line.strip()]
        if names:
            return names
    return CONFIG["yolo"]["class_names"]


# ============================================================
# 图像预处理
# ============================================================

def letterbox_resize(
    frame: np.ndarray,
    new_shape: Tuple[int, int] = (640, 640),
    color: Tuple[int, int, int] = (114, 114, 114),
) -> Tuple[np.ndarray, float, int, int]:
    """等比缩放 + 灰边填充"""
    h, w = frame.shape[:2]
    new_w, new_h = new_shape
    r = min(new_w / w, new_h / h)
    resized_w, resized_h = int(round(w * r)), int(round(h * r))
    resized = cv2.resize(frame, (resized_w, resized_h), interpolation=cv2.INTER_LINEAR)

    pad_w = new_w - resized_w
    pad_h = new_h - resized_h
    pad_left = pad_w // 2
    pad_top = pad_h // 2

    padded = cv2.copyMakeBorder(
        resized, pad_top, pad_h - pad_top, pad_left, pad_w - pad_left,
        cv2.BORDER_CONSTANT, value=color,
    )
    return padded, r, pad_left, pad_top


def preprocess_frame(frame: np.ndarray, img_size: int) -> Tuple[np.ndarray, float, int, int]:
    """letterbox → BGR2RGB → HWC2CHW → normalize → add batch"""
    padded, scale, pad_left, pad_top = letterbox_resize(frame, (img_size, img_size))
    blob = padded[:, :, ::-1].transpose(2, 0, 1).astype(np.float32) / 255.0
    return blob[None], scale, pad_left, pad_top


# ============================================================
# ONNX 输出解析
# ============================================================

def parse_output(
    pred_single: np.ndarray,
    frame_h: int, frame_w: int,
    img_size: int,
    num_classes: int,
    conf_thres: float,
    scale: float = 1.0,
    pad_left: int = 0,
    pad_top: int = 0,
) -> np.ndarray:
    """ONNX 输出 → [N, 6] (x1, y1, x2, y2, conf, cls_id)"""
    obj_conf = pred_single[:, 4]
    cls_scores = pred_single[:, 5:5 + num_classes]
    cls_ids = cls_scores.argmax(axis=1)
    total_conf = obj_conf * cls_scores.max(axis=1)

    keep = np.where(total_conf >= conf_thres)[0]
    if len(keep) == 0:
        return np.empty((0, 6), dtype=np.float32)

    total_conf = total_conf[keep]
    cls_ids = cls_ids[keep]
    boxes = pred_single[keep]

    inv_scale = 1.0 / scale if scale > 0 else 1.0
    cx = (boxes[:, 0] - pad_left) * inv_scale
    cy = (boxes[:, 1] - pad_top)  * inv_scale
    bw = boxes[:, 2] * inv_scale
    bh = boxes[:, 3] * inv_scale

    x1 = np.clip(cx - bw / 2, 0, frame_w)
    y1 = np.clip(cy - bh / 2, 0, frame_h)
    x2 = np.clip(cx + bw / 2, 0, frame_w)
    y2 = np.clip(cy + bh / 2, 0, frame_h)

    result = np.stack([x1, y1, x2, y2, total_conf,
                        cls_ids.astype(np.float32)], axis=1)
    # 防御 NaN/inf：ARM ONNX Runtime 可能输出非法浮点数
    valid = np.isfinite(result).all(axis=1)
    return result[valid]


def fast_nms(detections: np.ndarray, iou_thres: float = NMS_IOU_THRESHOLD) -> np.ndarray:
    """OpenCV DNN NMSBoxes"""
    if len(detections) <= 1:
        return detections

    boxes_xywh = detections[:, :4].copy()
    boxes_xywh[:, 2] -= boxes_xywh[:, 0]
    boxes_xywh[:, 3] -= boxes_xywh[:, 1]

    indices = cv2.dnn.NMSBoxes(
        bboxes=boxes_xywh.tolist(),
        scores=detections[:, 4].tolist(),
        score_threshold=0.0,
        nms_threshold=iou_thres,
    )
    if len(indices) == 0:
        return np.empty((0, 6), dtype=np.float32)
    return detections[indices.ravel()]


def run_inference(
    model: Dict, frame: np.ndarray, conf_thres: float,
) -> Tuple[np.ndarray, float]:
    """单帧推理 → (detections [N×6], 耗时 ms)"""
    t0 = time.perf_counter()
    blob, scale, pad_left, pad_top = preprocess_frame(frame, model["img_size"])
    outputs = model["session"].run(None, {model["input_name"]: blob})
    dets = parse_output(
        outputs[0][0], frame.shape[0], frame.shape[1],
        model["img_size"], model["num_classes"], conf_thres,
        scale=scale, pad_left=pad_left, pad_top=pad_top,
    )
    if len(dets) > 1:
        dets = fast_nms(dets)
    return dets, (time.perf_counter() - t0) * 1000.0


# ============================================================
# 显示
# ============================================================

_DETECTION_COLORS = [
    (0, 255, 0), (0, 180, 255), (0, 128, 255), (255, 128, 0),
]


def draw_results(
    frame: np.ndarray,
    detections_uv: np.ndarray,
    detections: List[Detection],
    class_names: List[str],
    infer_ms: float = 0.0,
    fps: float = 0.0,
) -> np.ndarray:
    """绘制检测框 + 标签"""
    for i in range(len(detections_uv)):
        det = detections_uv[i]
        cls_id = int(det[5])
        x1, y1, x2, y2 = map(int, det[:4])
        conf = float(det[4])
        cls_name = class_names[cls_id] if cls_id < len(class_names) else f"cls_{cls_id}"
        color = _DETECTION_COLORS[cls_id % len(_DETECTION_COLORS)]

        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        label = f"{cls_name} {conf:.2f}"
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(frame, (x1, y1 - th - 6), (x1 + tw + 6, y1), color, -1)
        cv2.putText(frame, label, (x1 + 2, y1 - 3),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

        if i < len(detections):
            d = detections[i]
            area = (d.x2 - d.x1) * (d.y2 - d.y1)
            plane = 1 if area > CONFIG["yolo"]["plane_area_threshold"] else 2
            cv2.putText(frame, f"u:{d.u:.0f} v:{d.v:.0f} area:{area:.0f} plane{plane}",
                        (x1, y2 + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)

    cv2.putText(frame, f"FPS:{fps:.0f} infer:{infer_ms:.0f}ms",
                (8, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
    return frame


# ============================================================
# ROS2 视觉检测服务
# ============================================================

class VisionDetectServer:
    """视觉检测 ROS2 服务节点"""

    def __init__(self, camera_id: int = 0):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import Bool, Float32MultiArray

        self._camera_id = camera_id

        # ── ONNX 模型 ──
        onnx_path = _find_onnx_path()
        print(f"[INFO] ONNX 模型: {onnx_path}")
        onnx_threads = CONFIG["optimization"]["onnx_threads"]
        self._model = load_onnx_model(onnx_path, onnx_threads)
        self._class_names = load_class_names()
        print(f"[INFO] 类别: {self._class_names}  "
              f"输入: {self._model['img_size']}x{self._model['img_size']}  "
              f"线程: {onnx_threads}")

        # ── 参数 ──
        self._conf_threshold = CONFIG["yolo"]["conf_threshold"]
        self._print_coords = CONFIG["display"]["print_coords"]
        self._show_preview = CONFIG["display"]["show_preview"]
        self._process_every_n = CONFIG["optimization"]["process_every_n"]
        self._publish_rate_hz = CONFIG["optimization"]["publish_rate_hz"]

        # ── 摄像头 ──
        cam_cfg = CONFIG["camera"]

        # X11 显示检查 (必须在任何 cv2 GUI 调用之前, Qt SIGABRT 无法被 except 捕获)
        if self._show_preview and not _x11_display_available():
            print("[WARN] X11 显示不可用，自动切换到无预览模式")
            self._show_preview = False

        if self._show_preview:
            cv2.namedWindow("Vision Detection", cv2.WINDOW_NORMAL)
        self._cap = self._init_camera(camera_id, cam_cfg)

        # ── 状态 ──
        self._lock = threading.Lock()
        self._detection_timer = None
        self._running = False
        self._latest_detections: List[Detection] = []
        self._latest_infer_ms: float = 0.0
        self._best_detections: List[Detection] = []   # 扫描期间置信度最高的一组
        self._best_score: float = -1.0
        self._best_frame_stamp = None                 # 最佳帧原始ROS时间戳
        self._total_frames: int = 0
        self._total_detections: int = 0
        self._start_time: Optional[float] = None
        self._last_status_time: float = 0.0
        self._last_fps_time: float = 0.0
        self._last_publish_time: float = 0.0
        self._fps: float = 0.0
        self._frame_count: int = 0
        self._skip_counter: int = 0
        self._camera_fail_count: int = 0
        self._require_initial_pose = bool(
            CONFIG["initial_pose_gate"]["required"])
        self._initial_pose_ready = not self._require_initial_pose

        # ── ROS2 ──
        if not rclpy.ok():
            rclpy.init(args=None)
        self._node = Node("vision_detect_server")

        topic_name = CONFIG["service"]["topic_name"]
        self._pub = self._node.create_publisher(Float32MultiArray, topic_name, 10)
        self._pub_msg = Float32MultiArray()

        # /detection_centers: 逆时针排序后的检测中心点 + 角标签 → handeye_bridge
        centers_topic = CONFIG["service"]["centers_topic"]
        self._centers_pub = self._node.create_publisher(
            DetectionCenterArray, centers_topic, 10)

        # bridge使用TRANSIENT_LOCAL发布，视觉节点晚启动也能立即收到最后状态。
        ready_qos = QoSProfile(depth=1)
        ready_qos.reliability = ReliabilityPolicy.RELIABLE
        ready_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        ready_topic = str(CONFIG["initial_pose_gate"]["ready_topic"])
        self._initial_ready_sub = self._node.create_subscription(
            Bool, ready_topic, self._on_initial_pose_ready, ready_qos)

        service_name = CONFIG["service"]["name"]
        self._srv = self._node.create_service(
            VisionDetect, service_name, self._on_detect_request,
        )

        self._node.get_logger().info(f"服务就绪: /{service_name}")
        self._node.get_logger().info(f"话题就绪: /{topic_name}")
        if self._require_initial_pose:
            self._node.get_logger().info(
                f"初始位置门禁已启用: 等待 {ready_topic}=true")
        self._node.get_logger().info("等待指令 (start=true 开始, start=false 停止)...")

    # ── 摄像头 ──────────────────────────────────────────────

    def _init_camera(self, camera_id: int, cam_cfg: Dict) -> cv2.VideoCapture:
        cap, _ = open_project_camera(
            camera_id,
            width=int(cam_cfg["width"]),
            height=int(cam_cfg["height"]),
            fps=float(cam_cfg["fps"]),
            fourcc=str(cam_cfg["fourcc"]),
            buffer_size=int(cam_cfg["buffer_size"]),
            strict_resolution=True,
            log=lambda text: print(f"[INFO] {text}"),
        )
        return cap

    # ── 定时器控制 ──────────────────────────────────────────

    def _on_initial_pose_ready(self, msg) -> None:
        ready = bool(msg.data)
        with self._lock:
            was_ready = self._initial_pose_ready
            self._initial_pose_ready = ready
            running = self._running
        if ready and not was_ready:
            self._node.get_logger().info("机械臂初始位置已就绪，允许启动检测")
        elif not ready and was_ready:
            self._node.get_logger().warn("机械臂离开初始位置，视觉检测门禁已关闭")
        if self._require_initial_pose and not ready and running:
            self._node.get_logger().error("检测期间初始位置失效，自动停止检测")
            self._stop_detection()

    def _start_detection(self) -> None:
        with self._lock:
            if self._running:
                self._node.get_logger().warn("检测已在运行中")
                return
            rate_hz = CONFIG["service"]["rate_hz"]
            self._detection_timer = self._node.create_timer(
                1.0 / rate_hz, self._detection_tick,
            )
            self._running = True
            self._start_time = time.time()
            self._total_frames = 0
            self._total_detections = 0
            self._skip_counter = 0
            self._last_status_time = time.time()
            self._last_publish_time = 0.0
            self._best_detections = []
            self._best_score = -1.0
            self._best_frame_stamp = None
            self._node.get_logger().info(f"━━━ 持续检测已启动 ({rate_hz} Hz) ━━━")

    def _stop_detection(self) -> None:
        with self._lock:
            if not self._running:
                self._node.get_logger().warn("检测未在运行")
                return
            self._node.destroy_timer(self._detection_timer)
            self._detection_timer = None
            self._running = False
            elapsed = time.time() - (self._start_time or time.time())
            self._node.get_logger().info(
                f"━━━ 持续检测已停止 — "
                f"{elapsed:.1f}s / {self._total_frames}帧 / "
                f"{self._total_detections}目标"
            )

    # ── 主循环 ──────────────────────────────────────────────

    def _detection_tick(self) -> None:
        """定时器回调：采集 → 推理 → 发布 → 缓存"""
        now = time.time()

        # 1. 采集
        ret, frame = self._cap.read()
        if not ret:
            self._camera_fail_count += 1
            self._node.get_logger().error(
                f"摄像头采集失败 (连续 {self._camera_fail_count} 次)",
                throttle_duration_sec=2.0,
            )
            if self._camera_fail_count >= 3:
                self._node.get_logger().error("尝试重新初始化摄像头...")
                try:
                    self._cap.release()
                except Exception:
                    pass
                try:
                    self._cap = self._init_camera(self._camera_id, CONFIG["camera"])
                    self._camera_fail_count = 0
                    self._node.get_logger().info("摄像头重新初始化成功")
                except Exception as e:
                    self._node.get_logger().error(f"重连失败: {e}")
                with self._lock:
                    self._latest_detections = []
                    self._best_detections = []
                    self._best_score = -1.0
                    self._best_frame_stamp = None
            return

        self._camera_fail_count = 0
        # USB 相机没有硬件时间戳时，以 read() 返回后的 ROS 时钟作为该帧时间。
        # 配合单帧缓冲和机械臂静止约束，可避免把旧检测与新位姿直接组合。
        frame_stamp = self._node.get_clock().now().to_msg()

        # 2. 跳帧
        self._skip_counter += 1
        self._total_frames += 1
        if self._skip_counter % self._process_every_n != 0:
            return

        # 3. 推理
        detections_uv, infer_ms = run_inference(
            self._model, frame, self._conf_threshold,
        )

        # 4. 构建检测结果（过滤 NaN，显式转为 Python float）
        detections: List[Detection] = []
        if len(detections_uv) > 0:
            for det in detections_uv:
                cls_id = int(det[5])
                # 显式 float() 转换：numpy 标量 → Python 原生 float，避免 ROS2 类型校验失败
                x1 = float(det[0]); y1 = float(det[1])
                x2 = float(det[2]); y2 = float(det[3])
                conf = float(det[4])
                if not (math.isfinite(x1) and math.isfinite(y1)
                        and math.isfinite(x2) and math.isfinite(y2)
                        and math.isfinite(conf)):
                    continue
                foot_u = (x1 + x2) * 0.5
                foot_v = (y1 + y2) * 0.5
                name = (
                    self._class_names[cls_id]
                    if cls_id < len(self._class_names)
                    else f"cls_{cls_id}"
                )
                detections.append(Detection(
                    cls_id, name, foot_u, foot_v, x1, y1, x2, y2, conf,
                ))

        # 4.5 过滤图像边缘检测（框中心距边缘太近 → 裁剪导致坐标不可靠）
        edge_margin = CONFIG["edge_margin_px"]
        frame_h, frame_w = frame.shape[:2]
        edge_filtered = 0
        keep = []
        for d in detections:
            if (edge_margin <= d.u <= frame_w - edge_margin
                    and edge_margin <= d.v <= frame_h - edge_margin):
                keep.append(d)
            else:
                edge_filtered += 1
        if edge_filtered > 0:
            _log.warning(
                "边缘过滤: 丢弃 %d 个靠近图像边缘的检测 (margin=%dpx, "
                "图像=%dx%d)", edge_filtered, edge_margin, frame_w, frame_h)
        detections = keep

        # 5. 缓存（保留置信度之和最高的一组，避免最后一帧为空丢失前面有效数据）
        frame_score = sum(d.conf for d in detections)
        with self._lock:
            self._latest_detections = detections
            self._latest_infer_ms = infer_ms
            self._total_detections += len(detections)
            if detections and frame_score > self._best_score:
                self._best_detections = list(detections)
                self._best_score = frame_score
                self._best_frame_stamp = frame_stamp

        # 6. 话题发布
        if (self._publish_rate_hz <= 0
                or (now - self._last_publish_time) >= 1.0 / self._publish_rate_hz):
            data = array.array("f")
            data.append(float(len(detections)))
            for d in detections:
                data.append(float(d.cls_id))
                data.append(d.u)
                data.append(d.v)
                data.append(d.conf)
            self._pub_msg.data = data
            self._pub.publish(self._pub_msg)
            self._last_publish_time = now

            # 发布实时帧到 DetectionCenterArray → handeye_bridge
            self._publish_centers(detections, frame_stamp, is_final_best=False)

        # 7. FPS
        self._frame_count += 1
        if self._last_fps_time == 0.0:
            self._last_fps_time = now
        elif now - self._last_fps_time >= 1.0:
            self._fps = self._frame_count / (now - self._last_fps_time)
            self._frame_count = 0
            self._last_fps_time = now

        # 8. 预览
        if self._show_preview:
            try:
                out = draw_results(frame, detections_uv, detections,
                                     self._class_names, infer_ms, self._fps)
                cv2.imshow("Vision Detection", out)
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    self._node.get_logger().info("预览窗口关闭 → 停止检测")
                    self._stop_detection()
            except Exception as e:
                self._node.get_logger().warn(f"显示异常: {e}", throttle_duration_sec=5.0)

        # 9. 状态日志
        if now - self._last_status_time >= 5.0:
            self._last_status_time = now
            with self._lock:
                td = self._total_detections  # 已在 _detection_tick #5 缓存区累加 (line ~686)
            self._node.get_logger().info(
                f"[运行中] FPS:{self._fps:.0f}  总帧:{self._total_frames}  "
                f"当前:{len(detections)}个  推理:{infer_ms:.0f}ms  "
                f"内存:{_mem_mb():.0f}MB"
            )

    def _publish_centers(self, detections: List[Detection], frame_stamp,
                         is_final_best: bool) -> None:
        """发布检测中心；最终最佳帧保留原始采集时间戳。"""
        if frame_stamp is None:
            return
        labeled = sort_ccw_and_label(detections)
        centers_msg = DetectionCenterArray()
        centers_msg.header.stamp = frame_stamp
        centers_msg.header.frame_id = str(CONFIG["camera"]["frame_id"])
        centers_msg.is_final_best = bool(is_final_best)
        for d, cidx in labeled:
            dc = DetectionCenter()
            dc.cls_id = int(d.cls_id)
            dc.u = float(d.u)
            dc.v = float(d.v)
            dc.conf = float(d.conf)
            dc.cls_name = str(d.cls_name)
            dc.corner_index = int(cidx)
            centers_msg.detections.append(dc)
        self._centers_pub.publish(centers_msg)

    # ── 服务回调 ────────────────────────────────────────────

    def _on_detect_request(self, request, response):
        """服务 /vision_detect: start=true 开启, start=false 停止并返回结果"""
        if request.start:
            with self._lock:
                was_running = self._running
                initial_ready = self._initial_pose_ready
            if self._require_initial_pose and not initial_ready:
                move_service = str(
                    CONFIG["initial_pose_gate"]["move_service"])
                response.success = False
                response.message = (
                    "机械臂尚未到达初始观察位，拒绝启动检测；请先调用 "
                    f"{move_service} 并等待 /initial_pose_ready=true")
                response.count = 0
                return response
            self._start_detection()
            response.success = True
            response.message = "检测已在运行中" if was_running else "持续检测已启动"
            response.count = 0
            return response

        # ── 停止 ──
        with self._lock:
            was_running = self._running
        self._stop_detection()

        if not was_running:
            response.success = False
            response.message = "检测未在运行，无结果"
            response.count = 0
            return response

        with self._lock:
            best = list(self._best_detections)
            best_frame_stamp = self._best_frame_stamp
            infer_ms = self._latest_infer_ms
            total_frames = self._total_frames
            total_dets = self._total_detections

        # 停止后重发最佳帧。header.stamp仍是原始图像时间，bridge据此复用
        # 实时阶段缓存的对应机械臂位姿，避免与停止时的当前位姿错误配对。
        if best and best_frame_stamp is not None:
            self._publish_centers(
                best, best_frame_stamp, is_final_best=True)
            self._node.get_logger().info(
                "已在 /detection_centers 重发最终最佳帧，可继续通过 /pick_target 选择")

        # 过滤 NaN/inf，同时强制转为纯 Python float（ROS2 不接受 numpy 类型）
        safe_cls_ids = []
        safe_u = []
        safe_v = []
        safe_names = []
        for d in best:
            try:
                u = float(d.u)
                v = float(d.v)
                if not (math.isfinite(u) and math.isfinite(v)):
                    continue
                safe_cls_ids.append(int(d.cls_id))
                safe_u.append(u)
                safe_v.append(v)
                safe_names.append(str(d.cls_name))
            except (TypeError, ValueError, OverflowError) as ex:
                self._node.get_logger().warn(f"跳过异常检测值: u={d.u} v={d.v} err={ex}")
                continue

        # try-except 兜底：防止极端情况导致节点崩溃
        try:
            response.count = len(safe_u)
            response.cls_ids = safe_cls_ids
            response.u_px = safe_u
            response.v_px = safe_v
            response.cls_names = safe_names
            response.success = True
        except (AssertionError, TypeError, ValueError, AttributeError) as e:
            self._node.get_logger().error(
                f"响应赋值异常: {e} "
                f"sample_u={safe_u[:3] if safe_u else 'EMPTY'} "
                f"sample_v={safe_v[:3] if safe_v else 'EMPTY'} "
                f"types=u:{type(safe_u[0]) if safe_u else 'N/A'} "
                f"v:{type(safe_v[0]) if safe_v else 'N/A'}"
            )
            response.count = 0
            response.u_px = []
            response.v_px = []
            response.cls_ids = []
            response.cls_names = []
            response.success = False
            response.message = "内部数据异常，已返回空结果"
            return response

        elapsed = time.time() - (self._start_time or time.time())
        response.message = (
            f"检测已停止 — {elapsed:.1f}s / "
            f"{total_frames}帧 / {total_dets}目标 / "
            f"最佳帧 {len(safe_u)}个 (score={self._best_score:.2f}) / 推理 {infer_ms:.0f}ms"
        )
        self._node.get_logger().info(response.message)
        return response

    # ── 生命周期 ────────────────────────────────────────────

    def spin(self) -> None:
        import rclpy
        try:
            while rclpy.ok():
                rclpy.spin_once(self._node, timeout_sec=0.1)
        except KeyboardInterrupt:
            print("\n[INFO] 收到中断信号")
        finally:
            self.shutdown()

    def shutdown(self) -> None:
        import rclpy
        if self._running:
            self._stop_detection()
        try:
            self._cap.release()
        except Exception:
            pass
        cv2.destroyAllWindows()
        try:
            self._node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
        print("[INFO] 视觉检测服务已退出")


# ============================================================
# 工具
# ============================================================

def _mem_mb() -> float:
    try:
        with open("/proc/self/status") as f:
            for line in f:
                if line.startswith("VmRSS:"):
                    return float(line.split()[1]) / 1024.0
    except Exception:
        pass
    return -1.0


def _x11_display_available() -> bool:
    """验证 X11 显示连通 + 认证是否正常。

    Qt XCB 插件在 X11 认证失败时直接 SIGABRT，Python except 无法捕获。
    必须在任何 cv2 GUI 调用之前做完整验证（含 MIT-MAGIC-COOKIE-1 认证）。
    """
    import subprocess as _sp
    display = os.environ.get("DISPLAY", "")
    if not display:
        return False
    # xset q 需要完整的 X11 认证握手，socket 探测不够（SSH forwarding 端口通但 cookie 错）
    try:
        result = _sp.run(
            ["xset", "q"],
            stdout=_sp.DEVNULL, stderr=_sp.DEVNULL,
            timeout=1.0,
        )
        return result.returncode == 0
    except Exception:
        return False


# ============================================================
# 入口
# ============================================================

def main() -> None:
    parser = argparse.ArgumentParser(description="视觉检测服务 (ONNX)")
    parser.add_argument("--camera", type=int, default=0, help="摄像头 ID")
    parser.add_argument("--no-preview", action="store_true", help="关闭 OpenCV 预览窗口，适合树莓派无桌面运行")
    parser.add_argument("--conf", type=float, default=None, help="置信度阈值，覆盖默认配置")
    parser.add_argument("--process-every-n", type=int, default=None, help="每 N 帧推理一次，树莓派上可调大降低负载")
    parser.add_argument("--rate-hz", type=float, default=None, help="检测定时器频率，覆盖默认配置")
    parser.add_argument("--service-name", type=str, default=None, help="检测服务名，默认 vision_detect")
    parser.add_argument("--topic-name", type=str, default=None, help="检测结果话题名，默认 vision_detections")
    parser.add_argument(
        "--allow-unprepared", action="store_true",
        help="仅相机/算法调试：允许机械臂未到初始位置时检测，完整应用禁止使用")
    args, _ = parser.parse_known_args()

    # 命令行只覆盖运行时需要经常调整的参数，模型路径和类别仍按包内资源查找。
    if args.no_preview:
        CONFIG["display"]["show_preview"] = False
    if args.conf is not None:
        CONFIG["yolo"]["conf_threshold"] = float(args.conf)
    if args.process_every_n is not None:
        CONFIG["optimization"]["process_every_n"] = max(1, int(args.process_every_n))
    if args.rate_hz is not None:
        CONFIG["service"]["rate_hz"] = max(1.0, float(args.rate_hz))
    if args.service_name:
        CONFIG["service"]["name"] = args.service_name
    if args.topic_name:
        CONFIG["service"]["topic_name"] = args.topic_name
    if args.allow_unprepared:
        CONFIG["initial_pose_gate"]["required"] = False

    server = VisionDetectServer(camera_id=args.camera)
    server.spin()


if __name__ == "__main__":
    main()
