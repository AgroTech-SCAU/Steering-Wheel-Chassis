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
import math
import os
import threading
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

from vison_topic_interfaces.srv import VisionDetect

# ============================================================
# 硬编码配置（不再依赖 config.yaml）
# ============================================================

CONFIG = {
    "camera": {
        "width": 640,
        "height": 640,
        "fps": 30,
    },
    "yolo": {
        "conf_threshold": 0.70,
        "class_names": ["luosi", "chilun"],
    },
    "service": {
        "name": "vision_detect",
        "topic_name": "vision_detections",
        "rate_hz": 15,
    },
    "optimization": {
        "onnx_threads": 2,
        "process_every_n": 10,
        "publish_rate_hz": 0,
    },
    "display": {
        "show_preview": True,
        "print_coords": False,
    },
}

NMS_IOU_THRESHOLD = 0.5


# ============================================================
# 数据类
# ============================================================

@dataclass
class Detection:
    """像素坐标系下的单个检测结果"""
    cls_id: int
    cls_name: str
    u: float            # 检测框底部中心 u (像素)
    v: float            # 检测框底部中心 v (像素)
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
            cv2.putText(frame, f"u:{d.u:.0f} v:{d.v:.0f}px",
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
        from std_msgs.msg import Float32MultiArray

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
        if self._show_preview:
            cv2.namedWindow("Vision Detection", cv2.WINDOW_NORMAL)
        self._cap = self._init_camera(camera_id, cam_cfg)

        # ── 状态 ──
        self._lock = threading.Lock()
        self._detection_timer = None
        self._running = False
        self._latest_detections: List[Detection] = []
        self._latest_infer_ms: float = 0.0
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

        # ── ROS2 ──
        if not rclpy.ok():
            rclpy.init(args=None)
        self._node = Node("vision_detect_server")

        topic_name = CONFIG["service"]["topic_name"]
        self._pub = self._node.create_publisher(Float32MultiArray, topic_name, 10)
        self._pub_msg = Float32MultiArray()

        service_name = CONFIG["service"]["name"]
        self._srv = self._node.create_service(
            VisionDetect, service_name, self._on_detect_request,
        )

        self._node.get_logger().info(f"服务就绪: /{service_name}")
        self._node.get_logger().info(f"话题就绪: /{topic_name}")
        self._node.get_logger().info("等待指令 (start=true 开始, start=false 停止)...")

    # ── 摄像头 ──────────────────────────────────────────────

    def _init_camera(self, camera_id: int, cam_cfg: Dict) -> cv2.VideoCapture:
        cap = cv2.VideoCapture(camera_id)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, cam_cfg["width"])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cam_cfg["height"])
        cap.set(cv2.CAP_PROP_FPS, cam_cfg.get("fps", 30))
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not cap.isOpened():
            raise RuntimeError(f"无法打开摄像头 #{camera_id}")

        w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        print(f"[INFO] 摄像头 #{camera_id}: {w:.0f}x{h:.0f}")
        return cap

    # ── 定时器控制 ──────────────────────────────────────────

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
            return

        self._camera_fail_count = 0

        # 2. 跳帧
        self._skip_counter += 1
        self._total_frames += 1
        if self._skip_counter % self._process_every_n != 0:
            return

        # 3. 推理
        detections_uv, infer_ms = run_inference(
            self._model, frame, self._conf_threshold,
        )

        # 4. 构建检测结果（过滤 NaN）
        detections: List[Detection] = []
        if len(detections_uv) > 0:
            for det in detections_uv:
                cls_id = int(det[5])
                x1, y1, x2, y2 = det[:4]
                conf = float(det[4])
                if not (np.isfinite(x1) and np.isfinite(y1)
                        and np.isfinite(x2) and np.isfinite(y2)
                        and np.isfinite(conf)):
                    continue
                foot_u = (x1 + x2) * 0.5
                foot_v = y2
                name = (
                    self._class_names[cls_id]
                    if cls_id < len(self._class_names)
                    else f"cls_{cls_id}"
                )
                detections.append(Detection(
                    cls_id, name, foot_u, foot_v, x1, y1, x2, y2, conf,
                ))

        # 5. 缓存
        with self._lock:
            self._latest_detections = detections
            self._latest_infer_ms = infer_ms
            self._total_detections += len(detections)

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
                td = self._total_detections  # 已在 line 514 累加过
            self._node.get_logger().info(
                f"[运行中] FPS:{self._fps:.0f}  总帧:{self._total_frames}  "
                f"当前:{len(detections)}个  推理:{infer_ms:.0f}ms  "
                f"内存:{_mem_mb():.0f}MB"
            )

    # ── 服务回调 ────────────────────────────────────────────

    def _on_detect_request(self, request, response):
        """服务 /vision_detect: start=true 开启, start=false 停止并返回结果"""
        if request.start:
            with self._lock:
                was_running = self._running
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
            latest = list(self._latest_detections)
            infer_ms = self._latest_infer_ms
            total_frames = self._total_frames
            total_dets = self._total_detections

        # 过滤 NaN/inf（最后防线）
        safe = [d for d in latest if _is_finite_detection(d)]

        # try-except 兜底：防止极端情况导致节点崩溃
        try:
            response.count = len(safe)
            response.cls_ids = [d.cls_id for d in safe]
            response.u_px = [d.u for d in safe]
            response.v_px = [d.v for d in safe]
            response.cls_names = [d.cls_name for d in safe]
            response.success = True
        except (AssertionError, TypeError, ValueError, AttributeError) as e:
            self._node.get_logger().error(f"响应赋值异常: {e}，返回空结果")
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
            f"最后帧 {len(safe)}个 / 推理 {infer_ms:.0f}ms"
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

    server = VisionDetectServer(camera_id=args.camera)
    server.spin()


if __name__ == "__main__":
    main()
