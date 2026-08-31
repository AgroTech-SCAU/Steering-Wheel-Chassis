"""项目统一摄像头初始化。

相机内参标定、手眼采样和在线检测必须经过本模块打开摄像头，避免
分辨率、编码格式、帧率、缓存深度或后端不一致造成内参和图像不匹配。
"""

from __future__ import annotations

import sys
from dataclasses import dataclass
from typing import Callable, Dict, Optional, Tuple

import cv2


DEFAULT_CAMERA_SETTINGS: Dict[str, object] = {
    "width": 640,
    "height": 480,
    "fps": 30,
    "fourcc": "MJPG",
    "buffer_size": 1,
}


@dataclass(frozen=True)
class CameraInfo:
    index: int
    width: int
    height: int
    fps: float
    backend: str
    fourcc: str
    buffer_size: int


def _preferred_backend() -> int:
    """Linux/树莓派固定使用V4L2，其他平台使用OpenCV默认后端。"""
    if sys.platform.startswith("linux") and hasattr(cv2, "CAP_V4L2"):
        return cv2.CAP_V4L2
    return cv2.CAP_ANY


def open_project_camera(
    camera_index: int,
    *,
    width: int = int(DEFAULT_CAMERA_SETTINGS["width"]),
    height: int = int(DEFAULT_CAMERA_SETTINGS["height"]),
    fps: float = float(DEFAULT_CAMERA_SETTINGS["fps"]),
    fourcc: str = str(DEFAULT_CAMERA_SETTINGS["fourcc"]),
    buffer_size: int = int(DEFAULT_CAMERA_SETTINGS["buffer_size"]),
    strict_resolution: bool = True,
    warmup_frames: int = 3,
    log: Optional[Callable[[str], None]] = None,
) -> Tuple[cv2.VideoCapture, CameraInfo]:
    """以项目统一参数打开摄像头并读取实际帧做强校验。

    Returns:
        ``(cap, info)``。调用方负责在结束时 ``cap.release()``。

    Raises:
        RuntimeError: 设备打不开、无法读取帧或实际分辨率不一致。
    """
    if width <= 0 or height <= 0 or fps <= 0:
        raise ValueError("相机width/height/fps必须大于0")
    if len(fourcc) != 4:
        raise ValueError("fourcc必须是4个字符，例如MJPG")

    backend = _preferred_backend()
    cap = cv2.VideoCapture(int(camera_index), backend)
    if not cap.isOpened():
        cap.release()
        raise RuntimeError(
            f"无法打开摄像头 #{camera_index} (backend={backend})，请检查设备号、连接和占用状态")

    # 顺序保持一致：先设编码，再设尺寸、帧率和缓存。
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(width))
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(height))
    cap.set(cv2.CAP_PROP_FPS, float(fps))
    cap.set(cv2.CAP_PROP_BUFFERSIZE, int(buffer_size))

    frame = None
    for _ in range(max(1, int(warmup_frames))):
        ok, candidate = cap.read()
        if ok and candidate is not None and candidate.size > 0:
            frame = candidate

    if frame is None:
        cap.release()
        raise RuntimeError(f"摄像头 #{camera_index} 已打开但无法读取有效图像")

    actual_height, actual_width = frame.shape[:2]
    if strict_resolution and (actual_width, actual_height) != (int(width), int(height)):
        cap.release()
        raise RuntimeError(
            f"摄像头实际分辨率 {actual_width}x{actual_height} 与要求的 "
            f"{int(width)}x{int(height)} 不一致")

    try:
        backend_name = cap.getBackendName()
    except Exception:
        backend_name = str(backend)

    actual_fps = float(cap.get(cv2.CAP_PROP_FPS))
    info = CameraInfo(
        index=int(camera_index),
        width=int(actual_width),
        height=int(actual_height),
        fps=actual_fps,
        backend=backend_name,
        fourcc=fourcc,
        buffer_size=int(buffer_size),
    )
    if log is not None:
        log(
            f"摄像头 #{info.index}: {info.width}x{info.height} @ {info.fps:.1f}fps "
            f"backend={info.backend} fourcc={info.fourcc} buffer={info.buffer_size}")
    return cap, info
