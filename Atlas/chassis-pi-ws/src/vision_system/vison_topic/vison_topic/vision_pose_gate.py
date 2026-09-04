from __future__ import annotations

from typing import Tuple


DEFAULT_READY_TOPIC = "/vision_pose_ready"


def detection_gate_ready_topic(config: dict | None = None) -> str:
    if not config:
        return DEFAULT_READY_TOPIC
    gate = config.get("vision_pose_gate", {})
    return str(gate.get("ready_topic", DEFAULT_READY_TOPIC))


def detection_gate_rejection(
    *,
    require_vision_pose: bool,
    vision_pose_ready: bool,
    ready_topic: str = DEFAULT_READY_TOPIC,
) -> Tuple[bool, str]:
    if not require_vision_pose or vision_pose_ready:
        return True, ""
    return (
        False,
        "机械臂尚未到达合法视觉观察位，拒绝启动检测；请先调用 "
        "/move_to_initial_pose、/move_to_sorting_scan_a 或 /move_to_sorting_scan_b，"
        f"并等待 {ready_topic}=true",
    )
