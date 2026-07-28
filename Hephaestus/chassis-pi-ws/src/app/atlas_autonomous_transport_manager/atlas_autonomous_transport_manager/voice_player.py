#!/usr/bin/env python3
"""把 Atlas 语音文本话题转换为本机扬声器播报。"""

from __future__ import annotations

import queue
import subprocess
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class VoicePlayer(Node):
    """串行播放语音文本，避免多个播报进程同时占用声卡。"""

    def __init__(self) -> None:
        super().__init__('atlas_voice_player')
        self.text_topic = str(
            self.declare_parameter('text_topic', '/atlas/voice/text').value
        )
        self.audio_device = str(
            self.declare_parameter('audio_device', 'default').value
        )
        self.voice = str(self.declare_parameter('voice', 'zh+m1').value)
        self.rate = str(self.declare_parameter('rate', '160').value)
        self.pitch = str(self.declare_parameter('pitch', '70').value)
        self.volume = str(self.declare_parameter('volume', '200').value)
        self.command_timeout_s = max(
            1.0, float(self.declare_parameter('command_timeout_s', 8.0).value)
        )
        self.queue: queue.Queue[str] = queue.Queue(maxsize=20)
        self.stop_event = threading.Event()
        self.worker = threading.Thread(target=self._worker, daemon=True)
        self.create_subscription(String, self.text_topic, self._on_text, 10)
        self.worker.start()
        self.get_logger().info(
            f'语音播报节点已启动 topic={self.text_topic} device={self.audio_device}'
        )

    def _on_text(self, message: String) -> None:
        text = str(message.data or '').strip()
        if not text:
            return
        try:
            self.queue.put_nowait(text)
        except queue.Full:
            self.get_logger().warning('语音队列已满，忽略本次播报')

    def _worker(self) -> None:
        while not self.stop_event.is_set():
            try:
                text = self.queue.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                espeak = subprocess.Popen(
                    [
                        'espeak', '-v', self.voice, '-a', self.volume,
                        '-p', self.pitch, '-s', self.rate, text, '--stdout',
                    ],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.DEVNULL,
                )
                aplay = subprocess.Popen(
                    ['aplay', '-D', self.audio_device],
                    stdin=espeak.stdout,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
                if espeak.stdout is not None:
                    espeak.stdout.close()
                aplay.communicate(timeout=self.command_timeout_s)
                espeak.wait(timeout=1.0)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f'语音播报失败: {exc}')
            finally:
                self.queue.task_done()

    def destroy_node(self):
        self.stop_event.set()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VoicePlayer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
