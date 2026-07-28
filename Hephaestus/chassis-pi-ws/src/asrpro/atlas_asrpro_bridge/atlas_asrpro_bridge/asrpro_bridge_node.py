#!/usr/bin/env python3
"""ASRPRO TWEN51 USB 串口桥"""

from __future__ import annotations

import queue
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Deque, Dict, Optional

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from atlas_mission_interfaces.msg import AsrproEvent, AsrproStatus
from atlas_mission_interfaces.srv import AsrproSpeak

from .protocol import ProtocolError, ProtocolFrame, decode_frame, encode_frame

try:
    import serial
except ImportError:
    serial = None


@dataclass
class PendingCommand:
    """等待 ASRPRO ACK 的树莓派下行命令"""

    sequence: int
    command: str
    arguments: tuple[str, ...]
    encoded: bytes
    phrase_id: str = ''
    sent_at_monotonic: float = 0.0
    retry_count: int = 0


class AsrproBridgeNode(Node):
    """维护 USB 串口、协议重试、播报服务和识别事件"""

    def __init__(self) -> None:
        super().__init__('atlas_asrpro_bridge')
        self.callback_group = ReentrantCallbackGroup()
        self._lock = threading.RLock()
        self._stop_event = threading.Event()
        self._serial = None
        self._rx_buffer = bytearray()
        self._tx_queue: queue.Queue[PendingCommand] = queue.Queue()
        self._pending: Dict[int, PendingCommand] = {}
        self._seen_event_keys: Deque[tuple[int, str, str]] = deque(maxlen=64)

        self.port = str(self.declare_parameter('port', '/dev/atlas_asrpro').value)
        self.baudrate = int(self.declare_parameter('baudrate', 115200).value)
        self.read_timeout_s = max(
            0.01, float(self.declare_parameter('read_timeout_s', 0.05).value)
        )
        self.reconnect_interval_s = max(
            0.1, float(self.declare_parameter('reconnect_interval_s', 1.0).value)
        )
        self.device_timeout_s = max(
            0.5, float(self.declare_parameter('device_timeout_s', 3.0).value)
        )
        self.heartbeat_period_s = max(
            0.2, float(self.declare_parameter('heartbeat_period_s', 1.0).value)
        )
        self.ack_timeout_s = max(
            0.05, float(self.declare_parameter('ack_timeout_s', 0.30).value)
        )
        self.command_retry_count = max(
            0, int(self.declare_parameter('command_retry_count', 3).value)
        )
        self.max_line_length = max(
            64, int(self.declare_parameter('max_line_length', 256).value)
        )
        self.protocol_version = max(
            1, int(self.declare_parameter('protocol_version', 1).value)
        )
        self.auto_enable_listen = bool(
            self.declare_parameter('auto_enable_listen', True).value
        )
        self.status_rate_hz = max(
            1.0, float(self.declare_parameter('status_rate_hz', 5.0).value)
        )

        self.status_topic = str(
            self.declare_parameter('status_topic', '/atlas/asrpro/status').value
        )
        self.event_topic = str(
            self.declare_parameter('event_topic', '/atlas/asrpro/event').value
        )
        self.recognized_topic = str(
            self.declare_parameter(
                'recognized_topic', '/atlas/asrpro/recognized'
            ).value
        )
        self.speak_service = str(
            self.declare_parameter('speak_service', '/atlas/asrpro/speak').value
        )

        self.serial_connected = False
        self.device_ready = False
        self.speech_busy = False
        self.listen_enabled = False
        self.state = AsrproStatus.STATE_OFFLINE
        self.firmware_version = ''
        self.boot_id = ''
        self.last_intent = ''
        self.status_message = '等待 ASRPRO USB 串口'
        self.reconnect_count = 0
        self.rx_sequence = 0
        self.tx_sequence = 0
        self._next_sequence = 1
        self._last_rx_monotonic = 0.0
        self._last_heartbeat_monotonic = 0.0
        self._last_connect_attempt_monotonic = 0.0

        self.status_publisher = self.create_publisher(
            AsrproStatus, self.status_topic, 10
        )
        self.event_publisher = self.create_publisher(
            AsrproEvent, self.event_topic, 20
        )
        self.recognized_publisher = self.create_publisher(
            String, self.recognized_topic, 10
        )
        self.speak_server = self.create_service(
            AsrproSpeak,
            self.speak_service,
            self._on_speak,
            callback_group=self.callback_group,
        )
        self.status_timer = self.create_timer(
            1.0 / self.status_rate_hz,
            self._publish_status,
            callback_group=self.callback_group,
        )

        self._worker = threading.Thread(
            target=self._serial_worker,
            name='atlas_asrpro_serial',
            daemon=True,
        )
        self._worker.start()
        self.get_logger().info(
            f'ASRPRO 串口桥已启动 port={self.port} baudrate={self.baudrate}'
        )

    def _allocate_sequence(self) -> int:
        with self._lock:
            sequence = self._next_sequence
            self._next_sequence += 1
            if self._next_sequence > 65535:
                self._next_sequence = 1
            self.tx_sequence = sequence
            return sequence

    def _queue_command(
        self,
        command: str,
        *arguments: str,
        phrase_id: str = '',
        sequence: Optional[int] = None,
    ) -> int:
        seq = self._allocate_sequence() if sequence is None else int(sequence)
        encoded = encode_frame(
            'A2P', self.protocol_version, seq, command, *arguments
        )
        self._tx_queue.put(
            PendingCommand(
                sequence=seq,
                command=command.upper(),
                arguments=tuple(arguments),
                encoded=encoded,
                phrase_id=phrase_id,
            )
        )
        return seq

    def _on_speak(self, request, response):
        phrase_id = str(request.phrase_id).strip()
        with self._lock:
            ready = self.serial_connected and self.device_ready
            busy = self.speech_busy
        if not phrase_id:
            response.success = False
            response.accepted = False
            response.sequence = 0
            response.message = 'phrase_id 不能为空'
            return response
        if not ready:
            response.success = False
            response.accepted = False
            response.sequence = 0
            response.message = 'ASRPRO 尚未就绪'
            return response
        if busy:
            response.success = False
            response.accepted = False
            response.sequence = 0
            response.message = 'ASRPRO 正在播报'
            return response
        try:
            sequence = self._queue_command(
                'SPEAK', phrase_id, phrase_id=phrase_id
            )
        except ProtocolError as exc:
            response.success = False
            response.accepted = False
            response.sequence = 0
            response.message = f'播报命令编码失败: {exc}'
            return response
        with self._lock:
            self.speech_busy = True
            self.state = AsrproStatus.STATE_SPEAKING
            self.status_message = f'播报命令已排队 phrase_id={phrase_id}'
        response.success = True
        response.accepted = True
        response.sequence = sequence
        response.message = self.status_message
        return response

    def _serial_worker(self) -> None:
        while not self._stop_event.is_set():
            if self._serial is None:
                self._try_open_serial()
                self._stop_event.wait(0.05)
                continue
            try:
                self._read_serial_once()
                self._drain_tx_queue()
                self._retry_pending_commands()
                self._heartbeat_tick()
                self._device_timeout_tick()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f'ASRPRO 串口循环异常: {exc}')
                self._disconnect(f'串口循环异常: {exc}')
            self._stop_event.wait(0.01)

    def _try_open_serial(self) -> None:
        now = time.monotonic()
        if now - self._last_connect_attempt_monotonic < self.reconnect_interval_s:
            return
        self._last_connect_attempt_monotonic = now
        with self._lock:
            self.state = AsrproStatus.STATE_CONNECTING
            self.status_message = f'正在连接 {self.port}'
        if serial is None:
            with self._lock:
                self.state = AsrproStatus.STATE_ERROR
                self.status_message = '缺少 pyserial 依赖'
            return
        try:
            handle = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.read_timeout_s,
                write_timeout=max(0.1, self.ack_timeout_s),
            )
            handle.reset_input_buffer()
            handle.reset_output_buffer()
        except Exception as exc:  # noqa: BLE001
            with self._lock:
                self.state = AsrproStatus.STATE_OFFLINE
                self.status_message = f'连接失败: {exc}'
            return
        with self._lock:
            self._serial = handle
            self.serial_connected = True
            self.device_ready = False
            self.speech_busy = False
            self.listen_enabled = False
            self._last_rx_monotonic = now
            self.reconnect_count += 1
            self.state = AsrproStatus.STATE_CONNECTING
            self.status_message = '串口已连接；等待 ASRPRO HELLO'
        self.get_logger().info(f'ASRPRO 串口已连接: {self.port}')

    def _clear_tx_queue(self) -> None:
        """清空尚未写入串口的旧命令；避免设备重连后执行失效播报"""
        while True:
            try:
                self._tx_queue.get_nowait()
            except queue.Empty:
                return

    def _disconnect(self, reason: str) -> None:
        with self._lock:
            handle = self._serial
            self._serial = None
            self.serial_connected = False
            self.device_ready = False
            self.speech_busy = False
            self.listen_enabled = False
            self.state = AsrproStatus.STATE_OFFLINE
            self.status_message = reason
            self._pending.clear()
            self._rx_buffer.clear()
        self._clear_tx_queue()
        if handle is not None:
            try:
                handle.close()
            except Exception:
                pass
        self._publish_event(
            AsrproEvent.EVENT_DISCONNECTED,
            0,
            payload=reason,
            message=reason,
        )

    def _read_serial_once(self) -> None:
        handle = self._serial
        if handle is None:
            return
        waiting = int(getattr(handle, 'in_waiting', 0))
        chunk = handle.read(waiting if waiting > 0 else 1)
        if not chunk:
            return
        with self._lock:
            self._last_rx_monotonic = time.monotonic()
        self._rx_buffer.extend(chunk)
        if len(self._rx_buffer) > self.max_line_length * 4:
            self._rx_buffer.clear()
            self._protocol_error('接收缓存超限；已清空')
            return
        while b'\n' in self._rx_buffer:
            line, _, remainder = self._rx_buffer.partition(b'\n')
            self._rx_buffer = bytearray(remainder)
            line = line.rstrip(b'\r')
            if not line:
                continue
            if len(line) > self.max_line_length:
                self._protocol_error('单帧长度超限')
                continue
            try:
                frame = decode_frame(line, expected_direction='P2A')
            except ProtocolError as exc:
                self._protocol_error(str(exc))
                continue
            self._handle_frame(frame)

    def _handle_frame(self, frame: ProtocolFrame) -> None:
        with self._lock:
            self.rx_sequence = frame.sequence
        if frame.version != self.protocol_version:
            self._protocol_error(
                f'协议版本不匹配 local={self.protocol_version} remote={frame.version}'
            )
            return
        command = frame.command
        args = frame.arguments
        if command == 'HELLO':
            firmware = args[0] if len(args) >= 1 else 'unknown'
            boot_id = args[1] if len(args) >= 2 else 'unknown'
            with self._lock:
                boot_changed = bool(self.boot_id and self.boot_id != boot_id)
                self.firmware_version = firmware
                self.boot_id = boot_id
                self.device_ready = True
                self.state = AsrproStatus.STATE_READY
                self.status_message = 'ASRPRO 已就绪'
                if boot_changed:
                    self._pending.clear()
                    self._seen_event_keys.clear()
                    self.speech_busy = False
                    self.listen_enabled = False
            if boot_changed:
                self._clear_tx_queue()
            self._queue_command('HELLO_ACK', boot_id)
            if self.auto_enable_listen:
                self._queue_command('LISTEN', '1')
            self._publish_event(
                AsrproEvent.EVENT_HELLO,
                frame.sequence,
                payload=','.join(args),
                message=self.status_message,
            )
            return
        if command == 'ACK':
            acknowledged = args[0].upper() if args else ''
            pending = None
            with self._lock:
                pending = self._pending.pop(frame.sequence, None)
                if acknowledged == 'LISTEN':
                    self.listen_enabled = True
                if acknowledged == 'SPEAK':
                    self.state = AsrproStatus.STATE_SPEAKING
                    self.speech_busy = True
                self.status_message = f'收到 ACK command={acknowledged}'
            self._publish_event(
                AsrproEvent.EVENT_ACK,
                frame.sequence,
                phrase_id=pending.phrase_id if pending else '',
                payload=acknowledged,
                message=self.status_message,
            )
            return
        if command == 'NACK':
            rejected = args[0].upper() if args else ''
            reason = args[1] if len(args) >= 2 else 'unknown'
            pending = None
            with self._lock:
                pending = self._pending.pop(frame.sequence, None)
                if rejected == 'SPEAK':
                    self.speech_busy = False
                    self.state = AsrproStatus.STATE_READY
                self.status_message = f'收到 NACK command={rejected} reason={reason}'
            self._publish_event(
                AsrproEvent.EVENT_NACK,
                frame.sequence,
                phrase_id=pending.phrase_id if pending else '',
                payload=','.join(args),
                message=self.status_message,
            )
            return
        if command == 'EVENT':
            event_type = args[0].upper() if args else ''
            payload = args[1] if len(args) >= 2 else ''
            self._send_event_ack(frame.sequence, event_type)
            key = (frame.sequence, event_type, payload)
            duplicate = key in self._seen_event_keys
            if not duplicate:
                self._seen_event_keys.append(key)
            if duplicate:
                return
            if event_type == 'ASR':
                intent = payload.strip().lower()
                with self._lock:
                    self.last_intent = intent
                    self.status_message = f'识别到 intent={intent}'
                text = String()
                text.data = intent
                self.recognized_publisher.publish(text)
                self._publish_event(
                    AsrproEvent.EVENT_ASR,
                    frame.sequence,
                    intent=intent,
                    payload=payload,
                    message=self.status_message,
                )
            elif event_type == 'SPEAK_DONE':
                phrase_id = payload.strip()
                with self._lock:
                    self.speech_busy = False
                    self.state = (
                        AsrproStatus.STATE_READY
                        if self.device_ready
                        else AsrproStatus.STATE_CONNECTING
                    )
                    self.status_message = f'播报完成 phrase_id={phrase_id}'
                self._publish_event(
                    AsrproEvent.EVENT_SPEAK_DONE,
                    frame.sequence,
                    phrase_id=phrase_id,
                    payload=payload,
                    message=self.status_message,
                )
            else:
                self._protocol_error(f'未知 EVENT 类型: {event_type}')
            return
        if command == 'PONG':
            # PONG 使用与 PING 相同的 sequence；收到后移除对应心跳等待项
            with self._lock:
                self._pending.pop(frame.sequence, None)
            self._publish_event(
                AsrproEvent.EVENT_PONG,
                frame.sequence,
                payload=','.join(args),
                message='收到 PONG',
            )
            return
        self._protocol_error(f'未知上行命令: {command}')

    def _send_event_ack(self, event_sequence: int, event_type: str) -> None:
        handle = self._serial
        if handle is None:
            return
        try:
            encoded = encode_frame(
                'A2P',
                self.protocol_version,
                int(event_sequence),
                'EVENT_ACK',
                event_type or 'UNKNOWN',
            )
            handle.write(encoded)
            handle.flush()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'EVENT_ACK 发送失败: {exc}')

    def _drain_tx_queue(self) -> None:
        handle = self._serial
        if handle is None:
            return
        for _ in range(8):
            try:
                pending = self._tx_queue.get_nowait()
            except queue.Empty:
                return
            try:
                handle.write(pending.encoded)
                handle.flush()
            except Exception:
                self._tx_queue.put(pending)
                raise
            pending.sent_at_monotonic = time.monotonic()
            with self._lock:
                self._pending[pending.sequence] = pending

    def _retry_pending_commands(self) -> None:
        handle = self._serial
        if handle is None:
            return
        now = time.monotonic()
        expired = []
        with self._lock:
            commands = list(self._pending.values())
        for pending in commands:
            if now - pending.sent_at_monotonic < self.ack_timeout_s:
                continue
            if pending.retry_count >= self.command_retry_count:
                expired.append(pending.sequence)
                if pending.command == 'SPEAK':
                    with self._lock:
                        self.speech_busy = False
                        self.state = AsrproStatus.STATE_READY
                        self.status_message = (
                            f'播报命令 ACK 超时 phrase_id={pending.phrase_id}'
                        )
                    self._publish_event(
                        AsrproEvent.EVENT_NACK,
                        pending.sequence,
                        phrase_id=pending.phrase_id,
                        payload='ACK_TIMEOUT',
                        message=self.status_message,
                    )
                continue
            handle.write(pending.encoded)
            handle.flush()
            pending.retry_count += 1
            pending.sent_at_monotonic = now
        if expired:
            with self._lock:
                for sequence in expired:
                    self._pending.pop(sequence, None)

    def _heartbeat_tick(self) -> None:
        now = time.monotonic()
        if not self.device_ready:
            return
        if now - self._last_heartbeat_monotonic < self.heartbeat_period_s:
            return
        self._last_heartbeat_monotonic = now
        self._queue_command('PING', str(int(now * 1000.0)))

    def _device_timeout_tick(self) -> None:
        with self._lock:
            last_rx = self._last_rx_monotonic
        if last_rx <= 0.0:
            return
        if time.monotonic() - last_rx > self.device_timeout_s:
            self._disconnect('ASRPRO 心跳超时')

    def _protocol_error(self, message: str) -> None:
        with self._lock:
            self.status_message = f'协议错误: {message}'
        self.get_logger().warn(self.status_message)
        self._publish_event(
            AsrproEvent.EVENT_PROTOCOL_ERROR,
            0,
            payload=message,
            message=self.status_message,
        )

    def _publish_event(
        self,
        event: int,
        sequence: int,
        *,
        intent: str = '',
        phrase_id: str = '',
        payload: str = '',
        message: str = '',
    ) -> None:
        event_message = AsrproEvent()
        event_message.header.stamp = self.get_clock().now().to_msg()
        event_message.event = int(event)
        event_message.sequence = int(sequence)
        event_message.intent = intent
        event_message.phrase_id = phrase_id
        event_message.payload = payload
        event_message.message = message
        self.event_publisher.publish(event_message)

    def _publish_status(self) -> None:
        with self._lock:
            message = AsrproStatus()
            message.header.stamp = self.get_clock().now().to_msg()
            message.state = int(self.state)
            message.serial_connected = bool(self.serial_connected)
            message.device_ready = bool(self.device_ready)
            message.speech_busy = bool(self.speech_busy)
            message.listen_enabled = bool(self.listen_enabled)
            message.rx_sequence = int(self.rx_sequence)
            message.tx_sequence = int(self.tx_sequence)
            message.reconnect_count = int(self.reconnect_count)
            message.firmware_version = self.firmware_version
            message.boot_id = self.boot_id
            message.last_intent = self.last_intent
            message.message = self.status_message
        self.status_publisher.publish(message)

    def destroy_node(self):
        self._stop_event.set()
        if self._worker.is_alive():
            self._worker.join(timeout=1.0)
        self._disconnect('节点关闭')
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AsrproBridgeNode()
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
