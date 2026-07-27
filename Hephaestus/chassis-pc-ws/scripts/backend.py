#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""PySide6/QML backend for the master-slave arm teleoperation console."""

from __future__ import annotations

import json
import math
import os
import struct
import sys
import threading
import time
import traceback
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Sequence

import serial
from dynamixel_sdk import COMM_SUCCESS, GroupSyncRead, PacketHandler, PortHandler
from serial.tools import list_ports

try:
    import crcmod
    import crcmod.predefined
except ImportError as exc:  # pragma: no cover - dependency error shown at startup
    raise SystemExit("缺少依赖 crcmod，请执行：python -m pip install crcmod") from exc

from PySide6.QtCore import (
    Property,
    QAbstractListModel,
    QModelIndex,
    QObject,
    Qt,
    QThread,
    QTimer,
    Signal,
    Slot,
)
from PySide6.QtWidgets import QFileDialog


IS_WINDOWS = os.name == "nt"
IS_POSIX = os.name == "posix"
PLATFORM_NAME = "Windows" if IS_WINDOWS else ("Linux / macOS" if IS_POSIX else sys.platform)

DEFAULT_LEADER_PORT = "" if IS_WINDOWS else "/dev/ttyUSB0"
DEFAULT_MCU_PORT = "" if IS_WINDOWS else "/dev/ttyACM0"
DEFAULT_LEADER_BAUD = 1_000_000
DEFAULT_MCU_BAUD = 921_600
DEFAULT_MASTER_SEND_FREQ_HZ = 50.0
DEFAULT_HEARTBEAT_FREQ_HZ = 1.0
DEFAULT_PRINT_FREQ_HZ = 1.0
DEFAULT_WRITE_TIMEOUT_S = 0.02
DEFAULT_READ_MCU_LOG = True
DEFAULT_MCU_LOG_ENCODING = "utf-8"
DEFAULT_MCU_LOG_PREFIX = "[MCU] "
DEFAULT_MCU_LOG_BUFFER_LIMIT = 8192
DEFAULT_JOINT_IDS = [1, 2, 3, 4, 5]
DEFAULT_GRIPPER_ID = 7
DEFAULT_GRIPPER_OPEN_POS = 2280
DEFAULT_GRIPPER_CLOSED_POS = 1670
DEFAULT_END_SWITCH_THRESHOLD = 0.50
DEFAULT_ENABLE_GRIPPER_TORQUE_ON_START = True
DEFAULT_GRIPPER_CURRENT_LIMIT = 100
DEFAULT_GRIPPER_GOAL_POSITION_ON_START = DEFAULT_GRIPPER_OPEN_POS
DEFAULT_JOINT_SIGNS = [-1.0, 1.0, -1.0, -1.0, 1.0]
DEFAULT_JOINT_OFFSETS_RAD = [3.14, 1.57, 6.2647, 3.14, 3.14]
DEFAULT_TICKS_PER_REV = 4096
DEFAULT_WRAP_TICKS = True
DEFAULT_SIGNED_POSITION = False
DEFAULT_CRC_NAME = "crc-ccitt-false"
DEFAULT_EXCLUSIVE = IS_POSIX
DEFAULT_DRY_RUN = False

END_SWITCH_OPEN = 0
END_SWITCH_CLOSED = 1

SOF = b"\xa5\x5a"
PROTOCOL_VER = 0x01
FLAG_NONE = 0x00
MSG_PC_HEARTBEAT = 0x10
MSG_PC_MASTER_JOINTS = 0x11
PC_MASTER_JOINTS_PAYLOAD_LEN = 25

DXL_PROTOCOL_VERSION = 2.0
ADDR_OPERATING_MODE = 11
ADDR_CURRENT_LIMIT = 38
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132
LEN_PRESENT_POSITION = 4
TORQUE_DISABLE = 0
TORQUE_ENABLE = 1
OPERATING_MODE_CURRENT_POSITION = 5

STARTUP_TIMEOUT_MS = 8000
STOP_INTERRUPT_TIMEOUT_MS = 2500
STOP_TERMINATE_TIMEOUT_MS = 2500

COMMON_BAUDRATES = [
    "9600",
    "19200",
    "38400",
    "57600",
    "115200",
    "230400",
    "460800",
    "500000",
    "576000",
    "921600",
    "1000000",
    "2000000",
    "3000000",
    "4000000",
]


def default_config_map() -> dict[str, Any]:
    return {
        "leader_port": DEFAULT_LEADER_PORT,
        "mcu_port": DEFAULT_MCU_PORT,
        "leader_baud": str(DEFAULT_LEADER_BAUD),
        "mcu_baud": str(DEFAULT_MCU_BAUD),
        "master_send_freq_hz": str(DEFAULT_MASTER_SEND_FREQ_HZ),
        "heartbeat_freq_hz": str(DEFAULT_HEARTBEAT_FREQ_HZ),
        "print_freq_hz": str(DEFAULT_PRINT_FREQ_HZ),
        "write_timeout_s": str(DEFAULT_WRITE_TIMEOUT_S),
        "joint_ids": ",".join(map(str, DEFAULT_JOINT_IDS)),
        "joint_signs": ",".join(f"{value:g}" for value in DEFAULT_JOINT_SIGNS),
        "joint_offsets_rad": ",".join(
            f"{value:g}" for value in DEFAULT_JOINT_OFFSETS_RAD
        ),
        "ticks_per_rev": str(DEFAULT_TICKS_PER_REV),
        "wrap_ticks": DEFAULT_WRAP_TICKS,
        "signed_position": DEFAULT_SIGNED_POSITION,
        "gripper_id": str(DEFAULT_GRIPPER_ID),
        "gripper_open_pos": str(DEFAULT_GRIPPER_OPEN_POS),
        "gripper_closed_pos": str(DEFAULT_GRIPPER_CLOSED_POS),
        "end_switch_threshold": str(DEFAULT_END_SWITCH_THRESHOLD),
        "enable_gripper_torque_on_start": DEFAULT_ENABLE_GRIPPER_TORQUE_ON_START,
        "gripper_current_limit": str(DEFAULT_GRIPPER_CURRENT_LIMIT),
        "gripper_goal_position_on_start": str(
            DEFAULT_GRIPPER_GOAL_POSITION_ON_START
        ),
        "read_mcu_log": DEFAULT_READ_MCU_LOG,
        "mcu_log_encoding": DEFAULT_MCU_LOG_ENCODING,
        "mcu_log_prefix": DEFAULT_MCU_LOG_PREFIX,
        "mcu_log_buffer_limit": str(DEFAULT_MCU_LOG_BUFFER_LIMIT),
        "exclusive": DEFAULT_EXCLUSIVE,
        "dry_run": DEFAULT_DRY_RUN,
        "crc_name": DEFAULT_CRC_NAME,
    }


@dataclass(frozen=True)
class AppConfig:
    leader_port: str
    mcu_port: str
    leader_baud: int
    mcu_baud: int
    master_send_freq_hz: float
    heartbeat_freq_hz: float
    print_freq_hz: float
    write_timeout_s: float
    joint_ids: tuple[int, int, int, int, int]
    joint_signs: tuple[float, float, float, float, float]
    joint_offsets_rad: tuple[float, float, float, float, float]
    ticks_per_rev: int
    wrap_ticks: bool
    signed_position: bool
    gripper_id: int
    gripper_open_pos: int
    gripper_closed_pos: int
    end_switch_threshold: float
    enable_gripper_torque_on_start: bool
    gripper_current_limit: int
    gripper_goal_position_on_start: int
    read_mcu_log: bool
    mcu_log_encoding: str
    mcu_log_prefix: str
    mcu_log_buffer_limit: int
    exclusive: bool
    dry_run: bool
    crc_name: str


@dataclass
class LeaderState:
    q_rad: list[float]
    gripper_raw: int
    gripper_norm: float
    end_switch: int


@dataclass
class RunStats:
    frames_sent: int = 0
    heartbeat_sent: int = 0
    errors: int = 0
    rx_bytes: int = 0
    rx_lines: int = 0


class RateLimiter:
    def __init__(self, rate_hz: float):
        self.enabled = rate_hz > 0.0
        self.period_s = 1.0 / rate_hz if self.enabled else float("inf")
        self.next_t = time.monotonic()

    def ready(self, now: float | None = None) -> bool:
        if not self.enabled:
            return False
        now = time.monotonic() if now is None else now
        if now < self.next_t:
            return False
        missed = max(1, int((now - self.next_t) // self.period_s) + 1)
        self.next_t += missed * self.period_s
        return True


class ProtocolPacker:
    def __init__(self, crc_name: str, version: int = PROTOCOL_VER):
        self.version = version & 0xFF
        self.seq = 0
        try:
            self._crc16 = crcmod.predefined.mkPredefinedCrcFun(crc_name)
        except Exception:
            if crc_name != DEFAULT_CRC_NAME:
                raise
            self._crc16 = crcmod.mkCrcFun(
                0x11021,
                initCrc=0xFFFF,
                rev=False,
                xorOut=0x0000,
            )

    def pack(self, msg_id: int, payload: bytes = b"", flags: int = FLAG_NONE) -> bytes:
        seq = self.seq & 0xFF
        self.seq = (self.seq + 1) & 0xFF
        body = bytes([self.version, msg_id & 0xFF, seq, flags & 0xFF]) + payload
        header = SOF + struct.pack(">H", len(body))
        crc = self._crc16(header + body) & 0xFFFF
        return header + body + struct.pack(">H", crc)

    def pack_heartbeat(self) -> bytes:
        return self.pack(MSG_PC_HEARTBEAT)

    def pack_master_joints(
        self,
        q_rad: Sequence[float],
        end_switch: int,
        stamp_ms: int | None = None,
    ) -> bytes:
        if len(q_rad) != 5:
            raise ValueError(f"需要 5 个关节角，实际得到 {len(q_rad)} 个")
        if end_switch not in (END_SWITCH_OPEN, END_SWITCH_CLOSED):
            raise ValueError(f"end_switch 只能为 0 或 1，实际为 {end_switch}")
        stamp_ms = monotonic_ms_u32() if stamp_ms is None else stamp_ms
        q_urad = [rad_to_urad(q) for q in q_rad]
        payload = struct.pack("<IiiiiiB", stamp_ms & 0xFFFFFFFF, *q_urad, end_switch)
        if len(payload) != PC_MASTER_JOINTS_PAYLOAD_LEN:
            raise RuntimeError(f"payload 长度错误：{len(payload)}")
        return self.pack(MSG_PC_MASTER_JOINTS, payload)


def monotonic_ms_u32() -> int:
    return int(time.monotonic() * 1000.0) & 0xFFFFFFFF


def rad_to_urad(rad: float) -> int:
    value = int(round(rad * 1_000_000.0))
    return max(-(2**31), min(2**31 - 1, value))


def parse_int_list(text: str, expected_len: int) -> list[int]:
    try:
        values = [int(part.strip(), 0) for part in text.split(",") if part.strip()]
    except ValueError as exc:
        raise ValueError(f"必须是逗号分隔的整数：{text}") from exc
    if len(values) != expected_len:
        raise ValueError(f"需要 {expected_len} 个整数，实际得到 {len(values)} 个")
    return values


def parse_float_list(text: str, expected_len: int) -> list[float]:
    try:
        values = [float(part.strip()) for part in text.split(",") if part.strip()]
    except ValueError as exc:
        raise ValueError(f"必须是逗号分隔的数字：{text}") from exc
    if len(values) != expected_len:
        raise ValueError(f"需要 {expected_len} 个数字，实际得到 {len(values)} 个")
    return values


def u32_to_s32(raw: int) -> int:
    raw &= 0xFFFFFFFF
    return raw - 0x100000000 if raw & 0x80000000 else raw


def format_q(q_rad: Sequence[float]) -> str:
    return "[" + ", ".join(f"{q:+.4f}" for q in q_rad) + "]"


@dataclass(frozen=True)
class JointMapping:
    signs: tuple[float, float, float, float, float]
    offsets_rad: tuple[float, float, float, float, float]
    ticks_per_rev: int
    wrap_ticks: bool
    signed_position: bool

    def ticks_to_rad(self, raw: int, index: int) -> float:
        value = u32_to_s32(raw) if self.signed_position else int(raw)
        if self.wrap_ticks:
            value %= self.ticks_per_rev
        rad = value / self.ticks_per_rev * 2.0 * math.pi - math.pi
        return self.signs[index] * rad + self.offsets_rad[index]


class DynamixelLeader:
    def __init__(self, config: AppConfig):
        self.config = config
        self.mapping = JointMapping(
            signs=config.joint_signs,
            offsets_rad=config.joint_offsets_rad,
            ticks_per_rev=config.ticks_per_rev,
            wrap_ticks=config.wrap_ticks,
            signed_position=config.signed_position,
        )
        self.port_handler = PortHandler(config.leader_port)
        self.packet_handler = PacketHandler(DXL_PROTOCOL_VERSION)
        self.group_reader = GroupSyncRead(
            self.port_handler,
            self.packet_handler,
            ADDR_PRESENT_POSITION,
            LEN_PRESENT_POSITION,
        )
        self._connected = False

    @property
    def all_read_ids(self) -> list[int]:
        ids = list(self.config.joint_ids)
        if self.config.gripper_id not in ids:
            ids.append(self.config.gripper_id)
        return ids

    def connect(self) -> None:
        if not self.port_handler.openPort():
            raise RuntimeError(f"无法打开主臂串口：{self.config.leader_port}")
        self._connected = True
        if not self.port_handler.setBaudRate(self.config.leader_baud):
            raise RuntimeError(f"无法设置主臂波特率：{self.config.leader_baud}")
        if self.config.enable_gripper_torque_on_start:
            self.configure_gripper_end_switch()
        for dxl_id in self.all_read_ids:
            if not self.group_reader.addParam(dxl_id):
                raise RuntimeError(f"无法将 Dynamixel ID {dxl_id} 加入同步读取组")

    def _check_comm(self, result: int, dxl_error: int, action: str, dxl_id: int) -> None:
        if result != COMM_SUCCESS:
            detail = self.packet_handler.getTxRxResult(result)
            raise RuntimeError(f"Dynamixel {action} 失败，ID={dxl_id}：{detail}")
        if dxl_error != 0:
            detail = self.packet_handler.getRxPacketError(dxl_error)
            raise RuntimeError(f"Dynamixel {action} 包错误，ID={dxl_id}：{detail}")

    def write1(self, dxl_id: int, address: int, value: int, label: str) -> None:
        result, dxl_error = self.packet_handler.write1ByteTxRx(
            self.port_handler, dxl_id, address, int(value) & 0xFF
        )
        self._check_comm(result, dxl_error, label, dxl_id)

    def write2(self, dxl_id: int, address: int, value: int, label: str) -> None:
        result, dxl_error = self.packet_handler.write2ByteTxRx(
            self.port_handler, dxl_id, address, int(value) & 0xFFFF
        )
        self._check_comm(result, dxl_error, label, dxl_id)

    def write4(self, dxl_id: int, address: int, value: int, label: str) -> None:
        result, dxl_error = self.packet_handler.write4ByteTxRx(
            self.port_handler, dxl_id, address, int(value) & 0xFFFFFFFF
        )
        self._check_comm(result, dxl_error, label, dxl_id)

    def configure_gripper_end_switch(self) -> None:
        gid = self.config.gripper_id
        self.write1(gid, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, "Torque_Enable=0")
        self.write1(
            gid,
            ADDR_OPERATING_MODE,
            OPERATING_MODE_CURRENT_POSITION,
            "Operating_Mode=CURRENT_POSITION",
        )
        self.write2(
            gid,
            ADDR_CURRENT_LIMIT,
            self.config.gripper_current_limit,
            "Current_Limit",
        )
        self.write1(gid, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, "Torque_Enable=1")
        self.write4(
            gid,
            ADDR_GOAL_POSITION,
            self.config.gripper_goal_position_on_start,
            "Goal_Position",
        )

    def gripper_raw_to_norm(self, raw: int) -> float:
        gripper_range = self.config.gripper_open_pos - self.config.gripper_closed_pos
        if gripper_range == 0:
            raise RuntimeError("夹爪打开位置与闭合位置不能相等")
        return 1.0 - (
            (float(raw) - float(self.config.gripper_closed_pos)) / float(gripper_range)
        )

    def read_state(self) -> LeaderState:
        result = self.group_reader.txRxPacket()
        if result != COMM_SUCCESS:
            detail = self.packet_handler.getTxRxResult(result)
            raise RuntimeError(f"Dynamixel 同步读取失败：{detail}")

        q_rad: list[float] = []
        for index, dxl_id in enumerate(self.config.joint_ids):
            if not self.group_reader.isAvailable(
                dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
            ):
                raise RuntimeError(f"Dynamixel ID {dxl_id} 的 Present_Position 不可用")
            raw = self.group_reader.getData(
                dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
            )
            q_rad.append(self.mapping.ticks_to_rad(raw, index))

        gid = self.config.gripper_id
        if not self.group_reader.isAvailable(
            gid, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
        ):
            raise RuntimeError(f"Dynamixel ID {gid} 的 Present_Position 不可用")

        gripper_raw = int(
            self.group_reader.getData(gid, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
        )
        gripper_norm = self.gripper_raw_to_norm(gripper_raw)
        end_switch = (
            END_SWITCH_CLOSED
            if gripper_norm > self.config.end_switch_threshold
            else END_SWITCH_OPEN
        )
        return LeaderState(q_rad, gripper_raw, gripper_norm, end_switch)

    def interrupt(self) -> None:
        """Close the SDK port immediately so a pending operation can return."""
        if not self._connected:
            return
        try:
            self.port_handler.closePort()
        except Exception:
            pass
        finally:
            self._connected = False

    def close(self) -> None:
        try:
            self.group_reader.clearParam()
        except Exception:
            pass
        self.interrupt()


class McuSerialSender:
    def __init__(self, config: AppConfig):
        self.config = config
        self.ser: serial.Serial | None = None
        self._rx_log_buffer = bytearray()
        self._rx_bytes = 0
        self._rx_lines = 0

    @property
    def rx_bytes(self) -> int:
        return self._rx_bytes

    @property
    def rx_lines(self) -> int:
        return self._rx_lines

    def connect(self) -> None:
        kwargs: dict[str, Any] = dict(
            port=self.config.mcu_port,
            baudrate=self.config.mcu_baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.0,
            write_timeout=self.config.write_timeout_s,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )
        if IS_POSIX:
            kwargs["exclusive"] = self.config.exclusive
        try:
            self.ser = serial.Serial(**kwargs)
        except TypeError:
            kwargs.pop("exclusive", None)
            self.ser = serial.Serial(**kwargs)
        try:
            self.ser.rts = False
            self.ser.dtr = False
        except Exception:
            pass
        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass

    def write_frame(self, frame: bytes) -> None:
        if self.ser is None or not self.ser.is_open:
            raise RuntimeError("从臂 MCU 串口未打开")
        written = self.ser.write(frame)
        if written != len(frame):
            raise RuntimeError(f"从臂串口短写：{written}/{len(frame)} bytes")

    def read_log_lines(self) -> list[str]:
        if not self.config.read_mcu_log:
            return []
        if self.ser is None or not self.ser.is_open:
            return []
        waiting = self.ser.in_waiting
        if waiting <= 0:
            return []
        data = self.ser.read(waiting)
        if not data:
            return []
        self._rx_bytes += len(data)
        self._rx_log_buffer.extend(data)
        lines: list[str] = []
        while True:
            try:
                index = self._rx_log_buffer.index(0x0A)
            except ValueError:
                break
            raw_line = bytes(self._rx_log_buffer[:index])
            del self._rx_log_buffer[: index + 1]
            text = raw_line.decode(
                self.config.mcu_log_encoding, errors="replace"
            ).rstrip("\r")
            if text:
                lines.append(text)
        if len(self._rx_log_buffer) > self.config.mcu_log_buffer_limit:
            keep = max(1, self.config.mcu_log_buffer_limit // 2)
            dropped = len(self._rx_log_buffer) - keep
            del self._rx_log_buffer[:dropped]
            tail = bytes(self._rx_log_buffer).decode(
                self.config.mcu_log_encoding, errors="replace"
            )
            lines.append(
                f"[RX_LOG_BUFFER_TRUNCATED] dropped={dropped} bytes, tail={tail}"
            )
        self._rx_lines += len(lines)
        return lines

    def interrupt(self) -> None:
        """Cancel pending serial I/O and close the handle."""
        ser = self.ser
        if ser is None:
            return
        for method_name in ("cancel_read", "cancel_write"):
            method = getattr(ser, method_name, None)
            if callable(method):
                try:
                    method()
                except Exception:
                    pass
        try:
            if ser.is_open:
                ser.close()
        except Exception:
            pass

    def close(self) -> None:
        self.interrupt()


class LogModel(QAbstractListModel):
    TimestampRole = Qt.UserRole + 1
    LevelRole = Qt.UserRole + 2
    MessageRole = Qt.UserRole + 3

    countChanged = Signal()

    def __init__(self, parent: QObject | None = None, max_rows: int = 5000):
        super().__init__(parent)
        self._rows: list[dict[str, str]] = []
        self._max_rows = max_rows

    def roleNames(self) -> dict[int, bytes]:
        return {
            self.TimestampRole: b"timestamp",
            self.LevelRole: b"level",
            self.MessageRole: b"message",
        }

    def rowCount(self, parent: QModelIndex = QModelIndex()) -> int:
        return 0 if parent.isValid() else len(self._rows)

    def data(self, index: QModelIndex, role: int = Qt.DisplayRole) -> Any:
        if not index.isValid() or not 0 <= index.row() < len(self._rows):
            return None
        row = self._rows[index.row()]
        if role == self.TimestampRole:
            return row["timestamp"]
        if role == self.LevelRole:
            return row["level"]
        if role == self.MessageRole:
            return row["message"]
        return None

    @Property(int, notify=countChanged)
    def count(self) -> int:
        return len(self._rows)

    @Slot(str, str, str)
    def append(self, timestamp: str, level: str, message: str) -> None:
        if len(self._rows) >= self._max_rows:
            remove_count = max(1, self._max_rows // 10)
            self.beginRemoveRows(QModelIndex(), 0, remove_count - 1)
            del self._rows[:remove_count]
            self.endRemoveRows()
        row = len(self._rows)
        self.beginInsertRows(QModelIndex(), row, row)
        self._rows.append(
            {"timestamp": timestamp, "level": level, "message": message}
        )
        self.endInsertRows()
        self.countChanged.emit()

    @Slot()
    def clear(self) -> None:
        if not self._rows:
            return
        self.beginResetModel()
        self._rows.clear()
        self.endResetModel()
        self.countChanged.emit()


class TeleopWorker(QThread):
    logEvent = Signal(str, str, str)
    statusEvent = Signal(str)
    statsEvent = Signal("QVariantMap")
    activeChanged = Signal(bool)
    stopped = Signal(str)

    def __init__(self, config: AppConfig, parent: QObject | None = None):
        super().__init__(parent)
        self.config = config
        self._stop_event = threading.Event()
        self._io_lock = threading.Lock()
        self._leader: DynamixelLeader | None = None
        self._sender: McuSerialSender | None = None

    def request_stop(self) -> None:
        self._stop_event.set()
        self.interrupt_io()

    def interrupt_io(self) -> None:
        """Force pending device I/O to return by closing both serial handles."""
        with self._io_lock:
            leader = self._leader
            sender = self._sender
        if sender is not None:
            sender.interrupt()
        if leader is not None:
            leader.interrupt()

    def log(self, message: str, level: str = "INFO") -> None:
        self.logEvent.emit(time.strftime("%H:%M:%S"), level, message)

    def run(self) -> None:
        leader = DynamixelLeader(self.config)
        sender = McuSerialSender(self.config)
        with self._io_lock:
            self._leader = leader
            self._sender = sender

        packer = ProtocolPacker(self.config.crc_name)
        stats = RunStats()
        fatal_error = ""

        try:
            self.statusEvent.emit("正在连接主臂")
            self.log(
                f"打开主臂串口：{self.config.leader_port} @ {self.config.leader_baud}"
            )
            leader.connect()
            if self._stop_event.is_set():
                return
            self.log("主臂 Dynamixel 连接成功")

            if self.config.dry_run:
                self.log("Dry-run 已开启：不打开从臂 MCU 串口", "WARN")
            else:
                self.statusEvent.emit("正在连接从臂")
                self.log(
                    f"打开从臂 MCU 串口：{self.config.mcu_port} @ {self.config.mcu_baud}"
                )
                sender.connect()
                if self._stop_event.is_set():
                    return
                self.log("从臂 MCU 串口连接成功")

            period_s = 1.0 / self.config.master_send_freq_hz
            heartbeat = RateLimiter(self.config.heartbeat_freq_hz)
            printer = RateLimiter(self.config.print_freq_hz)
            ui_refresh = RateLimiter(min(20.0, self.config.master_send_freq_hz))
            next_loop_t = time.monotonic()

            self.statusEvent.emit("运行中")
            self.activeChanged.emit(True)
            self.log("遥操作开始运行")

            while not self._stop_event.is_set():
                loop_start = time.monotonic()
                try:
                    if not self.config.dry_run:
                        for line in sender.read_log_lines():
                            self.log(f"{self.config.mcu_log_prefix}{line}", "MCU")

                    state = leader.read_state()
                    if self._stop_event.is_set():
                        break

                    frame = packer.pack_master_joints(state.q_rad, state.end_switch)
                    if not self.config.dry_run:
                        sender.write_frame(frame)
                    stats.frames_sent += 1

                    if heartbeat.ready(loop_start):
                        heartbeat_frame = packer.pack_heartbeat()
                        if not self.config.dry_run:
                            sender.write_frame(heartbeat_frame)
                        stats.heartbeat_sent += 1

                    if not self.config.dry_run:
                        for line in sender.read_log_lines():
                            self.log(f"{self.config.mcu_log_prefix}{line}", "MCU")

                    stats.rx_bytes = 0 if self.config.dry_run else sender.rx_bytes
                    stats.rx_lines = 0 if self.config.dry_run else sender.rx_lines

                    if printer.ready(loop_start):
                        self.log(
                            "q_rad="
                            f"{format_q(state.q_rad)} "
                            f"gripper_raw={state.gripper_raw} "
                            f"gripper_norm={state.gripper_norm:+.3f} "
                            f"end_switch={state.end_switch} "
                            f"frames={stats.frames_sent} "
                            f"heartbeat={stats.heartbeat_sent} "
                            f"errors={stats.errors}",
                            "DATA",
                        )

                    if ui_refresh.ready(loop_start):
                        self.statsEvent.emit(
                            {
                                "q_rad": list(state.q_rad),
                                "gripper_raw": state.gripper_raw,
                                "gripper_norm": state.gripper_norm,
                                "end_switch": state.end_switch,
                                "frames_sent": stats.frames_sent,
                                "heartbeat_sent": stats.heartbeat_sent,
                                "errors": stats.errors,
                                "rx_bytes": stats.rx_bytes,
                                "rx_lines": stats.rx_lines,
                            }
                        )

                except (
                    RuntimeError,
                    serial.SerialException,
                    serial.SerialTimeoutException,
                ) as exc:
                    if self._stop_event.is_set():
                        break
                    stats.errors += 1
                    self.log(str(exc), "WARN")
                    self.statsEvent.emit({"errors": stats.errors})
                    self._stop_event.wait(min(period_s, 0.05))

                next_loop_t += period_s
                now = time.monotonic()
                if next_loop_t < now - period_s:
                    next_loop_t = now
                remain = next_loop_t - time.monotonic()
                if remain > 0.0 and self._stop_event.wait(remain):
                    break

        except Exception as exc:
            if not self._stop_event.is_set():
                fatal_error = str(exc)
                self.log(f"启动或运行失败：{exc}", "ERROR")
                self.log(traceback.format_exc().rstrip(), "DEBUG")
                self.statusEvent.emit("运行失败")
        finally:
            self.statusEvent.emit("正在停止")
            try:
                if not self.config.dry_run:
                    for line in sender.read_log_lines():
                        self.log(f"{self.config.mcu_log_prefix}{line}", "MCU")
            except Exception:
                pass
            try:
                sender.close()
            except Exception as exc:
                self.log(f"关闭从臂串口时出错：{exc}", "WARN")
            try:
                leader.close()
            except Exception as exc:
                self.log(f"关闭主臂串口时出错：{exc}", "WARN")
            with self._io_lock:
                self._sender = None
                self._leader = None
            self.log("遥操作已停止")
            self.activeChanged.emit(False)
            self.stopped.emit(fatal_error)


class TeleopBackend(QObject):
    configChanged = Signal()
    portsChanged = Signal()
    portDetailsChanged = Signal()
    statusChanged = Signal()
    stateChanged = Signal()
    statsChanged = Signal()
    dialogRequested = Signal(str, str, bool)

    def __init__(self, log_model: LogModel, parent: QObject | None = None):
        super().__init__(parent)
        self.log_model = log_model
        self._config = default_config_map()
        self._ports: list[str] = []
        self._port_details: dict[str, str] = {}
        self._status = "未运行"
        self._running = False
        self._busy = False
        self._pending_fatal_error = ""
        self._config_dirty = True
        self._applied_config: AppConfig | None = None
        self._worker: TeleopWorker | None = None
        self._stop_requested = False
        self._stats: dict[str, Any] = self._empty_stats()

        self._startup_timer = QTimer(self)
        self._startup_timer.setSingleShot(True)
        self._startup_timer.timeout.connect(self._on_startup_timeout)

        self._stop_interrupt_timer = QTimer(self)
        self._stop_interrupt_timer.setSingleShot(True)
        self._stop_interrupt_timer.timeout.connect(self._on_stop_interrupt_timeout)

        self._stop_terminate_timer = QTimer(self)
        self._stop_terminate_timer.setSingleShot(True)
        self._stop_terminate_timer.timeout.connect(self._on_stop_terminate_timeout)

    @staticmethod
    def _empty_stats() -> dict[str, Any]:
        return {
            "q_rad": [0.0, 0.0, 0.0, 0.0, 0.0],
            "gripper_raw": 0,
            "gripper_norm": 0.0,
            "end_switch": 0,
            "frames_sent": 0,
            "heartbeat_sent": 0,
            "errors": 0,
            "rx_bytes": 0,
            "rx_lines": 0,
        }

    @Property("QVariantMap", notify=configChanged)
    def config(self) -> dict[str, Any]:
        return dict(self._config)

    @Property("QStringList", notify=portsChanged)
    def ports(self) -> list[str]:
        return list(self._ports)

    @Property("QVariantMap", notify=portDetailsChanged)
    def portDetails(self) -> dict[str, str]:
        return dict(self._port_details)

    @Property("QStringList", constant=True)
    def baudRates(self) -> list[str]:
        return COMMON_BAUDRATES

    @Property(str, constant=True)
    def platformName(self) -> str:
        return PLATFORM_NAME

    @Property(str, notify=statusChanged)
    def status(self) -> str:
        return self._status

    @Property(bool, notify=stateChanged)
    def running(self) -> bool:
        return self._running

    @Property(bool, notify=stateChanged)
    def configLocked(self) -> bool:
        return self._busy

    @Property(bool, notify=stateChanged)
    def configDirty(self) -> bool:
        return self._config_dirty

    @Property(bool, notify=stateChanged)
    def canRun(self) -> bool:
        return (
            not self.configLocked
            and not self._config_dirty
            and self._applied_config is not None
        )

    @Property(bool, notify=stateChanged)
    def canStop(self) -> bool:
        return self._busy and self._worker is not None and not self._stop_requested

    @Property(str, notify=stateChanged)
    def configState(self) -> str:
        if self.configLocked:
            return "运行期间配置已锁定"
        if self._config_dirty:
            return "配置已修改，等待应用"
        if self._applied_config is not None:
            return "配置已验证并应用"
        return "配置未应用"

    @Property("QVariantMap", notify=statsChanged)
    def stats(self) -> dict[str, Any]:
        return dict(self._stats)

    def _set_status(self, value: str) -> None:
        if self._status != value:
            self._status = value
            self.statusChanged.emit()

    def _append_log(self, level: str, message: str, timestamp: str | None = None) -> None:
        self.log_model.append(timestamp or time.strftime("%H:%M:%S"), level, message)

    @Slot(str, "QVariant")
    def setConfigValue(self, key: str, value: Any) -> None:
        if self.configLocked or key not in self._config:
            return
        normalized = value
        if isinstance(self._config[key], bool):
            normalized = bool(value)
        else:
            normalized = str(value)
        if self._config[key] == normalized:
            return
        self._config[key] = normalized
        self._config_dirty = True
        self._applied_config = None
        self.configChanged.emit()
        self.stateChanged.emit()

    @Slot()
    def scanPorts(self) -> None:
        if self.configLocked:
            return
        try:
            ports = sorted(list_ports.comports(), key=lambda item: item.device)
        except Exception as exc:
            self._append_log("ERROR", f"扫描串口失败：{exc}")
            self.dialogRequested.emit("串口扫描失败", str(exc), True)
            return

        self._ports = [item.device for item in ports]
        self._port_details = {
            item.device: (item.description or "未知设备") for item in ports
        }
        changed_config = False
        if self._ports and not str(self._config["leader_port"]).strip():
            self._config["leader_port"] = self._ports[0]
            changed_config = True
        if self._ports and not str(self._config["mcu_port"]).strip():
            remaining = [
                device
                for device in self._ports
                if device != self._config["leader_port"]
            ]
            self._config["mcu_port"] = remaining[0] if remaining else self._ports[0]
            changed_config = True

        if changed_config:
            self._config_dirty = True
            self._applied_config = None
            self.configChanged.emit()
            self.stateChanged.emit()

        self.portsChanged.emit()
        self.portDetailsChanged.emit()
        if ports:
            summary = "；".join(
                f"{item.device} ({item.description or '未知设备'})" for item in ports
            )
            self._append_log("INFO", f"扫描到 {len(ports)} 个串口：{summary}")
        else:
            self._append_log("WARN", "未扫描到串口，可手动输入端口名称")

    def _validate_config(self) -> AppConfig:
        def required_text(name: str, label: str) -> str:
            value = str(self._config[name]).strip()
            if not value:
                raise ValueError(f"{label}不能为空")
            return value

        def parse_int(name: str, label: str, minimum: int | None = None) -> int:
            text = required_text(name, label)
            try:
                value = int(text, 0)
            except ValueError as exc:
                raise ValueError(f"{label}必须是整数") from exc
            if minimum is not None and value < minimum:
                raise ValueError(f"{label}必须不小于 {minimum}")
            return value

        def parse_float(
            name: str,
            label: str,
            minimum: float | None = None,
            strictly_positive: bool = False,
        ) -> float:
            text = required_text(name, label)
            try:
                value = float(text)
            except ValueError as exc:
                raise ValueError(f"{label}必须是数字") from exc
            if not math.isfinite(value):
                raise ValueError(f"{label}必须是有限数")
            if strictly_positive and value <= 0.0:
                raise ValueError(f"{label}必须大于 0")
            if minimum is not None and value < minimum:
                raise ValueError(f"{label}必须不小于 {minimum}")
            return value

        leader_port = required_text("leader_port", "主臂串口")
        mcu_port = required_text("mcu_port", "从臂 MCU 串口")
        dry_run = bool(self._config["dry_run"])
        if not dry_run and leader_port == mcu_port:
            raise ValueError("主臂和从臂不能选择同一个串口")

        leader_baud = parse_int("leader_baud", "主臂波特率", 1)
        mcu_baud = parse_int("mcu_baud", "从臂波特率", 1)
        master_send_freq_hz = parse_float(
            "master_send_freq_hz", "关节发送频率", strictly_positive=True
        )
        heartbeat_freq_hz = parse_float(
            "heartbeat_freq_hz", "心跳频率", minimum=0.0
        )
        print_freq_hz = parse_float("print_freq_hz", "打印频率", minimum=0.0)
        write_timeout_s = parse_float(
            "write_timeout_s", "串口写超时", strictly_positive=True
        )

        joint_ids_list = parse_int_list(
            required_text("joint_ids", "关节 ID"), expected_len=5
        )
        if len(set(joint_ids_list)) != 5:
            raise ValueError("q0~q4 的 Dynamixel ID 不能重复")
        if any(value < 0 or value > 252 for value in joint_ids_list):
            raise ValueError("Dynamixel ID 必须位于 0~252")

        joint_signs_list = parse_float_list(
            required_text("joint_signs", "方向系数"), expected_len=5
        )
        if any(not math.isfinite(value) or value == 0.0 for value in joint_signs_list):
            raise ValueError("方向系数必须是 5 个有限且非零的数")

        joint_offsets_list = parse_float_list(
            required_text("joint_offsets_rad", "零位偏置"), expected_len=5
        )
        if any(not math.isfinite(value) for value in joint_offsets_list):
            raise ValueError("零位偏置必须是有限数")

        ticks_per_rev = parse_int("ticks_per_rev", "每圈 tick 数", 1)
        gripper_id = parse_int("gripper_id", "夹爪 ID", 0)
        if gripper_id > 252:
            raise ValueError("夹爪 Dynamixel ID 必须位于 0~252")
        if gripper_id in joint_ids_list:
            raise ValueError("夹爪 ID 不能与 q0~q4 的 ID 重复")

        gripper_open_pos = parse_int("gripper_open_pos", "夹爪打开位置")
        gripper_closed_pos = parse_int("gripper_closed_pos", "夹爪闭合位置")
        if gripper_open_pos == gripper_closed_pos:
            raise ValueError("夹爪打开位置和闭合位置不能相等")
        end_switch_threshold = parse_float(
            "end_switch_threshold", "末端开关阈值"
        )
        gripper_current_limit = parse_int(
            "gripper_current_limit", "夹爪电流限制", 0
        )
        if gripper_current_limit > 65535:
            raise ValueError("夹爪电流限制超出 16 位无符号范围")
        gripper_goal_position = parse_int(
            "gripper_goal_position_on_start", "夹爪启动目标位置"
        )

        encoding = required_text("mcu_log_encoding", "MCU 日志编码")
        try:
            "test".encode(encoding)
        except LookupError as exc:
            raise ValueError(f"未知日志编码：{encoding}") from exc

        log_buffer_limit = parse_int(
            "mcu_log_buffer_limit", "日志接收缓存上限", 128
        )
        crc_name = required_text("crc_name", "CRC 名称")
        try:
            ProtocolPacker(crc_name)
        except Exception as exc:
            raise ValueError(f"CRC 名称不可用：{crc_name}") from exc

        return AppConfig(
            leader_port=leader_port,
            mcu_port=mcu_port,
            leader_baud=leader_baud,
            mcu_baud=mcu_baud,
            master_send_freq_hz=master_send_freq_hz,
            heartbeat_freq_hz=heartbeat_freq_hz,
            print_freq_hz=print_freq_hz,
            write_timeout_s=write_timeout_s,
            joint_ids=tuple(joint_ids_list),  # type: ignore[arg-type]
            joint_signs=tuple(joint_signs_list),  # type: ignore[arg-type]
            joint_offsets_rad=tuple(joint_offsets_list),  # type: ignore[arg-type]
            ticks_per_rev=ticks_per_rev,
            wrap_ticks=bool(self._config["wrap_ticks"]),
            signed_position=bool(self._config["signed_position"]),
            gripper_id=gripper_id,
            gripper_open_pos=gripper_open_pos,
            gripper_closed_pos=gripper_closed_pos,
            end_switch_threshold=end_switch_threshold,
            enable_gripper_torque_on_start=bool(
                self._config["enable_gripper_torque_on_start"]
            ),
            gripper_current_limit=gripper_current_limit,
            gripper_goal_position_on_start=gripper_goal_position,
            read_mcu_log=bool(self._config["read_mcu_log"]),
            mcu_log_encoding=encoding,
            mcu_log_prefix=str(self._config["mcu_log_prefix"]),
            mcu_log_buffer_limit=log_buffer_limit,
            exclusive=bool(self._config["exclusive"]),
            dry_run=dry_run,
            crc_name=crc_name,
        )

    @Slot()
    def applyConfig(self) -> None:
        if self.configLocked:
            return
        try:
            config = self._validate_config()
        except ValueError as exc:
            self._applied_config = None
            self._config_dirty = True
            self.stateChanged.emit()
            self._append_log("ERROR", f"配置校验失败：{exc}")
            self.dialogRequested.emit("配置错误", str(exc), True)
            return

        self._applied_config = config
        self._config_dirty = False
        self.stateChanged.emit()
        self._append_log(
            "INFO",
            f"配置已应用：主臂 {config.leader_port} @ {config.leader_baud}；"
            f"从臂 {config.mcu_port} @ {config.mcu_baud}；"
            f"发送频率 {config.master_send_freq_hz:g} Hz",
        )

    @Slot()
    def startRun(self) -> None:
        if not self.canRun or self._applied_config is None:
            self.dialogRequested.emit(
                "配置未应用", "请先验证并应用当前配置", False
            )
            return
        self._stats = self._empty_stats()
        self.statsChanged.emit()
        self._set_status("正在启动")

        worker = TeleopWorker(self._applied_config, self)
        self._worker = worker
        self._busy = True
        self._stop_requested = False
        self._pending_fatal_error = ""
        worker.logEvent.connect(self._on_worker_log)
        worker.statusEvent.connect(self._set_status)
        worker.statsEvent.connect(self._on_worker_stats)
        worker.activeChanged.connect(self._on_worker_active)
        worker.stopped.connect(self._on_worker_stopped)
        worker.finished.connect(self._on_worker_finished)
        worker.finished.connect(worker.deleteLater)
        self.stateChanged.emit()
        worker.start()
        self._startup_timer.start(STARTUP_TIMEOUT_MS)

    @Slot()
    def stopRun(self) -> None:
        worker = self._worker
        if worker is None or not worker.isRunning() or self._stop_requested:
            return
        self._stop_requested = True
        self._startup_timer.stop()
        self._set_status("正在请求停止")
        self._append_log("INFO", "已发送停止请求，正在中断串口 I/O")
        worker.request_stop()
        self._stop_interrupt_timer.start(STOP_INTERRUPT_TIMEOUT_MS)
        self.stateChanged.emit()

    @Slot()
    def _on_startup_timeout(self) -> None:
        if not self._busy or self._running or self._worker is None:
            return
        self._append_log("ERROR", "设备连接超过 8 秒，已自动中止连接")
        self._set_status("连接超时，正在停止")
        self.stopRun()

    @Slot()
    def _on_stop_interrupt_timeout(self) -> None:
        worker = self._worker
        if worker is None or not worker.isRunning():
            return
        self._append_log("WARN", "停止等待超时，正在强制关闭主从串口")
        self._set_status("停止超时，正在强制释放串口")
        worker.interrupt_io()
        self._stop_terminate_timer.start(STOP_TERMINATE_TIMEOUT_MS)

    @Slot()
    def _on_stop_terminate_timeout(self) -> None:
        worker = self._worker
        if worker is None or not worker.isRunning():
            return
        self._append_log("ERROR", "串口强制关闭后线程仍未退出，执行最终线程终止")
        self._set_status("正在强制终止通信线程")
        worker.terminate()
        worker.wait(1000)

    @Slot()
    def resetDefaults(self) -> None:
        if self.configLocked:
            return
        self._config = default_config_map()
        self._config_dirty = True
        self._applied_config = None
        self.configChanged.emit()
        self.stateChanged.emit()
        self._append_log("INFO", "已恢复默认配置，需重新验证并应用")

    @Slot()
    def saveConfig(self) -> None:
        if self.configLocked:
            return
        path, _ = QFileDialog.getSaveFileName(
            None,
            "保存遥操作配置",
            "teleop_config.json",
            "JSON 配置 (*.json)",
        )
        if not path:
            return
        try:
            Path(path).write_text(
                json.dumps(self._config, ensure_ascii=False, indent=2),
                encoding="utf-8",
            )
            self._append_log("INFO", f"配置已保存：{path}")
        except Exception as exc:
            self._append_log("ERROR", f"保存配置失败：{exc}")
            self.dialogRequested.emit("保存失败", str(exc), True)

    @Slot()
    def loadConfig(self) -> None:
        if self.configLocked:
            return
        path, _ = QFileDialog.getOpenFileName(
            None,
            "载入遥操作配置",
            "",
            "JSON 配置 (*.json)",
        )
        if not path:
            return
        try:
            data = json.loads(Path(path).read_text(encoding="utf-8"))
            if not isinstance(data, dict):
                raise ValueError("配置文件根节点必须是 JSON 对象")
            defaults = default_config_map()
            merged = dict(defaults)
            for key, value in data.items():
                if key in merged:
                    merged[key] = value
            self._config = merged
            self._config_dirty = True
            self._applied_config = None
            self.configChanged.emit()
            self.stateChanged.emit()
            self._append_log("INFO", f"配置已载入：{path}；请重新验证并应用")
        except Exception as exc:
            self._append_log("ERROR", f"载入配置失败：{exc}")
            self.dialogRequested.emit("载入失败", str(exc), True)

    @Slot()
    def clearLogs(self) -> None:
        self.log_model.clear()

    @Slot(str, result=str)
    def portDescription(self, device: str) -> str:
        return self._port_details.get(device, "手动输入端口")

    @Slot()
    def shutdown(self) -> None:
        self._startup_timer.stop()
        self._stop_interrupt_timer.stop()
        self._stop_terminate_timer.stop()
        worker = self._worker
        if worker is None or not worker.isRunning():
            return
        worker.request_stop()
        if not worker.wait(2500):
            worker.interrupt_io()
        if worker.isRunning() and not worker.wait(1500):
            worker.terminate()
            worker.wait(1000)

    @Slot(str, str, str)
    def _on_worker_log(self, timestamp: str, level: str, message: str) -> None:
        self._append_log(level, message, timestamp)

    @Slot("QVariantMap")
    def _on_worker_stats(self, data: dict[str, Any]) -> None:
        self._stats.update(data)
        self.statsChanged.emit()

    @Slot(bool)
    def _on_worker_active(self, active: bool) -> None:
        self._running = active
        if active:
            self._startup_timer.stop()
        self.stateChanged.emit()

    @Slot(str)
    def _on_worker_stopped(self, fatal_error: str) -> None:
        self._startup_timer.stop()
        self._stop_interrupt_timer.stop()
        self._stop_terminate_timer.stop()
        self._running = False
        self._pending_fatal_error = fatal_error
        self._set_status("运行失败" if fatal_error else "已停止")
        self.stateChanged.emit()
        if fatal_error:
            self.dialogRequested.emit("运行失败", fatal_error, True)

    @Slot()
    def _on_worker_finished(self) -> None:
        self._startup_timer.stop()
        self._stop_interrupt_timer.stop()
        self._stop_terminate_timer.stop()
        self._running = False
        self._busy = False
        self._stop_requested = False
        self._worker = None
        self.stateChanged.emit()
