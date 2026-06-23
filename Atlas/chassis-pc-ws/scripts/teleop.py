#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
主从臂遥操作脚本

使用说明：
1. 打开主臂 Dynamixel 串口，默认 /dev/ttyUSB0
2. 启动时配置 ID7，并开启力矩，用作末端开关
3. 周期读取 ID1 ~ ID5 的 Present_Position，映射为 q0 ~ q4
4. 周期读取 ID7 的 Present_Position，按原双臂工程 gripper 逻辑映射为开关量
5. 将 q0 ~ q4 从 rad 转为 urad
6. 打开 MCU 串口，默认 /dev/ttyACM0
7. 按二进制协议持续发送 PC_MASTER_JOINTS 和 PC_HEARTBEAT

依赖：
pip install pyserial dynamixel-sdk crcmod

"""

from __future__ import annotations

import argparse
import math
import signal
import struct
import sys
import time
from dataclasses import dataclass
from typing import Sequence

import serial
from dynamixel_sdk import COMM_SUCCESS, GroupSyncRead, PacketHandler, PortHandler

try:
    import crcmod
    import crcmod.predefined
except ImportError as exc:
    raise SystemExit("缺少依赖 crcmod，请执行：pip install crcmod") from exc


# =============================================================================
# 用户配置区
# =============================================================================

# 主臂 Dynamixel 串口
DEFAULT_LEADER_PORT = "/dev/ttyUSB0"

# MCU 串口
DEFAULT_MCU_PORT = "/dev/ttyUSB1"

# 主臂 Dynamixel 波特率
DEFAULT_LEADER_BAUD = 115200

# MCU 串口波特率
DEFAULT_MCU_BAUD = 115200

# 主臂关节角发送频率
# 协议建议 PC_MASTER_JOINTS 为 30Hz ~ 100Hz
DEFAULT_MASTER_SEND_FREQ_HZ = 50.0

# PC 心跳发送频率
# 协议建议 PC_HEARTBEAT 为 1Hz，最低不建议低于 0.5Hz
DEFAULT_HEARTBEAT_FREQ_HZ = 1.0

# 调试打印频率
# 设置为 0 表示关闭打印
DEFAULT_PRINT_FREQ_HZ = 1.0

# 串口写超时时间
# 不建议设置太大，避免 MCU 串口异常时阻塞主循环
DEFAULT_WRITE_TIMEOUT_S = 0.02

# q0 ~ q4 对应的 Dynamixel ID
# 当前协议只发送 5 个主臂关节角
DEFAULT_JOINT_IDS = [1, 2, 3, 4, 5]

# ID7 用作末端开关输入
GRIPPER_ID = 7

# ID7 原始 gripper 映射参数
# 这两个值保持原双臂工程 DMLeaderConfig 的默认值
GRIPPER_OPEN_POS = 2280
GRIPPER_CLOSED_POS = 1670

# ID7 归一化阈值
# 原始映射后 gripper_norm 接近 0 表示打开，接近 1 表示闭合
# 当 gripper_norm > END_SWITCH_THRESHOLD 时认为末端开关触发
END_SWITCH_THRESHOLD = 0.50

# 末端开关协议取值
# 0 表示未触发 / 打开
# 1 表示触发 / 闭合
END_SWITCH_OPEN = 0
END_SWITCH_CLOSED = 1

# 启动时是否配置 ID7 并开启力矩
ENABLE_GRIPPER_TORQUE_ON_START = True

# ID7 电流限制
# 对应 Dynamixel 控制表 Current_Limit
GRIPPER_CURRENT_LIMIT = 100

# ID7 启动后的目标位置
# 原双臂工程中启动时会给 gripper 写入 open_pos
GRIPPER_GOAL_POSITION_ON_START = GRIPPER_OPEN_POS

# q0 ~ q4 方向修正
# 某个关节方向反了时，将对应位置改为 -1
DEFAULT_JOINT_SIGNS = [1, 1, 1, 1, 1]

# q0 ~ q4 零位偏置，单位 rad
# 读取角度后会执行：q = sign * q_raw + offset
DEFAULT_JOINT_OFFSETS_RAD = [0, 0, 0, 0, 0]

# Dynamixel 每圈编码 tick 数
DEFAULT_TICKS_PER_REV = 4096

# 默认是否对 Present_Position 做一圈取模
# 保持旧 DMLeader 的近似行为时建议保持 True
DEFAULT_WRAP_TICKS = True

# 默认是否将 Present_Position 按 int32 有符号数解释
# 普通 XL330 位置读取一般不需要打开
DEFAULT_SIGNED_POSITION = False

# CRC 名称
# 默认使用 CRC-16/CCITT-FALSE
DEFAULT_CRC_NAME = "crc-ccitt-false"


# =============================================================================
# 通信协议常量
# =============================================================================

SOF = b"\xa5\x5a"
PROTOCOL_VER = 0x01
FLAG_NONE = 0x00

MSG_PC_HEARTBEAT = 0x10
MSG_PC_MASTER_JOINTS = 0x11

# 本脚本扩展后的 PC_MASTER_JOINTS payload 长度
# 原协议为 24 字节：stamp_ms + 5 * q_urad
# 当前扩展为 25 字节：stamp_ms + 5 * q_urad + end_switch
PC_MASTER_JOINTS_PAYLOAD_LEN = 25


# =============================================================================
# Dynamixel XL330 Protocol 2.0 控制表常量
# =============================================================================

DXL_PROTOCOL_VERSION = 2.0

ADDR_OPERATING_MODE = 11
ADDR_CURRENT_LIMIT = 38
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132

LEN_PRESENT_POSITION = 4

TORQUE_DISABLE = 0
TORQUE_ENABLE = 1

# Current-based Position Control Mode
OPERATING_MODE_CURRENT_POSITION = 5


class RateLimiter:
    """周期触发器，用于控制心跳和调试打印等低频任务"""

    def __init__(self, rate_hz: float):
        """
        初始化周期触发器

        参数
        ----
        rate_hz:
            触发频率，单位 Hz
            当 rate_hz <= 0 时表示禁用
        """
        self.enabled = rate_hz > 0.0
        self.period_s = 1.0 / rate_hz if self.enabled else float("inf")
        self.next_t = time.monotonic()

    def ready(self, now: float | None = None) -> bool:
        """
        判断当前时刻是否达到触发时间

        参数
        ----
        now:
            当前单调时间，单位 s
            如果传入 None，则函数内部调用 time.monotonic()

        返回
        ----
        bool:
            True 表示应该触发一次
            False 表示还未到触发时间
        """
        if not self.enabled:
            return False

        now = time.monotonic() if now is None else now
        if now >= self.next_t:
            missed = max(1, int((now - self.next_t) // self.period_s) + 1)
            self.next_t += missed * self.period_s
            return True

        return False


class ProtocolPacker:
    """二进制通信协议打包器"""

    def __init__(self, crc_name: str = DEFAULT_CRC_NAME, version: int = PROTOCOL_VER):
        """
        初始化协议打包器

        参数
        ----
        crc_name:
            crcmod 支持的 CRC 名称，默认 crc-ccitt-false
        version:
            协议版本号，当前固定为 0x01
        """
        self.version = version & 0xFF
        self.seq = 0

        try:
            self._crc16 = crcmod.predefined.mkPredefinedCrcFun(crc_name)
        except Exception:
            if crc_name != DEFAULT_CRC_NAME:
                raise

            # CRC-16/CCITT-FALSE
            # poly=0x1021，init=0xFFFF，xorout=0x0000，refin=false，refout=false
            self._crc16 = crcmod.mkCrcFun(
                0x11021,
                initCrc=0xFFFF,
                rev=False,
                xorOut=0x0000,
            )

    def pack(self, msg_id: int, payload: bytes = b"", flags: int = FLAG_NONE) -> bytes:
        """
        打包通用二进制帧

        参数
        ----
        msg_id:
            消息 ID
        payload:
            业务载荷
        flags:
            标志位，bit0 为 NEED_ACK，当前 PC 发送帧不使用 ACK

        返回
        ----
        bytes:
            完整二进制帧，包含 SOF、LEN、BODY 和 CRC
        """
        seq = self.seq & 0xFF
        self.seq = (self.seq + 1) & 0xFF

        body = bytes([self.version, msg_id & 0xFF, seq, flags & 0xFF]) + payload
        header = SOF + struct.pack(">H", len(body))
        crc = self._crc16(header + body) & 0xFFFF

        return header + body + struct.pack(">H", crc)

    def pack_heartbeat(self) -> bytes:
        """
        打包 PC_HEARTBEAT 心跳帧

        返回
        ----
        bytes:
            MSG_ID = 0x10 的完整心跳帧
        """
        return self.pack(MSG_PC_HEARTBEAT)

    def pack_master_joints(
        self,
        q_rad: Sequence[float],
        end_switch: int,
        stamp_ms: int | None = None,
    ) -> bytes:
        """
        打包 PC_MASTER_JOINTS 主臂关节角帧

        参数
        ----
        q_rad:
            q0 ~ q4 关节角，单位 rad
        end_switch:
            末端开关状态
            0 表示未触发 / 打开
            1 表示触发 / 闭合
        stamp_ms:
            时间戳，单位 ms
            如果为 None，则使用本机 monotonic 时间生成 uint32 时间戳

        返回
        ----
        bytes:
            MSG_ID = 0x11 的完整主臂关节角帧
        """
        if len(q_rad) != 5:
            raise ValueError(
                f"PC_MASTER_JOINTS 需要 5 个关节角，实际得到 {len(q_rad)} 个"
            )

        if end_switch not in (END_SWITCH_OPEN, END_SWITCH_CLOSED):
            raise ValueError(f"end_switch 只能为 0 或 1，实际得到 {end_switch}")

        if stamp_ms is None:
            stamp_ms = monotonic_ms_u32()

        q_urad = [rad_to_urad(q) for q in q_rad]
        payload = struct.pack("<IiiiiiB", stamp_ms & 0xFFFFFFFF, *q_urad, end_switch)

        if len(payload) != PC_MASTER_JOINTS_PAYLOAD_LEN:
            raise RuntimeError(f"内部 payload 长度错误：{len(payload)}")

        return self.pack(MSG_PC_MASTER_JOINTS, payload)


def monotonic_ms_u32() -> int:
    """
    获取 uint32 毫秒时间戳

    返回
    ----
    int:
        基于 time.monotonic() 的毫秒时间戳
        按 uint32 范围自动回绕
    """
    return int(time.monotonic() * 1000.0) & 0xFFFFFFFF


def rad_to_urad(rad: float) -> int:
    """
    将弧度转换为微弧度整数

    参数
    ----
    rad:
        弧度值，单位 rad

    返回
    ----
    int:
        微弧度值，单位 urad
        范围被限制在 int32 可表达范围内
    """
    value = int(round(rad * 1_000_000.0))
    return max(-(2**31), min(2**31 - 1, value))


def parse_int_list(text: str, expected_len: int | None = None) -> list[int]:
    """
    解析逗号分隔的整数列表

    参数
    ----
    text:
        输入字符串，例如 "1,2,3,4,5"
    expected_len:
        期望元素个数
        如果为 None，则不检查长度

    返回
    ----
    list[int]:
        解析后的整数列表
    """
    values = [int(x.strip(), 0) for x in text.split(",") if x.strip()]

    if expected_len is not None and len(values) != expected_len:
        raise argparse.ArgumentTypeError(
            f"期望 {expected_len} 个逗号分隔整数，实际得到 {len(values)} 个"
        )

    return values


def parse_float_list(text: str, expected_len: int | None = None) -> list[float]:
    """
    解析逗号分隔的浮点数列表

    参数
    ----
    text:
        输入字符串，例如 "1,-1,1,1,-1"
    expected_len:
        期望元素个数
        如果为 None，则不检查长度

    返回
    ----
    list[float]:
        解析后的浮点数列表
    """
    values = [float(x.strip()) for x in text.split(",") if x.strip()]

    if expected_len is not None and len(values) != expected_len:
        raise argparse.ArgumentTypeError(
            f"期望 {expected_len} 个逗号分隔浮点数，实际得到 {len(values)} 个"
        )

    return values


def u32_to_s32(raw: int) -> int:
    """
    将 uint32 原始值按 int32 解释

    参数
    ----
    raw:
        uint32 原始值

    返回
    ----
    int:
        按补码规则解释后的 int32 值
    """
    raw &= 0xFFFFFFFF
    return raw - 0x100000000 if raw & 0x80000000 else raw


@dataclass
class JointMapping:
    """q0 ~ q4 关节角映射参数"""

    signs: list[float]
    offsets_rad: list[float]
    ticks_per_rev: int = DEFAULT_TICKS_PER_REV
    wrap_ticks: bool = DEFAULT_WRAP_TICKS
    signed_position: bool = DEFAULT_SIGNED_POSITION

    def ticks_to_rad(self, raw: int, index: int) -> float:
        """
        将 Dynamixel Present_Position tick 转为关节角 rad

        参数
        ----
        raw:
            Dynamixel Present_Position 原始值
        index:
            关节索引，范围 0 ~ 4

        返回
        ----
        float:
            映射后的关节角，单位 rad
        """
        value = u32_to_s32(raw) if self.signed_position else int(raw)

        if self.wrap_ticks:
            value %= self.ticks_per_rev

        # 保持旧 DMLeader 的核心转换方式
        # raw / 4096 * 2π - π
        rad = value / self.ticks_per_rev * 2.0 * math.pi - math.pi

        return self.signs[index] * rad + self.offsets_rad[index]


@dataclass
class LeaderState:
    """主臂当前读取状态"""

    q_rad: list[float]
    gripper_raw: int
    gripper_norm: float
    end_switch: int


class DynamixelLeader:
    """主臂 Dynamixel 读取与 ID7 末端开关配置类"""

    def __init__(
        self,
        port: str,
        baudrate: int,
        joint_ids: Sequence[int],
        gripper_id: int,
        mapping: JointMapping,
        end_switch_threshold: float,
    ):
        """
        初始化主臂读取器

        参数
        ----
        port:
            Dynamixel 主臂串口
        baudrate:
            Dynamixel 串口波特率
        joint_ids:
            q0 ~ q4 对应的 5 个 Dynamixel ID
        gripper_id:
            用作末端开关输入的 Dynamixel ID
        mapping:
            关节角映射参数
        end_switch_threshold:
            末端开关触发阈值
        """
        if len(joint_ids) != 5:
            raise ValueError("当前协议发送 q0 ~ q4，因此 joint_ids 必须包含 5 个 ID")

        self.port_name = port
        self.baudrate = baudrate
        self.joint_ids = list(joint_ids)
        self.gripper_id = int(gripper_id)
        self.mapping = mapping
        self.end_switch_threshold = float(end_switch_threshold)

        self.port_handler = PortHandler(port)
        self.packet_handler = PacketHandler(DXL_PROTOCOL_VERSION)
        self.group_reader = GroupSyncRead(
            self.port_handler,
            self.packet_handler,
            ADDR_PRESENT_POSITION,
            LEN_PRESENT_POSITION,
        )

    @property
    def all_read_ids(self) -> list[int]:
        """
        获取需要同步读取的全部 Dynamixel ID

        返回
        ----
        list[int]:
            q0 ~ q4 对应 ID 加上 ID7 末端开关 ID
        """
        ids = list(self.joint_ids)

        if self.gripper_id not in ids:
            ids.append(self.gripper_id)

        return ids

    def connect(self) -> None:
        """
        打开 Dynamixel 串口并配置读取参数

        流程
        ----
        1. 打开串口
        2. 设置波特率
        3. 配置 ID7 末端开关电机并开启力矩
        4. 将 q0 ~ q4 与 ID7 加入同步读取组
        """
        if not self.port_handler.openPort():
            raise RuntimeError(f"无法打开 Dynamixel 主臂串口：{self.port_name}")

        if not self.port_handler.setBaudRate(self.baudrate):
            raise RuntimeError(f"无法设置 Dynamixel 波特率：{self.baudrate}")

        if ENABLE_GRIPPER_TORQUE_ON_START:
            self.configure_gripper_end_switch()

        for dxl_id in self.all_read_ids:
            if not self.group_reader.addParam(dxl_id):
                raise RuntimeError(f"无法将 Dynamixel ID {dxl_id} 加入同步读取组")

    def _check_comm(
        self, result: int, dxl_error: int, action: str, dxl_id: int
    ) -> None:
        """
        检查 Dynamixel 读写通信结果

        参数
        ----
        result:
            SDK 返回的通信结果
        dxl_error:
            Dynamixel 返回的包错误码
        action:
            当前操作名称，用于错误提示
        dxl_id:
            当前操作对应的 Dynamixel ID
        """
        if result != COMM_SUCCESS:
            msg = self.packet_handler.getTxRxResult(result)
            raise RuntimeError(f"Dynamixel {action} 失败，ID={dxl_id}，原因：{msg}")

        if dxl_error != 0:
            msg = self.packet_handler.getRxPacketError(dxl_error)
            raise RuntimeError(
                f"Dynamixel {action} 返回包错误，ID={dxl_id}，原因：{msg}"
            )

    def write1(self, dxl_id: int, address: int, value: int, label: str) -> None:
        """
        写入 1 字节 Dynamixel 控制表参数

        参数
        ----
        dxl_id:
            Dynamixel ID
        address:
            控制表地址
        value:
            写入值
        label:
            操作说明，用于错误提示
        """
        result, dxl_error = self.packet_handler.write1ByteTxRx(
            self.port_handler,
            dxl_id,
            address,
            int(value) & 0xFF,
        )
        self._check_comm(result, dxl_error, label, dxl_id)

    def write2(self, dxl_id: int, address: int, value: int, label: str) -> None:
        """
        写入 2 字节 Dynamixel 控制表参数

        参数
        ----
        dxl_id:
            Dynamixel ID
        address:
            控制表地址
        value:
            写入值
        label:
            操作说明，用于错误提示
        """
        result, dxl_error = self.packet_handler.write2ByteTxRx(
            self.port_handler,
            dxl_id,
            address,
            int(value) & 0xFFFF,
        )
        self._check_comm(result, dxl_error, label, dxl_id)

    def write4(self, dxl_id: int, address: int, value: int, label: str) -> None:
        """
        写入 4 字节 Dynamixel 控制表参数

        参数
        ----
        dxl_id:
            Dynamixel ID
        address:
            控制表地址
        value:
            写入值
        label:
            操作说明，用于错误提示
        """
        result, dxl_error = self.packet_handler.write4ByteTxRx(
            self.port_handler,
            dxl_id,
            address,
            int(value) & 0xFFFFFFFF,
        )
        self._check_comm(result, dxl_error, label, dxl_id)

    def configure_gripper_end_switch(self) -> None:
        """
        配置 ID7 末端开关电机

        配置逻辑
        --------
        1. 关闭力矩
        2. 设置为 Current-based Position Control Mode
        3. 设置 Current_Limit
        4. 开启力矩
        5. 写入 Goal_Position 到打开位置

        说明
        ----
        该流程对应旧双臂工程 DMLeader.configure() 中对 gripper 的初始化逻辑
        """
        gid = self.gripper_id

        self.write1(gid, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, "Torque_Enable=0")
        self.write1(
            gid,
            ADDR_OPERATING_MODE,
            OPERATING_MODE_CURRENT_POSITION,
            "Operating_Mode=CURRENT_POSITION",
        )
        self.write2(gid, ADDR_CURRENT_LIMIT, GRIPPER_CURRENT_LIMIT, "Current_Limit")
        self.write1(gid, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, "Torque_Enable=1")
        self.write4(
            gid,
            ADDR_GOAL_POSITION,
            GRIPPER_GOAL_POSITION_ON_START,
            "Goal_Position=open",
        )

    @staticmethod
    def gripper_raw_to_norm(raw: int) -> float:
        """
        将 ID7 原始位置映射为 gripper_norm

        参数
        ----
        raw:
            ID7 Present_Position 原始值

        返回
        ----
        float:
            归一化后的 gripper 值
            接近 0 表示打开
            接近 1 表示闭合

        映射公式
        --------
        gripper_norm = 1 - (raw - GRIPPER_CLOSED_POS) / (GRIPPER_OPEN_POS - GRIPPER_CLOSED_POS)
        """
        gripper_range = GRIPPER_OPEN_POS - GRIPPER_CLOSED_POS

        if gripper_range == 0:
            raise RuntimeError(
                "ID7 映射参数错误：GRIPPER_OPEN_POS 与 GRIPPER_CLOSED_POS 不能相等"
            )

        return 1.0 - (float(raw) - float(GRIPPER_CLOSED_POS)) / float(gripper_range)

    def gripper_norm_to_switch(self, norm: float) -> int:
        """
        将 gripper_norm 转为一字节末端开关状态

        参数
        ----
        norm:
            ID7 归一化结果

        返回
        ----
        int:
            0 表示未触发 / 打开
            1 表示触发 / 闭合
        """
        return (
            END_SWITCH_CLOSED if norm > self.end_switch_threshold else END_SWITCH_OPEN
        )

    def read_state(self) -> LeaderState:
        """
        同步读取主臂 q0 ~ q4 与 ID7 末端开关状态

        返回
        ----
        LeaderState:
            包含 q0 ~ q4 弧度值、ID7 原始值、ID7 归一化值、末端开关状态
        """
        result = self.group_reader.txRxPacket()

        if result != COMM_SUCCESS:
            err = self.packet_handler.getTxRxResult(result)
            raise RuntimeError(f"Dynamixel 同步读取失败：{err}")

        q_rad: list[float] = []

        for i, dxl_id in enumerate(self.joint_ids):
            ok = self.group_reader.isAvailable(
                dxl_id,
                ADDR_PRESENT_POSITION,
                LEN_PRESENT_POSITION,
            )

            if not ok:
                raise RuntimeError(f"Dynamixel ID {dxl_id} 的 Present_Position 不可用")

            raw = self.group_reader.getData(
                dxl_id,
                ADDR_PRESENT_POSITION,
                LEN_PRESENT_POSITION,
            )
            q_rad.append(self.mapping.ticks_to_rad(raw, i))

        ok = self.group_reader.isAvailable(
            self.gripper_id,
            ADDR_PRESENT_POSITION,
            LEN_PRESENT_POSITION,
        )

        if not ok:
            raise RuntimeError(
                f"Dynamixel ID {self.gripper_id} 的 Present_Position 不可用"
            )

        gripper_raw = int(
            self.group_reader.getData(
                self.gripper_id,
                ADDR_PRESENT_POSITION,
                LEN_PRESENT_POSITION,
            )
        )
        gripper_norm = self.gripper_raw_to_norm(gripper_raw)
        end_switch = self.gripper_norm_to_switch(gripper_norm)

        return LeaderState(
            q_rad=q_rad,
            gripper_raw=gripper_raw,
            gripper_norm=gripper_norm,
            end_switch=end_switch,
        )

    def close(self) -> None:
        """
        关闭 Dynamixel 主臂串口

        说明
        ----
        本函数只关闭串口，不主动关闭 ID7 力矩
        如果需要退出时关闭力矩，可在这里补充 Torque_Enable=0
        """
        try:
            self.group_reader.clearParam()
        finally:
            self.port_handler.closePort()


class McuSerialSender:
    """MCU 串口发送器"""

    def __init__(self, port: str, baudrate: int, write_timeout_s: float):
        """
        初始化 MCU 串口发送器

        参数
        ----
        port:
            MCU 串口路径
        baudrate:
            MCU 串口波特率
        write_timeout_s:
            串口写超时时间，单位 s
        """
        self.port_name = port
        self.baudrate = baudrate
        self.write_timeout_s = write_timeout_s
        self.ser: serial.Serial | None = None

    def connect(self) -> None:
        """
        打开 MCU 串口

        串口配置
        --------
        8 数据位，无校验，1 停止位
        timeout = 0，表示读非阻塞
        write_timeout 用于避免写阻塞过久
        """
        self.ser = serial.Serial(
            port=self.port_name,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.0,
            write_timeout=self.write_timeout_s,
        )

    def write_frame(self, frame: bytes) -> None:
        """
        向 MCU 串口写入完整二进制帧

        参数
        ----
        frame:
            完整协议帧

        异常
        ----
        RuntimeError:
            串口未打开或发生短写入
        """
        if self.ser is None or not self.ser.is_open:
            raise RuntimeError("MCU 串口未打开")

        written = self.ser.write(frame)

        if written != len(frame):
            raise RuntimeError(f"MCU 串口写入不完整：{written}/{len(frame)} bytes")

    def close(self) -> None:
        """
        关闭 MCU 串口

        说明
        ----
        如果串口未打开，本函数不会抛出异常
        """
        if self.ser is not None and self.ser.is_open:
            self.ser.close()


def build_arg_parser() -> argparse.ArgumentParser:
    """
    构建命令行参数解析器

    返回
    ----
    argparse.ArgumentParser:
        已配置好的参数解析器
    """
    parser = argparse.ArgumentParser(
        description="读取 Dynamixel 主臂，并向 MCU 发送 PC_MASTER_JOINTS + 末端开关状态",
    )

    parser.add_argument(
        "--leader-port", default=DEFAULT_LEADER_PORT, help="主臂 Dynamixel 串口"
    )
    parser.add_argument("--mcu-port", default=DEFAULT_MCU_PORT, help="MCU 串口")
    parser.add_argument(
        "--leader-baud",
        type=int,
        default=DEFAULT_LEADER_BAUD,
        help="主臂 Dynamixel 波特率",
    )
    parser.add_argument(
        "--mcu-baud", type=int, default=DEFAULT_MCU_BAUD, help="MCU 串口波特率"
    )
    parser.add_argument(
        "--freq",
        type=float,
        default=DEFAULT_MASTER_SEND_FREQ_HZ,
        help="主臂关节角发送频率",
    )
    parser.add_argument(
        "--heartbeat-rate",
        type=float,
        default=DEFAULT_HEARTBEAT_FREQ_HZ,
        help="PC 心跳发送频率",
    )
    parser.add_argument(
        "--joint-ids",
        type=lambda s: parse_int_list(s, 5),
        default=DEFAULT_JOINT_IDS,
        help="q0~q4 对应的 5 个 Dynamixel ID",
    )
    parser.add_argument(
        "--gripper-id", type=int, default=GRIPPER_ID, help="用作末端开关的 Dynamixel ID"
    )
    parser.add_argument(
        "--end-switch-threshold",
        type=float,
        default=END_SWITCH_THRESHOLD,
        help="末端开关触发阈值",
    )
    parser.add_argument(
        "--joint-signs",
        type=lambda s: parse_float_list(s, 5),
        default=DEFAULT_JOINT_SIGNS,
        help="q0~q4 方向修正",
    )
    parser.add_argument(
        "--joint-offsets-rad",
        type=lambda s: parse_float_list(s, 5),
        default=DEFAULT_JOINT_OFFSETS_RAD,
        help="q0~q4 零位偏置，单位 rad",
    )
    parser.add_argument(
        "--ticks-per-rev",
        type=int,
        default=DEFAULT_TICKS_PER_REV,
        help="Dynamixel 每圈 tick 数",
    )
    parser.add_argument(
        "--no-wrap-ticks", action="store_true", help="不对 Present_Position 做一圈取模"
    )
    parser.add_argument(
        "--signed-position",
        action="store_true",
        help="将 Present_Position 按 int32 有符号数解释",
    )
    parser.add_argument("--crc-name", default=DEFAULT_CRC_NAME, help="CRC 名称")
    parser.add_argument(
        "--write-timeout",
        type=float,
        default=DEFAULT_WRITE_TIMEOUT_S,
        help="MCU 串口写超时时间",
    )
    parser.add_argument(
        "--print-rate",
        type=float,
        default=DEFAULT_PRINT_FREQ_HZ,
        help="调试打印频率，0 表示关闭",
    )
    parser.add_argument(
        "--dry-run", action="store_true", help="只读取主臂和打包协议，不打开 MCU 串口"
    )

    return parser


def sleep_until(next_t: float) -> float:
    """
    休眠到指定单调时间

    参数
    ----
    next_t:
        目标单调时间，单位 s

    返回
    ----
    float:
        输入的目标时间，方便调用方保留节拍变量
    """
    now = time.monotonic()
    remain = next_t - now

    if remain > 0.0:
        time.sleep(remain)

    return next_t


def format_q(q_rad: Sequence[float]) -> str:
    """
    将 q0 ~ q4 关节角格式化为调试字符串

    参数
    ----
    q_rad:
        q0 ~ q4 关节角，单位 rad

    返回
    ----
    str:
        便于打印的字符串
    """
    return "[" + ", ".join(f"{q:+.4f}" for q in q_rad) + "]"


def main() -> int:
    """
    程序入口函数

    返回
    ----
    int:
        进程退出码
        0 表示正常退出

    主循环逻辑
    ----------
    1. 读取主臂 q0 ~ q4 和 ID7 末端开关
    2. 打包并发送 PC_MASTER_JOINTS
    3. 按低频节拍发送 PC_HEARTBEAT
    4. 按调试节拍打印当前状态
    5. 单周期异常只丢弃当前周期，不直接退出程序
    """
    args = build_arg_parser().parse_args()

    if args.freq <= 0.0:
        raise SystemExit("--freq 必须大于 0")

    mapping = JointMapping(
        signs=list(args.joint_signs),
        offsets_rad=list(args.joint_offsets_rad),
        ticks_per_rev=args.ticks_per_rev,
        wrap_ticks=not args.no_wrap_ticks,
        signed_position=args.signed_position,
    )

    leader = DynamixelLeader(
        port=args.leader_port,
        baudrate=args.leader_baud,
        joint_ids=args.joint_ids,
        gripper_id=args.gripper_id,
        mapping=mapping,
        end_switch_threshold=args.end_switch_threshold,
    )
    sender = McuSerialSender(args.mcu_port, args.mcu_baud, args.write_timeout)
    packer = ProtocolPacker(args.crc_name)

    should_stop = False

    def handle_stop_signal(signum, frame) -> None:
        """
        处理中断信号

        参数
        ----
        signum:
            信号编号
        frame:
            当前栈帧

        说明
        ----
        收到 SIGINT 或 SIGTERM 后不在信号处理函数中直接关闭串口
        这里只设置 should_stop 标志，由主循环自然退出
        """
        nonlocal should_stop
        should_stop = True

    signal.signal(signal.SIGINT, handle_stop_signal)
    signal.signal(signal.SIGTERM, handle_stop_signal)

    print(f"打开主臂 Dynamixel 串口：{args.leader_port} @ {args.leader_baud}")
    print(
        f"ID{args.gripper_id} 末端开关配置："
        f"open_pos={GRIPPER_OPEN_POS}, "
        f"closed_pos={GRIPPER_CLOSED_POS}, "
        f"threshold={args.end_switch_threshold}"
    )
    leader.connect()

    if args.dry_run:
        print("dry-run 模式：不打开 MCU 串口，只读取主臂并打包协议")
    else:
        print(f"打开 MCU 串口：{args.mcu_port} @ {args.mcu_baud}")
        sender.connect()

    period_s = 1.0 / args.freq
    heartbeat = RateLimiter(args.heartbeat_rate)
    printer = RateLimiter(args.print_rate) if args.print_rate > 0.0 else None

    frames_sent = 0
    heartbeat_sent = 0
    errors = 0
    next_loop_t = time.monotonic()

    print("开始遥操作发送，按 Ctrl+C 退出")

    try:
        while not should_stop:
            loop_start = time.monotonic()

            try:
                state = leader.read_state()
                frame = packer.pack_master_joints(state.q_rad, state.end_switch)

                if not args.dry_run:
                    sender.write_frame(frame)

                frames_sent += 1

                if heartbeat.ready(loop_start):
                    heartbeat_frame = packer.pack_heartbeat()

                    if not args.dry_run:
                        sender.write_frame(heartbeat_frame)

                    heartbeat_sent += 1

                if printer is not None and printer.ready(loop_start):
                    print(
                        f"q_rad={format_q(state.q_rad)} "
                        f"gripper_raw={state.gripper_raw} "
                        f"gripper_norm={state.gripper_norm:+.3f} "
                        f"end_switch={state.end_switch} "
                        f"frames={frames_sent} "
                        f"heartbeat={heartbeat_sent} "
                        f"errors={errors}"
                    )

            except (
                RuntimeError,
                serial.SerialException,
                serial.SerialTimeoutException,
            ) as exc:
                errors += 1
                print(f"[警告] {exc}", file=sys.stderr)

                # 临时读写错误只丢弃当前周期，不阻塞太久，也不直接退出
                time.sleep(min(period_s, 0.05))

            next_loop_t += period_s
            now = time.monotonic()

            # 如果循环已经明显落后，重置节拍，避免追赶式空转
            if next_loop_t < now - period_s:
                next_loop_t = now

            sleep_until(next_loop_t)

    finally:
        print("正在停止遥操作发送")
        sender.close()
        leader.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
