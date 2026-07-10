"""ASRPRO 与树莓派之间的 ASCII 串口协议工具"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple


FRAME_PREFIX = '@'
CHECKSUM_SEPARATOR = '*'
LINE_ENDING = '\r\n'


class ProtocolError(ValueError):
    """协议帧格式或校验错误"""


@dataclass(frozen=True)
class ProtocolFrame:
    """一条已经校验通过的协议帧"""

    direction: str
    version: int
    sequence: int
    command: str
    arguments: Tuple[str, ...]


def checksum_xor(payload: str) -> int:
    """计算 payload UTF-8 字节的逐字节异或校验值"""

    value = 0
    for byte in payload.encode('utf-8'):
        value ^= byte
    return value & 0xFF


def validate_token(value: str, field_name: str) -> str:
    """限制字段不能破坏逗号分隔和行结束边界"""

    token = str(value).strip()
    if not token:
        raise ProtocolError(f'{field_name} 不能为空')
    if any(char in token for char in (',', '*', '\r', '\n')):
        raise ProtocolError(f'{field_name} 包含协议保留字符')
    return token


def encode_frame(
    direction: str,
    version: int,
    sequence: int,
    command: str,
    *arguments: str,
) -> bytes:
    """编码一条带异或校验的协议帧"""

    direction_token = validate_token(direction, 'direction')
    command_token = validate_token(command, 'command').upper()
    if direction_token not in ('A2P', 'P2A'):
        raise ProtocolError(f'不支持的 direction={direction_token}')
    if not 0 <= int(sequence) <= 65535:
        raise ProtocolError(f'sequence 超出范围: {sequence}')
    tokens = [direction_token, str(int(version)), str(int(sequence)), command_token]
    tokens.extend(validate_token(item, 'argument') for item in arguments)
    payload = ','.join(tokens)
    checksum = checksum_xor(payload)
    return f'{FRAME_PREFIX}{payload}{CHECKSUM_SEPARATOR}{checksum:02X}{LINE_ENDING}'.encode('utf-8')


def decode_frame(raw_line: bytes | str, expected_direction: str | None = None) -> ProtocolFrame:
    """解析并校验一条完整协议帧"""

    if isinstance(raw_line, bytes):
        try:
            line = raw_line.decode('utf-8')
        except UnicodeDecodeError as exc:
            raise ProtocolError('协议帧不是有效 UTF-8') from exc
    else:
        line = str(raw_line)
    line = line.strip('\r\n ')
    if not line.startswith(FRAME_PREFIX):
        raise ProtocolError('协议帧缺少 @ 前缀')
    if CHECKSUM_SEPARATOR not in line:
        raise ProtocolError('协议帧缺少 * 校验分隔符')
    payload, checksum_text = line[1:].rsplit(CHECKSUM_SEPARATOR, 1)
    if len(checksum_text) != 2:
        raise ProtocolError('协议帧校验值长度错误')
    try:
        received_checksum = int(checksum_text, 16)
    except ValueError as exc:
        raise ProtocolError('协议帧校验值不是十六进制') from exc
    calculated_checksum = checksum_xor(payload)
    if received_checksum != calculated_checksum:
        raise ProtocolError(
            f'协议帧校验失败 received={received_checksum:02X} calculated={calculated_checksum:02X}'
        )
    tokens = payload.split(',')
    if len(tokens) < 4:
        raise ProtocolError('协议帧字段数量不足')
    direction = tokens[0]
    if direction not in ('A2P', 'P2A'):
        raise ProtocolError(f'未知 direction={direction}')
    if expected_direction and direction != expected_direction:
        raise ProtocolError(f'方向不匹配 expected={expected_direction} actual={direction}')
    try:
        version = int(tokens[1])
        sequence = int(tokens[2])
    except ValueError as exc:
        raise ProtocolError('version 或 sequence 不是整数') from exc
    if not 0 <= sequence <= 65535:
        raise ProtocolError(f'sequence 超出范围: {sequence}')
    command = validate_token(tokens[3], 'command').upper()
    arguments = tuple(tokens[4:])
    return ProtocolFrame(direction, version, sequence, command, arguments)
