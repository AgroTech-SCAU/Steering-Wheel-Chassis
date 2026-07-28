import pytest

from atlas_asrpro_bridge.protocol import ProtocolError, decode_frame, encode_frame


def test_round_trip():
    raw = encode_frame('A2P', 1, 42, 'SPEAK', 'autonomous_start')
    frame = decode_frame(raw, expected_direction='A2P')
    assert frame.version == 1
    assert frame.sequence == 42
    assert frame.command == 'SPEAK'
    assert frame.arguments == ('autonomous_start',)


def test_round_trip_utf8_payload():
    raw = encode_frame('P2A', 1, 8, 'EVENT', 'ASR', '阿特拉斯启动')
    frame = decode_frame(raw, expected_direction='P2A')
    assert frame.arguments == ('ASR', '阿特拉斯启动')


def test_rejects_corrupt_checksum():
    raw = bytearray(encode_frame('P2A', 1, 7, 'EVENT', 'ASR', 'atlas_start'))
    raw[-4] = ord('0') if raw[-4] != ord('0') else ord('1')
    with pytest.raises(ProtocolError):
        decode_frame(bytes(raw), expected_direction='P2A')


def test_rejects_wrong_direction():
    raw = encode_frame('A2P', 1, 9, 'PING', '123')
    with pytest.raises(ProtocolError):
        decode_frame(raw, expected_direction='P2A')


def test_rejects_reserved_character():
    with pytest.raises(ProtocolError):
        encode_frame('A2P', 1, 10, 'SPEAK', 'bad,phrase')


def test_rejects_sequence_out_of_range():
    with pytest.raises(ProtocolError):
        encode_frame('A2P', 1, 65536, 'PING', '123')
