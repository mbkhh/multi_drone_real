import struct

from swarm_gimbal.protocol import (
    crc16_xmodem,
    encode_frame,
    FrameParser,
    MAGIC,
)


def test_encode_and_parse_round_trip():
    encoded = encode_frame(0x07, b'\x32\xce', 42)
    frames = FrameParser().feed(encoded)

    assert len(frames) == 1
    assert frames[0].command_id == 0x07
    assert frames[0].sequence == 42
    assert frames[0].payload == b'\x32\xce'


def test_split_frame_and_multiple_frames():
    first = encode_frame(0x0D, b'', 1)
    second = encode_frame(0x19, b'', 2)
    parser = FrameParser()

    assert parser.feed(first[:4]) == []
    frames = parser.feed(first[4:] + second)

    assert [frame.command_id for frame in frames] == [0x0D, 0x19]


def test_garbage_crc_failure_and_recovery():
    corrupt = bytearray(encode_frame(0x07, b'\x01\x02', 3))
    corrupt[-1] ^= 0xFF
    valid = encode_frame(0x0D, b'', 4)
    parser = FrameParser()

    frames = parser.feed(b'garbage' + bytes(corrupt) + valid)

    assert len(frames) == 1
    assert frames[0].sequence == 4
    assert parser.crc_errors == 1
    assert parser.discarded_bytes >= 8


def test_oversized_payload_length_is_rejected_and_parser_recovers():
    invalid_header = struct.pack('<2sBHHB', MAGIC, 1, 4096, 8, 0x0D)
    valid = encode_frame(0x19, b'', 9)
    parser = FrameParser(max_payload_size=64)

    frames = parser.feed(invalid_header + valid)

    assert len(frames) == 1
    assert frames[0].sequence == 9
    assert parser.malformed_frames == 1


def test_crc_reference_vector():
    assert crc16_xmodem(bytes.fromhex('556601010000000000')) == 0x8B59
