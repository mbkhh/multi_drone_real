import struct

import pytest

from swarm_gimbal.commands import (
    attitude_payload,
    CMD_CAMERA_SYSTEM_INFO,
    CMD_GIMBAL_ATTITUDE,
    decode_response,
    mode_payload,
    speed_payload,
)
from swarm_gimbal.protocol import SiyiFrame


def test_speed_payload_and_validation():
    assert speed_payload(50, -50) == b'\x32\xce'
    with pytest.raises(ValueError):
        speed_payload(101, 0)


def test_zr10_angle_limits():
    assert attitude_payload(30.0, -45.0) == struct.pack('<hh', 300, -450)
    with pytest.raises(ValueError):
        attitude_payload(136.0, 0.0)
    with pytest.raises(ValueError):
        attitude_payload(0.0, 26.0)


def test_mode_payload():
    assert mode_payload('lock') == b'\x03'
    assert mode_payload('follow') == b'\x04'
    assert mode_payload('fpv') == b'\x05'


def test_decode_attitude():
    payload = struct.pack('<hhhhhh', 123, -456, 7, 12, -20, 0)
    frame = SiyiFrame(1, 0, CMD_GIMBAL_ATTITUDE, payload)

    attitude = decode_response(frame)['attitude']

    assert attitude['yaw_deg'] == 12.3
    assert attitude['pitch_deg'] == -45.6
    assert attitude['pitch_rate_dps'] == -2.0


def test_decode_camera_info():
    payload = bytes((0, 1, 0, 1, 1, 1, 0, 1))
    frame = SiyiFrame(1, 0, CMD_CAMERA_SYSTEM_INFO, payload)

    camera = decode_response(frame)['camera']

    assert camera['hdr_enabled'] is True
    assert camera['recording_state'] == 'recording'
    assert camera['gimbal_mode'] == 'follow'
    assert camera['mounting_direction'] == 'normal'
