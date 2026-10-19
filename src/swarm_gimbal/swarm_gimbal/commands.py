"""SIYI ZR10 command payloads and response decoders."""

import math
import struct
from typing import Any, Dict

from .protocol import SiyiFrame


CMD_FIRMWARE_VERSION = 0x01
CMD_GIMBAL_ROTATION = 0x07
CMD_ONE_KEY_CENTERING = 0x08
CMD_CAMERA_SYSTEM_INFO = 0x0A
CMD_FUNCTION_FEEDBACK = 0x0B
CMD_CAMERA_FUNCTION = 0x0C
CMD_GIMBAL_ATTITUDE = 0x0D
CMD_SET_GIMBAL_ATTITUDE = 0x0E
CMD_GIMBAL_MODE = 0x19

YAW_MIN_DEG = -135.0
YAW_MAX_DEG = 135.0
PITCH_MIN_DEG = -90.0
PITCH_MAX_DEG = 25.0

GIMBAL_MODES = {0: 'lock', 1: 'follow', 2: 'fpv'}
MOUNT_DIRECTIONS = {0: 'reserved', 1: 'normal', 2: 'inverted'}
RECORD_STATES = {
    0: 'not_recording',
    1: 'recording',
    2: 'no_tf_card',
    3: 'video_data_loss',
}
VIDEO_OUTPUT_STATES = {0: 'hdmi', 1: 'cvbs'}
FUNCTION_FEEDBACK = {
    0: 'photo_success',
    1: 'photo_failed',
    2: 'hdr_on',
    3: 'hdr_off',
    4: 'recording_failed',
    5: 'recording_started',
    6: 'recording_stopped',
}


def _finite_number(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{name} must be a number')
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f'{name} must be finite')
    return result


def speed_payload(yaw: Any, pitch: Any) -> bytes:
    """Build the two signed speed bytes used by command 0x07."""
    yaw_value = round(_finite_number(yaw, 'yaw'))
    pitch_value = round(_finite_number(pitch, 'pitch'))
    if not -100 <= yaw_value <= 100:
        raise ValueError('yaw speed must be in [-100, 100]')
    if not -100 <= pitch_value <= 100:
        raise ValueError('pitch speed must be in [-100, 100]')
    return struct.pack('<bb', yaw_value, pitch_value)


def attitude_payload(yaw_deg: Any, pitch_deg: Any) -> bytes:
    """Build a ZR10 absolute target-angle payload with limit validation."""
    yaw = _finite_number(yaw_deg, 'yaw_deg')
    pitch = _finite_number(pitch_deg, 'pitch_deg')
    if not YAW_MIN_DEG <= yaw <= YAW_MAX_DEG:
        raise ValueError(
            f'yaw_deg must be in [{YAW_MIN_DEG}, {YAW_MAX_DEG}] for ZR10'
        )
    if not PITCH_MIN_DEG <= pitch <= PITCH_MAX_DEG:
        raise ValueError(
            f'pitch_deg must be in [{PITCH_MIN_DEG}, {PITCH_MAX_DEG}] for ZR10'
        )
    return struct.pack('<hh', round(yaw * 10.0), round(pitch * 10.0))


def mode_payload(mode: str) -> bytes:
    """Build the camera-function byte that selects a gimbal motion mode."""
    functions = {'lock': 3, 'follow': 4, 'fpv': 5}
    key = str(mode).strip().lower()
    if key not in functions:
        raise ValueError('mode must be one of: lock, follow, fpv')
    return struct.pack('<B', functions[key])


def decode_response(frame: SiyiFrame) -> Dict[str, Any]:
    """Decode status-bearing responses into fields suitable for JSON status."""
    payload = frame.payload
    command_id = frame.command_id

    if command_id == CMD_GIMBAL_ATTITUDE:
        if len(payload) != 12:
            raise ValueError('gimbal attitude response must be 12 bytes')
        yaw, pitch, roll, yaw_rate, pitch_rate, roll_rate = struct.unpack(
            '<hhhhhh', payload
        )
        return {
            'attitude': {
                'yaw_deg': yaw / 10.0,
                'pitch_deg': pitch / 10.0,
                'roll_deg': roll / 10.0,
                'yaw_rate_dps': yaw_rate / 10.0,
                'pitch_rate_dps': pitch_rate / 10.0,
                'roll_rate_dps': roll_rate / 10.0,
            }
        }

    if command_id == CMD_SET_GIMBAL_ATTITUDE:
        if len(payload) != 6:
            raise ValueError('set-attitude response must be 6 bytes')
        yaw, pitch, roll = struct.unpack('<hhh', payload)
        return {
            'attitude': {
                'yaw_deg': yaw / 10.0,
                'pitch_deg': pitch / 10.0,
                'roll_deg': roll / 10.0,
            },
            'ack': {'command': 'angle', 'success': True},
        }

    if command_id == CMD_CAMERA_SYSTEM_INFO:
        if len(payload) not in (7, 8):
            raise ValueError('camera system-info response must be 7 or 8 bytes')
        values = struct.unpack('<' + ('B' * len(payload)), payload)
        zoom_linkage = values[7] if len(values) == 8 else 0
        return {
            'camera': {
                'hdr_enabled': bool(values[1]),
                'recording_state': RECORD_STATES.get(
                    values[3], f'unknown_{values[3]}'
                ),
                'gimbal_mode': GIMBAL_MODES.get(
                    values[4], f'unknown_{values[4]}'
                ),
                'mounting_direction': MOUNT_DIRECTIONS.get(
                    values[5], f'unknown_{values[5]}'
                ),
                'video_output': VIDEO_OUTPUT_STATES.get(
                    values[6], f'unknown_{values[6]}'
                ),
                'zoom_linkage': bool(zoom_linkage),
            }
        }

    if command_id == CMD_GIMBAL_MODE:
        if len(payload) != 1:
            raise ValueError('gimbal-mode response must be 1 byte')
        return {
            'camera': {
                'gimbal_mode': GIMBAL_MODES.get(
                    payload[0], f'unknown_{payload[0]}'
                )
            }
        }

    if command_id == CMD_FIRMWARE_VERSION:
        if len(payload) not in (8, 12):
            raise ValueError('firmware response must be 8 or 12 bytes')
        words = struct.unpack('<' + ('I' * (len(payload) // 4)), payload)
        names = ('camera', 'gimbal', 'zoom')
        return {
            'firmware': {
                name: _format_firmware(word)
                for name, word in zip(names, words)
            }
        }

    if command_id == CMD_FUNCTION_FEEDBACK:
        if len(payload) != 1:
            raise ValueError('function-feedback response must be 1 byte')
        return {
            'function_feedback': FUNCTION_FEEDBACK.get(
                payload[0], f'unknown_{payload[0]}'
            )
        }

    if command_id in (CMD_GIMBAL_ROTATION, CMD_ONE_KEY_CENTERING):
        if len(payload) != 1:
            raise ValueError('gimbal command acknowledgement must be 1 byte')
        command = (
            'speed'
            if command_id == CMD_GIMBAL_ROTATION
            else 'center_or_down'
        )
        return {
            'ack': {'command': command, 'success': bool(payload[0])}
        }

    return {}


def _format_firmware(word: int) -> str:
    if word == 0:
        return 'n/a'
    major = (word >> 16) & 0xFF
    minor = (word >> 8) & 0xFF
    patch = word & 0xFF
    return f'v{major}.{minor}.{patch}'
