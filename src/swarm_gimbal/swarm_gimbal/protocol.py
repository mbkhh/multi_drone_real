"""Minimal SIYI SDK binary framing used by the ZR10 UDP interface."""

from dataclasses import dataclass
import struct
from typing import List


MAGIC = b'\x55\x66'
HEADER_SIZE = 8
CRC_SIZE = 2
MIN_FRAME_SIZE = HEADER_SIZE + CRC_SIZE
MAX_PAYLOAD_SIZE = 1024
CTRL_NEED_ACK = 0x01


def crc16_xmodem(data: bytes) -> int:
    """Return the CRC-16/XMODEM checksum required by the SIYI protocol."""
    crc = 0
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


@dataclass(frozen=True)
class SiyiFrame:
    """One decoded SIYI SDK frame."""

    control: int
    sequence: int
    command_id: int
    payload: bytes


def encode_frame(
    command_id: int,
    payload: bytes,
    sequence: int,
    need_ack: bool = True,
) -> bytes:
    """Encode a command and payload into a complete SIYI frame."""
    if not 0 <= command_id <= 0xFF:
        raise ValueError('command_id must be in [0, 255]')
    if not 0 <= sequence <= 0xFFFF:
        raise ValueError('sequence must be in [0, 65535]')
    if len(payload) > MAX_PAYLOAD_SIZE:
        raise ValueError('payload is too large')

    control = CTRL_NEED_ACK if need_ack else 0
    header = struct.pack(
        '<2sBHHB',
        MAGIC,
        control,
        len(payload),
        sequence,
        command_id,
    )
    without_crc = header + payload
    return without_crc + struct.pack('<H', crc16_xmodem(without_crc))


class FrameParser:
    """Incremental parser with CRC checking and stream resynchronization."""

    def __init__(self, max_payload_size: int = MAX_PAYLOAD_SIZE) -> None:
        self.max_payload_size = max_payload_size
        self._buffer = bytearray()
        self.crc_errors = 0
        self.malformed_frames = 0
        self.discarded_bytes = 0

    def reset(self) -> None:
        """Discard all buffered bytes."""
        self._buffer.clear()

    def feed(self, data: bytes) -> List[SiyiFrame]:
        """Consume any number of bytes and return every complete valid frame."""
        if data:
            self._buffer.extend(data)

        frames: List[SiyiFrame] = []
        while True:
            magic_index = self._buffer.find(MAGIC)
            if magic_index < 0:
                # Preserve a trailing 0x55 because it may be the first magic byte.
                keep = 1 if self._buffer.endswith(MAGIC[:1]) else 0
                discarded = len(self._buffer) - keep
                if discarded > 0:
                    self.discarded_bytes += discarded
                    del self._buffer[:discarded]
                break

            if magic_index > 0:
                self.discarded_bytes += magic_index
                del self._buffer[:magic_index]

            if len(self._buffer) < HEADER_SIZE:
                break

            _, control, payload_length, sequence, command_id = struct.unpack(
                '<2sBHHB', self._buffer[:HEADER_SIZE]
            )
            if payload_length > self.max_payload_size:
                self.malformed_frames += 1
                self.discarded_bytes += 1
                del self._buffer[0]
                continue

            frame_size = HEADER_SIZE + payload_length + CRC_SIZE
            if len(self._buffer) < frame_size:
                break

            candidate = bytes(self._buffer[:frame_size])
            expected_crc = struct.unpack('<H', candidate[-CRC_SIZE:])[0]
            actual_crc = crc16_xmodem(candidate[:-CRC_SIZE])
            if actual_crc != expected_crc:
                self.crc_errors += 1
                self.discarded_bytes += 1
                del self._buffer[0]
                continue

            payload = candidate[HEADER_SIZE:-CRC_SIZE]
            frames.append(
                SiyiFrame(
                    control=control,
                    sequence=sequence,
                    command_id=command_id,
                    payload=payload,
                )
            )
            del self._buffer[:frame_size]

        return frames
