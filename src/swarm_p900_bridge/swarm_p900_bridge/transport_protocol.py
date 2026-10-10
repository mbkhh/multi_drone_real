"""Binary framing, stream parsing, and sequence diagnostics for P900."""

from collections import defaultdict, deque
from dataclasses import dataclass
import struct
import zlib


MAGIC = b'P9R2'
PROTOCOL_VERSION = 1
BROADCAST_NODE_ID = 255
DEFAULT_MAX_PAYLOAD = 65536

# Network byte order:
# magic[4], version[u8], topic[u16], source_role[u8], source_node[u8],
# destination_node[u8], sequence[u32], payload_length[u32].
HEADER_STRUCT = struct.Struct('!4sBHBBBII')
CRC_STRUCT = struct.Struct('!I')
FRAME_OVERHEAD = HEADER_STRUCT.size + CRC_STRUCT.size


class ProtocolError(ValueError):
    """Raised when a caller tries to encode an invalid frame."""


@dataclass(frozen=True)
class Frame:
    """One validated P900 application frame."""

    topic_id: int
    source_role: int
    source_node_id: int
    destination_node_id: int
    sequence: int
    payload: bytes
    version: int = PROTOCOL_VERSION

    @property
    def wire_size(self):
        """Return encoded frame size in bytes."""
        return FRAME_OVERHEAD + len(self.payload)


@dataclass
class ParserStats:
    """Cumulative parser rejection and resynchronization counters."""

    frames: int = 0
    crc_errors: int = 0
    malformed_frames: int = 0
    invalid_lengths: int = 0
    unsupported_versions: int = 0
    unknown_topic_ids: int = 0
    discarded_bytes: int = 0


def _uint(value, maximum, field_name):
    try:
        result = int(value)
    except (TypeError, ValueError, OverflowError) as error:
        raise ProtocolError(f'{field_name} must be an integer') from error
    if not 0 <= result <= maximum:
        raise ProtocolError(
            f'{field_name} must be in the range 0..{maximum}'
        )
    return result


def encode_frame(
    *,
    topic_id,
    source_role,
    source_node_id,
    destination_node_id,
    sequence,
    payload,
    max_payload=DEFAULT_MAX_PAYLOAD,
):
    """Encode one payload into a versioned, CRC-protected binary frame."""
    if not isinstance(payload, (bytes, bytearray, memoryview)):
        raise ProtocolError('payload must be bytes-like')
    payload = bytes(payload)
    max_payload = _uint(max_payload, 0xFFFFFFFF, 'max_payload')
    if len(payload) > max_payload:
        raise ProtocolError(
            f'payload has {len(payload)} bytes; maximum is {max_payload}'
        )

    header = HEADER_STRUCT.pack(
        MAGIC,
        PROTOCOL_VERSION,
        _uint(topic_id, 0xFFFF, 'topic_id'),
        _uint(source_role, 0xFF, 'source_role'),
        _uint(source_node_id, 0xFF, 'source_node_id'),
        _uint(destination_node_id, 0xFF, 'destination_node_id'),
        _uint(sequence, 0xFFFFFFFF, 'sequence'),
        len(payload),
    )
    crc = zlib.crc32(header + payload) & 0xFFFFFFFF
    return header + payload + CRC_STRUCT.pack(crc)


class StreamParser:
    """Incrementally recover complete frames from arbitrary serial chunks."""

    def __init__(
        self,
        *,
        max_payload=DEFAULT_MAX_PAYLOAD,
        known_topic_ids=None,
    ):
        self.max_payload = _uint(
            max_payload, 0xFFFFFFFF, 'max_payload'
        )
        self.known_topic_ids = (
            None if known_topic_ids is None else frozenset(known_topic_ids)
        )
        self.buffer = bytearray()
        self.stats = ParserStats()

    def feed(self, data):
        """Consume bytes and return every complete, valid, known frame."""
        if not isinstance(data, (bytes, bytearray, memoryview)):
            raise TypeError('serial data must be bytes-like')
        if data:
            self.buffer.extend(data)

        frames = []
        while True:
            if not self._align_to_magic():
                break
            if len(self.buffer) < HEADER_STRUCT.size:
                break

            header_values = HEADER_STRUCT.unpack_from(self.buffer)
            (
                magic,
                version,
                topic_id,
                source_role,
                source_node_id,
                destination_node_id,
                sequence,
                payload_length,
            ) = header_values

            if magic != MAGIC:
                # _align_to_magic() makes this impossible unless the buffer is
                # modified incorrectly; retain a safe recovery path.
                self._reject_leading_byte()
                continue
            if version != PROTOCOL_VERSION:
                self.stats.unsupported_versions += 1
                self.stats.malformed_frames += 1
                self._reject_leading_byte()
                continue
            if payload_length > self.max_payload:
                self.stats.invalid_lengths += 1
                self.stats.malformed_frames += 1
                self._reject_leading_byte()
                continue

            frame_size = FRAME_OVERHEAD + payload_length
            if len(self.buffer) < frame_size:
                # A damaged in-range length could otherwise make the parser
                # wait through many later packets. Only jump to a later MAGIC
                # if it already begins a complete frame with a valid CRC.
                candidate = self._find_later_valid_frame()
                if candidate is not None:
                    self.stats.malformed_frames += 1
                    self.stats.discarded_bytes += candidate
                    del self.buffer[:candidate]
                    continue
                break

            frame_bytes = bytes(self.buffer[:frame_size])
            payload_end = HEADER_STRUCT.size + payload_length
            received_crc = CRC_STRUCT.unpack_from(
                frame_bytes, payload_end
            )[0]
            calculated_crc = zlib.crc32(
                frame_bytes[:payload_end]
            ) & 0xFFFFFFFF
            if received_crc != calculated_crc:
                self.stats.crc_errors += 1
                self._reject_leading_byte()
                continue

            del self.buffer[:frame_size]
            if (
                self.known_topic_ids is not None
                and topic_id not in self.known_topic_ids
            ):
                self.stats.unknown_topic_ids += 1
                continue

            frames.append(
                Frame(
                    version=version,
                    topic_id=topic_id,
                    source_role=source_role,
                    source_node_id=source_node_id,
                    destination_node_id=destination_node_id,
                    sequence=sequence,
                    payload=frame_bytes[
                        HEADER_STRUCT.size:payload_end
                    ],
                )
            )
            self.stats.frames += 1

        return frames

    def _align_to_magic(self):
        if len(self.buffer) < len(MAGIC):
            return False
        position = self.buffer.find(MAGIC)
        if position < 0:
            keep = len(MAGIC) - 1
            discard = max(0, len(self.buffer) - keep)
            if discard:
                self.stats.discarded_bytes += discard
                del self.buffer[:discard]
            return False
        if position:
            self.stats.discarded_bytes += position
            del self.buffer[:position]
        return True

    def _reject_leading_byte(self):
        self.stats.discarded_bytes += 1
        del self.buffer[0]

    def _find_later_valid_frame(self):
        position = self.buffer.find(MAGIC, 1)
        while position >= 0:
            available = len(self.buffer) - position
            if available >= HEADER_STRUCT.size:
                values = HEADER_STRUCT.unpack_from(self.buffer, position)
                version = values[1]
                payload_length = values[-1]
                candidate_size = FRAME_OVERHEAD + payload_length
                if (
                    version == PROTOCOL_VERSION
                    and payload_length <= self.max_payload
                    and available >= candidate_size
                ):
                    end = position + candidate_size
                    payload_end = end - CRC_STRUCT.size
                    received_crc = CRC_STRUCT.unpack_from(
                        self.buffer, payload_end
                    )[0]
                    calculated_crc = zlib.crc32(
                        self.buffer[position:payload_end]
                    ) & 0xFFFFFFFF
                    if received_crc == calculated_crc:
                        return position
            position = self.buffer.find(MAGIC, position + 1)
        return None


@dataclass(frozen=True)
class SequenceObservation:
    """Classification returned for one received sequence number."""

    status: str
    missing: int = 0


@dataclass
class SequenceStats:
    """Cumulative receive sequence diagnostics."""

    missing: int = 0
    duplicates: int = 0
    out_of_order: int = 0


class SequenceTracker:
    """Track gaps without rejecting useful frames."""

    def __init__(self, history_size=64):
        if history_size < 1:
            raise ValueError('history_size must be positive')
        self.history_size = history_size
        self.expected = {}
        self.recent = defaultdict(lambda: deque(maxlen=history_size))
        self.stats = SequenceStats()

    def observe(self, source_node_id, topic_id, sequence):
        """Record a frame and classify it as first/in-order/gap/etc."""
        key = (int(source_node_id), int(topic_id))
        sequence = _uint(sequence, 0xFFFFFFFF, 'sequence')
        recent = self.recent[key]
        if sequence in recent:
            self.stats.duplicates += 1
            return SequenceObservation('duplicate')

        expected = self.expected.get(key)
        if expected is None:
            self.expected[key] = (sequence + 1) & 0xFFFFFFFF
            recent.append(sequence)
            return SequenceObservation('first')

        if sequence == expected:
            self.expected[key] = (sequence + 1) & 0xFFFFFFFF
            recent.append(sequence)
            return SequenceObservation('in_order')

        forward_distance = (sequence - expected) & 0xFFFFFFFF
        if forward_distance < 0x80000000:
            self.stats.missing += forward_distance
            self.expected[key] = (sequence + 1) & 0xFFFFFFFF
            recent.append(sequence)
            return SequenceObservation('gap', missing=forward_distance)

        self.stats.out_of_order += 1
        recent.append(sequence)
        return SequenceObservation('out_of_order')
