"""Tests for P900 binary framing and stream recovery."""

import struct

from swarm_p900_bridge.transport_protocol import (
    CRC_STRUCT,
    encode_frame,
    HEADER_STRUCT,
    MAGIC,
    PROTOCOL_VERSION,
    SequenceTracker,
    StreamParser,
)


def make_frame(sequence=7, topic_id=1, payload=b'command'):
    """Return one valid station-to-drone test frame."""
    return encode_frame(
        topic_id=topic_id,
        source_role=0,
        source_node_id=0,
        destination_node_id=1,
        sequence=sequence,
        payload=payload,
        max_payload=1024,
    )


def test_valid_frame_encode_decode():
    parser = StreamParser(max_payload=1024, known_topic_ids={1})

    frames = parser.feed(make_frame())

    assert len(frames) == 1
    frame = frames[0]
    assert frame.version == PROTOCOL_VERSION
    assert frame.topic_id == 1
    assert frame.source_role == 0
    assert frame.source_node_id == 0
    assert frame.destination_node_id == 1
    assert frame.sequence == 7
    assert frame.payload == b'command'


def test_frame_split_over_multiple_serial_reads():
    encoded = make_frame(payload=b'split-message')
    parser = StreamParser(max_payload=1024, known_topic_ids={1})

    assert parser.feed(encoded[:3]) == []
    assert parser.feed(encoded[3:11]) == []
    frames = parser.feed(encoded[11:])

    assert [frame.payload for frame in frames] == [b'split-message']


def test_several_frames_in_one_read():
    parser = StreamParser(max_payload=1024, known_topic_ids={1})
    data = b''.join(make_frame(sequence=index) for index in range(3))

    frames = parser.feed(data)

    assert [frame.sequence for frame in frames] == [0, 1, 2]


def test_garbage_before_magic_is_discarded():
    parser = StreamParser(max_payload=1024, known_topic_ids={1})

    frames = parser.feed(b'noise\x00\xff' + make_frame())

    assert len(frames) == 1
    assert parser.stats.discarded_bytes >= 7


def test_corrupted_crc_is_discarded():
    encoded = bytearray(make_frame())
    encoded[-1] ^= 0x80
    parser = StreamParser(max_payload=1024, known_topic_ids={1})

    assert parser.feed(encoded) == []
    assert parser.stats.crc_errors == 1


def test_invalid_payload_length_is_rejected_before_payload_wait():
    parser = StreamParser(max_payload=32, known_topic_ids={1})
    invalid_header = HEADER_STRUCT.pack(
        MAGIC,
        PROTOCOL_VERSION,
        1,
        0,
        0,
        1,
        10,
        33,
    )

    assert parser.feed(invalid_header) == []
    assert parser.stats.invalid_lengths == 1
    assert parser.stats.malformed_frames == 1


def test_unknown_topic_id_is_discarded_cleanly():
    parser = StreamParser(max_payload=1024, known_topic_ids={1})

    assert parser.feed(make_frame(topic_id=99)) == []
    assert parser.stats.unknown_topic_ids == 1
    assert parser.stats.crc_errors == 0


def test_parser_recovers_after_corrupted_frame():
    bad = bytearray(make_frame(sequence=4, payload=b'bad'))
    bad[-CRC_STRUCT.size] ^= 0x01
    good = make_frame(sequence=5, payload=b'good')
    parser = StreamParser(max_payload=1024, known_topic_ids={1})

    frames = parser.feed(bytes(bad) + good)

    assert [frame.payload for frame in frames] == [b'good']
    assert parser.stats.crc_errors == 1


def test_parser_recovers_from_damaged_in_range_length():
    bad = bytearray(make_frame(sequence=4, payload=b'bad'))
    length_offset = HEADER_STRUCT.size - struct.calcsize('!I')
    bad[length_offset:HEADER_STRUCT.size] = struct.pack('!I', 900)
    good = make_frame(sequence=5, payload=b'good')
    parser = StreamParser(max_payload=1024, known_topic_ids={1})

    frames = parser.feed(bytes(bad) + good)

    assert [frame.payload for frame in frames] == [b'good']
    assert parser.stats.malformed_frames == 1


def test_sequence_gap_duplicate_and_out_of_order_are_diagnostic_only():
    tracker = SequenceTracker()

    assert tracker.observe(0, 1, 10).status == 'first'
    gap = tracker.observe(0, 1, 13)
    assert gap.status == 'gap'
    assert gap.missing == 2
    assert tracker.observe(0, 1, 13).status == 'duplicate'
    assert tracker.observe(0, 1, 12).status == 'out_of_order'
    assert tracker.stats.missing == 2
    assert tracker.stats.duplicates == 1
    assert tracker.stats.out_of_order == 1


def test_sequence_wraparound_is_in_order():
    tracker = SequenceTracker()

    tracker.observe(0, 1, 0xFFFFFFFF)

    assert tracker.observe(0, 1, 0).status == 'in_order'
