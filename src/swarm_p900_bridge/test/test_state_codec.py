"""Tests for the fixed-size P900 swarm-state codec."""

import math
import struct

from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
import pytest

from swarm_p900_bridge.state_codec import (
    COMPACT_STATE_SIZE,
    CompactStateError,
    decode_compact_state,
    encode_compact_state,
)


def make_odometry():
    message = Odometry()
    message.header.frame_id = 'world'
    message.child_frame_id = '2'
    message.pose.pose.position.x = 12.25
    message.pose.pose.position.y = -3.5
    message.pose.pose.position.z = 4.75
    yaw = -2.4
    message.pose.pose.orientation.z = math.sin(yaw / 2.0)
    message.pose.pose.orientation.w = math.cos(yaw / 2.0)
    message.twist.twist.linear.x = 0.4
    message.twist.twist.linear.y = -0.7
    message.twist.twist.linear.z = 0.2
    return message


def yaw_from_message(message):
    q = message.pose.pose.orientation
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def test_compact_state_is_exactly_28_bytes_and_reconstructs_odometry():
    original = make_odometry()
    stamp = Time(sec=42, nanosec=123)

    payload = encode_compact_state(original)
    rebuilt = decode_compact_state(payload, source_node_id=2, stamp=stamp)

    assert len(payload) == COMPACT_STATE_SIZE == 28
    assert rebuilt.header.stamp == stamp
    assert rebuilt.header.frame_id == 'world'
    assert rebuilt.child_frame_id == '2'
    assert rebuilt.pose.pose.position.x == pytest.approx(12.25)
    assert rebuilt.pose.pose.position.y == pytest.approx(-3.5)
    assert rebuilt.pose.pose.position.z == pytest.approx(4.75)
    assert rebuilt.twist.twist.linear.x == pytest.approx(0.4)
    assert rebuilt.twist.twist.linear.y == pytest.approx(-0.7)
    assert rebuilt.twist.twist.linear.z == pytest.approx(0.2)
    assert yaw_from_message(rebuilt) == pytest.approx(-2.4, abs=1e-6)
    assert list(rebuilt.pose.covariance) == [0.0] * 36
    assert list(rebuilt.twist.covariance) == [0.0] * 36


@pytest.mark.parametrize('size', [0, 27, 29, 100])
def test_compact_state_rejects_wrong_payload_size(size):
    with pytest.raises(CompactStateError, match='exactly 28 bytes'):
        decode_compact_state(bytes(size), 2, Time())


def test_compact_state_rejects_non_finite_payload():
    payload = struct.pack('!7f', 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, math.nan)
    with pytest.raises(CompactStateError, match='non-finite'):
        decode_compact_state(payload, 2, Time())


def test_compact_state_rejects_invalid_quaternion_and_source():
    message = make_odometry()
    message.pose.pose.orientation.z = 0.0
    message.pose.pose.orientation.w = 0.0
    with pytest.raises(CompactStateError, match='zero length'):
        encode_compact_state(message)

    payload = encode_compact_state(make_odometry())
    with pytest.raises(CompactStateError, match='0..254'):
        decode_compact_state(payload, 255, Time())
