"""Compact P900-only codec for ROS-facing swarm Odometry messages."""

import math
import struct

from nav_msgs.msg import Odometry


# Network byte order, with no implicit alignment or padding:
# position xyz, linear velocity xyz, ENU yaw.
COMPACT_STATE_STRUCT = struct.Struct('!7f')
COMPACT_STATE_SIZE = COMPACT_STATE_STRUCT.size


class CompactStateError(ValueError):
    """Raised when a local-state message or payload is not safe to use."""


def _finite(values):
    return all(math.isfinite(float(value)) for value in values)


def _normalized_yaw_from_quaternion(orientation):
    values = (
        float(orientation.x),
        float(orientation.y),
        float(orientation.z),
        float(orientation.w),
    )
    if not _finite(values):
        raise CompactStateError('orientation contains a non-finite value')
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 1e-9:
        raise CompactStateError('orientation quaternion has zero length')
    x, y, z, w = (value / norm for value in values)
    yaw = math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )
    return math.atan2(math.sin(yaw), math.cos(yaw))


def encode_compact_state(message):
    """Encode the seven swarm-state values from an Odometry message."""
    position = message.pose.pose.position
    velocity = message.twist.twist.linear
    values = (
        float(position.x),
        float(position.y),
        float(position.z),
        float(velocity.x),
        float(velocity.y),
        float(velocity.z),
        _normalized_yaw_from_quaternion(message.pose.pose.orientation),
    )
    if not _finite(values):
        raise CompactStateError('local state contains a non-finite value')
    try:
        return COMPACT_STATE_STRUCT.pack(*values)
    except (OverflowError, struct.error) as error:
        raise CompactStateError(
            f'local state is outside float32 range: {error}'
        ) from error


def decode_compact_state(payload, source_node_id, stamp):
    """Validate compact state and reconstruct normal ROS Odometry."""
    if len(payload) != COMPACT_STATE_SIZE:
        raise CompactStateError(
            f'compact state must be exactly {COMPACT_STATE_SIZE} bytes; '
            f'received {len(payload)}'
        )
    try:
        source_node_id = int(source_node_id)
    except (TypeError, ValueError, OverflowError) as error:
        raise CompactStateError('source node ID is not an integer') from error
    if not 0 <= source_node_id <= 254:
        raise CompactStateError('source node ID must be in 0..254')

    values = COMPACT_STATE_STRUCT.unpack(payload)
    if not _finite(values):
        raise CompactStateError('compact state contains a non-finite value')
    x, y, z, velocity_x, velocity_y, velocity_z, yaw = values
    yaw = math.atan2(math.sin(yaw), math.cos(yaw))

    message = Odometry()
    message.header.stamp = stamp
    message.header.frame_id = 'world'
    message.child_frame_id = str(source_node_id)
    message.pose.pose.position.x = float(x)
    message.pose.pose.position.y = float(y)
    message.pose.pose.position.z = float(z)
    message.pose.pose.orientation.x = 0.0
    message.pose.pose.orientation.y = 0.0
    message.pose.pose.orientation.z = math.sin(yaw / 2.0)
    message.pose.pose.orientation.w = math.cos(yaw / 2.0)
    message.twist.twist.linear.x = float(velocity_x)
    message.twist.twist.linear.y = float(velocity_y)
    message.twist.twist.linear.z = float(velocity_z)
    # Angular velocity and both covariance arrays retain ROS zero defaults.
    return message
