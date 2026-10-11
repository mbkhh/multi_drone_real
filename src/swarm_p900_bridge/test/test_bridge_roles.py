"""ROS integration smoke tests for all bridge roles."""

from nav_msgs.msg import Odometry
import rclpy
from std_msgs.msg import Header, String

from swarm_p900_bridge.bridge_node import P900BridgeNode
from swarm_p900_bridge.roles import role_code, ROLE_FOLLOWER
from swarm_p900_bridge.state_codec import (
    decode_compact_state,
    encode_compact_state,
)
from swarm_p900_bridge.topic_registry import (
    COMMAND_TOPIC_ID,
    FOLLOWER_COMMAND_TOPIC_ID,
    FORMATION_COMMAND_TOPIC_ID,
    FORMATION_TOPIC_ID,
    LIVENESS_TOPIC_ID,
    LOCAL_STATE_TOPIC_ID,
    STATUS_TOPIC_ID,
    VISION_COMMAND_TOPIC_ID,
    VISION_TRIGGER_TOPIC_ID,
)
from swarm_p900_bridge.transport_protocol import (
    BROADCAST_NODE_ID,
    Frame,
)


def make_node(role, node_id):
    """Create one loopback-backed bridge with ROS parameter overrides."""
    arguments = [
        '--ros-args',
        '-p', f'role:={role}',
        '-p', 'serial_port:=loop://',
        '-p', f'node_id:={node_id}',
    ]
    rclpy.init(args=arguments)
    return P900BridgeNode()


def destroy_node(node):
    """Destroy a test node while its ROS context is still valid."""
    if node is not None:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def test_station_role_creates_expected_topic_directions():
    node = None
    try:
        node = make_node('station', 100)
        assert [route.topic_id for route in node.tx_routes] == [
            COMMAND_TOPIC_ID,
            FORMATION_COMMAND_TOPIC_ID,
        ]
        assert [route.topic_id for route in node.rx_routes] == [
            STATUS_TOPIC_ID,
        ]
    finally:
        destroy_node(node)


def test_leader_role_creates_expected_topic_directions():
    node = None
    try:
        node = make_node('leader', 1)
        assert [route.topic_id for route in node.tx_routes] == [
            STATUS_TOPIC_ID,
            FOLLOWER_COMMAND_TOPIC_ID,
            FORMATION_TOPIC_ID,
            LIVENESS_TOPIC_ID,
            LOCAL_STATE_TOPIC_ID,
            VISION_TRIGGER_TOPIC_ID,
        ]
        assert [route.topic_id for route in node.rx_routes] == [
            COMMAND_TOPIC_ID,
            FORMATION_COMMAND_TOPIC_ID,
            LIVENESS_TOPIC_ID,
            LOCAL_STATE_TOPIC_ID,
            VISION_COMMAND_TOPIC_ID,
        ]
    finally:
        destroy_node(node)


def test_follower_role_creates_expected_topic_directions():
    node = None
    try:
        node = make_node('follower', 2)
        assert [route.topic_id for route in node.tx_routes] == [
            LIVENESS_TOPIC_ID,
            LOCAL_STATE_TOPIC_ID,
            VISION_COMMAND_TOPIC_ID,
        ]
        assert [route.topic_id for route in node.rx_routes] == [
            FOLLOWER_COMMAND_TOPIC_ID,
            FORMATION_TOPIC_ID,
            LIVENESS_TOPIC_ID,
            LOCAL_STATE_TOPIC_ID,
            VISION_TRIGGER_TOPIC_ID,
        ]
    finally:
        destroy_node(node)


def test_peer_identity_prevents_liveness_feedback():
    node = None
    try:
        node = make_node('follower', 2)
        route = next(
            item for item in node.tx_routes
            if item.topic_id == LIVENESS_TOPIC_ID
        )
        assert node._message_is_local_origin(route, Header(frame_id='2'))
        assert not node._message_is_local_origin(route, Header(frame_id='3'))
        assert node._counters.loop_suppressed == 1
    finally:
        destroy_node(node)


def make_state(source, x):
    message = Odometry()
    message.header.frame_id = 'world'
    message.child_frame_id = str(source)
    message.pose.pose.position.x = float(x)
    message.pose.pose.orientation.w = 1.0
    return message


def test_latest_state_replaces_intermediate_ros_samples():
    node = None
    try:
        node = make_node('follower', 2)
        route = next(
            item for item in node.tx_routes
            if item.topic_id == LOCAL_STATE_TOPIC_ID
        )
        for x in (1.0, 2.0, 5.0):
            node._transmit_message(route, make_state(2, x))

        assert tuple(node._latest_pending) == ((LOCAL_STATE_TOPIC_ID, 2),)
        pending = node._latest_pending[(LOCAL_STATE_TOPIC_ID, 2)]
        rebuilt = decode_compact_state(
            pending.payload, 2, node.get_clock().now().to_msg()
        )
        assert rebuilt.pose.pose.position.x == 5.0
        assert node._counters.state_samples_replaced == 2
    finally:
        destroy_node(node)


def test_leader_preserves_follower_identity_and_encoded_payload_for_relay():
    node = None
    try:
        node = make_node('leader', 1)
        payload = encode_compact_state(make_state(2, 4.5))
        frame = Frame(
            topic_id=LOCAL_STATE_TOPIC_ID,
            source_role=role_code(ROLE_FOLLOWER),
            source_node_id=2,
            destination_node_id=1,
            sequence=73,
            payload=payload,
        )

        node._publish_frame(frame)

        pending = node._latest_pending[(LOCAL_STATE_TOPIC_ID, 2)]
        assert pending.payload == payload
        assert pending.source_node_id == 2
        assert pending.source_role_code == role_code(ROLE_FOLLOWER)
        assert pending.sequence == 73
        assert pending.destination_node_id == BROADCAST_NODE_ID
        assert pending.relayed
        assert node._counters.rx_frames == 1
    finally:
        destroy_node(node)


def test_origin_follower_drops_its_own_returned_broadcast():
    node = None
    try:
        node = make_node('follower', 2)
        frame = Frame(
            topic_id=LOCAL_STATE_TOPIC_ID,
            source_role=role_code(ROLE_FOLLOWER),
            source_node_id=2,
            destination_node_id=BROADCAST_NODE_ID,
            sequence=9,
            payload=encode_compact_state(make_state(2, 1.0)),
        )

        node._publish_frame(frame)

        assert node._counters.dropped_own_source == 1
        assert node._counters.rx_frames == 0
        assert not node._latest_pending
    finally:
        destroy_node(node)


def test_other_follower_accepts_relay_without_relaying_it_again():
    node = None
    try:
        node = make_node('follower', 3)
        frame = Frame(
            topic_id=LOCAL_STATE_TOPIC_ID,
            source_role=role_code(ROLE_FOLLOWER),
            source_node_id=2,
            destination_node_id=BROADCAST_NODE_ID,
            sequence=10,
            payload=encode_compact_state(make_state(2, 6.0)),
        )

        node._publish_frame(frame)
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=0.02)

        assert node._counters.rx_frames == 1
        assert node._counters.relayed_frames == 0
        assert node._counters.loop_suppressed >= 1
        assert not node._latest_pending
    finally:
        destroy_node(node)


def test_message_summary_is_single_line_and_bounded():
    message = String(data='arm\n' + ('x' * 400))
    summary = P900BridgeNode._message_summary(message)
    assert summary.startswith("'arm\\n")
    assert '\n' not in summary
    assert len(summary) == 300
    assert summary.endswith('...')
