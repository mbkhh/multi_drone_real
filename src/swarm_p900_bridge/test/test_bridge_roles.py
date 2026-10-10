"""ROS integration smoke tests for both bridge roles."""

import rclpy
from std_msgs.msg import String

from swarm_p900_bridge.bridge_node import P900BridgeNode
from swarm_p900_bridge.topic_registry import (
    COMMAND_TOPIC_ID,
    FORMATION_COMMAND_TOPIC_ID,
    STATUS_TOPIC_ID,
)


def make_node(role, node_id, destination_node_id):
    """Create one loopback-backed bridge with ROS parameter overrides."""
    arguments = [
        '--ros-args',
        '-p', f'role:={role}',
        '-p', 'serial_port:=loop://',
        '-p', f'node_id:={node_id}',
        '-p', f'destination_node_id:={destination_node_id}',
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
        node = make_node('station', 0, 1)

        assert [route.topic_name for route in node.tx_routes] == [
            '/swarm/command',
            '/swarm/formation_command',
        ]
        assert [route.topic_name for route in node.rx_routes] == [
            '/swarm/status',
        ]
        assert len(node._route_subscriptions) == 2
        assert tuple(node._route_publishers) == (
            STATUS_TOPIC_ID,
        )
    finally:
        destroy_node(node)


def test_drone_role_creates_expected_topic_directions():
    node = None
    try:
        node = make_node('drone', 1, 0)

        assert [route.topic_name for route in node.tx_routes] == [
            '/swarm/status',
        ]
        assert [route.topic_name for route in node.rx_routes] == [
            '/swarm/command',
            '/swarm/formation_command',
        ]
        assert len(node._route_subscriptions) == 1
        assert tuple(node._route_publishers) == (
            COMMAND_TOPIC_ID,
            FORMATION_COMMAND_TOPIC_ID,
        )
    finally:
        destroy_node(node)


def test_message_summary_is_single_line_and_bounded():
    message = String(data='arm\n' + ('x' * 400))

    summary = P900BridgeNode._message_summary(message)

    assert summary.startswith("'arm\\n")
    assert '\n' not in summary
    assert len(summary) == 300
    assert summary.endswith('...')
