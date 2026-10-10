"""ROS integration smoke tests for both bridge roles."""

import rclpy

from swarm_p900_bridge.bridge_node import P900BridgeNode


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


def test_station_role_creates_only_command_transmitter():
    node = None
    try:
        node = make_node('station', 0, 1)

        assert [route.topic_name for route in node.tx_routes] == [
            '/swarm/command'
        ]
        assert node.rx_routes == ()
        assert len(node._route_subscriptions) == 1
        assert node._route_publishers == {}
    finally:
        destroy_node(node)


def test_drone_role_creates_only_command_receiver():
    node = None
    try:
        node = make_node('drone', 1, 0)

        assert node.tx_routes == ()
        assert [route.topic_name for route in node.rx_routes] == [
            '/swarm/command'
        ]
        assert node._route_subscriptions == []
        assert tuple(node._route_publishers) == (1,)
    finally:
        destroy_node(node)
