"""Direction tests for the centralized P900 topic registry."""

from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import String
from swarm_msgs.msg import FormationCommand, Status

from swarm_p900_bridge.roles import ROLE_DRONE, ROLE_STATION
from swarm_p900_bridge.topic_registry import (
    COMMAND_TOPIC_ID,
    FORMATION_COMMAND_TOPIC_ID,
    routes_received_by,
    routes_sent_by,
    STATUS_TOPIC_ID,
    TOPIC_REGISTRY,
)


def test_command_route_uses_existing_ros_type_and_one_way_direction():
    route = TOPIC_REGISTRY[COMMAND_TOPIC_ID]

    assert route.topic_name == '/swarm/command'
    assert route.message_type is String
    assert route.sender_roles == frozenset({ROLE_STATION})
    assert route.receiver_roles == frozenset({ROLE_DRONE})
    assert route.qos_reliability == ReliabilityPolicy.RELIABLE
    assert route.qos_depth == 10


def test_formation_route_uses_existing_type_and_station_to_drone():
    route = TOPIC_REGISTRY[FORMATION_COMMAND_TOPIC_ID]

    assert route.topic_name == '/swarm/formation_command'
    assert route.message_type is FormationCommand
    assert route.sender_roles == frozenset({ROLE_STATION})
    assert route.receiver_roles == frozenset({ROLE_DRONE})
    assert route.qos_reliability == ReliabilityPolicy.RELIABLE
    assert route.qos_depth == 10


def test_status_route_uses_existing_type_and_drone_to_station():
    route = TOPIC_REGISTRY[STATUS_TOPIC_ID]

    assert route.topic_name == '/swarm/status'
    assert route.message_type is Status
    assert route.sender_roles == frozenset({ROLE_DRONE})
    assert route.receiver_roles == frozenset({ROLE_STATION})
    assert route.qos_reliability == ReliabilityPolicy.BEST_EFFORT
    assert route.qos_depth == 1


def test_role_route_sets_prevent_topic_feedback_loops():
    command = TOPIC_REGISTRY[COMMAND_TOPIC_ID]
    formation = TOPIC_REGISTRY[FORMATION_COMMAND_TOPIC_ID]
    status = TOPIC_REGISTRY[STATUS_TOPIC_ID]

    assert routes_sent_by(ROLE_STATION) == (command, formation)
    assert routes_received_by(ROLE_STATION) == (status,)
    assert routes_sent_by(ROLE_DRONE) == (status,)
    assert routes_received_by(ROLE_DRONE) == (
        command, formation
    )
