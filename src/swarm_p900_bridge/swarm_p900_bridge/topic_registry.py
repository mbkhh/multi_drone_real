"""Central registry of ROS topics transported over the P900 link."""

from dataclasses import dataclass
from typing import FrozenSet, Type

from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import String
from swarm_msgs.msg import FormationCommand, Status

from swarm_p900_bridge.roles import ROLE_DRONE, ROLE_STATION, VALID_ROLES


COMMAND_TOPIC_ID = 1
FORMATION_COMMAND_TOPIC_ID = 2
STATUS_TOPIC_ID = 3


@dataclass(frozen=True)
class TopicRoute:
    """Describe one transported topic and its permitted direction."""

    topic_id: int
    topic_name: str
    message_type: Type
    sender_roles: FrozenSet[str]
    receiver_roles: FrozenSet[str]
    qos_reliability: ReliabilityPolicy
    qos_depth: int


# Add future transported topics here. The bridge node creates the correct ROS
# publishers/subscriptions directly from this single registry.
TOPIC_REGISTRY = {
    COMMAND_TOPIC_ID: TopicRoute(
        topic_id=COMMAND_TOPIC_ID,
        topic_name='/swarm/command',
        message_type=String,
        sender_roles=frozenset({ROLE_STATION}),
        receiver_roles=frozenset({ROLE_DRONE}),
        qos_reliability=ReliabilityPolicy.RELIABLE,
        qos_depth=10,
    ),
    FORMATION_COMMAND_TOPIC_ID: TopicRoute(
        topic_id=FORMATION_COMMAND_TOPIC_ID,
        topic_name='/swarm/formation_command',
        message_type=FormationCommand,
        sender_roles=frozenset({ROLE_STATION}),
        receiver_roles=frozenset({ROLE_DRONE}),
        qos_reliability=ReliabilityPolicy.RELIABLE,
        qos_depth=10,
    ),
    STATUS_TOPIC_ID: TopicRoute(
        topic_id=STATUS_TOPIC_ID,
        topic_name='/swarm/status',
        message_type=Status,
        sender_roles=frozenset({ROLE_DRONE}),
        receiver_roles=frozenset({ROLE_STATION}),
        qos_reliability=ReliabilityPolicy.BEST_EFFORT,
        qos_depth=1,
    ),
}


def _validate_registry():
    topic_names = set()
    for topic_id, route in TOPIC_REGISTRY.items():
        if topic_id != route.topic_id:
            raise ValueError(
                f'registry key {topic_id} does not match route ID '
                f'{route.topic_id}'
            )
        if not 0 <= topic_id <= 0xFFFF:
            raise ValueError(f'topic ID {topic_id} is outside uint16 range')
        if route.topic_name in topic_names:
            raise ValueError(f'duplicate topic name {route.topic_name!r}')
        topic_names.add(route.topic_name)
        configured_roles = route.sender_roles | route.receiver_roles
        unknown_roles = configured_roles - VALID_ROLES
        if unknown_roles:
            raise ValueError(
                f'topic {topic_id} contains unknown roles: '
                f'{sorted(unknown_roles)}'
            )
        if not route.sender_roles or not route.receiver_roles:
            raise ValueError(
                f'topic {topic_id} must have sender and receiver roles'
            )


_validate_registry()


def routes_sent_by(role):
    """Return routes this role must subscribe to and transmit."""
    return tuple(
        route for route in TOPIC_REGISTRY.values()
        if role in route.sender_roles
    )


def routes_received_by(role):
    """Return routes this role may receive and publish locally."""
    return tuple(
        route for route in TOPIC_REGISTRY.values()
        if role in route.receiver_roles
    )
