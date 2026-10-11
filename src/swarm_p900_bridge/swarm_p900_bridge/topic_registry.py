"""Central registry of ROS topics transported over the P900 link."""

from dataclasses import dataclass
from typing import FrozenSet, Type

from nav_msgs.msg import Odometry
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import Header, String
from swarm_msgs.msg import FormationCommand, Status

from swarm_p900_bridge.roles import (
    normalize_role,
    ROLE_FOLLOWER,
    ROLE_LEADER,
    ROLE_STATION,
    VALID_ROLES,
)
from swarm_p900_bridge.transport_protocol import BROADCAST_NODE_ID


STATION_NODE_ID = 100
LEADER_NODE_ID = 1

COMMAND_TOPIC_ID = 1
FORMATION_COMMAND_TOPIC_ID = 2
STATUS_TOPIC_ID = 3
FOLLOWER_COMMAND_TOPIC_ID = 4
FORMATION_TOPIC_ID = 5
LIVENESS_TOPIC_ID = 6
LOCAL_STATE_TOPIC_ID = 7
VISION_TRIGGER_TOPIC_ID = 8
VISION_COMMAND_TOPIC_ID = 9

CODEC_CDR = 'cdr'
CODEC_COMPACT_STATE = 'compact_state'
VALID_CODECS = frozenset({CODEC_CDR, CODEC_COMPACT_STATE})


@dataclass(frozen=True)
class TopicLink:
    """One locally-originated sender/receiver path for a topic."""

    sender_role: str
    receiver_roles: FrozenSet[str]
    destination_node_id: int


@dataclass(frozen=True)
class TopicRoute:
    """Describe one transported topic and all of its permitted paths."""

    topic_id: int
    topic_name: str
    message_type: Type
    links: tuple
    qos_reliability: ReliabilityPolicy
    qos_depth: int
    codec: str = CODEC_CDR
    leader_relay: bool = False
    max_tx_rate_hz: float = 0.0
    latest_sample: bool = False
    source_id_field: str = ''
    log_each_packet: bool = True

    @property
    def sender_roles(self):
        """Return roles that originate this ROS topic onto P900."""
        return frozenset(link.sender_role for link in self.links)

    @property
    def receiver_roles(self):
        """Return roles that publish received frames into local ROS."""
        roles = set()
        for link in self.links:
            roles.update(link.receiver_roles)
        return frozenset(roles)

    def link_for_sender(self, role):
        """Return the single local-origin TX link for ``role``."""
        role = normalize_role(role)
        matches = [link for link in self.links if link.sender_role == role]
        if len(matches) != 1:
            raise ValueError(
                f'topic {self.topic_name} has {len(matches)} TX links for '
                f'role {role}'
            )
        return matches[0]

    def accepts(self, sender_role, receiver_role, destination_node_id):
        """Return whether a received frame follows an approved registry path."""
        sender_role = normalize_role(sender_role)
        receiver_role = normalize_role(receiver_role)
        destination_node_id = int(destination_node_id)
        for link in self.links:
            if (
                link.sender_role == sender_role
                and receiver_role in link.receiver_roles
                and destination_node_id == link.destination_node_id
            ):
                return True

        # A leader relay retains the follower's original source role and node
        # ID while changing only the destination to application broadcast.
        return bool(
            self.leader_relay
            and sender_role == ROLE_FOLLOWER
            and receiver_role == ROLE_FOLLOWER
            and destination_node_id == BROADCAST_NODE_ID
        )

    def should_leader_relay(
        self,
        local_role,
        sender_role,
        destination_node_id,
    ):
        """Return whether this is a follower-origin frame to relay once."""
        return bool(
            self.leader_relay
            and normalize_role(local_role) == ROLE_LEADER
            and normalize_role(sender_role) == ROLE_FOLLOWER
            and int(destination_node_id) == LEADER_NODE_ID
        )


def _link(sender, receivers, destination):
    return TopicLink(
        sender_role=sender,
        receiver_roles=frozenset(receivers),
        destination_node_id=destination,
    )


# This is the only place that defines transported topic IDs, ROS types,
# origin/receiver roles, application destinations, relay behavior and codecs.
TOPIC_REGISTRY = {
    COMMAND_TOPIC_ID: TopicRoute(
        topic_id=COMMAND_TOPIC_ID,
        topic_name='/swarm/command',
        message_type=String,
        links=(
            _link(ROLE_STATION, {ROLE_LEADER}, LEADER_NODE_ID),
        ),
        qos_reliability=ReliabilityPolicy.RELIABLE,
        qos_depth=10,
    ),
    FORMATION_COMMAND_TOPIC_ID: TopicRoute(
        topic_id=FORMATION_COMMAND_TOPIC_ID,
        topic_name='/swarm/formation_command',
        message_type=FormationCommand,
        links=(
            _link(ROLE_STATION, {ROLE_LEADER}, LEADER_NODE_ID),
        ),
        qos_reliability=ReliabilityPolicy.RELIABLE,
        qos_depth=10,
    ),
    STATUS_TOPIC_ID: TopicRoute(
        topic_id=STATUS_TOPIC_ID,
        topic_name='/swarm/status',
        message_type=Status,
        links=(
            _link(ROLE_LEADER, {ROLE_STATION}, STATION_NODE_ID),
        ),
        qos_reliability=ReliabilityPolicy.BEST_EFFORT,
        qos_depth=1,
    ),
    FOLLOWER_COMMAND_TOPIC_ID: TopicRoute(
        topic_id=FOLLOWER_COMMAND_TOPIC_ID,
        topic_name='/command',
        message_type=String,
        links=(
            _link(ROLE_LEADER, {ROLE_FOLLOWER}, BROADCAST_NODE_ID),
        ),
        qos_reliability=ReliabilityPolicy.BEST_EFFORT,
        qos_depth=10,
    ),
    FORMATION_TOPIC_ID: TopicRoute(
        topic_id=FORMATION_TOPIC_ID,
        topic_name='/formation',
        message_type=FormationCommand,
        links=(
            _link(ROLE_LEADER, {ROLE_FOLLOWER}, BROADCAST_NODE_ID),
        ),
        qos_reliability=ReliabilityPolicy.BEST_EFFORT,
        qos_depth=10,
    ),
    LIVENESS_TOPIC_ID: TopicRoute(
        topic_id=LIVENESS_TOPIC_ID,
        topic_name='/swarm/live',
        message_type=Header,
        links=(
            _link(ROLE_LEADER, {ROLE_FOLLOWER}, BROADCAST_NODE_ID),
            _link(ROLE_FOLLOWER, {ROLE_LEADER}, LEADER_NODE_ID),
        ),
        qos_reliability=ReliabilityPolicy.BEST_EFFORT,
        qos_depth=8,
        leader_relay=True,
        source_id_field='frame_id',
        log_each_packet=False,
    ),
    LOCAL_STATE_TOPIC_ID: TopicRoute(
        topic_id=LOCAL_STATE_TOPIC_ID,
        topic_name='/swarm/local_state',
        message_type=Odometry,
        links=(
            _link(ROLE_LEADER, {ROLE_FOLLOWER}, BROADCAST_NODE_ID),
            _link(ROLE_FOLLOWER, {ROLE_LEADER}, LEADER_NODE_ID),
        ),
        qos_reliability=ReliabilityPolicy.BEST_EFFORT,
        qos_depth=1,
        codec=CODEC_COMPACT_STATE,
        leader_relay=True,
        max_tx_rate_hz=10.0,
        latest_sample=True,
        source_id_field='child_frame_id',
        log_each_packet=False,
    ),
    VISION_TRIGGER_TOPIC_ID: TopicRoute(
        topic_id=VISION_TRIGGER_TOPIC_ID,
        topic_name='/swarm/vision_trigger',
        message_type=String,
        links=(
            _link(ROLE_LEADER, {ROLE_FOLLOWER}, BROADCAST_NODE_ID),
        ),
        qos_reliability=ReliabilityPolicy.RELIABLE,
        qos_depth=10,
    ),
    VISION_COMMAND_TOPIC_ID: TopicRoute(
        topic_id=VISION_COMMAND_TOPIC_ID,
        topic_name='/swarm/vision_command',
        message_type=String,
        links=(
            _link(ROLE_FOLLOWER, {ROLE_LEADER}, LEADER_NODE_ID),
        ),
        qos_reliability=ReliabilityPolicy.RELIABLE,
        qos_depth=10,
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
        if not route.links:
            raise ValueError(f'topic {topic_id} must have at least one link')
        if route.codec not in VALID_CODECS:
            raise ValueError(f'topic {topic_id} has invalid codec {route.codec}')
        if route.latest_sample and route.max_tx_rate_hz <= 0.0:
            raise ValueError(
                f'topic {topic_id} latest-sample mode needs a positive rate'
            )

        sender_roles = set()
        for link in route.links:
            configured_roles = {link.sender_role} | set(link.receiver_roles)
            unknown_roles = configured_roles - VALID_ROLES
            if unknown_roles:
                raise ValueError(
                    f'topic {topic_id} contains unknown roles: '
                    f'{sorted(unknown_roles)}'
                )
            if link.sender_role in sender_roles:
                raise ValueError(
                    f'topic {topic_id} has multiple TX links for '
                    f'{link.sender_role}'
                )
            sender_roles.add(link.sender_role)
            if not link.receiver_roles:
                raise ValueError(
                    f'topic {topic_id} link has no receiver roles'
                )
            if not 0 <= link.destination_node_id <= BROADCAST_NODE_ID:
                raise ValueError(
                    f'topic {topic_id} has invalid destination '
                    f'{link.destination_node_id}'
                )


_validate_registry()


def routes_sent_by(role):
    """Return routes this role must subscribe to and transmit."""
    role = normalize_role(role)
    return tuple(
        route for route in TOPIC_REGISTRY.values()
        if role in route.sender_roles
    )


def routes_received_by(role):
    """Return routes this role may receive and publish locally."""
    role = normalize_role(role)
    return tuple(
        route for route in TOPIC_REGISTRY.values()
        if role in route.receiver_roles
    )
