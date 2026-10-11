"""Direction tests for the centralized P900 topic registry."""

from nav_msgs.msg import Odometry
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import Header, String
from swarm_msgs.msg import FormationCommand, Status

from swarm_p900_bridge.roles import (
    ROLE_DRONE,
    ROLE_FOLLOWER,
    ROLE_LEADER,
    ROLE_STATION,
)
from swarm_p900_bridge.topic_registry import (
    CODEC_COMPACT_STATE,
    COMMAND_TOPIC_ID,
    FOLLOWER_COMMAND_TOPIC_ID,
    FORMATION_COMMAND_TOPIC_ID,
    FORMATION_TOPIC_ID,
    LEADER_NODE_ID,
    LIVENESS_TOPIC_ID,
    LOCAL_STATE_TOPIC_ID,
    routes_received_by,
    routes_sent_by,
    STATION_NODE_ID,
    STATUS_TOPIC_ID,
    TOPIC_REGISTRY,
    VISION_COMMAND_TOPIC_ID,
    VISION_TRIGGER_TOPIC_ID,
)
from swarm_p900_bridge.transport_protocol import BROADCAST_NODE_ID


def test_existing_station_leader_topics_keep_ids_types_and_directions():
    command = TOPIC_REGISTRY[COMMAND_TOPIC_ID]
    formation = TOPIC_REGISTRY[FORMATION_COMMAND_TOPIC_ID]
    status = TOPIC_REGISTRY[STATUS_TOPIC_ID]

    assert command.topic_name == '/swarm/command'
    assert command.message_type is String
    assert command.sender_roles == frozenset({ROLE_STATION})
    assert command.receiver_roles == frozenset({ROLE_LEADER})
    assert command.link_for_sender(ROLE_STATION).destination_node_id == 1
    assert command.qos_reliability == ReliabilityPolicy.RELIABLE

    assert formation.topic_name == '/swarm/formation_command'
    assert formation.message_type is FormationCommand
    assert formation.sender_roles == frozenset({ROLE_STATION})
    assert formation.receiver_roles == frozenset({ROLE_LEADER})

    assert status.topic_name == '/swarm/status'
    assert status.message_type is Status
    assert status.sender_roles == frozenset({ROLE_LEADER})
    assert status.receiver_roles == frozenset({ROLE_STATION})
    assert status.link_for_sender(ROLE_LEADER).destination_node_id == 100
    assert status.qos_reliability == ReliabilityPolicy.BEST_EFFORT


def test_new_leader_follower_command_and_vision_routes():
    command = TOPIC_REGISTRY[FOLLOWER_COMMAND_TOPIC_ID]
    formation = TOPIC_REGISTRY[FORMATION_TOPIC_ID]
    trigger = TOPIC_REGISTRY[VISION_TRIGGER_TOPIC_ID]
    detection = TOPIC_REGISTRY[VISION_COMMAND_TOPIC_ID]

    assert (command.topic_name, command.message_type) == ('/command', String)
    assert formation.topic_name == '/formation'
    assert formation.message_type is FormationCommand
    assert trigger.topic_name == '/swarm/vision_trigger'
    assert detection.topic_name == '/swarm/vision_command'
    for route in (command, formation, trigger):
        assert route.sender_roles == frozenset({ROLE_LEADER})
        assert route.receiver_roles == frozenset({ROLE_FOLLOWER})
        assert (
            route.link_for_sender(ROLE_LEADER).destination_node_id
            == BROADCAST_NODE_ID
        )
    assert detection.sender_roles == frozenset({ROLE_FOLLOWER})
    assert detection.receiver_roles == frozenset({ROLE_LEADER})
    assert (
        detection.link_for_sender(ROLE_FOLLOWER).destination_node_id
        == LEADER_NODE_ID
    )


def test_liveness_and_compact_state_have_explicit_relay_policy():
    liveness = TOPIC_REGISTRY[LIVENESS_TOPIC_ID]
    state = TOPIC_REGISTRY[LOCAL_STATE_TOPIC_ID]

    assert liveness.message_type is Header
    assert liveness.leader_relay
    assert liveness.source_id_field == 'frame_id'
    assert state.message_type is Odometry
    assert state.codec == CODEC_COMPACT_STATE
    assert state.leader_relay
    assert state.latest_sample
    assert state.max_tx_rate_hz == 10.0
    assert state.source_id_field == 'child_frame_id'

    for route in (liveness, state):
        assert route.sender_roles == frozenset({ROLE_LEADER, ROLE_FOLLOWER})
        assert route.receiver_roles == frozenset({ROLE_LEADER, ROLE_FOLLOWER})
        assert (
            route.link_for_sender(ROLE_LEADER).destination_node_id
            == BROADCAST_NODE_ID
        )
        assert (
            route.link_for_sender(ROLE_FOLLOWER).destination_node_id
            == LEADER_NODE_ID
        )
        assert route.accepts(ROLE_FOLLOWER, ROLE_LEADER, LEADER_NODE_ID)
        assert route.accepts(
            ROLE_FOLLOWER, ROLE_FOLLOWER, BROADCAST_NODE_ID
        )
        assert not route.accepts(
            ROLE_FOLLOWER, ROLE_LEADER, BROADCAST_NODE_ID
        )


def test_complete_role_route_sets_and_legacy_drone_alias():
    assert [route.topic_id for route in routes_sent_by(ROLE_STATION)] == [1, 2]
    assert [route.topic_id for route in routes_received_by(ROLE_STATION)] == [3]
    assert [route.topic_id for route in routes_sent_by(ROLE_LEADER)] == [
        3, 4, 5, 6, 7, 8,
    ]
    assert [route.topic_id for route in routes_received_by(ROLE_LEADER)] == [
        1, 2, 6, 7, 9,
    ]
    assert [route.topic_id for route in routes_sent_by(ROLE_FOLLOWER)] == [
        6, 7, 9,
    ]
    assert [route.topic_id for route in routes_received_by(ROLE_FOLLOWER)] == [
        4, 5, 6, 7, 8,
    ]
    assert routes_sent_by(ROLE_DRONE) == routes_sent_by(ROLE_LEADER)
    assert STATION_NODE_ID == 100
    assert LEADER_NODE_ID == 1
