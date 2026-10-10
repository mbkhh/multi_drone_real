"""Direction tests for the centralized P900 topic registry."""

from std_msgs.msg import String

from swarm_p900_bridge.roles import ROLE_DRONE, ROLE_STATION
from swarm_p900_bridge.topic_registry import (
    COMMAND_TOPIC_ID,
    routes_received_by,
    routes_sent_by,
    TOPIC_REGISTRY,
)


def test_command_route_uses_existing_ros_type_and_one_way_direction():
    route = TOPIC_REGISTRY[COMMAND_TOPIC_ID]

    assert route.topic_name == '/swarm/command'
    assert route.message_type is String
    assert routes_sent_by(ROLE_STATION) == (route,)
    assert routes_received_by(ROLE_STATION) == ()
    assert routes_sent_by(ROLE_DRONE) == ()
    assert routes_received_by(ROLE_DRONE) == (route,)
