import json
import math
from types import SimpleNamespace

from std_msgs.msg import String
from px4_msgs.msg import VehicleStatus

import swarm_single_decenter.communication as communication_module
from swarm_single_decenter.communication import Communication
from swarm_config.config_utils import get_mission_waypoints


class DummyLogger:
    def __init__(self):
        self.errors = []
        self.warnings = []
        self.infos = []

    def error(self, message):
        self.errors.append(message)

    def warning(self, message):
        self.warnings.append(message)

    def info(self, message):
        self.infos.append(message)


class DummyPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def make_communication(drone_id=2):
    logger = DummyLogger()
    node = SimpleNamespace(
        frame_id=str(drone_id),
        get_logger=lambda: logger,
    )
    communication = object.__new__(Communication)
    communication.parent_node = node
    communication.drone_id = drone_id
    return communication, node, logger


def test_commands_are_filtered_by_all_or_numeric_target():
    communication, _, _ = make_communication(2)
    executed = []
    communication.execute_command = (
        lambda command_type, command: executed.append(
            (command_type, command['target'])
        )
    )

    for target in ('all', 2, '2', 1):
        message = String()
        message.data = json.dumps({'target': target, 'command': 'arm'})
        communication.command_callback(message)

    assert executed == [('arm', 'all'), ('arm', 2), ('arm', '2')]


def test_mission_loads_only_the_reporting_drone_section(monkeypatch):
    communication, node, _ = make_communication(2)
    loaded = []
    started = []

    def load(file_name, drone_id):
        loaded.append((file_name, drone_id))
        return [[0.0, 5.0, 2.0, 0.0], [10.0, 5.0, 2.0, -22.5]]

    monkeypatch.setattr(communication_module, 'get_mission_waypoints', load)
    node.start_mission = lambda points, relative_to_start, yaw_relative: (
        started.append((points, relative_to_start, yaw_relative)) or True
    )

    communication.execute_mission({
        'waypoint_file': 'waypoints_decentralized_36x34.yaml',
        'waypoint_counts': {'1': 20, '2': 2, '3': 21},
        'relative_to_start': False,
        'yaw_relative': True,
    })

    assert loaded == [('waypoints_decentralized_36x34.yaml', 2)]
    assert started == [(
        [[0.0, 5.0, 2.0, 0.0], [10.0, 5.0, 2.0, -22.5]],
        False,
        True,
    )]


def test_default_mission_file_and_mode_are_decentralized(monkeypatch):
    communication, node, _ = make_communication(3)
    loaded = []
    started = []
    monkeypatch.setattr(
        communication_module,
        'get_mission_waypoints',
        lambda file_name, drone_id: (
            loaded.append((file_name, drone_id)) or [[0.0, 10.0, 2.0, 0.0]]
        ),
    )
    node.start_mission = lambda points, relative_to_start, yaw_relative: (
        started.append((relative_to_start, yaw_relative)) or True
    )

    communication.execute_mission({})

    assert loaded == [('waypoints_decentralized_36x34.yaml', 3)]
    assert started == [(False, True)]


def test_installed_decentralized_file_has_three_bounded_independent_paths():
    starts = {
        1: [0.0, 0.0, 0.0],
        2: [0.0, 5.0, 0.0],
        3: [0.0, 10.0, 0.0],
    }
    for drone_id in (1, 2, 3):
        points = get_mission_waypoints(
            'waypoints_decentralized_36x34.yaml', drone_id
        )
        assert points
        assert all(len(point) == 4 for point in points)
        previous = starts[drone_id]
        for point in points:
            assert all(math.isfinite(float(value)) for value in point)
            leg = math.hypot(
                float(point[0]) - previous[0],
                float(point[1]) - previous[1],
            )
            assert leg <= 50.0
            previous = [float(value) for value in point[:3]]


def test_takeoff_rejects_nonfinite_local_position():
    communication, node, logger = make_communication(1)
    vehicle_status = VehicleStatus()
    vehicle_status.arming_state = VehicleStatus.ARMING_STATE_ARMED
    vehicle_status.nav_state = VehicleStatus.NAVIGATION_STATE_OFFBOARD
    node.state = 'TAKEOFF'
    node.vehicle_status = vehicle_status
    node.manual_control = False
    node.navigation = SimpleNamespace(current_pos=[0.0, 0.0, float('nan')])
    node.goal_callback_temp = lambda _goal: (_ for _ in ()).throw(
        AssertionError('invalid takeoff goal was forwarded')
    )

    assert not communication.execute_takeoff(2.0)
    assert 'not finite' in logger.errors[-1]


def test_start_detection_publishes_unchanged_vision_trigger():
    communication, node, _ = make_communication(1)
    communication.vision_trigger_publisher = DummyPublisher()
    communication.vision_return_land_active = False
    communication.vision_return_target = None
    communication.vision_detection_action = 'report'
    node.message = ''

    communication.execute_start_detection({
        'class_ids': [32, 29],
        'on_detection': 'return_land',
    })

    assert communication.vision_detection_action == 'return_land'
    assert communication.vision_trigger_publisher.messages[-1].data == (
        'START:32,29'
    )


def test_detection_returns_each_drone_to_its_configured_home_then_lands(
    monkeypatch,
):
    homes = {
        1: [0.0, 0.0, 0.0],
        2: [0.0, 5.0, 0.0],
        3: [0.0, 10.0, 0.0],
    }
    monkeypatch.setattr(
        communication_module,
        'get_config',
        lambda key: 0.1 if key == 'swarm_single.goal_tolerance' else None,
    )

    for drone_id, home in homes.items():
        communication, node, _ = make_communication(drone_id)
        status = VehicleStatus()
        status.arming_state = VehicleStatus.ARMING_STATE_ARMED
        status.nav_state = VehicleStatus.NAVIGATION_STATE_OFFBOARD
        goals = []
        land_requests = []
        mission_aborts = []
        node.state = 'TAKEOFF'
        node.vehicle_status = status
        node.manual_control = False
        node.safety_violation_reason = lambda: None
        node.initial_world_position = home
        node.navigation = SimpleNamespace(current_pos=[12.0, 18.0, 2.4])
        node.min_goal_altitude = -0.5
        node.max_goal_altitude = 5.0
        node.mission_active = True
        node.abort_mission = lambda reason: mission_aborts.append(reason)
        node.set_local_goal = lambda goal: goals.append(list(goal))
        node.motion_enabled = False
        node.mission_goal_tolerance = 0.3
        node.message = ''
        node.request_land = lambda: land_requests.append(True) or True
        communication.vision_trigger_publisher = DummyPublisher()
        communication.vision_detection_action = 'return_land'
        communication.vision_return_land_active = False
        communication.vision_return_target = None

        detection = String()
        detection.data = 'UAV_2:TARGET_DETECTED'
        communication.vision_detection_callback(detection)

        expected_target = [home[0], home[1], 2.4]
        assert goals == [expected_target]
        assert mission_aborts
        assert node.motion_enabled
        assert communication.vision_return_land_active
        assert communication.vision_trigger_publisher.messages[-1].data == 'STOP'

        node.navigation.current_pos = expected_target
        communication.update_vision_return_land()

        assert land_requests == [True]
        assert not communication.vision_return_land_active
        assert communication.vision_return_target is None
