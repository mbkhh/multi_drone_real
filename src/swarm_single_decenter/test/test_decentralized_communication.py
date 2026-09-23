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
