import json
from types import SimpleNamespace

import swarm_station_decenter.station_node as station_module
from swarm_station_decenter.station_node import DecentralizedStationNode


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


def make_station():
    station = object.__new__(DecentralizedStationNode)
    station.command_publisher = DummyPublisher()
    station.logger = DummyLogger()
    station.get_logger = lambda: station.logger
    return station


def last_payload(station):
    return json.loads(station.command_publisher.messages[-1].data)


def test_bare_arm_targets_all_drones():
    station = make_station()

    station.handle_command('arm')

    assert last_payload(station) == {'target': 'all', 'command': 'arm'}


def test_numeric_prefix_targets_one_drone():
    station = make_station()

    station.handle_command('2 takeoff 2.5')

    assert last_payload(station) == {
        'target': 2,
        'command': 'takeoff',
        'height': 2.5,
    }


def test_drone_prefix_targets_one_goal():
    station = make_station()

    station.handle_command('drone 3 move 8 9 2')

    assert last_payload(station) == {
        'target': 3,
        'command': 'fly',
        'x': 8.0,
        'y': 9.0,
        'z': 2.0,
        'absolute': True,
    }


def test_all_mission_validates_and_sends_each_drone_count(monkeypatch):
    station = make_station()
    loads = []
    monkeypatch.setattr(
        station_module,
        'get_config',
        lambda key: 3 if key == 'swarm_sim.drone_count' else None,
    )
    monkeypatch.setattr(
        station_module,
        'get_mission_waypoints',
        lambda file_name, drone_id: (
            loads.append((file_name, drone_id)) or [[drone_id, 0.0, 2.0, 0.0]]
        ),
    )

    station.handle_command('all mission')

    assert loads == [
        ('waypoints_decentralized_36x34.yaml', 1),
        ('waypoints_decentralized_36x34.yaml', 2),
        ('waypoints_decentralized_36x34.yaml', 3),
    ]
    assert last_payload(station) == {
        'target': 'all',
        'command': 'mission',
        'waypoint_file': 'waypoints_decentralized_36x34.yaml',
        'waypoint_counts': {'1': 1, '2': 1, '3': 1},
        'relative_to_start': False,
        'yaw_relative': True,
    }


def test_status_command_prints_all_cached_drones():
    station = make_station()
    station.statuses = {
        1: SimpleNamespace(
            mission_state='RUNNING', mission_index=2, mission_count=20,
            control_state='TAKEOFF', armed=True, offboard=True,
            leader_x=1.0, leader_y=2.0, leader_z=3.0,
            swarm_members=[1, 2, 3],
        ),
        2: SimpleNamespace(
            mission_state='IDLE', mission_index=0, mission_count=0,
            control_state='TAKEOFF', armed=True, offboard=True,
            leader_x=4.0, leader_y=5.0, leader_z=3.0,
            swarm_members=[1, 2, 3],
        ),
    }

    station.handle_command('status')

    output = '\n'.join(station.logger.infos)
    assert 'Connected drones: [1, 2]' in output
    assert '[1] state=TAKEOFF' in output
    assert '[2] state=TAKEOFF' in output
