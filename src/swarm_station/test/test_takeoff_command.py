from importlib import import_module
import json
from types import SimpleNamespace


class DummyLogger:
    def __init__(self):
        self.errors = []
        self.infos = []

    def error(self, message):
        self.errors.append(message)

    def info(self, message):
        self.infos.append(message)


class DummyPublisher:
    def __init__(self):
        self.last_message = None

    def publish(self, message):
        self.last_message = message


def test_takeoff_is_sent_when_fewer_drones_than_config_are_connected(
    monkeypatch,
):
    monkeypatch.setenv('PYNPUT_BACKEND', 'dummy')
    station_module = import_module('swarm_station.station_node')
    configured_values = {
        'swarm_sim.drone_count': 3,
        'swarm_single.control.takeoff_height': 2.5,
    }
    monkeypatch.setattr(
        station_module, 'get_config', configured_values.get
    )
    logger = DummyLogger()
    publisher = DummyPublisher()
    station = SimpleNamespace(
        last_status=SimpleNamespace(
            control_state='TAKEOFF',
            armed=True,
            offboard=True,
            swarm_members=[1, 2],
        ),
        command_publisher=publisher,
        get_logger=lambda: logger,
    )

    station_module.StationNode.send_takeoff_command(station)

    assert logger.errors == []
    assert publisher.last_message is not None
    assert json.loads(publisher.last_message.data) == {
        'command': 'takeoff',
        'height': 2.5,
    }


def test_relative_yaw_move_is_sent_to_the_leader(monkeypatch):
    monkeypatch.setenv('PYNPUT_BACKEND', 'dummy')
    station_module = import_module('swarm_station.station_node')
    publisher = DummyPublisher()
    station = SimpleNamespace(command_publisher=publisher)

    station_module.StationNode.send_yaw_command(station, -20.0)

    assert publisher.last_message is not None
    assert json.loads(publisher.last_message.data) == {
        'command': 'yaw',
        'delta_degrees': -20.0,
    }


def test_start_detection_is_sent_to_the_leader(monkeypatch):
    monkeypatch.setenv('PYNPUT_BACKEND', 'dummy')
    station_module = import_module('swarm_station.station_node')
    publisher = DummyPublisher()
    station = SimpleNamespace(command_publisher=publisher)

    station_module.StationNode.send_start_detection_command(
        station, [32, 0]
    )

    assert json.loads(publisher.last_message.data) == {
        'command': 'start_detection',
        'class_ids': [32, 0],
    }


def test_stop_detection_is_sent_to_the_leader(monkeypatch):
    monkeypatch.setenv('PYNPUT_BACKEND', 'dummy')
    station_module = import_module('swarm_station.station_node')
    publisher = DummyPublisher()
    station = SimpleNamespace(command_publisher=publisher)

    station_module.StationNode.send_stop_detection_command(station)

    assert json.loads(publisher.last_message.data) == {
        'command': 'stop_detection',
    }


def test_station_input_parses_detection_class_list(monkeypatch):
    monkeypatch.setenv('PYNPUT_BACKEND', 'dummy')
    station_module = import_module('swarm_station.station_node')
    logger = DummyLogger()
    sent_class_ids = []
    station = SimpleNamespace(
        leader_is_connected=True,
        get_logger=lambda: logger,
        send_start_detection_command=lambda values: sent_class_ids.append(
            values
        ),
    )

    station_module.StationNode.check_for_input(
        station, 'start_detection 32,0'
    )

    assert sent_class_ids == [[32, 0]]
    assert logger.errors == []


def test_mission_sends_small_absolute_position_relative_yaw_file_command(
    monkeypatch,
):
    monkeypatch.setenv('PYNPUT_BACKEND', 'dummy')
    station_module = import_module('swarm_station.station_node')
    configured_values = {
        'swarm_single.mission.waypoint_file': (
            'leader_waypoints_xyzyaw-3.txt'
        ),
        'swarm_single.mission.max_waypoints': 200,
    }
    monkeypatch.setattr(
        station_module, 'get_config', configured_values.get
    )
    monkeypatch.setattr(
        station_module,
        'get_mission_waypoints',
        lambda filename, leader_id: [
            [0.0, 0.0, 2.0, 0.0, False],
            [1.0, 0.0, 2.0, -22.5, True],
        ],
    )
    logger = DummyLogger()
    publisher = DummyPublisher()
    station = SimpleNamespace(
        last_status=SimpleNamespace(
            control_state='TAKEOFF',
            armed=True,
            offboard=True,
            leader_id=1,
        ),
        command_publisher=publisher,
        get_logger=lambda: logger,
    )

    assert station_module.StationNode.send_mission(station)

    assert logger.errors == []
    payload = json.loads(publisher.last_message.data)
    assert payload == {
        'command': 'mission',
        'waypoint_file': 'leader_waypoints_xyzyaw-3.txt',
        'leader_id': 1,
        'waypoint_count': 2,
        'relative_to_start': False,
        'yaw_relative': True,
    }
    assert 'points' not in payload
