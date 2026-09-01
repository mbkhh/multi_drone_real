from importlib import import_module
import json
from types import SimpleNamespace


class DummyLogger:
    def __init__(self):
        self.errors = []

    def error(self, message):
        self.errors.append(message)


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
