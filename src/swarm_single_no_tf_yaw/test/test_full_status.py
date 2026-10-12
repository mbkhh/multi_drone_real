import json
from types import SimpleNamespace

from std_msgs.msg import String

from swarm_single_no_tf_yaw.communication import Communication
from swarm_single_no_tf_yaw import system_report


class DummyLogger:
    def __init__(self):
        self.infos = []
        self.warnings = []
        self.errors = []

    def info(self, message):
        self.infos.append(message)

    def warning(self, message):
        self.warnings.append(message)

    def error(self, message):
        self.errors.append(message)


class DummyPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def test_topic_counts_use_the_drone_specific_uav_namespace():
    counts = system_report.count_ros_topics(
        [
            '/swarm/live',
            '/uav_2/fmu/out/vehicle_status_v1',
            '/UAV_2/fmu/out/vehicle_local_position_v1',
            '/uav_1/fmu/out/vehicle_status_v1',
        ],
        2,
    )

    assert counts == {
        'total': 4,
        'uav_marker': 'UAV_2',
        'uav_count': 2,
        'uav_fmu_count': 2,
    }


def test_systemd_services_are_queried_in_one_bounded_call():
    calls = []

    def runner(command, **kwargs):
        calls.append((command, kwargs))
        return SimpleNamespace(
            stdout='active\ninactive\nfailed\nactive\n', stderr=''
        )

    states, error = system_report.query_service_states(runner=runner)

    assert error == ''
    assert states['swarm-xrce-agent.service'] == 'active'
    assert states['swarm-p900-bridge.service'] == 'inactive'
    assert states['swarm-control.service'] == 'failed'
    assert states['swarm-vision.service'] == 'active'
    assert len(calls) == 1
    assert calls[0][1]['timeout'] == 2.0


def test_leader_forwards_each_follower_report_only_once():
    publisher = DummyPublisher()
    logger = DummyLogger()
    communication = object.__new__(Communication)
    communication.parent_node = SimpleNamespace(
        is_leader=True,
        frame_id='1',
        get_logger=lambda: logger,
    )
    communication.full_status_report_publisher = publisher
    communication._full_status_forwarded_reports = []
    incoming = String()
    incoming.data = json.dumps({
        'kind': 'full_status_response',
        'request_id': 'abc-123',
        'report': {'schema': 1, 'drone_id': 2},
    })

    Communication._full_status_response_callback(communication, incoming)
    Communication._full_status_response_callback(communication, incoming)

    assert len(publisher.messages) == 1
    forwarded = json.loads(publisher.messages[0].data)
    assert forwarded['kind'] == 'full_status_report'
    assert forwarded['request_id'] == 'abc-123'
    assert forwarded['report']['drone_id'] == 2


def test_full_status_command_only_starts_report_and_broadcasts_request():
    publisher = DummyPublisher()
    logger = DummyLogger()
    communication = object.__new__(Communication)
    communication.parent_node = SimpleNamespace(get_logger=lambda: logger)
    communication.command_publisher = publisher
    started = []
    communication._start_full_status_collection = (
        lambda request_id: started.append(request_id) or True
    )
    incoming = String()
    incoming.data = json.dumps({
        'command': 'full_status',
        'request_id': 'request-1',
    })

    Communication.command_leader_callback(communication, incoming)

    assert started == ['request-1']
    assert json.loads(publisher.messages[0].data) == {
        'command': 'full_status',
        'request_id': 'request-1',
    }
