import math
from types import SimpleNamespace

from swarm_single.single_control_node import SingleControlNode


class DummyLogger:
    def info(self, _message):
        pass

    def error(self, _message):
        pass


class DummyNow:
    nanoseconds = 123456789000

    def to_msg(self):
        return SimpleNamespace()


class DummyClock:
    def now(self):
        return DummyNow()


class DummyPublisher:
    def __init__(self):
        self.last_message = None

    def publish(self, message):
        self.last_message = message


class DummyBroadcaster:
    def __init__(self):
        self.last_transform = None

    def sendTransform(self, transform):
        self.last_transform = transform


def make_controller_stub():
    controller = SimpleNamespace(
        yaw_initialized=True,
        yaw=0.25,
        velocity_goal=[3.0, 4.0, 2.0],
        max_horizontal_speed=1.0,
        max_vertical_speed=0.5,
        trajectory_setpoint_publisher=DummyPublisher(),
        get_clock=lambda: DummyClock(),
        released_reason=None,
    )

    def release(reason):
        controller.released_reason = reason

    controller.release_to_pilot = release
    return controller


def test_setpoint_is_clamped_and_unused_fields_are_nan():
    controller = make_controller_stub()

    SingleControlNode.publish_position_setpoint(controller)

    message = controller.trajectory_setpoint_publisher.last_message
    assert message is not None
    assert math.isclose(
        math.hypot(message.velocity[0], message.velocity[1]),
        1.0,
        rel_tol=1e-6,
    )
    assert math.isclose(message.velocity[2], 0.5, rel_tol=1e-6)
    assert all(math.isnan(value) for value in message.position)
    assert all(math.isnan(value) for value in message.acceleration)
    assert all(math.isnan(value) for value in message.jerk)
    assert math.isnan(message.yawspeed)
    assert controller.released_reason is None


def test_non_finite_velocity_is_rejected():
    controller = make_controller_stub()
    controller.velocity_goal = [math.nan, 0.0, 0.0]

    SingleControlNode.publish_position_setpoint(controller)

    assert controller.trajectory_setpoint_publisher.last_message is None
    assert controller.released_reason == 'Non-finite velocity setpoint rejected'


def make_goal_stub():
    return SimpleNamespace(
        navigation=SimpleNamespace(
            current_pos=[0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        ),
        max_goal_distance=10.0,
        min_goal_altitude=-0.5,
        max_goal_altitude=5.0,
        get_logger=lambda: DummyLogger(),
        get_clock=lambda: DummyClock(),
        px4_model='real_drone',
        frame_id='1',
        goal_frame='goal',
        goal_tf_broadcaster=DummyBroadcaster(),
        leader_goal=[0.0, 0.0, 0.0],
        sended_goal_ack=True,
    )


def test_goal_safety_envelope():
    controller = make_goal_stub()

    assert not SingleControlNode.goal_callback_temp(
        controller, [math.nan, 0.0, 1.0]
    )
    assert not SingleControlNode.goal_callback_temp(controller, [11.0, 0.0, 1.0])
    assert not SingleControlNode.goal_callback_temp(controller, [0.0, 0.0, 6.0])

    assert SingleControlNode.goal_callback_temp(controller, [2.0, 3.0, 2.0])
    assert controller.leader_goal == [2.0, 3.0, 2.0]
    assert controller.goal_tf_broadcaster.last_transform is not None
