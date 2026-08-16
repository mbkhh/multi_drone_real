import importlib
import math
from types import SimpleNamespace

from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped


navigation_module = importlib.import_module('swarm_single.navigation')
Navigation = navigation_module.navigation


class DummyLogger:
    def __init__(self):
        self.warnings = []

    def warning(self, message):
        self.warnings.append(message)


class DummyNow:
    def __init__(self, nanoseconds):
        self.nanoseconds = nanoseconds

    def to_msg(self):
        seconds, nanoseconds = divmod(self.nanoseconds, 1_000_000_000)
        return Time(sec=int(seconds), nanosec=int(nanoseconds))

    def __sub__(self, other):
        return SimpleNamespace(
            nanoseconds=self.nanoseconds - other.nanoseconds
        )


class DummyClock:
    def __init__(self):
        self.nanoseconds = 10_000_000_000

    def now(self):
        return DummyNow(self.nanoseconds)

    def advance(self, seconds):
        self.nanoseconds += int(seconds * 1_000_000_000)


class FailingTFBuffer:
    def __init__(self):
        self.calls = 0

    def lookup_transform(self, *_args, **_kwargs):
        self.calls += 1
        raise AssertionError('local navigation must not perform a TF lookup')


class StaticTFBuffer:
    def __init__(self, transform):
        self.transform = transform
        self.calls = []

    def lookup_transform(self, *args, **kwargs):
        self.calls.append((args, kwargs))
        return self.transform


def make_parent(clock, tf_buffer=None):
    goal = TransformStamped()
    goal.header.frame_id = 'world'
    goal.child_frame_id = 'x500_depth_1/goal'
    goal.transform.translation.z = 2.0
    goal.transform.rotation.w = 1.0
    logger = DummyLogger()
    return SimpleNamespace(
        get_clock=lambda: clock,
        get_logger=lambda: logger,
        last_local_position_time=clock.now(),
        active_goal_transform=goal,
        is_leader=True,
        manual_control=False,
        manual_velocity=[0.0, 0.0, 0.0],
        last_seen_neighbors={},
        velocity_goal=[0.0, 0.0, 0.0],
        px4_model='x500_depth',
        frame_id='1',
        goal_frame='goal',
        tf_buffer=tf_buffer or FailingTFBuffer(),
        logger=logger,
    )


def test_navigation_configuration_is_cached_and_local_goal_avoids_tf(
    monkeypatch,
):
    calls = []

    def load_config(key):
        calls.append(key)
        return None

    monkeypatch.setattr(navigation_module, 'get_config', load_config)
    clock = DummyClock()
    parent = make_parent(clock)
    navigation = Navigation(parent)
    navigation.current_pos[:3] = [0.0, 0.0, 0.0]
    initialization_call_count = len(calls)

    monkeypatch.setattr(
        navigation_module,
        'get_config',
        lambda _key: (_ for _ in ()).throw(
            AssertionError('configuration was reread during navigation')
        ),
    )

    assert navigation.navigate_to_goal()
    assert len(calls) == initialization_call_count
    assert parent.tf_buffer.calls == 0
    assert parent.velocity_goal[2] < 0.0


def test_stale_local_position_commands_zero_velocity(monkeypatch):
    monkeypatch.setattr(navigation_module, 'get_config', lambda _key: None)
    clock = DummyClock()
    parent = make_parent(clock)
    navigation = Navigation(parent)
    navigation.current_pos[:3] = [0.0, 0.0, 0.0]
    parent.velocity_goal = [0.2, -0.1, -0.7]
    clock.advance(navigation.feedback_timeout + 0.01)

    assert not navigation.navigate_to_goal()
    assert parent.velocity_goal == [0.0, 0.0, 0.0]
    assert 'commanding zero velocity' in parent.logger.warnings[-1]


def test_neighbor_lookup_is_nonblocking_and_expires_when_stream_stops(
    monkeypatch,
):
    monkeypatch.setattr(navigation_module, 'get_config', lambda _key: None)
    clock = DummyClock()
    transform = TransformStamped()
    transform.header.stamp = Time(sec=123, nanosec=456)
    transform.header.frame_id = 'world'
    transform.child_frame_id = 'x500_depth_2/odom'
    transform.transform.rotation.w = 1.0
    tf_buffer = StaticTFBuffer(transform)
    parent = make_parent(clock, tf_buffer=tf_buffer)
    navigation = Navigation(parent)

    assert navigation.get_drone_pos(2) is not None
    assert tf_buffer.calls[0][1] == {}

    clock.advance(navigation.neighbor_transform_timeout + 0.01)
    assert navigation.get_drone_pos(2) is None


def test_relative_goal_is_resolved_from_fresh_leader_pose(monkeypatch):
    monkeypatch.setattr(navigation_module, 'get_config', lambda _key: None)
    clock = DummyClock()
    leader = TransformStamped()
    leader.header.stamp = Time(sec=1, nanosec=0)
    leader.transform.translation.x = 10.0
    leader.transform.translation.y = 20.0
    leader.transform.translation.z = 3.0
    leader.transform.rotation.w = 1.0
    parent = make_parent(clock, tf_buffer=StaticTFBuffer(leader))
    parent.active_goal_transform.header.frame_id = 'x500_depth_1/odom'
    parent.active_goal_transform.transform.translation.x = -5.0
    parent.active_goal_transform.transform.translation.y = 2.0
    parent.active_goal_transform.transform.translation.z = 1.0
    navigation = Navigation(parent)

    goal = navigation.resolve_active_goal()

    assert goal.tolist() == [5.0, 22.0, 4.0]


def test_braking_velocity_reduces_near_stopping_radius(monkeypatch):
    monkeypatch.setattr(navigation_module, 'get_config', lambda _key: None)
    navigation = Navigation(make_parent(DummyClock()))

    velocity = navigation._preferred_velocity(
        navigation_module.np.array([0.31, 0.0, 0.0]), 0.31
    )

    assert 0.0 < velocity[0] < 0.2
    assert math.isclose(velocity[1], 0.0)
    assert math.isclose(velocity[2], 0.0)
