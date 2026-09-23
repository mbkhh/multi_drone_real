import importlib
import math
from types import SimpleNamespace

from builtin_interfaces.msg import Time


navigation_module = importlib.import_module('swarm_single_decenter.navigation')
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


def make_parent(clock):
    logger = DummyLogger()
    return SimpleNamespace(
        get_clock=lambda: clock,
        get_logger=lambda: logger,
        last_local_position_time=clock.now(),
        active_goal=[0.0, 0.0, 2.0],
        active_goal_mode='absolute',
        formation_offset=None,
        leader_id=1,
        peer_states={},
        is_leader=True,
        manual_control=False,
        manual_velocity=[0.0, 0.0, 0.0],
        last_seen_neighbors={},
        velocity_goal=[0.0, 0.0, 0.0],
        frame_id='1',
        logger=logger,
    )


def test_navigation_configuration_is_cached_and_goal_is_local(monkeypatch):
    calls = []

    def load_config(key):
        calls.append(key)
        return None

    monkeypatch.setattr(navigation_module, 'get_config', load_config)
    clock = DummyClock()
    parent = make_parent(clock)
    navigation = Navigation(parent)
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
    assert parent.velocity_goal[2] < 0.0


def test_collision_geometry_is_loaded_from_shared_config(monkeypatch):
    configured = {
        'swarm_single.navigation.neighbor_dist': 7.0,
        'swarm_single.navigation.radius': 0.8,
        'swarm_single.navigation.time_horizon': 6.0,
    }
    monkeypatch.setattr(
        navigation_module,
        'get_config',
        lambda key: configured.get(key),
    )

    navigation = Navigation(make_parent(DummyClock()))

    assert navigation.neighbor_dist == 7.0
    assert navigation.radius == 0.8
    assert navigation.time_horizon == 6.0


def test_stale_local_position_commands_zero_velocity(monkeypatch):
    monkeypatch.setattr(navigation_module, 'get_config', lambda _key: None)
    clock = DummyClock()
    parent = make_parent(clock)
    navigation = Navigation(parent)
    parent.velocity_goal = [0.2, -0.1, -0.7]
    clock.advance(navigation.feedback_timeout + 0.01)

    assert not navigation.navigate_to_goal()
    assert parent.velocity_goal == [0.0, 0.0, 0.0]
    assert 'commanding zero velocity' in parent.logger.warnings[-1]


def test_peer_state_expires_using_local_receipt_time(monkeypatch):
    monkeypatch.setattr(navigation_module, 'get_config', lambda _key: None)
    clock = DummyClock()
    parent = make_parent(clock)
    parent.peer_states[2] = {
        'position': [3.0, 4.0, 1.0],
        'orientation': [0.0, 0.0, 0.0, 1.0],
        'velocity': [0.0, 0.0, 0.0],
        'received_at': clock.now(),
    }
    navigation = Navigation(parent)

    assert navigation.get_drone_pos(2).tolist() == [3.0, 4.0, 1.0]
    clock.advance(navigation.neighbor_state_timeout + 0.01)
    assert navigation.get_drone_pos(2) is None


def test_decentralized_navigation_does_not_resolve_formation_goal(monkeypatch):
    monkeypatch.setattr(navigation_module, 'get_config', lambda _key: None)
    clock = DummyClock()
    parent = make_parent(clock)
    parent.frame_id = '2'
    parent.is_leader = False
    parent.active_goal = None
    parent.active_goal_mode = 'formation'
    parent.formation_offset = [-5.0, 2.0, 1.0]
    parent.peer_states[1] = {
        'position': [10.0, 20.0, 3.0],
        'orientation': [0.0, 0.0, 0.0, 1.0],
        'velocity': [0.0, 0.0, 0.0],
        'received_at': clock.now(),
    }
    navigation = Navigation(parent)

    assert navigation.resolve_active_goal() is None


def test_braking_velocity_reduces_near_stopping_radius(monkeypatch):
    monkeypatch.setattr(navigation_module, 'get_config', lambda _key: None)
    navigation = Navigation(make_parent(DummyClock()))

    velocity = navigation._preferred_velocity(
        navigation_module.np.array([0.31, 0.0, 0.0]), 0.31
    )

    assert 0.0 < velocity[0] < 0.2
    assert math.isclose(velocity[1], 0.0)
    assert math.isclose(velocity[2], 0.0)


def test_head_on_peer_is_seen_early_and_gets_opposite_path_velocity(monkeypatch):
    monkeypatch.setattr(navigation_module, 'get_config', lambda _key: None)
    clock = DummyClock()
    parent = make_parent(clock)
    parent.active_goal = [10.0, 0.0, 2.0]
    parent.last_seen_neighbors[2] = clock.now()
    parent.peer_states[2] = {
        'position': [2.0, 0.0, 2.0],
        'orientation': [0.0, 0.0, 0.0, 1.0],
        'velocity': [-1.0, 0.0, 0.0],
        'received_at': clock.now(),
    }
    navigation = Navigation(parent)
    navigation.current_pos[:3] = [-2.0, 0.0, 2.0]
    navigation.local_velocity = [1.0, 0.0, 0.0]

    assert navigation.neighbor_dist > 2.0 * navigation.radius
    assert navigation.navigate_to_goal()

    # velocity_goal is NED. A head-on ENU-X command is removed and replaced
    # by an ENU-Y/NED-North escape before the safety volumes overlap.
    assert abs(parent.velocity_goal[1]) < 0.1
    assert parent.velocity_goal[0] > 0.3
    assert '[COLLISION AVOIDANCE]' in parent.logger.warnings[-1]

    other_parent = make_parent(clock)
    other_parent.frame_id = '2'
    other_parent.active_goal = [-10.0, 0.0, 2.0]
    other_parent.last_seen_neighbors[1] = clock.now()
    other_parent.peer_states[1] = {
        'position': [-2.0, 0.0, 2.0],
        'orientation': [0.0, 0.0, 0.0, 1.0],
        'velocity': [1.0, 0.0, 0.0],
        'received_at': clock.now(),
    }
    other_navigation = Navigation(other_parent)
    other_navigation.current_pos[:3] = [2.0, 0.0, 2.0]
    other_navigation.local_velocity = [-1.0, 0.0, 0.0]

    assert other_navigation.navigate_to_goal()
    assert abs(other_parent.velocity_goal[1]) < 0.1
    assert other_parent.velocity_goal[0] < -0.3


def test_overlapping_peer_gets_deterministic_separation_velocity(monkeypatch):
    monkeypatch.setattr(navigation_module, 'get_config', lambda _key: None)
    clock = DummyClock()
    parent = make_parent(clock)
    navigation = Navigation(parent)

    safe_velocity = navigation._apply_collision_safety(
        navigation_module.np.zeros(3),
        navigation_module.np.zeros(3),
        [(
            2,
            navigation_module.np.zeros(3),
            navigation_module.np.zeros(3),
        )],
    )

    assert navigation_module.np.linalg.norm(safe_velocity) > 0.3
    assert navigation_module.np.all(
        navigation_module.np.isfinite(safe_velocity)
    )
