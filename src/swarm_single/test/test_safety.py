import json
import math
from types import SimpleNamespace

from builtin_interfaces.msg import Time
from px4_msgs.msg import VehicleStatus
from std_msgs.msg import String

from swarm_single.communication import Communication
from swarm_single.single_control_node import SingleControlNode


class DummyLogger:
    def info(self, _message):
        pass

    def error(self, _message):
        pass

    def warning(self, _message):
        pass


class DummyNow:
    def __init__(self, nanoseconds=123456789000):
        self.nanoseconds = nanoseconds

    def to_msg(self):
        seconds, nanoseconds = divmod(self.nanoseconds, 1_000_000_000)
        return Time(sec=int(seconds), nanosec=int(nanoseconds))

    def __sub__(self, other):
        return SimpleNamespace(nanoseconds=self.nanoseconds - other.nanoseconds)


class DummyClock:
    def __init__(self):
        self.nanoseconds = 123456789000

    def now(self):
        return DummyNow(self.nanoseconds)

    def advance(self, seconds):
        self.nanoseconds += int(seconds * 1_000_000_000)


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


def test_goal_horizontal_safety_envelope():
    controller = make_goal_stub()
    controller.set_goal_transform = lambda goal, log_received=True: (
        SingleControlNode.set_goal_transform(controller, goal, log_received)
    )

    assert not SingleControlNode.goal_callback_temp(
        controller, [math.nan, 0.0, 1.0]
    )
    assert not SingleControlNode.goal_callback_temp(controller, [11.0, 0.0, 1.0])

    assert SingleControlNode.goal_callback_temp(controller, [2.0, 3.0, 2.0])
    assert controller.leader_goal == [2.0, 3.0, 2.0]
    assert controller.goal_tf_broadcaster.last_transform is not None


def test_current_hold_pose_can_be_outside_commanded_goal_envelope():
    controller = make_goal_stub()
    current_pose = [25.0, -30.0, 8.0]

    SingleControlNode.set_goal_transform(
        controller, current_pose, log_received=False
    )

    assert controller.leader_goal == current_pose
    assert controller.goal_tf_broadcaster.last_transform is not None


def make_world_position_stub(initial_position):
    return SimpleNamespace(
        use_configured_world_origin=True,
        initial_world_position=list(initial_position),
        px4_position_origin_enu=None,
    )


def test_independent_px4_origins_map_to_two_configured_drone_positions():
    drone_1 = make_world_position_stub([0.0, 0.0, 0.0])
    drone_2 = make_world_position_stub([0.0, 5.0, 0.0])

    assert SingleControlNode.world_position_from_px4_enu(
        drone_1, [120.0, -30.0, 4.0]
    ) == [0.0, 0.0, 0.0]
    assert SingleControlNode.world_position_from_px4_enu(
        drone_2, [-50.0, 400.0, -2.0]
    ) == [0.0, 5.0, 0.0]

    assert SingleControlNode.world_position_from_px4_enu(
        drone_1, [121.0, -32.0, 7.0]
    ) == [1.0, -2.0, 3.0]
    assert SingleControlNode.world_position_from_px4_enu(
        drone_2, [-49.0, 398.0, 1.0]
    ) == [1.0, 3.0, 3.0]


def test_disarmed_arm_world_origin_calibration_is_repeated():
    controller = make_world_position_stub([0.0, 5.0, 0.0])
    controller.latest_px4_position_enu = [10.0, 20.0, 2.0]
    controller.world_origin_calibrated = False
    controller.navigation = SimpleNamespace(
        current_pos=[99.0, 99.0, 99.0, 0.0, 0.0, 0.0, 1.0]
    )
    controller.get_logger = lambda: DummyLogger()
    controller.drone_id = '2'

    assert SingleControlNode.calibrate_world_origin(controller)
    assert controller.navigation.current_pos[:3] == [0.0, 5.0, 0.0]
    assert controller.px4_position_origin_enu == [10.0, 20.0, 2.0]

    controller.latest_px4_position_enu = [50.0, 60.0, 7.0]
    assert SingleControlNode.calibrate_world_origin(controller)
    assert controller.navigation.current_pos[:3] == [0.0, 5.0, 0.0]
    assert controller.px4_position_origin_enu == [50.0, 60.0, 7.0]


def make_takeoff_parent(position):
    vehicle_status = VehicleStatus()
    vehicle_status.arming_state = VehicleStatus.ARMING_STATE_ARMED
    vehicle_status.nav_state = VehicleStatus.NAVIGATION_STATE_OFFBOARD
    parent = SimpleNamespace(
        state='TAKEOFF',
        vehicle_status=vehicle_status,
        manual_control=False,
        navigation=SimpleNamespace(
            current_pos=[*position, 0.0, 0.0, 0.0, 1.0]
        ),
        mission_active=False,
        motion_enabled=False,
        accepted_goal=None,
        get_logger=lambda: DummyLogger(),
    )

    def accept_goal(goal):
        parent.accepted_goal = list(goal)
        return True

    parent.goal_callback_temp = accept_goal
    parent.abort_mission = lambda _reason: None
    return parent


def test_leader_takeoff_is_relayed_and_each_drone_climbs_from_own_position():
    leader = make_takeoff_parent([0.0, 0.0, 0.0])
    follower = make_takeoff_parent([0.0, 5.0, 0.0])
    leader_communication = object.__new__(Communication)
    leader_communication.parent_node = leader
    leader_communication.command_publisher = DummyPublisher()
    follower_communication = object.__new__(Communication)
    follower_communication.parent_node = follower

    station_command = String()
    station_command.data = json.dumps({"command": "takeoff", "height": 3.0})
    leader_communication.command_leader_callback(station_command)
    follower_communication.command_callback(
        leader_communication.command_publisher.last_message
    )

    assert leader.accepted_goal == [0.0, 0.0, 3.0]
    assert follower.accepted_goal == [0.0, 5.0, 3.0]
    assert leader.motion_enabled
    assert follower.motion_enabled


def test_takeoff_is_rejected_before_offboard_arming_completes():
    parent = make_takeoff_parent([0.0, 0.0, 0.0])
    parent.state = 'ARMING'
    communication = object.__new__(Communication)
    communication.parent_node = parent

    assert not communication.execute_takeoff(3.0)
    assert parent.accepted_goal is None
    assert not parent.motion_enabled


def make_mission_stub():
    clock = DummyClock()
    controller = SimpleNamespace(
        state='TAKEOFF',
        manual_control=False,
        navigation=SimpleNamespace(
            current_pos=[10.0, 20.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        ),
        max_mission_waypoints=10,
        max_goal_distance=10.0,
        min_goal_altitude=-0.5,
        max_goal_altitude=5.0,
        mission_goal_tolerance=0.4,
        mission_waypoint_dwell=1.0,
        mission_timeout=60.0,
        mission=[],
        mission_active=False,
        mission_index=0,
        mission_target=None,
        mission_start_time=None,
        mission_dwell_start=None,
        mission_state='IDLE',
        message='',
        motion_enabled=False,
        velocity_goal=[0.0, 0.0, 0.0],
        get_clock=lambda: clock,
        get_logger=lambda: DummyLogger(),
        telemetry_is_fresh=lambda: True,
        accepted_goals=[],
    )

    def accept_goal(goal):
        controller.accepted_goals.append(list(goal))
        return True

    controller.goal_callback_temp = accept_goal
    controller.activate_current_mission_waypoint = lambda: (
        SingleControlNode.activate_current_mission_waypoint(controller)
    )
    controller.abort_mission = lambda reason: (
        SingleControlNode.abort_mission(controller, reason)
    )
    controller.complete_mission = lambda: (
        SingleControlNode.complete_mission(controller)
    )
    return controller, clock


def test_relative_mission_reuses_normal_goal_path():
    controller, _clock = make_mission_stub()

    accepted = SingleControlNode.start_mission(
        controller,
        [[0.0, 0.0, 2.0], [2.0, 0.0, 2.0]],
        relative_to_start=True,
    )

    assert accepted
    assert controller.mission == [[10.0, 20.0, 2.0], [12.0, 20.0, 2.0]]
    assert controller.accepted_goals == [[10.0, 20.0, 2.0]]
    assert controller.mission_active
    assert controller.motion_enabled


def test_mission_advances_and_completes_after_dwell():
    controller, clock = make_mission_stub()
    assert SingleControlNode.start_mission(
        controller,
        [[0.0, 0.0, 1.0]],
        relative_to_start=True,
    )
    controller.navigation.current_pos[:3] = controller.mission_target

    SingleControlNode.update_mission_progress(controller)
    clock.advance(1.1)
    SingleControlNode.update_mission_progress(controller)

    assert not controller.mission_active
    assert controller.mission_state == 'COMPLETED'
    assert controller.mission_index == 1
    assert controller.velocity_goal == [0.0, 0.0, 0.0]


def test_mission_requires_active_offboard_state():
    controller, _clock = make_mission_stub()
    controller.state = 'IDLE'

    assert not SingleControlNode.start_mission(
        controller,
        [[0.0, 0.0, 1.0]],
        relative_to_start=True,
    )
    assert not controller.mission_active


def test_rc_mode_change_aborts_mission_and_latches_pilot_control():
    controller, _clock = make_mission_stub()
    controller.mission = [[10.0, 20.0, 2.0]]
    controller.mission_active = True
    controller.mission_target = list(controller.mission[0])
    controller.motion_enabled = True
    controller.manual_control = True
    controller.manual_velocity = [0.2, 0.1, -0.1]
    controller.offboard_setpoint_counter = 42
    controller.offboard_was_confirmed = True
    controller.release_to_pilot = lambda reason: (
        SingleControlNode.release_to_pilot(controller, reason)
    )

    vehicle_status = VehicleStatus()
    vehicle_status.nav_state = VehicleStatus.NAVIGATION_STATE_POSCTL
    SingleControlNode.vehicle_status_callback(controller, vehicle_status)

    assert controller.state == 'PILOT_CONTROL'
    assert not controller.mission_active
    assert controller.mission_state == 'ABORTED'
    assert not controller.motion_enabled
    assert not controller.manual_control
    assert controller.manual_velocity == [0.0, 0.0, 0.0]
    assert controller.velocity_goal == [0.0, 0.0, 0.0]
    assert controller.offboard_setpoint_counter == 0
