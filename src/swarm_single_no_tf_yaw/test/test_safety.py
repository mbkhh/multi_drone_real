import json
import math
from types import SimpleNamespace

from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
from px4_msgs.msg import (
    FailsafeFlags,
    VehicleCommand,
    VehicleLandDetected,
    VehicleStatus,
)
from std_msgs.msg import String

from swarm_single_no_tf_yaw.communication import Communication
from swarm_single_no_tf_yaw.formation import PatternController
from swarm_single_no_tf_yaw.single_control_node import DroneState, SingleControlNode


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


class DummyFormation:
    def __init__(self):
        self.refresh_count = 0

    def refresh_pattern(self):
        self.refresh_count += 1


def make_controller_stub():
    clock = DummyClock()
    controller = SimpleNamespace(
        yaw_initialized=True,
        yaw=0.25,
        velocity_goal=[3.0, 4.0, 2.0],
        max_horizontal_speed=1.0,
        max_vertical_speed=0.5,
        trajectory_setpoint_publisher=DummyPublisher(),
        last_published_velocity_setpoint=None,
        velocity_setpoints_since_debug=0,
        get_clock=lambda: clock,
        max_horizontal_acceleration=1.0,
        max_vertical_acceleration=0.5,
        max_yaw_rate=math.radians(90.0),
        max_control_interval=0.25,
        last_setpoint_time=None,
        last_yaw_setpoint_time=None,
        commanded_yaw=0.0,
        last_commanded_velocity_setpoint=[0.0, 0.0, 0.0],
        released_reason=None,
        clock=clock,
    )

    def release(reason):
        controller.released_reason = reason

    controller.release_to_pilot = release
    controller.limit_velocity_change = lambda desired, now: (
        SingleControlNode.limit_velocity_change(controller, desired, now)
    )
    controller.limit_yaw_change = lambda now: (
        SingleControlNode.limit_yaw_change(controller, now)
    )
    return controller


def test_referendum_keeps_role_logic_without_leader_px4_subscription():
    formation = DummyFormation()
    parent = SimpleNamespace(
        frame_id='2',
        leader_id=2,
        is_leader=True,
        last_seen_neighbors={1: DummyNow()},
        formation=formation,
        get_logger=lambda: DummyLogger(),
    )
    communication = object.__new__(Communication)
    communication.parent_node = parent
    role_changes = []
    communication.become_leader = lambda: role_changes.append('leader')
    communication.step_down_as_leader = lambda: role_changes.append('follower')

    Communication.referendum_conducting(communication)

    assert parent.leader_id == 1
    assert role_changes == ['follower']
    assert formation.refresh_count == 1


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
    assert all(
        math.isclose(actual, expected, rel_tol=1e-6)
        for actual, expected in zip(
            controller.last_published_velocity_setpoint,
            (0.6, 0.8, 0.5),
        )
    )
    assert controller.velocity_setpoints_since_debug == 1
    assert controller.released_reason is None


def test_non_finite_velocity_is_rejected():
    controller = make_controller_stub()
    controller.velocity_goal = [math.nan, 0.0, 0.0]

    SingleControlNode.publish_position_setpoint(controller)

    assert controller.trajectory_setpoint_publisher.last_message is None
    assert controller.released_reason == 'Non-finite velocity setpoint rejected'


def test_velocity_reversal_is_slew_limited():
    controller = make_controller_stub()
    controller.last_setpoint_time = controller.clock.now()
    controller.clock.advance(0.1)

    first = SingleControlNode.limit_velocity_change(
        controller, [1.0, 0.0, 0.5], controller.clock.now()
    )
    assert math.isclose(first[0], 0.1, abs_tol=1e-9)
    assert math.isclose(first[2], 0.05, abs_tol=1e-9)

    controller.last_setpoint_time = controller.clock.now()
    controller.clock.advance(0.1)
    reversed_command = SingleControlNode.limit_velocity_change(
        controller, [-1.0, 0.0, -0.5], controller.clock.now()
    )
    assert math.isclose(reversed_command[0], 0.0, abs_tol=1e-9)
    assert math.isclose(reversed_command[2], 0.0, abs_tol=1e-9)


def test_yaw_setpoint_is_slew_limited_and_uses_shortest_wrap_path():
    controller = make_controller_stub()
    controller.yaw = math.pi - 0.1
    controller.commanded_yaw = -math.pi + 0.1
    controller.last_yaw_setpoint_time = controller.clock.now()

    controller.clock.advance(0.5)
    limited = SingleControlNode.limit_yaw_change(
        controller, controller.clock.now()
    )

    # The target is only 0.2 rad away across the wrap boundary, so it is
    # reached in one update; a long 2*pi rotation must never be commanded.
    assert math.isclose(limited, math.pi - 0.1, abs_tol=1e-9)

    controller.yaw = 0.0
    controller.clock.advance(0.1)
    limited = SingleControlNode.limit_yaw_change(
        controller, controller.clock.now()
    )
    assert math.isclose(limited, math.pi - 0.1 - math.radians(9.0), abs_tol=1e-9)


def test_invalid_feedback_forces_immediate_zero_setpoint():
    controller = make_controller_stub()
    controller.last_commanded_velocity_setpoint = [0.4, -0.2, -0.5]
    controller.last_setpoint_time = controller.clock.now()

    SingleControlNode.publish_position_setpoint(controller, force_zero=True)

    message = controller.trajectory_setpoint_publisher.last_message
    assert list(message.velocity) == [0.0, 0.0, 0.0]
    assert controller.last_commanded_velocity_setpoint == [0.0, 0.0, 0.0]


def test_control_loop_gap_is_detected_without_rejecting_ten_hz_operation():
    clock = DummyClock()
    controller = SimpleNamespace(
        get_clock=lambda: clock,
        get_logger=lambda: DummyLogger(),
        last_control_loop_time=clock.now(),
        last_control_interval=None,
        last_control_gap_warning_time=None,
        max_control_interval=0.25,
    )

    clock.advance(0.1)
    assert SingleControlNode.update_control_loop_timing(controller)

    clock.advance(0.26)
    assert not SingleControlNode.update_control_loop_timing(controller)
    assert math.isclose(controller.last_control_interval, 0.26)


def make_offboard_safety_stub():
    clock = DummyClock()
    vehicle_status = VehicleStatus()
    vehicle_status.arming_state = VehicleStatus.ARMING_STATE_DISARMED
    vehicle_status.pre_flight_checks_pass = True
    vehicle_status.failsafe = False
    failsafe_flags = FailsafeFlags()
    failsafe_flags.manual_control_signal_lost = False
    failsafe_flags.local_position_invalid = False
    failsafe_flags.local_velocity_invalid = False
    controller = SimpleNamespace(
        state='IDLE',
        vehicle_status=vehicle_status,
        failsafe_flags=failsafe_flags,
        require_manual_control_signal=True,
        last_vehicle_status_time=clock.now(),
        last_local_position_time=clock.now(),
        last_failsafe_flags_time=clock.now(),
        telemetry_timeout=2.5,
        use_configured_world_origin=False,
        navigation=SimpleNamespace(
            current_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        ),
        motion_enabled=True,
        manual_control=True,
        velocity_goal=[0.2, 0.1, -0.1],
        last_commanded_velocity_setpoint=[0.2, 0.1, -0.1],
        last_setpoint_time=clock.now(),
        offboard_setpoint_counter=12,
        offboard_was_confirmed=True,
        get_clock=lambda: clock,
        get_logger=lambda: DummyLogger(),
        set_local_goal=lambda _goal, log_received=False: None,
        reset_mission_state=lambda: None,
    )
    controller.telemetry_is_fresh = lambda: (
        SingleControlNode.telemetry_is_fresh(controller)
    )
    controller.safety_violation_reason = lambda require_preflight=False: (
        SingleControlNode.safety_violation_reason(
            controller,
            require_preflight=require_preflight,
        )
    )
    return controller


def test_real_flight_safety_gates_reject_offboard_arm():
    cases = (
        (
            lambda controller: setattr(
                controller, 'last_failsafe_flags_time', None
            ),
            'missing/stale',
        ),
        (
            lambda controller: setattr(
                controller.vehicle_status, 'failsafe', True
            ),
            'active failsafe',
        ),
        (
            lambda controller: setattr(
                controller.failsafe_flags,
                'manual_control_signal_lost',
                True,
            ),
            'RC/manual-control',
        ),
        (
            lambda controller: setattr(
                controller.failsafe_flags, 'local_position_invalid', True
            ),
            'local position/velocity',
        ),
    )

    for make_unsafe, expected_reason in cases:
        controller = make_offboard_safety_stub()
        make_unsafe(controller)

        reason = SingleControlNode.safety_violation_reason(
            controller,
            require_preflight=True,
        )
        assert expected_reason in reason
        assert not SingleControlNode.request_offboard_control(controller)
        assert controller.state == 'IDLE'


def test_safe_real_flight_telemetry_allows_offboard_arming_sequence():
    controller = make_offboard_safety_stub()

    assert SingleControlNode.request_offboard_control(controller)
    assert controller.state == 'ARMING'
    assert not controller.motion_enabled
    assert not controller.manual_control
    assert controller.velocity_goal == [0.0, 0.0, 0.0]


def test_explicit_simulation_mode_bypasses_companion_safety_gates():
    controller = make_offboard_safety_stub()
    controller.simulation_disable_safety_checks = True
    controller.last_vehicle_status_time = None
    controller.vehicle_status.failsafe = True
    controller.failsafe_flags.manual_control_signal_lost = True

    assert SingleControlNode.safety_violation_reason(
        controller,
        require_preflight=True,
    ) is None
    assert SingleControlNode.request_offboard_control(controller)
    assert controller.state == 'ARMING'


def make_arm_command_parent(accepted=True):
    parent = SimpleNamespace(
        arm_requests=0,
        get_logger=lambda: DummyLogger(),
    )

    def request_offboard_control():
        parent.arm_requests += 1
        return accepted

    parent.request_offboard_control = request_offboard_control
    return parent


def test_leader_relays_arm_only_after_its_own_safety_checks_accept():
    leader = make_arm_command_parent(accepted=True)
    follower = make_arm_command_parent(accepted=True)
    leader_communication = object.__new__(Communication)
    leader_communication.parent_node = leader
    leader_communication.command_publisher = DummyPublisher()
    follower_communication = object.__new__(Communication)
    follower_communication.parent_node = follower

    station_command = String()
    station_command.data = json.dumps({"command": "arm"})
    leader_communication.command_leader_callback(station_command)
    follower_communication.command_callback(
        leader_communication.command_publisher.last_message
    )

    assert leader.arm_requests == 1
    assert follower.arm_requests == 1

    rejected_leader = make_arm_command_parent(accepted=False)
    rejected_communication = object.__new__(Communication)
    rejected_communication.parent_node = rejected_leader
    rejected_communication.command_publisher = DummyPublisher()
    rejected_communication.command_leader_callback(station_command)

    assert rejected_leader.arm_requests == 1
    assert rejected_communication.command_publisher.last_message is None


def test_leader_accepts_relative_yaw_command_from_station():
    received = []
    leader = SimpleNamespace(
        get_logger=lambda: DummyLogger(),
        request_relative_yaw=lambda delta: (
            received.append(float(delta)) or True
        ),
    )
    communication = object.__new__(Communication)
    communication.parent_node = leader

    station_command = String()
    station_command.data = json.dumps({
        'command': 'yaw',
        'delta_degrees': -20.0,
    })
    communication.command_leader_callback(station_command)

    assert received == [-20.0]


def make_goal_stub():
    controller = SimpleNamespace(
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
        active_goal=None,
        active_goal_mode=None,
        formation_offset=None,
        leader_goal=[0.0, 0.0, 0.0],
        sended_goal_ack=True,
    )
    return controller


def test_relative_yaw_command_updates_target_from_measured_heading():
    controller = SimpleNamespace(
        state=DroneState.TAKEOFF,
        current_yaw_ned=math.radians(10.0),
        yaw=0.0,
        yaw_initialized=True,
        mission_active=False,
        get_logger=lambda: DummyLogger(),
    )

    assert SingleControlNode.request_relative_yaw(controller, -20.0)
    assert math.isclose(controller.yaw, math.radians(-10.0), abs_tol=1e-9)
    assert controller.yaw_initialized


def test_goal_horizontal_safety_envelope():
    controller = make_goal_stub()
    controller.set_local_goal = lambda goal, log_received=True: (
        SingleControlNode.set_local_goal(controller, goal, log_received)
    )

    assert not SingleControlNode.goal_callback_temp(
        controller, [math.nan, 0.0, 1.0]
    )
    assert not SingleControlNode.goal_callback_temp(controller, [11.0, 0.0, 1.0])

    assert SingleControlNode.goal_callback_temp(controller, [2.0, 3.0, 2.0])
    assert controller.leader_goal == [2.0, 3.0, 2.0]
    assert controller.active_goal == [2.0, 3.0, 2.0]
    assert controller.active_goal_mode == 'absolute'


def test_local_goal_is_changeable_without_publishing_it():
    controller = make_goal_stub()

    SingleControlNode.set_local_goal(controller, [0.0, 0.0, 0.0])
    SingleControlNode.set_local_goal(controller, [0.0, 0.0, 3.0])

    assert controller.active_goal == [0.0, 0.0, 3.0]
    assert controller.active_goal_mode == 'absolute'
    assert controller.formation_offset is None


def test_current_hold_pose_can_be_outside_commanded_goal_envelope():
    controller = make_goal_stub()
    current_pose = [25.0, -30.0, 8.0]

    SingleControlNode.set_local_goal(
        controller, current_pose, log_received=False
    )

    assert controller.leader_goal == current_pose
    assert controller.active_goal == current_pose


def test_local_state_is_published_as_common_enu_odometry():
    publisher = DummyPublisher()
    controller = SimpleNamespace(
        frame_id='2',
        navigation=SimpleNamespace(
            current_pos=[1.0, 5.0, 3.0, 0.0, 0.0, 0.5, 0.866],
            local_velocity=[0.2, -0.1, 0.3],
        ),
        swarm_state_publisher=publisher,
        get_clock=lambda: DummyClock(),
    )

    SingleControlNode.publish_swarm_state(controller)

    message = publisher.last_message
    assert message.header.frame_id == 'world'
    assert message.child_frame_id == '2'
    assert message.pose.pose.position.x == 1.0
    assert message.pose.pose.position.y == 5.0
    assert message.pose.pose.position.z == 3.0
    assert message.pose.pose.orientation.z == 0.5
    assert message.pose.pose.orientation.w == 0.866
    assert message.twist.twist.linear.x == 0.2
    assert message.twist.twist.linear.y == -0.1
    assert message.twist.twist.linear.z == 0.3


def test_peer_state_cache_uses_local_receipt_time_and_ignores_self():
    clock = DummyClock()
    controller = SimpleNamespace(
        frame_id='1',
        peer_states={},
        get_clock=lambda: clock,
    )
    message = Odometry()
    message.header.frame_id = 'world'
    message.child_frame_id = '2'
    message.pose.pose.position.x = 4.0
    message.pose.pose.position.y = 5.0
    message.pose.pose.position.z = 6.0
    message.pose.pose.orientation.w = 1.0

    SingleControlNode.swarm_state_callback(controller, message)

    assert controller.peer_states[2]['position'] == [4.0, 5.0, 6.0]
    assert (
        controller.peer_states[2]['received_at'].nanoseconds
        == clock.now().nanoseconds
    )

    message.child_frame_id = '1'
    message.pose.pose.position.x = 99.0
    SingleControlNode.swarm_state_callback(controller, message)
    assert set(controller.peer_states) == {2}


def test_follower_formation_stores_only_a_local_offset():
    parent = SimpleNamespace(
        frame_id='2',
        leader_id=1,
        last_seen_neighbors={1: DummyNow(), 3: DummyNow()},
        stored_offset=None,
        get_logger=lambda: DummyLogger(),
    )
    parent.set_formation_offset = lambda offset: setattr(
        parent, 'stored_offset', list(offset)
    )
    formation = PatternController(parent)

    formation.set_pattern('square', spacing=5.0, rotation_z=180.0)

    assert parent.stored_offset is not None
    assert math.isclose(parent.stored_offset[0], -5.0, abs_tol=1e-9)
    assert math.isclose(parent.stored_offset[1], 0.0, abs_tol=1e-9)
    assert math.isclose(parent.stored_offset[2], 0.0, abs_tol=1e-9)


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


def make_land_command_parent(accepted=True):
    parent = SimpleNamespace(
        land_requests=0,
        get_logger=lambda: DummyLogger(),
    )

    def request_land():
        parent.land_requests += 1
        return accepted

    parent.request_land = request_land
    return parent


def test_leader_land_is_relayed_and_each_drone_requests_its_own_landing():
    leader = make_land_command_parent()
    follower = make_land_command_parent()
    leader_communication = object.__new__(Communication)
    leader_communication.parent_node = leader
    leader_communication.command_publisher = DummyPublisher()
    follower_communication = object.__new__(Communication)
    follower_communication.parent_node = follower

    station_command = String()
    station_command.data = json.dumps({"command": "land"})
    leader_communication.command_leader_callback(station_command)
    follower_communication.command_callback(
        leader_communication.command_publisher.last_message
    )

    assert leader.land_requests == 1
    assert follower.land_requests == 1
    assert json.loads(
        leader_communication.command_publisher.last_message.data
    ) == {"command": "land"}


def test_rejected_leader_land_is_not_relayed_to_followers():
    leader = make_land_command_parent(accepted=False)
    communication = object.__new__(Communication)
    communication.parent_node = leader
    communication.command_publisher = DummyPublisher()

    station_command = String()
    station_command.data = json.dumps({"command": "land"})
    communication.command_leader_callback(station_command)

    assert leader.land_requests == 1
    assert communication.command_publisher.last_message is None


def make_controlled_landing_stub(height=3.0, landed=False):
    clock = DummyClock()
    vehicle_status = VehicleStatus()
    vehicle_status.arming_state = VehicleStatus.ARMING_STATE_ARMED
    vehicle_status.nav_state = VehicleStatus.NAVIGATION_STATE_OFFBOARD
    land_detected = VehicleLandDetected()
    land_detected.landed = landed
    controller = SimpleNamespace(
        vehicle_status=vehicle_status,
        vehicle_land_detected=land_detected,
        last_land_detected_time=clock.now(),
        telemetry_timeout=2.5,
        mission_active=False,
        motion_enabled=True,
        manual_control=True,
        velocity_goal=[0.2, -0.1, 0.3],
        offboard_setpoint_counter=15,
        state='TAKEOFF',
        landing_fast_height=2.0,
        landing_fast_speed=0.7,
        landing_slow_speed=0.3,
        landing_timeout=45.0,
        landing_started_time=None,
        landing_speed_stage=None,
        landing_disarm_last_sent_time=None,
        distance_to_ground=height,
        initial_world_position=[0.0, 0.0, 0.0],
        navigation=SimpleNamespace(current_pos=[0.0, 0.0, height]),
        published_commands=[],
        heartbeat_count=0,
        setpoint_force_zero_values=[],
        disarm_count=0,
        released_reason=None,
        get_clock=lambda: clock,
        get_logger=lambda: DummyLogger(),
        publish_vehicle_command=lambda command: (
            controller.published_commands.append(command)
        ),
        safety_violation_reason=lambda: None,
    )
    controller.land_detection_is_fresh = lambda: (
        SingleControlNode.land_detection_is_fresh(controller)
    )
    controller.landing_height_above_ground = lambda: (
        SingleControlNode.landing_height_above_ground(controller)
    )
    controller.publish_offboard_control_heartbeat_signal = lambda: setattr(
        controller, 'heartbeat_count', controller.heartbeat_count + 1
    )
    controller.publish_position_setpoint = lambda force_zero=False: (
        controller.setpoint_force_zero_values.append(force_zero)
    )
    controller.disarm = lambda: setattr(
        controller, 'disarm_count', controller.disarm_count + 1
    )
    controller.release_to_pilot = lambda reason: setattr(
        controller, 'released_reason', reason
    )
    return controller, clock


def test_request_land_starts_controlled_offboard_descent():
    controller, _clock = make_controlled_landing_stub()

    assert SingleControlNode.request_land(controller)
    assert controller.published_commands == []
    assert controller.state == 'LANDING'
    assert not controller.motion_enabled
    assert not controller.manual_control
    assert controller.velocity_goal == [0.0, 0.0, 0.0]
    assert controller.offboard_setpoint_counter == 0
    assert controller.landing_started_time is not None


def test_land_rejects_missing_landing_detector_telemetry():
    controller, _clock = make_controlled_landing_stub()
    controller.last_land_detected_time = None

    assert not SingleControlNode.request_land(controller)
    assert controller.state == 'TAKEOFF'


def test_controlled_land_uses_positive_ned_down_and_latches_slow_phase():
    controller, _clock = make_controlled_landing_stub(height=3.0)
    assert SingleControlNode.request_land(controller)

    SingleControlNode.run_landing_control(controller)
    assert controller.velocity_goal == [0.0, 0.0, 0.7]
    assert controller.landing_speed_stage == 'fast'

    controller.distance_to_ground = 1.9
    SingleControlNode.run_landing_control(controller)
    assert controller.velocity_goal == [0.0, 0.0, 0.3]
    assert controller.landing_speed_stage == 'slow'

    # A noisy height estimate must not accelerate descent again near ground.
    controller.distance_to_ground = 2.2
    SingleControlNode.run_landing_control(controller)
    assert controller.velocity_goal == [0.0, 0.0, 0.3]
    assert controller.landing_speed_stage == 'slow'


def test_controlled_land_disarms_only_after_fresh_px4_touchdown():
    controller, clock = make_controlled_landing_stub(height=0.1, landed=False)
    assert SingleControlNode.request_land(controller)

    SingleControlNode.run_landing_control(controller)
    assert controller.disarm_count == 0

    controller.vehicle_land_detected.landed = True
    SingleControlNode.run_landing_control(controller)
    assert controller.velocity_goal == [0.0, 0.0, 0.0]
    assert controller.setpoint_force_zero_values[-1]
    assert controller.disarm_count == 1

    # Do not flood PX4, but retry a possibly dropped safe-disarm command.
    SingleControlNode.run_landing_control(controller)
    assert controller.disarm_count == 1
    clock.advance(1.1)
    controller.last_land_detected_time = clock.now()
    SingleControlNode.run_landing_control(controller)
    assert controller.disarm_count == 2


def test_controlled_land_stops_on_timeout():
    controller, clock = make_controlled_landing_stub(height=3.0)
    assert SingleControlNode.request_land(controller)
    clock.advance(45.1)
    controller.last_land_detected_time = clock.now()

    SingleControlNode.run_landing_control(controller)

    assert controller.released_reason == 'Controlled LAND timed out'


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


def test_mission_accepts_optional_leader_yaw_without_changing_position_shape():
    controller, _clock = make_mission_stub()
    controller.yaw = 0.0
    controller.yaw_initialized = False

    accepted = SingleControlNode.start_mission(
        controller,
        [[0.0, 0.0, 2.0, 90.0], [2.0, 0.0, 2.0]],
        relative_to_start=True,
    )

    assert accepted
    assert controller.mission == [[10.0, 20.0, 2.0], [12.0, 20.0, 2.0]]
    assert controller.mission_yaws == [math.pi / 2.0, None]
    assert controller.mission_target_yaw == math.pi / 2.0
    assert controller.yaw == math.pi / 2.0
    assert controller.yaw_initialized


def test_local_state_leader_yaw_rotates_formation_without_changing_geometry():
    clock = DummyClock()
    parent = SimpleNamespace(
        frame_id='2',
        leader_id=1,
        is_leader=False,
        peer_states={},
        leader_yaw_enu=None,
        leader_yaw_received_at=None,
        active_goal_mode='formation',
        last_seen_neighbors={1: DummyNow()},
        stored_offset=None,
        get_clock=lambda: clock,
        get_logger=lambda: DummyLogger(),
    )
    parent.set_formation_offset = lambda offset: setattr(
        parent, 'stored_offset', list(offset)
    )
    formation = PatternController(parent)
    parent.formation = formation

    formation.set_pattern('circle', spacing=3.0)

    def publish_peer_yaw(peer_id, yaw_degrees):
        yaw = math.radians(yaw_degrees)
        message = Odometry()
        message.header.frame_id = 'world'
        message.child_frame_id = str(peer_id)
        message.pose.pose.orientation.z = math.sin(yaw / 2.0)
        message.pose.pose.orientation.w = math.cos(yaw / 2.0)
        SingleControlNode.swarm_state_callback(parent, message)

    publish_peer_yaw(1, 0.0)
    initial_offset = list(parent.stored_offset)
    publish_peer_yaw(1, 30.0)
    rotated_offset = list(parent.stored_offset)

    initial_radius = math.hypot(initial_offset[0], initial_offset[1])
    rotated_radius = math.hypot(rotated_offset[0], rotated_offset[1])
    initial_bearing = math.atan2(initial_offset[1], initial_offset[0])
    rotated_bearing = math.atan2(rotated_offset[1], rotated_offset[0])

    assert math.isclose(initial_radius, 3.0, abs_tol=1e-9)
    assert math.isclose(rotated_radius, initial_radius, abs_tol=1e-9)
    assert math.isclose(
        rotated_bearing - initial_bearing,
        math.radians(30.0),
        abs_tol=1e-9,
    )
    assert math.isclose(
        rotated_bearing - parent.leader_yaw_enu,
        initial_bearing,
        abs_tol=1e-9,
    )

    # A different follower's orientation must not rotate this formation.
    publish_peer_yaw(3, 90.0)
    assert parent.stored_offset == rotated_offset
    assert math.isclose(parent.leader_yaw_enu, math.radians(30.0))

    # A leader update must not overwrite a follower's unrelated absolute goal.
    parent.active_goal_mode = 'absolute'
    publish_peer_yaw(1, 90.0)
    assert parent.stored_offset == rotated_offset


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


def test_px4_ground_auto_disarm_allows_a_later_explicit_arm():
    controller, _clock = make_mission_stub()
    controller.manual_velocity = [0.0, 0.0, 0.0]
    controller.offboard_setpoint_counter = 0
    controller.offboard_was_confirmed = True
    controller.release_to_pilot = lambda reason: (
        SingleControlNode.release_to_pilot(controller, reason)
    )

    vehicle_status = VehicleStatus()
    vehicle_status.nav_state = VehicleStatus.NAVIGATION_STATE_OFFBOARD
    vehicle_status.arming_state = VehicleStatus.ARMING_STATE_DISARMED
    SingleControlNode.vehicle_status_callback(controller, vehicle_status)

    assert controller.state == 'PILOT_CONTROL'
    assert not controller.motion_enabled
    assert not controller.offboard_was_confirmed
