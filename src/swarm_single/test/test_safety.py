import math
from types import SimpleNamespace

from builtin_interfaces.msg import Time
from px4_msgs.msg import VehicleCommand, VehicleLocalPosition, VehicleStatus

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


def test_arm_activates_goal_tracking_only_in_single_drone_mode():
    for group_enabled, expected_motion_enabled in ((False, True), (True, False)):
        controller = SimpleNamespace(
            state='IDLE',
            group_enabled=group_enabled,
            vehicle_status=SimpleNamespace(failsafe=False),
            failsafe_flags=SimpleNamespace(
                manual_control_signal_lost=False,
                local_position_invalid=False,
                local_velocity_invalid=False,
            ),
            navigation=SimpleNamespace(
                current_pos=[1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0]
            ),
            motion_enabled=False,
            group_motion_active=True,
            manual_control=True,
            velocity_goal=[1.0, 1.0, 1.0],
            offboard_setpoint_counter=10,
            offboard_was_confirmed=True,
            telemetry_is_fresh=lambda: True,
            shared_frame_is_ready=lambda: True,
            set_goal_transform=lambda _goal, log_received=True: None,
            reset_mission_state=lambda: None,
            get_logger=lambda: DummyLogger(),
        )

        SingleControlNode.request_offboard_control(controller)

        assert controller.state == 'ARMING'
        assert controller.motion_enabled is expected_motion_enabled
        assert not controller.group_motion_active
        assert not controller.manual_control


def test_non_finite_velocity_is_rejected():
    controller = make_controller_stub()
    controller.velocity_goal = [math.nan, 0.0, 0.0]

    SingleControlNode.publish_position_setpoint(controller)

    assert controller.trajectory_setpoint_publisher.last_message is None
    assert controller.released_reason == 'Non-finite velocity setpoint rejected'


def test_vehicle_command_targets_this_drone_mav_sys_id():
    controller = SimpleNamespace(
        drone_id='2',
        vehicle_command_publisher=DummyPublisher(),
        get_clock=lambda: DummyClock(),
    )

    SingleControlNode.publish_vehicle_command(
        controller,
        VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
        param1=1.0,
    )

    message = controller.vehicle_command_publisher.last_message
    assert message.target_system == 2
    assert message.target_component == 1
    assert message.param1 == 1.0


def test_configured_origin_converts_px4_local_ned_to_shared_enu():
    controller = SimpleNamespace(
        group_enabled=True,
        shared_frame_mode='configured_offsets',
        configured_origin_offset=[4.0, 1.0, 0.5],
        shared_frame_ready=False,
        shared_frame_error='waiting',
    )
    local_position = VehicleLocalPosition()
    local_position.x = 2.0   # north
    local_position.y = 1.5   # east
    local_position.z = -3.0  # down

    shared = SingleControlNode.local_position_to_shared_enu(
        controller, local_position
    )

    assert shared == [5.5, 3.0, 3.5]
    assert controller.shared_frame_ready


def test_leader_waits_for_follower_group_motion_acknowledgement():
    clock = DummyClock()
    parent = SimpleNamespace(
        state='TAKEOFF',
        group_enabled=True,
        required_drone_ids=[1, 2],
        drone_id='1',
        group_status_timeout=2.5,
        group_motion_active=False,
        motion_enabled=False,
        is_leader=True,
        message='',
        get_clock=lambda: clock,
        get_logger=lambda: DummyLogger(),
    )
    communication = Communication.__new__(Communication)
    communication.parent_node = parent
    communication.pending_group_motion_reason = None
    communication.pending_group_motion_start = None
    communication.drone_statuses = {
        2: ({'group_motion_active': False}, clock.now())
    }
    communication.group_readiness = lambda require_offboard: {
        'ready': True,
        'state': 'READY',
    }
    communication.broadcast_swarm_command = lambda *_args, **_kwargs: None
    communication._own_drone_status = lambda: {
        'group_motion_active': parent.group_motion_active
    }

    assert communication.begin_group_motion('test move')
    assert parent.group_motion_active
    assert not parent.motion_enabled

    communication.drone_statuses[2] = (
        {'group_motion_active': True},
        clock.now(),
    )
    communication._monitor_group_motion()

    assert parent.motion_enabled
    assert communication.pending_group_motion_reason is None


def test_follower_holds_when_leader_group_status_is_inactive():
    clock = DummyClock()
    parent = SimpleNamespace(
        group_enabled=True,
        drone_id='2',
        leader_id=1,
        is_leader=False,
        group_motion_active=True,
        group_status_timeout=2.5,
        stopped_reason=None,
        get_clock=lambda: clock,
    )
    parent.stop_group_motion = lambda reason: setattr(
        parent, 'stopped_reason', reason
    )
    communication = Communication.__new__(Communication)
    communication.parent_node = parent
    communication.follower_group_start_time = None
    communication.drone_statuses = {
        1: (
            {
                'group_motion_active': False,
                'offboard': True,
                'control_state': 'TAKEOFF',
            },
            clock.now(),
        )
    }

    communication._monitor_group_motion()

    assert parent.stopped_reason == 'leader group status is inactive or stale'


def test_single_drone_mode_does_not_require_group_motion_acknowledgement():
    parent = SimpleNamespace(
        group_enabled=False,
        group_motion_active=True,
        stopped_reason=None,
    )
    parent.stop_group_motion = lambda reason: setattr(
        parent, 'stopped_reason', reason
    )
    communication = Communication.__new__(Communication)
    communication.parent_node = parent

    communication._monitor_group_motion()

    assert parent.group_motion_active
    assert parent.stopped_reason is None


def make_goal_stub():
    return SimpleNamespace(
        navigation=SimpleNamespace(
            current_pos=[0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        ),
        max_goal_distance=10.0,
        get_logger=lambda: DummyLogger(),
        get_clock=lambda: DummyClock(),
        px4_model='real_drone',
        frame_id='1',
        goal_frame='goal',
        goal_tf_broadcaster=DummyBroadcaster(),
        leader_goal=[0.0, 0.0, 0.0],
        sended_goal_ack=True,
    )


def test_goal_validation_has_no_software_altitude_limit():
    controller = make_goal_stub()
    controller.set_goal_transform = lambda goal, log_received=True: (
        SingleControlNode.set_goal_transform(controller, goal, log_received)
    )

    assert not SingleControlNode.goal_callback_temp(
        controller, [math.nan, 0.0, 1.0]
    )
    assert not SingleControlNode.goal_callback_temp(controller, [11.0, 0.0, 1.0])
    assert SingleControlNode.goal_callback_temp(controller, [0.0, 0.0, 6.0])

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


def test_mission_has_no_software_altitude_limit():
    controller, _clock = make_mission_stub()

    accepted = SingleControlNode.start_mission(
        controller,
        [[0.0, 0.0, 20.0]],
        relative_to_start=True,
    )

    assert accepted
    assert controller.mission == [[10.0, 20.0, 20.0]]


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
    controller.group_motion_active = True
    controller.is_leader = True
    controller.group_stop_reason = None
    controller.communication = SimpleNamespace(
        broadcast_group_stop=lambda reason: setattr(
            controller, 'group_stop_reason', reason
        )
    )
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
    assert controller.group_stop_reason == (
        'PX4 left Offboard mode (RC/pilot takeover)'
    )
