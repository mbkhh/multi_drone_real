#!/usr/bin/env python3

import rclpy
import math
from typing import Dict
from rclpy.node import Node
from nav_msgs.msg import Odometry
from swarm_config.config_utils import get_config
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from px4_msgs.msg import (
    FailsafeFlags,
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLandDetected,
    VehicleLocalPosition,
    VehicleStatus,
)

from swarm_single_no_tf.navigation import navigation
from swarm_single_no_tf.formation import PatternController
# from swarm_single_no_tf.lidar_handler import lidarHandler
from swarm_single_no_tf.communication import Communication

class DroneState:
    IDLE = "IDLE"
    ARMING = "ARMING"
    TAKEOFF = "TAKEOFF"
    LANDING = "LANDING"
    PILOT_CONTROL = "PILOT_CONTROL"

class SingleControlNode(Node):

    def __init__(self):
        super().__init__('control_node')
        self.declare_parameter('frame_id', 1)
        
        self.frame_id = str(
            self.get_parameter('frame_id').get_parameter_value().integer_value
        )
        self.px4_model = get_config('swarm_sim.px4_model')
        self.declare_parameter('swarm_state_topic', '/swarm/local_state')
        self.swarm_state_topic = (
            self.get_parameter('swarm_state_topic')
            .get_parameter_value().string_value
        )
        self.declare_parameter('use_configured_world_origin', True)
        self.use_configured_world_origin = (
            self.get_parameter('use_configured_world_origin')
            .get_parameter_value().bool_value
        )
        self.declare_parameter('simulation_disable_safety_checks', False)
        self.simulation_disable_safety_checks = (
            self.get_parameter('simulation_disable_safety_checks')
            .get_parameter_value().bool_value
        )
        if self.simulation_disable_safety_checks:
            self.get_logger().warning(
                'ALL companion safety gates are DISABLED by launch parameter. '
                'This mode is for SITL only.'
            )
        self.declare_parameter('require_manual_control_signal', True)
        self.require_manual_control_signal = (
            self.get_parameter('require_manual_control_signal')
            .get_parameter_value().bool_value
        )
        if self.require_manual_control_signal:
            self.get_logger().info(
                'Real-flight RC/manual-control safety gate is ENABLED.'
            )
        else:
            self.get_logger().warning(
                'RC/manual-control safety gate is DISABLED by launch parameter. '
                'Use this override only in SITL.'
            )
        configured_start = [0.0, 0.0, 0.0]
        if self.use_configured_world_origin:
            configured_start = self.config_vector3(
                f'swarm_single.real_world.initial_positions.{self.frame_id}'
            )
        self.declare_parameter('initial_world_position', configured_start)
        self.initial_world_position = [
            float(value) for value in (
                self.get_parameter('initial_world_position')
                .get_parameter_value().double_array_value
            )
        ]
        if (
            len(self.initial_world_position) != 3
            or not all(
                math.isfinite(value) for value in self.initial_world_position
            )
        ):
            raise ValueError(
                'initial_world_position must contain three finite ENU values.'
            )
        self.latest_px4_position_enu = None
        self.px4_position_origin_enu = None
        self.world_origin_calibrated = False

        self.last_seen_neighbors: Dict[int, rclpy.time.Time] = {}
        self.peer_states = {}
                    
        self.leader_id = None
        self.is_leader = False
        self.leader_yaw_enu = None
        self.leader_yaw_received_at = None

        self.active_goal = None
        self.active_goal_mode = None
        self.formation_offset = None

        self.velocity_goal = [0.0, 0.0, 0.0]
        self.manual_velocity = [0.0, 0.0, 0.0]
        self.leader_goal = [0.0, 0.0, 0.0]
        self.yaw = 0.0
        # Measured yaw in the common ENU world frame. This is published by
        # the leader; self.yaw remains the PX4/NED setpoint used locally.
        self.current_yaw_enu = None
        self.current_yaw_ned = None
        # self.yaw is the requested PX4/NED yaw. This separate value is the
        # rate-limited yaw actually sent in TrajectorySetpoint messages.
        self.commanded_yaw = 0.0
        self.last_yaw_setpoint_time = None
        self.yaw_initialized = False

        self.mission = []
        # Keep the existing three-value position representation intact. Yaw
        # is stored in a parallel list so old missions and callers remain
        # compatible. Mission-file yaw is supplied in degrees and converted
        # to PX4/NED radians while parsing.
        self.mission_yaws = []
        self.mission_active = False
        self.mission_index = 0
        self.mission_target = None
        self.mission_target_yaw = None
        self.mission_start_time = None
        self.mission_dwell_start = None
        self.mission_state = "IDLE"
        self.message = ""
        self.manual_control = False
        self.motion_enabled = False
        self.state = DroneState.IDLE
        self.offboard_was_confirmed = False
        self.last_vehicle_status_time = None
        self.last_local_position_time = None
        self.last_failsafe_flags_time = None
        self.last_land_detected_time = None
        self.distance_to_ground = None
        self.failsafe_flags = FailsafeFlags()
        self.vehicle_land_detected = VehicleLandDetected()
        self.telemetry_timeout = self.config_float(
            'swarm_single.control.telemetry_timeout', 2.5
        )
        self.max_horizontal_speed = self.config_float(
            'swarm_single.control.max_horizontal_speed', 1.0
        )
        self.max_vertical_speed = self.config_float(
            'swarm_single.control.max_vertical_speed', 0.5
        )
        self.landing_fast_height = self.config_float(
            'swarm_single.control.landing_fast_height', 2.0
        )
        self.landing_fast_speed = self.config_float(
            'swarm_single.control.landing_fast_speed', self.max_vertical_speed
        )
        self.landing_slow_speed = self.config_float(
            'swarm_single.control.landing_slow_speed', 0.3
        )
        self.landing_timeout = self.config_float(
            'swarm_single.control.landing_timeout', 45.0
        )
        landing_values = (
            self.landing_fast_height,
            self.landing_fast_speed,
            self.landing_slow_speed,
            self.landing_timeout,
        )
        if not all(
            math.isfinite(value) and value > 0.0 for value in landing_values
        ):
            raise ValueError(
                'Landing height, speeds, and timeout must be finite and positive.'
            )
        if self.landing_slow_speed > self.landing_fast_speed:
            raise ValueError(
                'landing_slow_speed cannot exceed landing_fast_speed.'
            )
        if self.landing_fast_speed > self.max_vertical_speed:
            raise ValueError(
                'landing_fast_speed cannot exceed max_vertical_speed.'
            )
        self.max_horizontal_acceleration = self.config_float(
            'swarm_single.control.max_horizontal_acceleration', 1.0
        )
        self.max_vertical_acceleration = self.config_float(
            'swarm_single.control.max_vertical_acceleration', 0.7
        )
        self.max_yaw_rate_deg_s = self.config_float(
            'swarm_single.control.max_yaw_rate_deg_s', 20.0
        )
        if (
            not math.isfinite(self.max_yaw_rate_deg_s)
            or self.max_yaw_rate_deg_s <= 0.0
        ):
            raise ValueError(
                'max_yaw_rate_deg_s must be finite and greater than zero.'
            )
        self.max_yaw_rate = math.radians(self.max_yaw_rate_deg_s)
        self.max_control_interval = self.config_float(
            'swarm_single.control.max_control_interval', 0.25
        )
        self.minimum_setpoint_rate = self.config_float(
            'swarm_single.control.minimum_setpoint_rate', 8.0
        )
        self.velocity_debug_interval = self.config_float(
            'swarm_single.control.velocity_debug_interval', 0.5
        )
        self.max_goal_distance = self.config_float(
            'swarm_single.control.max_goal_distance', 10.0
        )
        self.min_goal_altitude = self.config_float(
            'swarm_single.control.min_goal_altitude', -0.5
        )
        self.max_goal_altitude = self.config_float(
            'swarm_single.control.max_goal_altitude', 5.0
        )
        self.mission_goal_tolerance = self.config_float(
            'swarm_single.mission.goal_tolerance', 0.4
        )
        self.mission_waypoint_dwell = self.config_float(
            'swarm_single.mission.waypoint_dwell', 1.0
        )
        self.mission_timeout = self.config_float(
            'swarm_single.mission.timeout', 180.0
        )
        self.max_mission_waypoints = self.config_int(
            'swarm_single.mission.max_waypoints', 100
        )

        self.navigation = navigation(self)
        self.formation = PatternController(self)
        self.communication = Communication(self)

        self.drone_id = self.frame_id
        
        # QoS Profile for telemetry and setpoints (Depth = 1)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # QoS Profile strictly for Vehicle Commands (Depth = 10) like in the working code
        qos_profile_cmd = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        swarm_state_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=max(10, int(get_config('swarm_sim.drone_count') or 1) * 2),
        )

        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, f"/uav_{self.drone_id}/fmu/in/offboard_control_mode", qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, f"/uav_{self.drone_id}/fmu/in/trajectory_setpoint", qos_profile)
        
        # Applying the deeper QoS profile here
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, f"/uav_{self.drone_id}/fmu/in/vehicle_command", qos_profile_cmd)

        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, f"/uav_{self.drone_id}/fmu/out/vehicle_status_v1", self.vehicle_status_callback, qos_profile)
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition,
            f"/uav_{self.drone_id}/fmu/out/vehicle_local_position_v1",
            self.vehicle_local_position_callback,
            qos_profile,
        )
        self.failsafe_flags_subscriber = self.create_subscription(
            FailsafeFlags,
            f"/uav_{self.drone_id}/fmu/out/failsafe_flags",
            self.failsafe_flags_callback,
            qos_profile,
        )
        self.vehicle_land_detected_subscriber = self.create_subscription(
            VehicleLandDetected,
            f"/uav_{self.drone_id}/fmu/out/vehicle_land_detected",
            self.vehicle_land_detected_callback,
            qos_profile,
        )
        self.swarm_state_publisher = self.create_publisher(
            Odometry,
            self.swarm_state_topic,
            swarm_state_qos,
        )
        self.swarm_state_subscriber = self.create_subscription(
            Odometry,
            self.swarm_state_topic,
            self.swarm_state_callback,
            swarm_state_qos,
        )

        self.offboard_setpoint_counter = 0
        self.vehicle_status = VehicleStatus()
        self.last_published_velocity_setpoint = None
        self.last_commanded_velocity_setpoint = [0.0, 0.0, 0.0]
        self.velocity_setpoints_since_debug = 0
        self.velocity_debug_last_time = self.get_clock().now()
        self.last_setpoint_time = None
        self.last_control_loop_time = None
        self.last_control_interval = None
        self.last_control_gap_warning_time = None
        self.landing_started_time = None
        self.landing_speed_stage = None
        self.landing_disarm_last_sent_time = None

        # Timer runs at 20Hz
        self.timer = self.create_timer(0.05, self.control_loop_callback)
        self.velocity_debug_timer = self.create_timer(
            self.velocity_debug_interval,
            self.velocity_setpoint_debug_callback,
        )

        self.sended_goal_ack = True

        if self.use_configured_world_origin:
            self.get_logger().info(
                f'Drone {self.drone_id} configured ARM position (ENU): '
                f'{self.initial_world_position}. Waiting for ARM calibration.'
            )
        self.get_logger().info(f"SingleControlNode successfully initialized for Drone {self.drone_id}.")

    def control_loop_callback(self):
        """Contain controller errors so PX4 can execute its configured failsafe."""
        try:
            self.run_control_loop()
        except Exception as error:
            self.get_logger().error(
                f'Controller exception: {type(error).__name__}: {error}'
            )
            self.release_to_pilot('Controller exception')

    def velocity_setpoint_debug_callback(self):
        now = self.get_clock().now()
        elapsed = (
            now - self.velocity_debug_last_time
        ).nanoseconds / 1e9
        self.velocity_debug_last_time = now
        sample_count = self.velocity_setpoints_since_debug
        self.velocity_setpoints_since_debug = 0

        if sample_count == 0:
            if self.state not in (DroneState.IDLE, DroneState.PILOT_CONTROL):
                self.get_logger().warning(
                    f'[VELOCITY DEBUG] drone={self.drone_id} state={self.state} '
                    f'published=0 during {elapsed:.3f} s.'
                )
            else :
                self.get_logger().warning("[VELOCITY DEBUG] IN Idle or pilot mode")
            return

        north, east, down = self.last_published_velocity_setpoint
        total_speed = math.sqrt(north ** 2 + east ** 2 + down ** 2)
        publish_rate = sample_count / elapsed if elapsed > 0.0 else math.inf
        message = (
            f'[VELOCITY DEBUG] drone={self.drone_id} state={self.state} '
            f'published={sample_count} rate={publish_rate:.1f} Hz | '
            f'NED velocity: north={north:+.3f}, east={east:+.3f}, '
            f'down={down:+.3f} m/s | ENU up={-down:+.3f} m/s | '
            f'speed={total_speed:.3f} m/s'
        )
        if publish_rate < self.minimum_setpoint_rate:
            self.get_logger().warning(f'[CONTROL RATE LOW] {message}')
        else:
            self.get_logger().info(message)

    def update_control_loop_timing(self):
        """Return False for a scheduling gap unsafe for closed-loop motion."""
        now = self.get_clock().now()
        if self.last_control_loop_time is None:
            self.last_control_loop_time = now
            return True

        interval = (
            now - self.last_control_loop_time
        ).nanoseconds / 1e9
        self.last_control_loop_time = now
        self.last_control_interval = interval
        if 0.0 <= interval <= self.max_control_interval:
            return True

        should_warn = self.last_control_gap_warning_time is None
        if self.last_control_gap_warning_time is not None:
            elapsed = (
                now - self.last_control_gap_warning_time
            ).nanoseconds / 1e9
            should_warn = elapsed >= 1.0
        if should_warn:
            self.get_logger().warning(
                f'[CONTROL GAP] loop interval was {interval:.3f} s; '
                'commanding zero velocity until fresh feedback is processed.'
            )
            self.last_control_gap_warning_time = now
        return False

    def run_control_loop(self):
        """Run the explicit, RC-preemptible Offboard state machine at 20 Hz."""

        control_timing_ok = self.update_control_loop_timing()

        # PX4 may auto-disarm while waiting on the ground (for example after
        # COM_DISARM_PRFLT expires). Do not leave the companion controller
        # latched in TAKEOFF, where a later explicit ARM would be ignored.
        if (
            self.state == DroneState.TAKEOFF
            and self.vehicle_status.arming_state
            != VehicleStatus.ARMING_STATE_ARMED
        ):
            self.release_to_pilot('PX4 disarmed while companion control was active')
            return

        # Once PX4 accepts an RC mode change, stop all Offboard traffic
        # immediately. The controller remains latched out until a new explicit
        # station ARM command calls request_offboard_control().
        if (
            self.state == DroneState.TAKEOFF
            and self.vehicle_status.nav_state
            != VehicleStatus.NAVIGATION_STATE_OFFBOARD
        ):
            self.release_to_pilot('PX4 left Offboard mode (RC/pilot takeover)')
            return

        if self.state in (DroneState.IDLE, DroneState.PILOT_CONTROL):
            return

        safety_violation = self.safety_violation_reason()
        if safety_violation is not None:
            self.release_to_pilot(safety_violation)
            return

        if self.state == DroneState.ARMING:
            # Stream a zero-velocity setpoint while preparing Offboard. An ARM
            # command must never also be an implicit takeoff command.
            self.velocity_goal = [0.0, 0.0, 0.0]
            self.publish_offboard_control_heartbeat_signal()
            self.publish_position_setpoint()
            self.offboard_setpoint_counter += 1

            if (
                self.vehicle_status.nav_state
                == VehicleStatus.NAVIGATION_STATE_OFFBOARD
            ):
                self.offboard_was_confirmed = True
            elif self.offboard_was_confirmed:
                self.release_to_pilot(
                    'Pilot changed mode during the Offboard arming sequence'
                )
                return

            # PX4 requires the Offboard proof-of-life stream before mode entry.
            if self.offboard_setpoint_counter == 25:
                self.get_logger().info("Requesting Offboard mode...")
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)

            elif (
                self.offboard_setpoint_counter == 35
                and self.vehicle_status.arming_state
                != VehicleStatus.ARMING_STATE_ARMED
            ):
                self.arm()

            elif (
                self.offboard_setpoint_counter >= 40
                and self.vehicle_status.nav_state
                == VehicleStatus.NAVIGATION_STATE_OFFBOARD
                and self.vehicle_status.arming_state
                == VehicleStatus.ARMING_STATE_ARMED
            ):
                self.state = DroneState.TAKEOFF
                self.offboard_setpoint_counter = 0
                self.get_logger().info(
                    "Offboard armed in HOLD. Waiting for an explicit movement command."
                )

            elif self.offboard_setpoint_counter >= 100:
                self.release_to_pilot(
                    'Offboard/arming confirmation timed out'
                )

        elif self.state == DroneState.TAKEOFF:
            self.publish_offboard_control_heartbeat_signal()
            if self.motion_enabled:
                if control_timing_ok:
                    feedback_valid = self.navigation.navigate_to_goal()
                else:
                    self.velocity_goal = [0.0, 0.0, 0.0]
                    feedback_valid = False
            else:
                self.velocity_goal = [0.0, 0.0, 0.0]
                feedback_valid = True
            if feedback_valid:
                self.update_mission_progress()
            self.publish_position_setpoint(force_zero=not feedback_valid)

        elif self.state == DroneState.LANDING:
            self.run_landing_control(control_timing_ok)

    def request_offboard_control(self):
        """Start Offboard only after an explicit station command."""
        if self.state in (DroneState.ARMING, DroneState.TAKEOFF):
            self.get_logger().info('Offboard control is already active or starting.')
            return False

        safety_violation = self.safety_violation_reason(
            require_preflight=True
        )
        if safety_violation is not None:
            self.get_logger().error(
                f'Offboard rejected: {safety_violation}.'
            )
            return False

        if self.use_configured_world_origin:
            if (
                self.vehicle_status.arming_state
                == VehicleStatus.ARMING_STATE_ARMED
            ):
                if not self.world_origin_calibrated:
                    self.get_logger().error(
                        'Offboard rejected: the shared world origin cannot be '
                        'initialized for the first time while the vehicle is armed.'
                    )
                    return False
                self.get_logger().warning(
                    'ARM received while already armed; preserving the existing '
                    'world origin to prevent an in-flight TF jump.'
                )
            elif not self.calibrate_world_origin():
                return False

        # Reset the relative-goal origin to the measured current pose. This
        # prevents an old goal from causing motion after pilot takeover.
        current_pose = [float(value) for value in self.navigation.current_pos[:3]]
        if not all(math.isfinite(value) for value in current_pose):
            self.get_logger().error(
                'Offboard rejected: current PX4 local pose is not finite.'
            )
            return False
        self.set_local_goal(current_pose, log_received=False)
        self.reset_mission_state()
        self.motion_enabled = False
        self.manual_control = False
        self.velocity_goal = [0.0, 0.0, 0.0]
        self.last_commanded_velocity_setpoint = [0.0, 0.0, 0.0]
        self.last_setpoint_time = None
        self.last_yaw_setpoint_time = None
        self.offboard_setpoint_counter = 0
        self.offboard_was_confirmed = False
        self.landing_started_time = None
        self.landing_speed_stage = None
        self.landing_disarm_last_sent_time = None
        self.state = DroneState.ARMING
        self.get_logger().warning(
            'Explicit ARM received: preparing Offboard with zero velocity, '
            'then switching mode and arming PX4.'
        )
        return True

    def release_to_pilot(self, reason):
        """Latch this node out and cease heartbeat/setpoint publication."""
        if self.mission_active:
            self.abort_mission(reason)
        self.state = DroneState.PILOT_CONTROL
        self.motion_enabled = False
        self.manual_control = False
        self.manual_velocity = [0.0, 0.0, 0.0]
        self.velocity_goal = [0.0, 0.0, 0.0]
        self.last_commanded_velocity_setpoint = [0.0, 0.0, 0.0]
        self.last_setpoint_time = None
        self.last_yaw_setpoint_time = None
        self.offboard_setpoint_counter = 0
        self.offboard_was_confirmed = False
        self.landing_started_time = None
        self.landing_speed_stage = None
        self.landing_disarm_last_sent_time = None
        self.get_logger().warning(
            f'{reason}. ROS control released; PX4/RC/failsafe owns control. '
            'A new station ARM command is required to re-enter Offboard.'
        )

    def request_land(self):
        """Start a locally controlled Offboard descent on this vehicle."""
        if self.vehicle_status.arming_state != VehicleStatus.ARMING_STATE_ARMED:
            self.get_logger().warning('LAND ignored: vehicle is not armed.')
            return False
        if self.state == DroneState.LANDING:
            self.get_logger().info('LAND is already active on this vehicle.')
            return True
        if (
            self.state != DroneState.TAKEOFF
            or self.vehicle_status.nav_state
            != VehicleStatus.NAVIGATION_STATE_OFFBOARD
        ):
            self.get_logger().error(
                'LAND rejected: companion Offboard control is not active.'
            )
            return False
        safety_violation = self.safety_violation_reason()
        if safety_violation is not None:
            self.get_logger().error(f'LAND rejected: {safety_violation}.')
            return False
        if not self.land_detection_is_fresh():
            self.get_logger().error(
                'LAND rejected: PX4 landing-detector telemetry is missing/stale.'
            )
            return False
        if self.mission_active:
            self.abort_mission('LAND requested')
        self.motion_enabled = False
        self.manual_control = False
        self.velocity_goal = [0.0, 0.0, 0.0]
        self.offboard_setpoint_counter = 0
        self.landing_started_time = self.get_clock().now()
        self.landing_speed_stage = None
        self.landing_disarm_last_sent_time = None
        self.state = DroneState.LANDING
        self.get_logger().warning(
            'LAND requested: starting controlled Offboard descent. PX4 NED '
            f'down speed is {self.landing_fast_speed:.2f} m/s above '
            f'{self.landing_fast_height:.2f} m and '
            f'{self.landing_slow_speed:.2f} m/s below it.'
        )
        return True

    def land_detection_is_fresh(self):
        """Return whether PX4's received landing state is recent enough to trust."""
        if self.last_land_detected_time is None:
            return False
        age = (
            self.get_clock().now() - self.last_land_detected_time
        ).nanoseconds / 1e9
        return 0.0 <= age <= self.telemetry_timeout

    def landing_height_above_ground(self):
        """Prefer PX4 terrain distance, then fall back to calibrated launch Z."""
        distance = self.distance_to_ground
        if distance is not None and math.isfinite(distance) and distance >= 0.0:
            return distance

        current_height = float(self.navigation.current_pos[2])
        launch_height = float(self.initial_world_position[2])
        if not all(
            math.isfinite(value) for value in (current_height, launch_height)
        ):
            return None
        return max(0.0, current_height - launch_height)

    def run_landing_control(self, control_timing_ok=True):
        """Descend in Offboard and disarm only after PX4 reports landed."""
        if self.vehicle_status.arming_state != VehicleStatus.ARMING_STATE_ARMED:
            self.get_logger().info('Landed and disarmed. Returning to IDLE.')
            self.state = DroneState.IDLE
            self.offboard_setpoint_counter = 0
            self.landing_started_time = None
            self.landing_speed_stage = None
            self.landing_disarm_last_sent_time = None
            return

        # If PX4 itself enters AUTO_LAND (for example due to a failsafe), stop
        # sending Offboard traffic and let the autopilot own the descent.
        if (
            self.vehicle_status.nav_state
            == VehicleStatus.NAVIGATION_STATE_AUTO_LAND
        ):
            return
        if (
            self.vehicle_status.nav_state
            != VehicleStatus.NAVIGATION_STATE_OFFBOARD
        ):
            self.release_to_pilot('PX4 entered another mode during LAND')
            return
        if not self.land_detection_is_fresh():
            self.release_to_pilot(
                'PX4 landing-detector telemetry became missing/stale during LAND'
            )
            return

        now = self.get_clock().now()
        if self.landing_started_time is None:
            self.landing_started_time = now
        elapsed = (now - self.landing_started_time).nanoseconds / 1e9
        if elapsed > self.landing_timeout:
            self.release_to_pilot('Controlled LAND timed out')
            return

        if self.vehicle_land_detected.landed:
            self.velocity_goal = [0.0, 0.0, 0.0]
            self.publish_offboard_control_heartbeat_signal()
            self.publish_position_setpoint(force_zero=True)

            should_send_disarm = self.landing_disarm_last_sent_time is None
            if self.landing_disarm_last_sent_time is not None:
                since_disarm = (
                    now - self.landing_disarm_last_sent_time
                ).nanoseconds / 1e9
                should_send_disarm = since_disarm >= 1.0
            if should_send_disarm:
                self.disarm()
                self.landing_disarm_last_sent_time = now
                self.get_logger().warning(
                    'PX4 landing detector confirms touchdown; safe disarm sent.'
                )
            return

        if not control_timing_ok:
            self.velocity_goal = [0.0, 0.0, 0.0]
            self.publish_offboard_control_heartbeat_signal()
            self.publish_position_setpoint(force_zero=True)
            return

        height = self.landing_height_above_ground()
        if height is None:
            self.release_to_pilot(
                'Controlled LAND has no finite height-above-ground estimate'
            )
            return

        # Latch the slow phase so noisy terrain estimates cannot accelerate the
        # vehicle again near the ground.
        if (
            self.landing_speed_stage == 'slow'
            or height <= self.landing_fast_height
        ):
            stage = 'slow'
            down_speed = self.landing_slow_speed
        else:
            stage = 'fast'
            down_speed = self.landing_fast_speed
        if stage != self.landing_speed_stage:
            self.landing_speed_stage = stage
            self.get_logger().warning(
                f'Controlled LAND {stage} phase: height={height:.2f} m, '
                f'PX4 NED down={down_speed:.2f} m/s.'
            )

        # PX4 velocity setpoints are NED: positive Z velocity means down.
        self.velocity_goal = [0.0, 0.0, down_speed]
        self.publish_offboard_control_heartbeat_signal()
        self.publish_position_setpoint()

    def request_safe_disarm(self):
        """Disarm only when PX4's landing detector confirms the vehicle is down."""
        if self.vehicle_status.arming_state != VehicleStatus.ARMING_STATE_ARMED:
            self.state = DroneState.IDLE
            self.get_logger().info('Vehicle is already disarmed.')
            return
        if (
            self.last_land_detected_time is None
            or (
                self.get_clock().now() - self.last_land_detected_time
            ).nanoseconds / 1e9 > self.telemetry_timeout
            or not self.vehicle_land_detected.landed
        ):
            self.get_logger().error(
                'DISARM REFUSED: fresh PX4 landing confirmation is unavailable.'
            )
            return
        self.disarm()
        self.state = DroneState.PILOT_CONTROL
        self.motion_enabled = False
        self.get_logger().warning('Safe disarm sent while landed.')

    def telemetry_is_fresh(self):
        now = self.get_clock().now()
        timestamps = (
            self.last_vehicle_status_time,
            self.last_local_position_time,
            self.last_failsafe_flags_time,
        )
        if any(timestamp is None for timestamp in timestamps):
            return False
        return all(
            0.0 <= (now - timestamp).nanoseconds / 1e9 <= self.telemetry_timeout
            for timestamp in timestamps
        )

    def safety_violation_reason(self, require_preflight=False):
        """Return the first PX4/RC condition that makes Offboard unsafe."""
        if getattr(self, 'simulation_disable_safety_checks', False):
            return None
        if not self.telemetry_is_fresh():
            return (
                'PX4 vehicle status, local position, or failsafe telemetry '
                'is missing/stale'
            )
        #if require_preflight and not self.vehicle_status.pre_flight_checks_pass:
        #    return 'PX4 preflight checks have not passed'
        if self.vehicle_status.failsafe:
            return 'PX4 reports an active failsafe'
        if (
            self.require_manual_control_signal
            and self.failsafe_flags.manual_control_signal_lost
        ):
            return 'RC/manual-control signal is unavailable'
        if (
            self.failsafe_flags.local_position_invalid
            or self.failsafe_flags.local_velocity_invalid
        ):
            return 'PX4 local position/velocity is invalid'
        return None

    @staticmethod
    def config_float(key, default):
        value = get_config(key)
        return float(default if value is None else value)

    @staticmethod
    def config_int(key, default):
        value = get_config(key)
        return int(default if value is None else value)

    @staticmethod
    def config_vector3(key):
        value = get_config(key)
        if not isinstance(value, (list, tuple)) or len(value) != 3:
            raise ValueError(
                f"Configuration '{key}' must contain exactly three values."
            )
        try:
            vector = [float(component) for component in value]
        except (TypeError, ValueError) as error:
            raise ValueError(
                f"Configuration '{key}' must contain numeric values."
            ) from error
        if not all(math.isfinite(component) for component in vector):
            raise ValueError(
                f"Configuration '{key}' must contain finite values."
            )
        return vector

    def world_position_from_px4_enu(self, px4_position_enu):
        if not self.use_configured_world_origin:
            return list(px4_position_enu)
        if self.px4_position_origin_enu is None:
            self.px4_position_origin_enu = list(px4_position_enu)
        return [
            self.initial_world_position[axis]
            + px4_position_enu[axis]
            - self.px4_position_origin_enu[axis]
            for axis in range(3)
        ]

    def calibrate_world_origin(self):
        if not self.use_configured_world_origin:
            return True
        if self.latest_px4_position_enu is None or not all(
            math.isfinite(value) for value in self.latest_px4_position_enu
        ):
            self.get_logger().error(
                'Offboard rejected: no finite PX4 local position is available '
                'for world-frame calibration.'
            )
            return False

        self.px4_position_origin_enu = list(self.latest_px4_position_enu)
        self.world_origin_calibrated = True
        self.navigation.current_pos[:3] = list(self.initial_world_position)
        self.get_logger().warning(
            f'Drone {self.drone_id} world origin calibrated: current position '
            f'is {self.initial_world_position} ENU.'
        )
        return True

    def reset_mission_state(self):
        """Clear mission execution without changing the active flight mode."""
        self.mission = []
        self.mission_yaws = []
        self.mission_active = False
        self.mission_index = 0
        self.mission_target = None
        self.mission_target_yaw = None
        self.mission_start_time = None
        self.mission_dwell_start = None
        self.mission_state = "IDLE"

    def start_mission(self, points, relative_to_start=True):
        """Validate and start a waypoint mission using the normal goal path."""
        if self.state != DroneState.TAKEOFF:
            self.get_logger().error(
                'Mission rejected: ARM Offboard and wait for confirmation first.'
            )
            return False
        if self.manual_control:
            self.get_logger().error(
                'Mission rejected: manual control is active.'
            )
            return False
        if not self.telemetry_is_fresh():
            self.get_logger().error(
                'Mission rejected: PX4 telemetry is missing or stale.'
            )
            return False
        if not isinstance(points, (list, tuple)) or not points:
            self.get_logger().error(
                'Mission rejected: at least one waypoint is required.'
            )
            return False
        if len(points) > self.max_mission_waypoints:
            self.get_logger().error(
                f'Mission rejected: {len(points)} waypoints exceeds the '
                f'{self.max_mission_waypoints} waypoint limit.'
            )
            return False

        mission_origin = [
            float(value) for value in self.navigation.current_pos[:3]
        ]
        if not all(math.isfinite(value) for value in mission_origin):
            self.get_logger().error(
                'Mission rejected: current local position is not finite.'
            )
            return False

        resolved_points = []
        resolved_yaws = []
        previous = mission_origin
        for index, point in enumerate(points):
            if not isinstance(point, (list, tuple)) or len(point) not in (3, 4):
                self.get_logger().error(
                    f'Mission rejected: waypoint {index + 1} must contain '
                    'x, y, z and optionally yaw.'
                )
                return False
            try:
                waypoint = [float(value) for value in point[:3]]
            except (TypeError, ValueError):
                self.get_logger().error(
                    f'Mission rejected: waypoint {index + 1} is not numeric.'
                )
                return False
            if not all(math.isfinite(value) for value in waypoint):
                self.get_logger().error(
                    f'Mission rejected: waypoint {index + 1} is not finite.'
                )
                return False

            waypoint_yaw = None
            if len(point) == 4:
                try:
                    waypoint_yaw = float(point[3])
                except (TypeError, ValueError):
                    self.get_logger().error(
                        f'Mission rejected: waypoint {index + 1} yaw '
                        '(degrees) is not numeric.'
                    )
                    return False
                if not math.isfinite(waypoint_yaw):
                    self.get_logger().error(
                        f'Mission rejected: waypoint {index + 1} yaw '
                        '(degrees) is not finite.'
                    )
                    return False
                # Mission files use degrees for readability. PX4 setpoints
                # use radians, so convert and normalize equivalent complete
                # turns before storing the internal value.
                waypoint_yaw = math.radians(waypoint_yaw)
                waypoint_yaw = math.atan2(
                    math.sin(waypoint_yaw), math.cos(waypoint_yaw)
                )

            if relative_to_start:
                target = [
                    mission_origin[axis] + waypoint[axis]
                    for axis in range(3)
                ]
            else:
                target = waypoint

            leg_distance = math.hypot(
                target[0] - previous[0], target[1] - previous[1]
            )
            if leg_distance > self.max_goal_distance:
                self.get_logger().error(
                    f'Mission rejected: waypoint {index + 1} has a '
                    f'{leg_distance:.2f} m horizontal leg, exceeding the '
                    f'{self.max_goal_distance:.2f} m goal limit.'
                )
                return False
            resolved_points.append(target)
            resolved_yaws.append(waypoint_yaw)
            previous = target

        self.mission = resolved_points
        self.mission_yaws = resolved_yaws
        self.mission_active = True
        self.mission_index = 0
        self.mission_target = None
        self.mission_target_yaw = None
        self.mission_start_time = self.get_clock().now()
        self.mission_dwell_start = None
        self.mission_state = "RUNNING"
        self.message = f'MISSION STARTED: {len(self.mission)} waypoints'
        return self.activate_current_mission_waypoint()

    def activate_current_mission_waypoint(self):
        """Send the current waypoint through the tested normal goal handler."""
        if not self.mission_active or self.mission_index >= len(self.mission):
            return False

        target = self.mission[self.mission_index]
        if not self.goal_callback_temp(target):
            self.abort_mission(
                f'waypoint {self.mission_index + 1} was rejected'
            )
            return False

        self.mission_target = list(target)
        mission_yaws = getattr(self, 'mission_yaws', [])
        target_yaw = (
            mission_yaws[self.mission_index]
            if self.mission_index < len(mission_yaws)
            else None
        )
        self.mission_target_yaw = target_yaw
        if target_yaw is not None:
            # self.yaw is the local PX4/NED yaw setpoint. Followers do not
            # copy this value; they receive measured leader ENU yaw instead.
            self.yaw = float(target_yaw)
            self.yaw_initialized = True
        self.mission_dwell_start = None
        self.motion_enabled = True
        yaw_log = (
            f', yaw={target_yaw:.3f} rad (PX4/NED)'
            if target_yaw is not None
            else ''
        )
        self.get_logger().info(
            f'Mission waypoint {self.mission_index + 1}/{len(self.mission)} '
            f'activated: [{target[0]:.2f}, {target[1]:.2f}, {target[2]:.2f}]'
            f'{yaw_log}'
        )
        return True

    def update_mission_progress(self):
        """Advance after the vehicle remains within tolerance at a waypoint."""
        if not self.mission_active or self.mission_target is None:
            return

        now = self.get_clock().now()
        if (
            self.mission_timeout > 0.0
            and self.mission_start_time is not None
            and (now - self.mission_start_time).nanoseconds / 1e9
            > self.mission_timeout
        ):
            self.abort_mission('mission timeout')
            return

        current = [float(value) for value in self.navigation.current_pos[:3]]
        if not all(math.isfinite(value) for value in current):
            self.abort_mission('current local position is not finite')
            return

        distance = math.dist(current, self.mission_target)
        if distance > self.mission_goal_tolerance:
            self.mission_dwell_start = None
            return

        if self.mission_dwell_start is None:
            self.mission_dwell_start = now
            return
        if (
            (now - self.mission_dwell_start).nanoseconds / 1e9
            < self.mission_waypoint_dwell
        ):
            return

        self.mission_index += 1
        if self.mission_index >= len(self.mission):
            self.complete_mission()
            return
        self.activate_current_mission_waypoint()

    def complete_mission(self):
        """End at the final waypoint and keep Offboard armed in zero-velocity hold."""
        waypoint_count = len(self.mission)
        self.mission_active = False
        self.mission_index = waypoint_count
        self.mission_target = None
        self.mission_target_yaw = None
        self.mission_dwell_start = None
        self.motion_enabled = False
        self.velocity_goal = [0.0, 0.0, 0.0]
        self.mission_state = "COMPLETED"
        self.message = f'MISSION COMPLETE: {waypoint_count} waypoints'
        self.get_logger().info(self.message)

    def abort_mission(self, reason):
        """Stop mission-generated motion while leaving PX4 failsafes in charge."""
        if not self.mission_active:
            return
        self.mission_active = False
        self.mission_target = None
        self.mission_target_yaw = None
        self.mission_dwell_start = None
        self.motion_enabled = False
        self.velocity_goal = [0.0, 0.0, 0.0]
        self.mission_state = "ABORTED"
        self.message = f'MISSION ABORTED: {reason}'
        self.get_logger().warning(self.message)

    def publish_offboard_control_heartbeat_signal(self):
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def vehicle_status_callback(self, vehicle_status):
        self.vehicle_status = vehicle_status
        self.last_vehicle_status_time = self.get_clock().now()
        if (
            self.state == DroneState.TAKEOFF
            and vehicle_status.arming_state
            != VehicleStatus.ARMING_STATE_ARMED
        ):
            self.release_to_pilot(
                'PX4 disarmed while companion control was active'
            )
            return
        # PX4 owns RC mode selection. Release the companion controller as soon
        # as PX4 confirms a switch away from Offboard; the control-loop check
        # remains as a redundant guard. No station/network connection is
        # required for this takeover path.
        if (
            self.state == DroneState.TAKEOFF
            and vehicle_status.nav_state
            != VehicleStatus.NAVIGATION_STATE_OFFBOARD
        ):
            self.release_to_pilot(
                'PX4 left Offboard mode (RC/pilot takeover)'
            )

    def failsafe_flags_callback(self, failsafe_flags):
        self.failsafe_flags = failsafe_flags
        self.last_failsafe_flags_time = self.get_clock().now()

    def vehicle_land_detected_callback(self, land_detected):
        self.vehicle_land_detected = land_detected
        self.last_land_detected_time = self.get_clock().now()

    def vehicle_local_position_callback(self, local_position):
        """Update local ENU state and share it directly with the swarm."""
        if not (local_position.xy_valid and local_position.z_valid):
            return
        px4_position_enu = [
            float(local_position.y),
            float(local_position.x),
            float(-local_position.z),
        ]
        if not all(math.isfinite(value) for value in px4_position_enu):
            return
        self.latest_px4_position_enu = px4_position_enu
        world_position = self.world_position_from_px4_enu(px4_position_enu)
        self.last_local_position_time = self.get_clock().now()
        if getattr(local_position, 'dist_bottom_valid', False):
            distance = float(local_position.dist_bottom)
            self.distance_to_ground = (
                distance if math.isfinite(distance) and distance >= 0.0 else None
            )
        else:
            self.distance_to_ground = None

        # PX4 local position is NED. Swarm state and navigation use ENU.
        heading_ned = float(local_position.heading)
        yaw_enu = (math.pi / 2.0) - heading_ned
        orientation_z = math.sin(yaw_enu / 2.0)
        orientation_w = math.cos(yaw_enu / 2.0)
        if math.isfinite(heading_ned):
            self.current_yaw_ned = math.atan2(
                math.sin(heading_ned), math.cos(heading_ned)
            )
            self.current_yaw_enu = math.atan2(
                math.sin(yaw_enu), math.cos(yaw_enu)
            )

        self.navigation.local_velocity = [
            float(local_position.vy),
            float(local_position.vx),
            float(-local_position.vz),
        ]
        self.navigation.current_pos = [
            world_position[0],
            world_position[1],
            world_position[2],
            0.0,
            0.0,
            orientation_z,
            orientation_w,
        ]
        self.publish_swarm_state()
        if self.state in (
            DroneState.IDLE,
            DroneState.ARMING,
            DroneState.PILOT_CONTROL,
        ):
            self.yaw = heading_ned
            self.commanded_yaw = heading_ned
            self.yaw_initialized = math.isfinite(self.yaw)

    def publish_swarm_state(self):
        """Publish common-ENU position, orientation, and velocity without TF."""
        current = self.navigation.current_pos
        velocity = self.navigation.local_velocity
        values = [*current, *velocity]
        if not all(math.isfinite(float(value)) for value in values):
            return

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.child_frame_id = self.frame_id
        msg.pose.pose.position.x = float(current[0])
        msg.pose.pose.position.y = float(current[1])
        msg.pose.pose.position.z = float(current[2])
        msg.pose.pose.orientation.x = float(current[3])
        msg.pose.pose.orientation.y = float(current[4])
        msg.pose.pose.orientation.z = float(current[5])
        msg.pose.pose.orientation.w = float(current[6])
        msg.twist.twist.linear.x = float(velocity[0])
        msg.twist.twist.linear.y = float(velocity[1])
        msg.twist.twist.linear.z = float(velocity[2])
        self.swarm_state_publisher.publish(msg)

    @staticmethod
    def yaw_from_enu_quaternion(orientation):
        """Extract normalized ENU yaw from an [x, y, z, w] quaternion."""
        try:
            x, y, z, w = (float(value) for value in orientation)
        except (TypeError, ValueError):
            return None
        if not all(math.isfinite(value) for value in (x, y, z, w)):
            return None

        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm <= 1e-9:
            return None
        x, y, z, w = (value / norm for value in (x, y, z, w))

        sin_yaw = 2.0 * (w * z + x * y)
        cos_yaw = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(sin_yaw, cos_yaw)

    def swarm_state_callback(self, msg):
        """Cache peer odometry and apply the elected leader's embedded yaw."""
        try:
            peer_id = int(msg.child_frame_id)
        except (TypeError, ValueError):
            return
        if peer_id == int(self.frame_id) or msg.header.frame_id != 'world':
            return

        position = [
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            float(msg.pose.pose.position.z),
        ]
        orientation = [
            float(msg.pose.pose.orientation.x),
            float(msg.pose.pose.orientation.y),
            float(msg.pose.pose.orientation.z),
            float(msg.pose.pose.orientation.w),
        ]
        velocity = [
            float(msg.twist.twist.linear.x),
            float(msg.twist.twist.linear.y),
            float(msg.twist.twist.linear.z),
        ]
        if not all(
            math.isfinite(value)
            for value in [*position, *orientation, *velocity]
        ):
            return

        received_at = self.get_clock().now()
        yaw_enu = SingleControlNode.yaw_from_enu_quaternion(orientation)
        self.peer_states[peer_id] = {
            'position': position,
            'orientation': orientation,
            'yaw_enu': yaw_enu,
            'velocity': velocity,
            'received_at': received_at,
        }

        # Every drone publishes orientation in /swarm/local_state, but a
        # follower rotates its fixed formation offset only from the currently
        # elected leader. Therefore non-leader yaw cannot disturb formation.
        leader_id = getattr(self, 'leader_id', None)
        if getattr(self, 'is_leader', False) or leader_id is None:
            return
        try:
            leader_id = int(leader_id)
        except (TypeError, ValueError):
            return
        if peer_id != leader_id or yaw_enu is None:
            return

        self.leader_yaw_enu = yaw_enu
        self.leader_yaw_received_at = received_at
        formation = getattr(self, 'formation', None)
        if (
            formation is not None
            and getattr(self, 'active_goal_mode', None) == 'formation'
        ):
            formation.update_leader_yaw(yaw_enu)
        
    def arm(self):
        self.get_logger().info("Sending ARM command.")
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

    def disarm(self):
        self.get_logger().info('Sending DISARM command.')
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)

    def publish_position_setpoint(self, force_zero=False):
        if not self.yaw_initialized:
            self.release_to_pilot('No valid PX4 heading for Offboard setpoint')
            return

        north = float(self.velocity_goal[0])
        east = float(self.velocity_goal[1])
        down = float(self.velocity_goal[2])
        if not all(math.isfinite(value) for value in (north, east, down)):
            self.release_to_pilot('Non-finite velocity setpoint rejected')
            return

        horizontal_speed = math.hypot(north, east)
        if horizontal_speed > self.max_horizontal_speed:
            scale = self.max_horizontal_speed / horizontal_speed
            north *= scale
            east *= scale
        down = max(-self.max_vertical_speed, min(self.max_vertical_speed, down))

        now = self.get_clock().now()
        if force_zero:
            north, east, down = 0.0, 0.0, 0.0
            self.last_commanded_velocity_setpoint = [0.0, 0.0, 0.0]
        else:
            north, east, down = self.limit_velocity_change(
                [north, east, down], now
            )
        self.last_setpoint_time = now
        commanded_yaw = self.limit_yaw_change(now)

        msg = TrajectorySetpoint()
        msg.position = [math.nan, math.nan, math.nan]
        msg.velocity = [north, east, down]
        msg.acceleration = [math.nan, math.nan, math.nan]
        msg.jerk = [math.nan, math.nan, math.nan]
        msg.yaw = commanded_yaw
        msg.yawspeed = math.nan

        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.last_published_velocity_setpoint = (north, east, down)
        self.velocity_setpoints_since_debug += 1

    def limit_yaw_change(self, now):
        """Move the outgoing yaw setpoint toward the requested yaw safely."""
        target = float(self.yaw)
        if not math.isfinite(target):
            self.release_to_pilot('Non-finite yaw setpoint rejected')
            return 0.0
        target = math.atan2(math.sin(target), math.cos(target))

        current = getattr(self, 'commanded_yaw', target)
        if not math.isfinite(float(current)):
            current = target
        current = math.atan2(math.sin(float(current)), math.cos(float(current)))

        previous_time = getattr(self, 'last_yaw_setpoint_time', None)
        if previous_time is None:
            current = target
        else:
            interval = (now - previous_time).nanoseconds / 1e9
            interval = max(0.0, min(interval, self.max_control_interval))
            max_delta = self.max_yaw_rate * interval
            # Use the shortest path across the +/-pi wrap boundary.
            error = math.atan2(
                math.sin(target - current), math.cos(target - current)
            )
            if abs(error) <= max_delta:
                current = target
            elif error > 0.0:
                current += max_delta
            else:
                current -= max_delta
            current = math.atan2(math.sin(current), math.cos(current))

        self.commanded_yaw = current
        self.last_yaw_setpoint_time = now
        return current

    def request_relative_yaw(self, delta_degrees):
        """Request a relative PX4/NED yaw change from the station."""
        try:
            delta_degrees = float(delta_degrees)
        except (TypeError, ValueError):
            self.get_logger().error(
                'YAW rejected: relative angle must be numeric degrees.'
            )
            return False
        if not math.isfinite(delta_degrees):
            self.get_logger().error(
                'YAW rejected: relative angle must be finite degrees.'
            )
            return False
        if self.state != DroneState.TAKEOFF:
            self.get_logger().warning(
                'YAW rejected: the leader must be armed in Offboard TAKEOFF '
                'state before a yaw move is accepted.'
            )
            return False

        current = getattr(self, 'current_yaw_ned', None)
        if current is None or not math.isfinite(float(current)):
            current = getattr(self, 'yaw', None)
        if current is None or not math.isfinite(float(current)):
            self.get_logger().error(
                'YAW rejected: no finite current PX4 heading is available.'
            )
            return False

        target = float(current) + math.radians(delta_degrees)
        self.yaw = math.atan2(math.sin(target), math.cos(target))
        self.yaw_initialized = True
        if self.mission_active:
            self.abort_mission('replaced by a station yaw command')
        self.get_logger().info(
            f'Relative yaw command accepted: delta={delta_degrees:+.1f} '
            f'deg, target={math.degrees(self.yaw):+.1f} PX4/NED deg.'
        )
        return True

    def limit_velocity_change(self, desired, now):
        """Slew-limit NED velocity so delayed feedback cannot reverse instantly."""
        previous = list(getattr(
            self, 'last_commanded_velocity_setpoint', [0.0, 0.0, 0.0]
        ))
        previous_time = getattr(self, 'last_setpoint_time', None)
        if previous_time is None:
            limited = [float(value) for value in desired]
            self.last_commanded_velocity_setpoint = limited
            return tuple(limited)

        interval = (now - previous_time).nanoseconds / 1e9
        interval = max(0.0, min(interval, self.max_control_interval))

        horizontal_delta = [
            float(desired[0]) - previous[0],
            float(desired[1]) - previous[1],
        ]
        horizontal_delta_norm = math.hypot(*horizontal_delta)
        max_horizontal_delta = self.max_horizontal_acceleration * interval
        if (
            horizontal_delta_norm > max_horizontal_delta
            and horizontal_delta_norm > 0.0
        ):
            scale = max_horizontal_delta / horizontal_delta_norm
            horizontal_delta = [value * scale for value in horizontal_delta]

        vertical_delta = float(desired[2]) - previous[2]
        max_vertical_delta = self.max_vertical_acceleration * interval
        vertical_delta = max(
            -max_vertical_delta,
            min(max_vertical_delta, vertical_delta),
        )

        limited = [
            previous[0] + horizontal_delta[0],
            previous[1] + horizontal_delta[1],
            previous[2] + vertical_delta,
        ]
        self.last_commanded_velocity_setpoint = limited
        return tuple(limited)

    def publish_vehicle_command(self, command, **params):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = float(params.get("param1", 0.0))
        msg.param2 = float(params.get("param2", 0.0))
        msg.param3 = float(params.get("param3", 0.0))
        msg.param4 = float(params.get("param4", 0.0))
        msg.param5 = float(params.get("param5", 0.0))
        msg.param6 = float(params.get("param6", 0.0))
        msg.param7 = float(params.get("param7", 0.0))
        msg.target_system = int(self.vehicle_status.system_id)
        msg.target_component = 1 #int(self.vehicle_status.component_id)
        msg.source_system = 255#1 # Changed to 1 to match the working code
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def goal_callback_temp(self, goal):
        if len(goal) != 3 or not all(math.isfinite(float(v)) for v in goal):
            self.get_logger().error('Goal rejected: coordinates must be finite.')
            return False

        goal = [float(v) for v in goal]
        current = self.navigation.current_pos
        horizontal_distance = math.hypot(
            goal[0] - current[0], goal[1] - current[1]
        )
        if horizontal_distance > self.max_goal_distance:
            self.get_logger().error(
                f'Goal rejected: {horizontal_distance:.2f} m horizontal move '
                f'exceeds {self.max_goal_distance:.2f} m safety limit.'
            )
            return False
        self.set_local_goal(goal)
        return True

    def set_local_goal(self, goal, log_received=True):
        if log_received:
            self.get_logger().info(
                f'Received goal request: [{goal[0]:.2f}, {goal[1]:.2f}, '
                f'{goal[2]:.2f}]. Storing local absolute goal.'
            )
        self.sended_goal_ack = False
        self.active_goal = [float(goal[0]), float(goal[1]), float(goal[2])]
        self.active_goal_mode = 'absolute'
        self.formation_offset = None
        self.leader_goal = list(self.active_goal)

    def set_formation_offset(self, offset):
        """Store a follower offset; its absolute goal is resolved locally."""
        values = [float(value) for value in offset]
        if len(values) != 3 or not all(math.isfinite(value) for value in values):
            raise ValueError('Formation offset must contain three finite values.')
        self.formation_offset = values
        self.active_goal = None
        self.active_goal_mode = 'formation'

def main(args=None):
    rclpy.init(args=args)
    node = SingleControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Caught KeyboardInterrupt, shutting down node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
