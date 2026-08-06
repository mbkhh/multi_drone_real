#!/usr/bin/env python3

import rclpy
import tf2_ros
import math
from typing import Dict
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from swarm_config.config_utils import get_config
from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
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

from swarm_single.navigation import navigation
from swarm_single.formation import PatternController
# from swarm_single.lidar_handler import lidarHandler
from swarm_single.communication import Communication

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
        self.drone_id = self.frame_id
        self.px4_model = get_config('swarm_sim.px4_model')
        self.goal_frame = get_config('swarm_single.goal_frame_name')

        self.last_seen_neighbors: Dict[int, rclpy.time.Time] = {}
                    
        self.leader_id = None
        self.is_leader = False

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.local_position_tf_broadcaster = TransformBroadcaster(self)
        self.goal_tf_broadcaster = StaticTransformBroadcaster(self)

        self.velocity_goal = [0.0, 0.0, 0.0]
        self.manual_velocity = [0.0, 0.0, 0.0]
        self.leader_goal = [0.0, 0.0, 0.0]
        self.yaw = 0.0
        self.yaw_initialized = False

        self.mission = []
        self.mission_active = False
        self.mission_index = 0
        self.mission_target = None
        self.mission_start_time = None
        self.mission_dwell_start = None
        self.mission_state = "IDLE"
        self.message = ""
        self.manual_control = False
        self.motion_enabled = False
        self.group_motion_active = False
        self.state = DroneState.IDLE
        self.offboard_was_confirmed = False
        self.last_vehicle_status_time = None
        self.last_local_position_time = None
        self.last_failsafe_flags_time = None
        self.last_land_detected_time = None
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
        self.max_goal_distance = self.config_float(
            'swarm_single.control.max_goal_distance', 10.0
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

        self.group_enabled = self.config_bool(
            'swarm_single.group.enabled', False
        )
        configured_ids = get_config(
            'swarm_single.group.required_drone_ids'
        )
        if not isinstance(configured_ids, list) or not configured_ids:
            drone_count = self.config_int('swarm_sim.drone_count', 1)
            configured_ids = list(range(1, drone_count + 1))
        self.required_drone_ids = sorted(
            {int(drone_id) for drone_id in configured_ids}
        )
        self.group_status_timeout = self.config_float(
            'swarm_single.group.status_timeout', self.telemetry_timeout
        )
        self.shared_frame_mode = str(
            get_config('swarm_single.group.shared_frame.mode')
            or 'configured_offsets'
        ).lower()
        self.shared_frame_reference_id = self.config_int(
            'swarm_single.group.shared_frame.reference_drone_id',
            self.required_drone_ids[0],
        )
        self.shared_frame_max_eph = self.config_float(
            'swarm_single.group.shared_frame.max_eph', 2.0
        )
        self.shared_frame_max_epv = self.config_float(
            'swarm_single.group.shared_frame.max_epv', 3.0
        )
        self.shared_frame_reference = None
        self.last_shared_frame_reference_time = None
        self.configured_origin_offset = self.get_configured_origin_offset()
        self.shared_frame_ready = not self.group_enabled
        self.shared_frame_error = ''
        if self.group_enabled:
            if self.shared_frame_mode == 'configured_offsets':
                self.shared_frame_ready = self.configured_origin_offset is not None
                if not self.shared_frame_ready:
                    self.shared_frame_error = (
                        f'no configured ENU origin offset for drone {self.drone_id}'
                    )
            elif self.shared_frame_mode == 'gps':
                self.shared_frame_ready = False
                self.shared_frame_error = 'waiting for global EKF references'
            else:
                self.shared_frame_ready = False
                self.shared_frame_error = (
                    f'unsupported shared-frame mode {self.shared_frame_mode!r}'
                )

        self.vehicle_status = VehicleStatus()
        self.navigation = navigation(self)
        self.formation = PatternController(self)
        self.communication = Communication(self)
        
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
        self.reference_local_position_subscriber = None
        if (
            self.group_enabled
            and self.shared_frame_mode == 'gps'
            and int(self.drone_id) != self.shared_frame_reference_id
        ):
            self.reference_local_position_subscriber = self.create_subscription(
                VehicleLocalPosition,
                f'/uav_{self.shared_frame_reference_id}/fmu/out/'
                'vehicle_local_position_v1',
                self.reference_local_position_callback,
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

        self.offboard_setpoint_counter = 0
        # Timer runs at 20Hz
        self.timer = self.create_timer(0.05, self.control_loop_callback)

        self.sended_goal_ack = True
        
        self.get_logger().info(f"SingleControlNode successfully initialized for Drone {self.drone_id}.")
        if self.group_enabled:
            self.get_logger().warning(
                f'Group mode enabled for drones {self.required_drone_ids}; '
                f'shared frame mode={self.shared_frame_mode}, '
                f'ready={self.shared_frame_ready}.'
            )

    def control_loop_callback(self):
        """Contain controller errors so PX4 can execute its configured failsafe."""
        try:
            self.run_control_loop()
        except Exception as error:
            self.get_logger().error(
                f'Controller exception: {type(error).__name__}: {error}'
            )
            self.release_to_pilot('Controller exception')

    def run_control_loop(self):
        """Run the explicit, RC-preemptible Offboard state machine at 20 Hz."""

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

        if not self.telemetry_is_fresh():
            self.release_to_pilot('PX4 telemetry timeout')
            return

        if self.state == DroneState.ARMING:
            # Match the original single-drone controller: keep resolving the
            # current goal while PX4 enters Offboard instead of forcing a
            # separate zero-velocity HOLD state. A real group still waits for
            # its coordinated movement handshake before tracking formation.
            if self.group_enabled:
                self.velocity_goal = [0.0, 0.0, 0.0]
            else:
                self.navigation.navigate_to_goal()
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
                if self.group_enabled:
                    self.get_logger().info(
                        'Offboard armed; waiting for coordinated group movement.'
                    )
                else:
                    self.get_logger().info(
                        'Offboard armed; local goal tracking is active.'
                    )

            elif self.offboard_setpoint_counter >= 100:
                self.release_to_pilot(
                    'Offboard/arming confirmation timed out'
                )

        elif self.state == DroneState.TAKEOFF:
            self.publish_offboard_control_heartbeat_signal()
            self.update_mission_progress()
            if self.motion_enabled:
                self.navigation.navigate_to_goal()
            else:
                self.velocity_goal = [0.0, 0.0, 0.0]
            self.publish_position_setpoint()

        elif self.state == DroneState.LANDING:
            self.offboard_setpoint_counter += 1
            if self.vehicle_status.arming_state != VehicleStatus.ARMING_STATE_ARMED:
                self.get_logger().info("Landed and disarmed. Returning to IDLE.")
                self.state = DroneState.IDLE
                self.offboard_setpoint_counter = 0
            elif (
                self.vehicle_status.nav_state
                == VehicleStatus.NAVIGATION_STATE_AUTO_LAND
            ):
                return
            elif (
                self.vehicle_status.nav_state
                != VehicleStatus.NAVIGATION_STATE_OFFBOARD
            ):
                self.release_to_pilot(
                    'PX4 entered another mode during LAND request'
                )
            elif self.offboard_setpoint_counter <= 20:
                # Keep Offboard alive only while waiting briefly for PX4 to
                # accept LAND. Repeat a best-effort command three times.
                self.velocity_goal = [0.0, 0.0, 0.0]
                self.publish_offboard_control_heartbeat_signal()
                self.publish_position_setpoint()
                if self.offboard_setpoint_counter in (5, 10):
                    self.publish_vehicle_command(
                        VehicleCommand.VEHICLE_CMD_NAV_LAND
                    )
            else:
                self.release_to_pilot(
                    'PX4 did not accept AUTO_LAND; LAND will not be forced repeatedly'
                )

    def request_offboard_control(self):
        """Start Offboard only after an explicit station command."""
        if self.state in (DroneState.ARMING, DroneState.TAKEOFF):
            self.get_logger().info('Offboard control is already active or starting.')
            return
        if not self.telemetry_is_fresh():
            self.get_logger().error(
                'Offboard rejected: vehicle status, local position, or '
                'failsafe telemetry is missing/stale.'
            )
            return
        if self.group_enabled and not self.shared_frame_is_ready():
            self.get_logger().error(
                f'Offboard rejected: shared group frame is not ready '
                f'({self.shared_frame_error}).'
            )
            return
        # if not self.vehicle_status.pre_flight_checks_pass:
        #     self.get_logger().error(
        #         'Offboard rejected: PX4 preflight checks have not passed.'
        #     )
        #     return
        if self.vehicle_status.failsafe:
            self.get_logger().error('Offboard rejected: PX4 is in failsafe.')
            return
        if self.failsafe_flags.manual_control_signal_lost:
            self.get_logger().error(
                'Offboard rejected: RC/manual-control signal is unavailable.'
            )
            return
        if (
            self.failsafe_flags.local_position_invalid
            or self.failsafe_flags.local_velocity_invalid
        ):
            self.get_logger().error(
                'Offboard rejected: PX4 local position/velocity is invalid.'
            )
            return

        # Reset the relative-goal origin to the measured current pose. This
        # prevents an old goal from causing motion after pilot takeover.
        current_pose = [float(value) for value in self.navigation.current_pos[:3]]
        if not all(math.isfinite(value) for value in current_pose):
            self.get_logger().error(
                'Offboard rejected: current PX4 local pose is not finite.'
            )
            return
        self.set_goal_transform(current_pose, log_received=False)
        self.reset_mission_state()
        # Single-drone operation follows the original controller and tracks
        # its current goal immediately. Group operation remains gated until
        # every member acknowledges the coordinated movement command.
        self.motion_enabled = not self.group_enabled
        self.group_motion_active = False
        self.manual_control = False
        self.velocity_goal = [0.0, 0.0, 0.0]
        self.offboard_setpoint_counter = 0
        self.offboard_was_confirmed = False
        self.state = DroneState.ARMING
        self.get_logger().warning(
            'Explicit ARM received: preparing Offboard, then arming PX4.'
        )

    def release_to_pilot(self, reason):
        """Latch this node out and cease heartbeat/setpoint publication."""
        was_group_motion_active = getattr(
            self, 'group_motion_active', False
        )
        if self.mission_active:
            self.abort_mission(reason)
        self.state = DroneState.PILOT_CONTROL
        self.motion_enabled = False
        self.group_motion_active = False
        self.manual_control = False
        self.manual_velocity = [0.0, 0.0, 0.0]
        self.velocity_goal = [0.0, 0.0, 0.0]
        self.offboard_setpoint_counter = 0
        self.offboard_was_confirmed = False
        self.get_logger().warning(
            f'{reason}. ROS control released; PX4/RC/failsafe owns control. '
            'A new station ARM command is required to re-enter Offboard.'
        )
        if (
            was_group_motion_active
            and getattr(self, 'is_leader', False)
            and hasattr(self, 'communication')
        ):
            self.communication.broadcast_group_stop(reason)

    def stop_group_motion(self, reason):
        """Hold this vehicle without taking control away from PX4 or the RC."""
        if self.mission_active:
            self.abort_mission(reason)
        self.motion_enabled = False
        self.group_motion_active = False
        self.manual_control = False
        self.manual_velocity = [0.0, 0.0, 0.0]
        self.velocity_goal = [0.0, 0.0, 0.0]
        self.message = f'GROUP MOTION STOPPED: {reason}'
        self.get_logger().warning(self.message)

    def request_land(self):
        """Ask PX4 to enter its native AUTO_LAND mode once."""
        if self.vehicle_status.arming_state != VehicleStatus.ARMING_STATE_ARMED:
            self.get_logger().warning('LAND ignored: vehicle is not armed.')
            return
        if self.mission_active:
            self.abort_mission('LAND requested')
        self.motion_enabled = False
        self.group_motion_active = False
        self.manual_control = False
        self.velocity_goal = [0.0, 0.0, 0.0]
        self.offboard_setpoint_counter = 0
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.state = DroneState.LANDING
        self.get_logger().warning(
            'LAND requested; waiting briefly for PX4 AUTO_LAND confirmation.'
        )

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
            (now - timestamp).nanoseconds / 1e9 <= self.telemetry_timeout
            for timestamp in timestamps
        )

    @staticmethod
    def config_float(key, default):
        value = get_config(key)
        return float(default if value is None else value)

    @staticmethod
    def config_int(key, default):
        value = get_config(key)
        return int(default if value is None else value)

    @staticmethod
    def config_bool(key, default):
        value = get_config(key)
        return bool(default if value is None else value)

    def get_configured_origin_offset(self):
        """Return this drone's measured origin in the common ENU frame."""
        offsets = get_config(
            'swarm_single.group.shared_frame.origin_offsets'
        )
        if not isinstance(offsets, dict):
            return None
        raw_offset = offsets.get(int(self.drone_id))
        if raw_offset is None:
            raw_offset = offsets.get(str(self.drone_id))
        if not isinstance(raw_offset, (list, tuple)) or len(raw_offset) != 3:
            return None
        try:
            offset = [float(value) for value in raw_offset]
        except (TypeError, ValueError):
            return None
        return offset if all(math.isfinite(value) for value in offset) else None

    def global_reference_is_valid(self, local_position):
        """Check whether PX4's EKF global reference is usable for alignment."""
        values = (
            local_position.ref_lat,
            local_position.ref_lon,
            local_position.ref_alt,
            local_position.eph,
            local_position.epv,
        )
        return (
            local_position.xy_global
            and local_position.z_global
            and not local_position.dead_reckoning
            and all(math.isfinite(float(value)) for value in values)
            and float(local_position.eph) <= self.shared_frame_max_eph
            and float(local_position.epv) <= self.shared_frame_max_epv
        )

    def reference_local_position_callback(self, local_position):
        """Capture the reference drone's global EKF origin for GPS alignment."""
        if not self.global_reference_is_valid(local_position):
            self.shared_frame_ready = False
            self.shared_frame_error = (
                f'drone {self.shared_frame_reference_id} global EKF reference '
                'is invalid or inaccurate'
            )
            return
        self.shared_frame_reference = (
            float(local_position.ref_lat),
            float(local_position.ref_lon),
            float(local_position.ref_alt),
        )
        self.last_shared_frame_reference_time = self.get_clock().now()

    def shared_frame_is_ready(self):
        if not self.group_enabled:
            return True
        if self.shared_frame_mode == 'configured_offsets':
            return self.configured_origin_offset is not None
        if (
            self.shared_frame_mode != 'gps'
            or self.shared_frame_reference is None
            or self.last_shared_frame_reference_time is None
        ):
            return False
        age = (
            self.get_clock().now() - self.last_shared_frame_reference_time
        ).nanoseconds / 1e9
        if age > self.group_status_timeout:
            self.shared_frame_ready = False
            self.shared_frame_error = 'reference drone global EKF data is stale'
            return False
        return self.shared_frame_ready

    def local_position_to_shared_enu(self, local_position):
        """Convert this PX4-local NED position into the configured common ENU."""
        local_enu = [
            float(local_position.y),
            float(local_position.x),
            float(-local_position.z),
        ]
        if not self.group_enabled:
            self.shared_frame_ready = True
            return local_enu

        if self.shared_frame_mode == 'configured_offsets':
            if self.configured_origin_offset is None:
                self.shared_frame_ready = False
                return None
            self.shared_frame_ready = True
            self.shared_frame_error = ''
            return [
                self.configured_origin_offset[axis] + local_enu[axis]
                for axis in range(3)
            ]

        if self.shared_frame_mode != 'gps':
            self.shared_frame_ready = False
            return None
        if not self.global_reference_is_valid(local_position):
            self.shared_frame_ready = False
            self.shared_frame_error = (
                f'drone {self.drone_id} global EKF reference is invalid or '
                'inaccurate'
            )
            return None
        if (
            self.shared_frame_reference is None
            or self.last_shared_frame_reference_time is None
        ):
            self.shared_frame_ready = False
            self.shared_frame_error = 'waiting for reference drone global EKF data'
            return None
        reference_age = (
            self.get_clock().now() - self.last_shared_frame_reference_time
        ).nanoseconds / 1e9
        if reference_age > self.group_status_timeout:
            self.shared_frame_ready = False
            self.shared_frame_error = 'reference drone global EKF data is stale'
            return None

        reference_lat, reference_lon, reference_alt = (
            self.shared_frame_reference
        )
        own_lat = float(local_position.ref_lat)
        own_lon = float(local_position.ref_lon)
        own_alt = float(local_position.ref_alt)
        earth_radius = 6378137.0
        mean_lat = math.radians((reference_lat + own_lat) / 2.0)
        north_offset = earth_radius * math.radians(own_lat - reference_lat)
        east_offset = (
            earth_radius
            * math.cos(mean_lat)
            * math.radians(own_lon - reference_lon)
        )
        up_offset = own_alt - reference_alt
        self.shared_frame_ready = True
        self.shared_frame_error = ''
        return [
            east_offset + local_enu[0],
            north_offset + local_enu[1],
            up_offset + local_enu[2],
        ]

    def reset_mission_state(self):
        """Clear mission execution without changing the active flight mode."""
        self.mission = []
        self.mission_active = False
        self.mission_index = 0
        self.mission_target = None
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
        previous = mission_origin
        for index, point in enumerate(points):
            if not isinstance(point, (list, tuple)) or len(point) != 3:
                self.get_logger().error(
                    f'Mission rejected: waypoint {index + 1} must contain x, y, z.'
                )
                return False
            try:
                waypoint = [float(value) for value in point]
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
            previous = target

        self.mission = resolved_points
        self.mission_active = True
        self.mission_index = 0
        self.mission_target = None
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
        self.mission_dwell_start = None
        self.motion_enabled = True
        self.get_logger().info(
            f'Mission waypoint {self.mission_index + 1}/{len(self.mission)} '
            f'activated: [{target[0]:.2f}, {target[1]:.2f}, {target[2]:.2f}]'
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
        """Publish the real PX4 NED pose into the ENU TF tree used by navigation."""
        if not (local_position.xy_valid and local_position.z_valid):
            return
        self.last_local_position_time = self.get_clock().now()
        if (
            self.group_enabled
            and self.shared_frame_mode == 'gps'
            and int(self.drone_id) == self.shared_frame_reference_id
        ):
            self.reference_local_position_callback(local_position)

        shared_position = self.local_position_to_shared_enu(local_position)
        if shared_position is None:
            return

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'world'
        transform.child_frame_id = f'{self.px4_model}_{self.frame_id}/odom'

        # PX4 local position is NED. The navigation code and TF tree use ENU.
        transform.transform.translation.x = shared_position[0]
        transform.transform.translation.y = shared_position[1]
        transform.transform.translation.z = shared_position[2]

        yaw_enu = (math.pi / 2.0) - float(local_position.heading)
        transform.transform.rotation.z = math.sin(yaw_enu / 2.0)
        transform.transform.rotation.w = math.cos(yaw_enu / 2.0)
        self.local_position_tf_broadcaster.sendTransform(transform)

        # Formation offsets are defined in the shared ENU axes and must not
        # rotate when the leader yaws. This translation-only frame follows the
        # vehicle position while retaining world orientation.
        formation_origin = TransformStamped()
        formation_origin.header.stamp = transform.header.stamp
        formation_origin.header.frame_id = 'world'
        formation_origin.child_frame_id = (
            f'{self.px4_model}_{self.frame_id}/formation_origin'
        )
        formation_origin.transform.translation.x = shared_position[0]
        formation_origin.transform.translation.y = shared_position[1]
        formation_origin.transform.translation.z = shared_position[2]
        formation_origin.transform.rotation.w = 1.0
        self.local_position_tf_broadcaster.sendTransform(formation_origin)

        self.navigation.vel = [
            float(local_position.vx),
            float(local_position.vy),
            float(local_position.vz),
        ]
        self.navigation.current_pos = [
            shared_position[0],
            shared_position[1],
            shared_position[2],
            0.0,
            0.0,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        ]
        if self.state in (
            DroneState.IDLE,
            DroneState.ARMING,
            DroneState.PILOT_CONTROL,
        ):
            self.yaw = float(local_position.heading)
            self.yaw_initialized = math.isfinite(self.yaw)
        
    def arm(self):
        self.get_logger().info("Sending ARM command.")
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

    def disarm(self):
        self.get_logger().info('Sending DISARM command.')
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)

    def publish_position_setpoint(self):
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

        msg = TrajectorySetpoint()
        msg.position = [math.nan, math.nan, math.nan]
        msg.velocity = [north, east, down]
        msg.acceleration = [math.nan, math.nan, math.nan]
        msg.jerk = [math.nan, math.nan, math.nan]
        msg.yaw = self.yaw
        msg.yawspeed = math.nan

        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

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
        # PX4 accepts commands addressed to its MAV_SYS_ID. frame_id is also
        # used for the /uav_<id> DDS namespace, so each node targets only its
        # own flight controller in a multi-vehicle ROS domain.
        msg.target_system = int(self.drone_id)
        msg.target_component = 1
        msg.source_system = 1 # Changed to 1 to match the working code
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
        self.set_goal_transform(goal)
        return True

    def set_goal_transform(self, goal, log_received=True):
        if log_received:
            self.get_logger().info(f'Received goal request: [{goal[0]:.2f}, {goal[1]:.2f}, {goal[2]:.2f}]. Broadcasting static transform.')
        self.sended_goal_ack = False
        static_transform_stamped = TransformStamped()

        static_transform_stamped.header.stamp = self.get_clock().now().to_msg()
        static_transform_stamped.header.frame_id = 'world'
        static_transform_stamped.child_frame_id = f'{self.px4_model}_{self.frame_id}/{self.goal_frame}'

        static_transform_stamped.transform.translation.x = goal[0]
        static_transform_stamped.transform.translation.y = goal[1]
        static_transform_stamped.transform.translation.z = goal[2]
        self.leader_goal = [goal[0], goal[1], goal[2]]
        static_transform_stamped.transform.rotation.x = 0.0
        static_transform_stamped.transform.rotation.y = 0.0
        static_transform_stamped.transform.rotation.z = 0.0
        static_transform_stamped.transform.rotation.w = 1.0
        self.goal_tf_broadcaster.sendTransform(static_transform_stamped)

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
