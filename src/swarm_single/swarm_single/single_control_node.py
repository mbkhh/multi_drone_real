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
        self.message = ""
        self.manual_control = False
        self.motion_enabled = False
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
        self.min_goal_altitude = self.config_float(
            'swarm_single.control.min_goal_altitude', -0.5
        )
        self.max_goal_altitude = self.config_float(
            'swarm_single.control.max_goal_altitude', 5.0
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

        self.offboard_setpoint_counter = 0
        self.vehicle_status = VehicleStatus()

        # Timer runs at 20Hz
        self.timer = self.create_timer(0.05, self.control_loop_callback)

        self.mission_goal_tolerance = 0.4
        self.sended_goal_ack = True
        
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
        if not self.vehicle_status.pre_flight_checks_pass:
            self.get_logger().error(
                'Offboard rejected: PX4 preflight checks have not passed.'
            )
            return
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
        if not self.goal_callback_temp(self.navigation.current_pos[:3]):
            self.get_logger().error(
                'Offboard rejected: current pose is outside the configured '
                'goal safety envelope.'
            )
            return
        self.motion_enabled = False
        self.manual_control = False
        self.velocity_goal = [0.0, 0.0, 0.0]
        self.offboard_setpoint_counter = 0
        self.offboard_was_confirmed = False
        self.state = DroneState.ARMING
        self.get_logger().warning(
            'Explicit OFFBOARD request received: preparing with zero velocity; '
            'PX4 will be armed only if currently disarmed.'
        )

    def release_to_pilot(self, reason):
        """Latch this node out and cease heartbeat/setpoint publication."""
        self.state = DroneState.PILOT_CONTROL
        self.motion_enabled = False
        self.manual_control = False
        self.manual_velocity = [0.0, 0.0, 0.0]
        self.velocity_goal = [0.0, 0.0, 0.0]
        self.offboard_setpoint_counter = 0
        self.offboard_was_confirmed = False
        self.get_logger().warning(
            f'{reason}. ROS control released; PX4/RC/failsafe owns control. '
            'A new station ARM command is required to re-enter Offboard.'
        )

    def request_land(self):
        """Ask PX4 to enter its native AUTO_LAND mode once."""
        if self.vehicle_status.arming_state != VehicleStatus.ARMING_STATE_ARMED:
            self.get_logger().warning('LAND ignored: vehicle is not armed.')
            return
        self.motion_enabled = False
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

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'world'
        transform.child_frame_id = f'{self.px4_model}_{self.frame_id}/odom'

        # PX4 local position is NED. The navigation code and TF tree use ENU.
        transform.transform.translation.x = float(local_position.y)
        transform.transform.translation.y = float(local_position.x)
        transform.transform.translation.z = float(-local_position.z)

        yaw_enu = (math.pi / 2.0) - float(local_position.heading)
        transform.transform.rotation.z = math.sin(yaw_enu / 2.0)
        transform.transform.rotation.w = math.cos(yaw_enu / 2.0)
        self.local_position_tf_broadcaster.sendTransform(transform)

        self.navigation.vel = [
            float(local_position.vx),
            float(local_position.vy),
            float(local_position.vz),
        ]
        self.navigation.current_pos = [
            float(local_position.y),
            float(local_position.x),
            float(-local_position.z),
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
        msg.target_system = 1
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
        if not self.min_goal_altitude <= goal[2] <= self.max_goal_altitude:
            self.get_logger().error(
                f'Goal rejected: altitude {goal[2]:.2f} m is outside '
                f'[{self.min_goal_altitude:.2f}, '
                f'{self.max_goal_altitude:.2f}] m.'
            )
            return False

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
        return True

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
