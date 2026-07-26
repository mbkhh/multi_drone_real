#!/usr/bin/env python3

import rclpy
import tf2_ros
import math
from typing import Dict
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from swarm_config.config_utils import get_config
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus

from swarm_single.navigation import navigation
from swarm_single.formation import PatternController
# from swarm_single.lidar_handler import lidarHandler
from swarm_single.communication import Communication

class DroneState:
    IDLE = "IDLE"
    ARMING = "ARMING"
    TAKEOFF = "TAKEOFF"
    LANDING = "LANDING"

class SingleControlNode(Node):

    def __init__(self):
        super().__init__('control_node')
        self.declare_parameter('frame_id', '1')
        
        self.frame_id = "1"
        self.px4_model = get_config('swarm_sim.px4_model')
        self.goal_frame = get_config('swarm_single.goal_frame_name')

        self.last_seen_neighbors: Dict[int, rclpy.time.Time] = {}
                    
        self.leader_id = None
        self.is_leader = False

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.velocity_goal = [0.0, 0.0, 0.0]
        self.manual_velocity = [0.0, 0.0, 0.0]
        self.leader_goal = [0.0, 0.0, 0.0]
        self.yaw = 1.57

        self.mission = []
        self.message = ""

        self.navigation = navigation(self)
        self.formation = PatternController(self)
        self.communication = Communication(self)

        self.manual_control = False
        self.state = DroneState.IDLE
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
            VehicleStatus, f"/uav_{self.drone_id}/fmu/out/vehicle_status", self.vehicle_status_callback, qos_profile)

        self.offboard_setpoint_counter = 0
        self.vehicle_status = VehicleStatus()

        # Timer runs at 20Hz
        self.timer = self.create_timer(0.05, self.control_loop_callback)

        self.mission_goal_tolerance = 0.4
        self.sended_goal_ack = True
        
        self.get_logger().info(f"SingleControlNode successfully initialized for Drone {self.drone_id}.")

    def control_loop_callback(self):
        """Main callback that runs at 20Hz, sends heartbeats, and executes state logic."""
        
        # Always publish heartbeats to keep Offboard alive
        self.publish_offboard_control_heartbeat_signal()
        self.navigation.navigate_to_goal()

        # Always publish position setpoints if we are trying to fly or arming
        if self.state != DroneState.IDLE:
            self.publish_position_setpoint()

        if self.state == DroneState.IDLE and self.velocity_goal[2] < -0.1:
            self.get_logger().info("Takeoff command detected. Starting arming sequence.")
            self.state = DroneState.ARMING
            self.offboard_setpoint_counter = 0 
    
        elif self.state == DroneState.ARMING:
            self.offboard_setpoint_counter += 1
            
            # Step 1: Let setpoints stream for 5 ticks (~0.25s), then request Offboard mode
            if self.offboard_setpoint_counter == 5:
                self.get_logger().info("Requesting Offboard mode...")
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
                
            # Step 2: Let PX4 process Offboard for 15 ticks (~0.75s), then Arm
            elif self.offboard_setpoint_counter == 20:
                self.arm()
                
            # Step 3: Give it a moment to confirm arming, then transition to TAKEOFF
            elif self.offboard_setpoint_counter >= 30:
                self.state = DroneState.TAKEOFF
                self.offboard_setpoint_counter = 0
                self.get_logger().info("Sequence complete. Transitioning to TAKEOFF state.")
            
        elif self.state == DroneState.TAKEOFF:
            # Check if pilot took over via physical RC
            if self.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.offboard_setpoint_counter += 1
                if self.offboard_setpoint_counter > 20: 
                    self.get_logger().info("Pilot took control or Offboard lost! Resetting to IDLE.")
                    self.state = DroneState.IDLE
                    self.velocity_goal = [0.0, 0.0, 0.0]
                    self.offboard_setpoint_counter = 0
            else:
                self.offboard_setpoint_counter = 0
                
        elif self.state == DroneState.LANDING:
            if self.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
                self.get_logger().info("PX4 dropped out of AUTO_LAND. Re-issuing LAND command.")
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            if self.vehicle_status.arming_state != VehicleStatus.ARMING_STATE_ARMED:
                self.get_logger().info("Landed and disarmed. Returning to IDLE.")
                self.state = DroneState.IDLE
                self.disarm()
                self.offboard_setpoint_counter = 0

    def publish_offboard_control_heartbeat_signal(self):
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def vehicle_status_callback(self, vehicle_status):
        self.vehicle_status = vehicle_status
        
    def arm(self):
        self.get_logger().info("Sending ARM command.")
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

    def disarm(self):
        self.get_logger().info('Sending DISARM command.')
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)

    def publish_position_setpoint(self):
        msg = TrajectorySetpoint()
        msg.position = [math.nan, math.nan, math.nan] 
        msg.velocity = [self.velocity_goal[0], self.velocity_goal[1], self.velocity_goal[2]]
        msg.yaw = self.yaw

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
        self.get_logger().info(f'Received goal request: [{goal[0]:.2f}, {goal[1]:.2f}, {goal[2]:.2f}]. Broadcasting static transform.')
        self.sended_goal_ack = False
        tf_static_broadcaster = StaticTransformBroadcaster(self)
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
        tf_static_broadcaster.sendTransform(static_transform_stamped)

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