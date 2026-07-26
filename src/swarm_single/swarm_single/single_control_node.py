#!/usr/bin/env python3

import rclpy
import tf2_ros
import math
import math
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

        self.velocity_goal = [0.0, 0.0, 0.0]#Vector3(0.0, 0.0, 0.0)
        self.manual_velocity = [0.0, 0.0, 0.0]
        self.leader_goal = [0.0, 0.0, 0.0]
        self.yaw = 1.57


        self.mission = []

        self.message = ""

        self.navigation = navigation(self)
        self.formation = PatternController(self)
        # self.lidar = lidarHandler(self)
        self.communication = Communication(self)

        #######################################################

        self.manual_control = False

        self.state = DroneState.IDLE
        self.drone_id = self.frame_id
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, f"/uav_{self.drone_id}/fmu/in/offboard_control_mode", qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, f"/uav_{self.drone_id}/fmu/in/trajectory_setpoint", qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, f"/uav_{self.drone_id}/fmu/in/vehicle_command", qos_profile)

        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, f"/uav_{self.drone_id}/fmu/out/vehicle_status", self.vehicle_status_callback, qos_profile)

        self.offboard_setpoint_counter = 0
        self.vehicle_status = VehicleStatus()

        self.timer = self.create_timer(0.05, self.control_loop_callback)

        self.mission_goal_tolerance = 0.4

        self.sended_goal_ack = True
        
        self.get_logger().info(f"SingleControlNode successfully initialized for Drone {self.drone_id}.")

    def control_loop_callback(self):
        """Main callback that runs at 10Hz, sends heartbeats, and executes state logic."""
        self.publish_offboard_control_heartbeat_signal()

        # if self.is_leader and len(self.mission)!= 0:
        #   if self.mission[0][0] != self.leader_goal[0] or self.mission[0][1] != self.leader_goal[1] or self.mission[0][2] != self.leader_goal[2] :
        #       self.goal_callback_temp(self.mission[0])

        #   distance = math.dist(self.mission[0], self.navigation.current_pos[:3])

        #   if distance < self.mission_goal_tolerance:
        #       self.mission.pop(0)
        #       self.communication.send_mission()
        # if self.is_leader and len(self.mission) == 0:
        #   if not self.sended_goal_ack:
        #       if self.navigation.current_pos[0] != 0.0 or self.navigation.current_pos[1] != 0.0 or self.navigation.current_pos[2] != 0.0:
        #           distance = math.dist(self.leader_goal, self.navigation.current_pos[:3])
        #           if distance < self.mission_goal_tolerance:
        #               self.sended_goal_ack = True
        #               self.message = f"Achieved goal {self.leader_goal[0]:.2f}, {self.leader_goal[1]:.2f}, {self.leader_goal[2]:.2f}"

        self.navigation.navigate_to_goal()

        if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            #if(int(self.drone_id) == 1):
            #   self.get_logger().info(f"time is {int(self.get_clock().now().nanoseconds / 1000000)}")
            if self.state == DroneState.IDLE and self.velocity_goal[2] < -0.1:
                self.get_logger().info("Takeoff command detected (Z velocity < 0). Starting arming sequence.")
                self.state = DroneState.ARMING
                self.offboard_setpoint_counter = 0 
        
            if self.state == DroneState.ARMING:
                if self.offboard_setpoint_counter >= 10:
                    self.arm()
                    self.state = DroneState.TAKEOFF
                    self.get_logger().info("Offboard setpoint counter reached. Transitioning to TAKEOFF state.")
                self.offboard_setpoint_counter += 1
                
            elif self.state == DroneState.TAKEOFF:
                if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                    self.publish_position_setpoint()
                else:
                    self.get_logger().warn("Lost Offboard state during TAKEOFF! Reverting to ARMING state.")
                    self.state = DroneState.ARMING

            elif self.state == DroneState.LANDING:
                if self.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
                    # If PX4 exits land mode for some reason, re-issue the command
                    self.get_logger().warn("PX4 dropped out of AUTO_LAND. Re-issuing LAND command.")
                    self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                if self.vehicle_status.arming_state != VehicleStatus.ARMING_STATE_ARMED:
                    self.get_logger().info("Landed and disarmed. Returning to IDLE.")
                    self.state = DroneState.IDLE
                    self.disarm()
                    self.offboard_setpoint_counter = 0 # Reset for next takeoff
        else:
            if self.state != DroneState.IDLE:
                self.get_logger().info("Pilot took control or Offboard lost. Resetting state to IDLE.")
                self.state = DroneState.IDLE
                # You might want to reset other variables here too
                self.velocity_goal = [0.0, 0.0, 0.0]

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
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status
        
    def arm(self):
        self.get_logger().info("Sending VEHICLE_CMD_DO_SET_MODE (Offboard) and ARM command.")
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

    def disarm(self):
        self.get_logger().info('Sending DISARM command.')
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)

    def publish_position_setpoint(self):
        msg = TrajectorySetpoint()
        msg.position = [math.nan, math.nan, math.nan] 
        # if self.manual_control:
        #   msg.velocity = [self.manual_velocity[0], self.manual_velocity[1], self.manual_velocity[2]]
        #   msg.yaw = self.yaw
        # else:
        msg.velocity = [self.velocity_goal[0],self.velocity_goal[1],self.velocity_goal[2]]#self.velocity_goal.xyz
        msg.yaw = self.yaw #self.velocity_goal[3] #1.57079  # (90 degrees)

        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_vehicle_command(self, command, **params):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = int(self.drone_id)+1
        msg.target_component = 1
        msg.source_system = 255
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