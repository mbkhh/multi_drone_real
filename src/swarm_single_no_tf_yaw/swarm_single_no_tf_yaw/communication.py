import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from std_msgs.msg import String
from swarm_msgs.msg import Status, ManualControl, FormationCommand
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
)
from swarm_msgs.action import Fly
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from px4_msgs.msg import VehicleStatus
from swarm_config.config_utils import get_config
import math
import json
import numpy as np
import asyncio
from rclpy.callback_groups import ReentrantCallbackGroup

class Communication():
    def __init__(self, parent_node: Node):
        self.parent_node = parent_node
        
        self.formation_topic_name = get_config('swarm_single.formation_topic_name')
        self.command_topic_name = get_config('swarm_single.command_topic_name')
        self.liveness_topic_name = get_config('swarm_single.liveness_topic_name')
        self.manual_control_topic_name = get_config('swarm_single.manual_control_topic_name')
        self.leader_formation_topic_name = get_config('swarm_single.formation_command_topic_name')
        self.leader_command_topic_name = get_config('swarm_single.leader_command_topic_name')
        
        self.broadcast_interval = get_config('swarm_single.broadcast_interval')
        self.status_interval = get_config('swarm_single.status_interval')
        self.neighbor_timeout = self.broadcast_interval * 4


        self.GOAL_TOLERANCE = get_config('swarm_single.goal_tolerance')

        self.liveness_topic = f"/swarm/{self.liveness_topic_name}"
        self.liveness_publisher = self.parent_node.create_publisher(Header, self.liveness_topic, 10)
        self.liveness_subscriber = self.parent_node.create_subscription(Header, self.liveness_topic, self._neighbor_liveness_callback, 10)
        self.liveness_broadcast_timer = self.parent_node.create_timer(self.broadcast_interval, self._broadcast_liveness)
        self.timeout_timer = self.parent_node.create_timer(self.neighbor_timeout, self._check_neighbor_timeouts)
        
        self.qos_profile_reliable = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=int(int(get_config('swarm_sim.drone_count'))*2)
        )

        self.formation_subscriber = self.parent_node.create_subscription(FormationCommand, self.formation_topic_name, self.formation_command_callback, 10)
        self.formation_publisher = None


        self.command_subscriber = self.parent_node.create_subscription(String, self.command_topic_name, self.command_callback, 10)
        self.command_publisher = None

        self.status_publisher = None
        self.leader_formation_subscriber = None
        self.leader_command_subscriber = None
        self.manual_control_subscriber = None
        self.status_timer = None

        # --- Action Server State --- ### NEW ###
        self._action_goal_handle = None
        self._action_target_pos = None
        self._goal_check_timer = None
        
        self.referendum_conducting()
        self.parent_node.get_logger().info("Communication module initialized. Listening for swarm and station heartbeat...")

    def manual_control_handler(self, msg: ManualControl):
        """Handles manual control commands from the user."""
        #self.parent_node.get_logger().info(f"Manual Control: {msg.vx}, {msg.vy}, {msg.vz}, {msg.vyaw}")
        if msg.manual_mode:
            if self.parent_node.mission_active:
                self.parent_node.abort_mission('manual control requested')
            self.parent_node.manual_control = True
            if self.parent_node.state == "TAKEOFF":
                self.parent_node.motion_enabled = True
            yaw = self.parent_node.yaw + msg.vyaw/30.0
            if yaw > math.pi:
                yaw -= 2 * math.pi
            elif yaw < -math.pi:
                yaw += 2 * math.pi
            self.parent_node.manual_velocity = [msg.vy, msg.vx, msg.vz]
            self.parent_node.yaw = yaw
        else:
            self.parent_node.manual_control = False
            self.parent_node.manual_velocity = [0.0, 0.0, 0.0]
            self.parent_node.get_logger().info(f"Turning manual mode off")
            
    def _broadcast_liveness(self):
        msg = Header()
        msg.stamp = self.parent_node.get_clock().now().to_msg()
        msg.frame_id = self.parent_node.frame_id
        self.liveness_publisher.publish(msg)

    def _broadcast_status(self):
        # self.parent_node.get_logger().info("publishing status")

        msg = Status()
        msg.timestamp = int(self.parent_node.get_clock().now().nanoseconds / 1000)
        msg.swarm_members = list(self.parent_node.last_seen_neighbors.keys()) + [int(self.parent_node.frame_id)]
        msg.leader_id       = self.parent_node.leader_id
        msg.pattern_name    = str(self.parent_node.formation.pattern_name)
        msg.spacing         = float(self.parent_node.formation.spacing)
        msg.rotation_x      = float(self.parent_node.formation.rotation_x)
        msg.rotation_y      = float(self.parent_node.formation.rotation_y)
        msg.rotation_z      = float(self.parent_node.formation.rotation_z)
        msg.leader_x        = float(self.parent_node.navigation.current_pos[0])
        msg.leader_y        = float(self.parent_node.navigation.current_pos[1])
        msg.leader_z        = float(self.parent_node.navigation.current_pos[2])
        # Goals are intentionally process-local in this package. The legacy
        # Status goal fields remain at their message defaults for wire/schema
        # compatibility, but no active goal is shared with other drones.
        msg.message         = self.parent_node.message
        msg.control_state   = self.parent_node.state
        msg.armed           = (
            self.parent_node.vehicle_status.arming_state
            == VehicleStatus.ARMING_STATE_ARMED
        )
        msg.offboard        = (
            self.parent_node.vehicle_status.nav_state
            == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        )
        msg.mission_active  = self.parent_node.mission_active
        msg.mission_count   = len(self.parent_node.mission)
        if self.parent_node.mission_active:
            msg.mission_index = self.parent_node.mission_index + 1
        else:
            msg.mission_index = min(
                self.parent_node.mission_index,
                len(self.parent_node.mission),
            )
        msg.mission_state   = self.parent_node.mission_state
        self.parent_node.message = "" 
        self.status_publisher.publish(msg)

    def _neighbor_liveness_callback(self, msg: Header):
        count = len(list(self.parent_node.last_seen_neighbors.keys()))
        if not msg.frame_id==self.parent_node.frame_id: 
            self.parent_node.last_seen_neighbors[int(msg.frame_id)] = self.parent_node.get_clock().now()
        if not count==len(list(self.parent_node.last_seen_neighbors.keys())): 
            self.parent_node.get_logger().info(f"Connection established with neighbor {msg.frame_id}! Swarm peers count: {len(self.parent_node.last_seen_neighbors)}")
            self.referendum_conducting()

    def _check_neighbor_timeouts(self):
        now = self.parent_node.get_clock().now()
        timed_out_neighbors = set()
        for neighbor_id, last_seen_time in self.parent_node.last_seen_neighbors.items():
            if (now - last_seen_time).nanoseconds / 1e9 > self.neighbor_timeout:
                timed_out_neighbors.add(neighbor_id)
        
        for neighbor_id in timed_out_neighbors:
            self.parent_node.get_logger().info(f"Connection lost with neighbor {neighbor_id}! Removing from swarm topology.")
            del self.parent_node.last_seen_neighbors[neighbor_id]
            self.parent_node.peer_states.pop(neighbor_id, None)
        
        if bool(timed_out_neighbors): self.referendum_conducting()

    def referendum_conducting(self):
        swarm_members = list(self.parent_node.last_seen_neighbors.keys()) + [int(self.parent_node.frame_id)]
        leader_id = min(swarm_members)
        if leader_id != self.parent_node.leader_id:
            self.parent_node.leader_id = leader_id

            self.parent_node.get_logger().info(f"Topology update: This is drone {self.parent_node.frame_id} and leader is {leader_id}")

            if leader_id == int(self.parent_node.frame_id) and not self.parent_node.is_leader:
                self.become_leader()
            elif leader_id != int(self.parent_node.frame_id) and self.parent_node.is_leader:
                self.step_down_as_leader()
        self.parent_node.formation.refresh_pattern()

    def become_leader(self, ):
        self.parent_node.get_logger().info('Transitioning to LEADER role. Establishing Station connection...')
        self.parent_node.goal_callback_temp( [self.parent_node.navigation.current_pos[0], self.parent_node.navigation.current_pos[1], self.parent_node.navigation.current_pos[2]])
        self.parent_node.is_leader = True
        self.send_formation_command('square', 4)
        self.status_publisher = self.parent_node.create_publisher(Status, "/swarm/status",self.qos_profile_reliable)
        self.manual_control_subscriber = self.parent_node.create_subscription(ManualControl, self.manual_control_topic_name, self.manual_control_handler, 10)
        self.status_timer = self.parent_node.create_timer(self.status_interval, self._broadcast_status)
        self._fly_action = ActionServer(self.parent_node, Fly, '/fly_to', execute_callback=self.execute_callback,callback_group=ReentrantCallbackGroup(),goal_callback=self.goal_callback,cancel_callback=self.cancel_callback)

        self.formation_publisher = self.parent_node.create_publisher(FormationCommand, self.formation_topic_name, 10)
        self.command_publisher = self.parent_node.create_publisher(String, self.command_topic_name, 10)
        if (self.formation_subscriber is not None):
            self.parent_node.destroy_subscription(self.formation_subscriber)
            self.parent_node.destroy_subscription(self.command_subscriber)

        self.formation_subscriber = None
        self.command_subscriber = None

        self.leader_formation_subscriber = self.parent_node.create_subscription(FormationCommand, self.leader_formation_topic_name, self.formation_leader_callback, 10)
        self.leader_command_subscriber = self.parent_node.create_subscription(String, self.leader_command_topic_name, self.command_leader_callback, 10)
    
    def step_down_as_leader(self, ):
        self.parent_node.get_logger().info('Stepping down as leader. Reverting to FOLLOWER role.')
        self.parent_node.is_leader = False
        self.parent_node.destroy_publisher(self.status_publisher)
        self.parent_node.destroy_subscription(self.manual_control_subscriber)
        self.parent_node.destroy_subscription(self.leader_formation_subscriber)
        self.parent_node.destroy_subscription(self.leader_command_subscriber)
        self.status_timer.cancel()
        self.parent_node.destroy_publisher(self.formation_publisher)
        self.parent_node.destroy_publisher(self.command_publisher)
        self.formation_subscriber = self.parent_node.create_subscription(FormationCommand, self.formation_topic_name, self.formation_command_callback, 10)
        self.command_subscriber = self.parent_node.create_subscription(String, self.command_topic_name, self.command_callback, 10)
        self._fly_action.destroy()
        self._fly_action = None
        self.formation_publisher = None
        self.command_publisher = None
        self.leader_formation_subscriber = None
        self.leader_command_subscriber = None
        self.status_publisher = None
        self.manual_control_subscriber = None
        self.status_timer = None

    def command_callback(self, msg: String):
        self.parent_node.get_logger().info(f"Follower received command payload: {msg.data}")
        try:
            cmd = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.parent_node.get_logger().error(f"JSON Decode Error: {e}")
            return
        command_type = cmd.get("command")
        if command_type == "arm":
            if self.parent_node.request_offboard_control():
                self.parent_node.get_logger().info("Received ARM command.")

        elif command_type == "takeoff":
            self.execute_takeoff(cmd.get("height", 3.0))

        elif command_type == "land":
            self.parent_node.request_land()

        elif command_type == "start_animation":
            start_time = int(cmd.get("start_time"))

            speed_x = float(cmd.get("speed_x"))
            speed_y = float(cmd.get("speed_y"))
            speed_z = float(cmd.get("speed_z"))

            self.parent_node.formation.start_animation(start_time=start_time, speed_x= speed_x, speed_y= speed_y, speed_z= speed_z)
        elif command_type == "stop_animation":
            self.parent_node.formation.stop_animation()
        elif command_type == 'mission':
            # Followers do not execute the leader's coordinates. They enable
            # their existing formation goal and follow the leader instead.
            if self.parent_node.state == "TAKEOFF":
                self.parent_node.motion_enabled = True
                self.parent_node.get_logger().info(
                    'Follower mission tracking enabled.'
                )
            else:
                self.parent_node.get_logger().warning(
                    'Follower ignored mission start because Offboard is not active.'
                )
        elif command_type == "leader_is_dead":
            del self.parent_node.last_seen_neighbors[self.parent_node.leader_id]
            self.referendum_conducting()


    def command_leader_callback(self, msg: String):
        self.parent_node.get_logger().info(f"Leader received Ground Station command: {msg.data}")
        try:
            cmd = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.parent_node.get_logger().error(f"JSON Decode Error: {e}")
            return
        command_type = cmd.get("command")
        if command_type == "arm":
            if self.parent_node.request_offboard_control():
                out_msg = String()
                out_msg.data = json.dumps({"command": "arm"})
                self.command_publisher.publish(out_msg)
                self.parent_node.get_logger().info(
                    "Leader: publishing ARM command."
                )
        elif command_type == "takeoff":
            height = cmd.get("height", 3.0)
            if self.execute_takeoff(height):
                out_msg = String()
                out_msg.data = json.dumps({
                    "command": "takeoff",
                    "height": float(height),
                })
                self.command_publisher.publish(out_msg)
                self.parent_node.get_logger().info(
                    f"Leader: publishing {float(height):.2f} m TAKEOFF command."
                )
        elif command_type == "yaw":
            delta_degrees = cmd.get("delta_degrees")
            if self.parent_node.request_relative_yaw(delta_degrees):
                self.parent_node.get_logger().info(
                    f"Leader: accepted relative yaw move of "
                    f"{float(delta_degrees):+.1f} degrees."
                )
        elif command_type == "fly":
            x = float(cmd.get("x"))
            y = float(cmd.get("y"))
            z = float(cmd.get("z"))
            absolute = cmd.get("absolute", True)

            if absolute:
                goal_accepted = self.parent_node.goal_callback_temp([x, y, z])
            else:
                goal_accepted = self.parent_node.goal_callback_temp([self.parent_node.leader_goal[0] + x, self.parent_node.leader_goal[1] + y, self.parent_node.leader_goal[2] + z])
            if goal_accepted and self.parent_node.state == "TAKEOFF":
                if self.parent_node.mission_active:
                    self.parent_node.abort_mission(
                        'replaced by a station move/set_goal command'
                    )
                self.parent_node.motion_enabled = True
            elif goal_accepted:
                self.parent_node.get_logger().warning(
                    "Goal stored but motion is disabled: explicitly ARM Offboard first."
                )
                
        elif command_type == "disarm_leader":
            self.parent_node.request_safe_disarm()
        elif command_type == "land":
            if self.parent_node.request_land():
                out_msg = String()
                out_msg.data = json.dumps({"command": "land"})
                self.command_publisher.publish(out_msg)
                self.parent_node.get_logger().info(
                    "Leader: publishing LAND command."
                )
        elif command_type == "start_animation":
            start_time = int(self.parent_node.get_clock().now().nanoseconds / 1000000)

            speed_x = float(cmd.get("speed_x"))
            speed_y = float(cmd.get("speed_y"))
            speed_z = float(cmd.get("speed_z"))

            #self.parent_node.formation.start_animation()
            command = {
                "command": "start_animation",
                "start_time": start_time,
                "speed_x": speed_x,
                "speed_y": speed_y,
                "speed_z": speed_z
            }
            msg = String()
            msg.data = json.dumps(command)
            self.command_publisher.publish(msg)

            self.parent_node.get_logger().info(f"Leader: Publishing animation command.")
        elif command_type == "stop_animation":
            out_msg = String()
            out_msg.data = json.dumps({"command": "stop_animation"})
            self.command_publisher.publish(out_msg)
            self.parent_node.get_logger().info("Leader: Publishing stop animation command.")
        elif command_type == 'mission':
            mission = cmd.get("points")
            relative_to_start = cmd.get("relative_to_start", True)
            if self.parent_node.start_mission(
                mission,
                relative_to_start=relative_to_start,
            ):
                self.parent_node.get_logger().info(
                    f"Leader accepted mission with {len(mission)} waypoints."
                )
                self.send_mission()

    def execute_takeoff(self, height):
        try:
            height = float(height)
        except (TypeError, ValueError):
            self.parent_node.get_logger().error(
                'TAKEOFF rejected: height must be numeric.'
            )
            return False
        if not math.isfinite(height) or height <= 0.0:
            self.parent_node.get_logger().error(
                'TAKEOFF rejected: height must be finite and greater than zero.'
            )
            return False
        if (
            self.parent_node.state != "TAKEOFF"
            or self.parent_node.vehicle_status.arming_state
            != VehicleStatus.ARMING_STATE_ARMED
            or self.parent_node.vehicle_status.nav_state
            != VehicleStatus.NAVIGATION_STATE_OFFBOARD
        ):
            self.parent_node.get_logger().warning(
                'TAKEOFF rejected: this drone is not armed in Offboard mode.'
            )
            return False
        if self.parent_node.manual_control:
            self.parent_node.get_logger().warning(
                'TAKEOFF rejected: manual control is active.'
            )
            return False

        current = [
            float(value)
            for value in self.parent_node.navigation.current_pos[:3]
        ]
        if not all(math.isfinite(value) for value in current):
            self.parent_node.get_logger().error(
                'TAKEOFF rejected: current position is not finite.'
            )
            return False
        target = [current[0], current[1], current[2] + height]
        if not self.parent_node.goal_callback_temp(target):
            return False
        if self.parent_node.mission_active:
            self.parent_node.abort_mission('replaced by TAKEOFF command')
        self.parent_node.motion_enabled = True
        self.parent_node.get_logger().warning(
            f'TAKEOFF accepted: climbing {height:.2f} m to ENU '
            f'z={target[2]:.2f}.'
        )
        return True

    def send_mission(self):
        command = {
            "command": "mission",
            "points": self.parent_node.mission,
            "relative_to_start": False,
        }
        msg = String()
        msg.data = json.dumps(command)
        self.command_publisher.publish(msg)

    def formation_leader_callback(self, msg: FormationCommand):
        self.parent_node.formation.set_pattern(
            pattern_name=msg.pattern_name,
            spacing=msg.spacing,
            rotation_x=msg.rotation_x,
            rotation_y=msg.rotation_y,
            rotation_z=msg.rotation_z
        )
        self.send_formation_command(msg.pattern_name, msg.spacing, rotation_x=msg.rotation_x, rotation_y=msg.rotation_y, rotation_z=msg.rotation_z)
    
    def send_formation_command(self, name, spacing, rotation_x=0.0, rotation_y=0.0, rotation_z=0.0, line_type="X"):
        """Populates and publishes a FormationCommand message."""
        if self.formation_publisher:
            msg = FormationCommand()
            msg.pattern_name = name
            msg.spacing = float(spacing)
            msg.rotation_x = float(rotation_x)
            msg.rotation_y = float(rotation_y)
            msg.rotation_z = float(rotation_z)
            
            self.formation_publisher.publish(msg)
            self.parent_node.get_logger().info(f"Published formation command: '{name}' with spacing {spacing}")

    def formation_command_callback(self, msg: ManualControl):
        self.parent_node.get_logger().info(
            f"Received new formation command: '{msg.pattern_name}' with spacing {msg.spacing}"
        )
        self.parent_node.formation.set_pattern(
            pattern_name=msg.pattern_name,
            spacing=msg.spacing,
            rotation_x=msg.rotation_x,
            rotation_y=msg.rotation_y,
            rotation_z=msg.rotation_z
        )

    def goal_callback(self, goal_request):
        """Accept or reject a client request to start an action."""
        self.parent_node.get_logger().info(f"Received goal request: {goal_request.goal}")
        if self.parent_node.state != "TAKEOFF":
            self.parent_node.get_logger().warning(
                'Fly action rejected: Offboard is not active.'
            )
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT
    def cancel_callback(self, goal_handle):
        self.parent_node.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT
    
    async def execute_callback(self, goal_handle):
        goal = goal_handle.request.goal
        absolute = goal_handle.request.absolute
        self.parent_node.get_logger().info(f"Executing fly_to goal: [{goal.x}, {goal.y}, {goal.z}]")


        # Send goal to drone
        if absolute:
            goal_accepted = self.parent_node.goal_callback_temp(
                [goal.x, goal.y, goal.z]
            )
        else:
            goal_accepted = self.parent_node.goal_callback_temp(
                [
                    self.parent_node.leader_goal[0] + goal.x,
                    self.parent_node.leader_goal[1] + goal.y,
                    self.parent_node.leader_goal[2] + goal.z,
                ]
            )
        if not goal_accepted:
            goal_handle.abort()
            result_msg = Fly.Result()
            result_msg.result = 'Rejected by safety limits'
            return result_msg
        if self.parent_node.state == "TAKEOFF":
            self.parent_node.motion_enabled = True
        

        # Create feedback message
        feedback_msg = Fly.Feedback()
        result_msg = Fly.Result()

        # Feedback loop
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.parent_node.get_logger().info('Goal canceled by client')
                goal_handle.canceled()
                result_msg.result = 'Cancelled'
                return result_msg

            # Get current position
            current_pos_vec = np.array([self.parent_node.navigation.current_pos[0], self.parent_node.navigation.current_pos[1], self.parent_node.navigation.current_pos[2]])
        
            distance = np.linalg.norm([goal.x, goal.y, goal.z]- current_pos_vec)

            feedback_msg.distance_remaining = float(distance)
            goal_handle.publish_feedback(feedback_msg)

            self.parent_node.get_logger().info(f'Distance remaining: {distance:.2f}')

            if distance < self.GOAL_TOLERANCE:  # Reached the goal
                self.parent_node.get_logger().info("Goal reached!")
                break

            await asyncio.sleep(1.0)  # Non-blocking sleep
            #time.sleep(1)
        goal_handle.succeed()
        result_msg.result = 'Reached'
        return result_msg
