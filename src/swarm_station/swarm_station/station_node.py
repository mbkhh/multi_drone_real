import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
import sys, os
import select 
import json
import readline
from rclpy.action import ActionClient

from swarm_msgs.msg import Status, FormationCommand
from swarm_msgs.action import Fly 
from swarm_config.config_utils import get_config
from swarm_config.config_utils import get_scenario
from swarm_station.manual_controller import ManualController

class StationNode(Node):
	"""
	A Ground Control Station node to send high-level commands to the drone swarm.
	This version uses a non-blocking input check inside a ROS timer.
	"""

	# --- Configuration ---
	LEADER_TIMEOUT_SEC = 5.0 # Seconds to wait before declaring leader lost
	
	def __init__(self):
		super().__init__('station_node')
		
		self.declare_parameter('scenario_name', 'none')

		# --- State Variables ---
		self.leader_is_connected = False
		self.last_status_time = None

		self.manual_controller = ManualController(self)

		self._action_client = ActionClient(self, Fly, '/fly_to')
		self.get_logger().info("Action client for '/fly_to' created.")

		# --- QoS Profile for Reliable Commands ---
		self.qos_profile_reliable = QoSProfile(
			reliability=ReliabilityPolicy.BEST_EFFORT,
			history=HistoryPolicy.KEEP_LAST,
			depth=int(int(get_config('swarm_sim.drone_count'))*2)
		)

		self.AckQueue = {}
		self.last_status = None

		self.status_subscriber = self.create_subscription(Status, "/swarm/status", self._swarm_status_callback, self.qos_profile_reliable)
		self.formation_publisher = self.create_publisher(FormationCommand, get_config('swarm_single.formation_command_topic_name'), 10 )
		self.command_publisher = self.create_publisher(String, get_config('swarm_single.leader_command_topic_name'), 10 )
		self.get_logger().info("Station Node initialized.")
		

		# Timer to check for leader timeout 
		self.leader_check_timer = self.create_timer(1.0, self._check_leader_connection) # Check every second
		
		# --- Timer for checking keyboard input ---
		# This timer runs at 10 Hz to check for user input without blocking.
		self.input_check_timer = self.create_timer(0.1, self.check_for_input)
		self.counter = 0
		self.scenario = []

		self.scenario_name = self.get_parameter('scenario_name').get_parameter_value().string_value

		if self.scenario_name != "none":
			self.start_scenario()
	def start_scenario(self):
		self.create_timer(1,self.run_scenario)
		self.counter = 0 

		self.scenario =  get_scenario(self.scenario_name)

	def run_scenario(self):
		if self.leader_is_connected:
			self.counter = self.counter + 1
			for i, command in self.scenario:
				if (i == self.counter):
					self.check_for_input(command)
					self.get_logger().info(f"Sending Command: {command}")

	"""Callback for receiving status messages from the swarm leader."""
	def _swarm_status_callback(self, msg: Status):
		if not self.leader_is_connected:
			self.get_logger().info("Connection to swarm leader established.")
		
		self.leader_is_connected = True
		self.last_status_time = self.get_clock().now()

		self.last_status = msg
		if msg.message != "":
			self.get_logger().info(f"Message from leader: {msg.message}")
		
	
	"""Periodically checks if the leader has timed out."""
	def _check_leader_connection(self):
		if self.leader_is_connected and self.last_status_time is not None:
			elapsed_time = (self.get_clock().now() - self.last_status_time).nanoseconds / 1e9
			if elapsed_time > self.LEADER_TIMEOUT_SEC:
				self.get_logger().info("Connection to swarm leader lost (timeout).")
				self.leader_is_connected = False
	def send_formation(self, pattern_name, spacing = 3.0, rotation_x = 0.0,rotation_y = 0.0,rotation_z = 0.0):
		msg = FormationCommand()
		msg.pattern_name = pattern_name

		if spacing is not None:
			msg.spacing = float(spacing)
		else:
			msg.spacing = 3.0
		if rotation_x is not None:
			msg.rotation_x = float(rotation_x)
		else:
			msg.rotation_x = 0.0
		if rotation_y is not None:
			msg.rotation_y = float(rotation_y)
		else:
			msg.rotation_y = 0.0
		if rotation_z is not None:
			msg.rotation_z = float(rotation_z)
		else:
			msg.rotation_z = 0.0
		
		self.formation_publisher.publish(msg)
	def send_fly_goal(self, x, y, z , absolute = True):
		self.get_logger().info("Waiting for action server...")
		self._action_client.wait_for_server(timeout_sec=5.0)

		goal_msg = Fly.Goal()
		goal_msg.goal.x = x
		goal_msg.goal.y = y
		goal_msg.goal.z = z
		goal_msg.absolute = absolute

		self.get_logger().info(f'Sending goal request: [{x}, {y}, {z}]')

		# Send the goal asynchronously and add callbacks to handle the response
		self._send_goal_future = self._action_client.send_goal_async(
			goal_msg,
			feedback_callback=self.feedback_callback)

		self._send_goal_future.add_done_callback(self.goal_response_callback)
	def send_fly_goal2(self, x, y, z , absolute = True):
		command = {
			"command": "fly",
			"x": x,
			"y": y,
			"z": z,
			"absolute": absolute
		}
		msg = String()	
		msg.data = json.dumps(command)
		self.command_publisher.publish(msg)

	def goal_response_callback(self, future):
		goal_handle = future.result()
		if not goal_handle.accepted:
			self.get_logger().info('Goal rejected by the leader :(')
			return

		self.get_logger().info('Goal accepted by the leader :)')

		# Now we wait for the action to be completed
		self._get_result_future = goal_handle.get_result_async()
		self._get_result_future.add_done_callback(self.get_result_callback)

	def get_result_callback(self, future):
		result = future.result().result
		self.get_logger().info(f'Result from leader: {result.result}')

	def feedback_callback(self, feedback_msg):
		# This is where you would update your progress bar in the future
		self.get_logger().info(f'Received feedback from leader: distance_remaining {feedback_msg.feedback.distance_remaining}')
	def send_arm_command(self):
		msg = String()
		msg.data =  json.dumps({"command": "arm"})
		self.command_publisher.publish(msg)
	def send_takeoff_command(self):
		if self.last_status is None:
			self.get_logger().error(
				"TAKEOFF not sent: no current leader status is available."
			)
			return
		if not (
			self.last_status.control_state == "TAKEOFF"
			and self.last_status.armed
			and self.last_status.offboard
		):
			self.get_logger().error(
				"TAKEOFF not sent: run 'arm' first and wait until status "
				"reports armed=True, offboard=True."
			)
			return
		expected_drones = int(get_config('swarm_sim.drone_count'))
		connected_drones = len(set(self.last_status.swarm_members))
		if connected_drones < expected_drones:
			self.get_logger().error(
				f"TAKEOFF not sent: {connected_drones}/{expected_drones} "
				"configured drones are connected."
			)
			return
		height = get_config('swarm_single.control.takeoff_height')
		height = 3.0 if height is None else float(height)
		msg = String()
		msg.data = json.dumps({"command": "takeoff", "height": height})
		self.command_publisher.publish(msg)
		
	def send_disarm_leader_command(self):
		msg = String()
		msg.data =  json.dumps({"command": "disarm_leader"})
		self.command_publisher.publish(msg)
	def send_land_command(self):
		msg = String()
		msg.data = json.dumps({"command": "land"})
		self.command_publisher.publish(msg)
	def stop_animation_command(self):
		msg = String()
		msg.data =  json.dumps({"command": "stop_animation"})
		self.command_publisher.publish(msg)
	def start_animation_command(self, speed_x, speed_y, speed_z):
		command = {
			"command": "start_animation",
			"speed_x": speed_x,
			"speed_y": speed_y,
			"speed_z": speed_z
		}
		msg = String()
		msg.data = json.dumps(command)
		self.command_publisher.publish(msg)
	
	def send_mission(self):
		if self.last_status is None:
			self.get_logger().error(
				"Mission not sent: no current leader status is available."
			)
			return
		if not (
			self.last_status.control_state == "TAKEOFF"
			and self.last_status.armed
			and self.last_status.offboard
		):
			self.get_logger().error(
				"Mission not sent: run 'arm' first and wait until status "
				"reports armed=True, offboard=True."
			)
			return

		points = get_config('swarm_single.mission.waypoints')
		relative_to_start = get_config(
			'swarm_single.mission.relative_to_start'
		)
		if not isinstance(points, list) or not points:
			self.get_logger().error(
				"Mission not sent: configure swarm_single.mission.waypoints."
			)
			return

		command = {
			"command": "mission",
			"points": points,
			"relative_to_start": (
				True if relative_to_start is None else bool(relative_to_start)
			),
		}
		msg = String()
		msg.data = json.dumps(command)
		self.command_publisher.publish(msg)
		self.get_logger().info(
			f"Mission sent with {len(points)} configured waypoints."
		)

	def check_for_input(self, input=None):
		"""
		Called by a timer to check for keyboard input without blocking.
		"""
		# select.select() checks if sys.stdin has data to be read.
		# The last argument '0' makes it non-blocking (it returns immediately).
		# We check if sys.stdin is in the list of readable streams.
		if input != None or sys.stdin in select.select([sys.stdin], [], [], 0)[0] :
			if input != None:
				line = input
			else:
				line = sys.stdin.readline()
			
			if line :
				command_input = line.strip()
				command = command_input.lower().split()
				if not self.leader_is_connected and command_input.upper() in ['START', 'RETURN','STATUS', ]:
					self.get_logger().error("Cannot send command: No swarm leader is connected.")

				elif command_input.upper() == 'STATUS':
					self.get_logger().info("The swarm leader is connected.")
					self.get_logger().info(f"Status: leader_id :{self.last_status.leader_id} , total members of swarm: {len(self.last_status.swarm_members)}")
					self.get_logger().info(f"Leader Position: X:{self.last_status.leader_x:.2f}, Y:{self.last_status.leader_y:.2f}, Z:{self.last_status.leader_z:.2f}")
					self.get_logger().info(f"Swarm Goal: X:{self.last_status.goal_x:.2f}, Y:{self.last_status.goal_y:.2f}, Z:{self.last_status.goal_z:.2f}")
					self.get_logger().info(f"Formation Status: Type:{self.last_status.pattern_name}, Spacing:{self.last_status.spacing}, Rotate_X:{self.last_status.rotation_x}, Rotate_Y:{self.last_status.rotation_y}, Rotate_Z:{self.last_status.rotation_z}")
					self.get_logger().info(
						f"Control: state={self.last_status.control_state}, "
						f"armed={self.last_status.armed}, "
						f"offboard={self.last_status.offboard}"
					)
					self.get_logger().info(
						f"Mission: state={self.last_status.mission_state}, "
						f"waypoint={self.last_status.mission_index}/"
						f"{self.last_status.mission_count}"
					)
					
				elif command_input.lower() == 'clear' or command_input.lower() == 'cls':
					os.system('clear')

				elif command_input.lower() == 'exit' or command_input.lower() == 'shutdown' or command_input.lower() == 'quit':
					print("Shutting down station node.")
					rclpy.shutdown()
				elif len(command) > 0:
					if not self.leader_is_connected:
						self.get_logger().error("Cannot send goal: No swarm leader is connected.")
					else:
						match command[0].lower():
							case 'arm':
								self.send_arm_command()
							case 'takeoff':
								self.send_takeoff_command()
							case 'disarm_leader':
								self.send_disarm_leader_command()
							case 'land':
								self.send_land_command()
							case 'stop_animation':
								self.stop_animation_command()
							case 'mission':
								self.send_mission()
							case 'start_animation':
								params = {
									"speed_x": 0.0,
									"speed_y": 0.0,
									"speed_z": 0.0
								}
								for arg in command[1:]:
									if '=' in arg:
										key, value = arg.split('=', 1)
										if key in params:
											try:
												params[key] = float(value)
											except ValueError:
												self.get_logger().error(f"Invalid value for {key}: must be a number")
												break
										else:
											self.get_logger().info(f"Ignoring unknown parameter: {key}")
								else:
									self.start_animation_command(params["speed_x"], params["speed_y"], params["speed_z"])
							case 'set_goal':
								if self.manual_controller.manual_mode:
									self.get_logger().error("Manual mode is activated can't set goal.")
								elif len(command) != 4:
									self.get_logger().error("Invalid command. Usage: set_goal <x> <y> <z>")
								else:
									try:
										x = float(command[1])
										y = float(command[2])
										z = float(command[3])
										self.send_fly_goal2(x, y, z, True)
									except ValueError:
										self.get_logger().error("Invalid coordinates. X, Y, and Z must be numbers.")
							case 'move':
								params = {
									"x": 0.0,
									"y": 0.0,
									"z": 0.0
								}
								for arg in command[1:]:
									if '=' in arg:
										key, value = arg.split('=', 1)
										if key in params:
											try:
												params[key] = float(value)
											except ValueError:
												self.get_logger().error(f"Invalid value for {key}: must be a number")
												break
										else:
											self.get_logger().info(f"Ignoring unknown parameter: {key}")
								else:
									self.send_fly_goal2(params["x"], params["y"], params["z"], False)
							case 'manual':
								if len(command) > 1 and command[1] == 'off':
									self.manual_controller.manual_mode = False
									self.manual_controller.publish_cmd(True)
								else:
									self.get_logger().info("Entering manual control mode. Press 'q' to exit.")
									self.manual_controller.manual_mode = True
							case 'set_formation':
								if len(command) < 2:
									self.get_logger().error("Invalid command. Usage: set_formation <pattern_name> [spacing=<value>] [rotation_x=<value>] [rotation_y=<value>] [rotation_z=<value>]")
								else:
									pattern_name = command[1]
									params = {
										"spacing": None,
										"rotation_x": None,
										"rotation_y": None,
										"rotation_z": None
									}
									for arg in command[2:]:
										if '=' in arg:
											key, value = arg.split('=', 1)
											if key in params:
												try:
													params[key] = float(value)
												except ValueError:
													self.get_logger().error(f"Invalid value for {key}: must be a number")
													break
											else:
												self.get_logger().info(f"Ignoring unknown parameter: {key}")
									else:
										# Only if we didn't break due to error
										self.send_formation(
											pattern_name,
											spacing=params["spacing"],
											rotation_x=params["rotation_x"],
											rotation_y=params["rotation_y"],
											rotation_z=params["rotation_z"]
										)
							case 'lidar':
								pass			
							
				
				# Reprint the input prompt after processing a command
				print("station>  ", end="")
				sys.stdout.flush()


def main(args=None):
	rclpy.init(args=args)
	station_node = StationNode()
	
	print("Station node spinning.")
	
	try:
		rclpy.spin(station_node)
	except KeyboardInterrupt:
		station_node.get_logger().info("Station node stopped via KeyboardInterrupt.")
	finally:
		station_node.destroy_node()
		if rclpy.ok():
			rclpy.shutdown()

if __name__ == '__main__':
	main()
