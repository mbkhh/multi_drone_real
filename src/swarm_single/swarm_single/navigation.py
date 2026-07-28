import rclpy
import rvo23d
import numpy as np
from rclpy.node import Node
from tf2_ros import TransformException
from swarm_config.config_utils import get_config
from geometry_msgs.msg import PointStamped
import tf2_geometry_msgs

class navigation():
	def __init__(self, parent_node: Node):
		self.parent_node = parent_node

		self.vel = [0.0, 0.0, 0.0]
		self.current_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

	def navigate_to_goal(self):

		maxSpeed = get_config('swarm_single.navigation.max_speed')
		good_dist_to_goal = get_config('swarm_single.navigation.good_dist_to_goal')
		sim = rvo23d.PyRVOSimulator( get_config('swarm_single.navigation.time_step'), get_config('swarm_single.navigation.neighbor_dist'), get_config('swarm_single.navigation.max_neighbors'), get_config('swarm_single.navigation.time_horizon'),get_config('swarm_single.navigation.radius') ,maxSpeed )
		
		agents = []
		goals = []
		drone_loc = self.get_drone_pos(self.parent_node.frame_id)
		
		if not drone_loc  is None:
			self.current_pos = [drone_loc.translation.x, drone_loc.translation.y, drone_loc.translation.z, drone_loc.rotation.x, drone_loc.rotation.y, drone_loc.rotation.z, drone_loc.rotation.w]

		if self.parent_node.is_leader and self.parent_node.manual_control:
			goal_loc = None
		else:
			goal_loc = self.get_drone_absolute_goal(self.parent_node.frame_id)
		if drone_loc == None :
			return 
		else:
			if self.parent_node.is_leader:
				if not self.parent_node.manual_control and goal_loc == None:
					return
			else:
				if goal_loc == None:
					return

		moving_agent = sim.addAgent((drone_loc.translation.x, drone_loc.translation.y, drone_loc.translation.z))
		agents.append(moving_agent)
		if self.parent_node.is_leader and self.parent_node.manual_control:
			goals.append(np.array([0.0,0.0,0.0]))
		else:
			goals.append(np.array([goal_loc.translation.x, goal_loc.translation.y, goal_loc.translation.z]))
		if not self.parent_node.is_leader:
			for i in self.parent_node.last_seen_neighbors.keys():
				neighbor_pos = self.get_drone_pos(i)

				if neighbor_pos == None:
					agent = sim.addAgent((0.0, 0.0, 1000.0))  
					agents.append(agent)
					goals.append(np.array([0.0,0.0,1000.0]))
				else:
					agent = sim.addAgent((neighbor_pos.translation.x, neighbor_pos.translation.y, neighbor_pos.translation.z))
					agents.append(agent)
					goals.append(np.array([neighbor_pos.translation.x, neighbor_pos.translation.y, neighbor_pos.translation.z]))

		obstacles = self.get_obstacle_absolute()
		for i, center_point in enumerate(obstacles):
			agent = sim.addAgent((center_point[0], center_point[1], center_point[2]))
			agents.append(agent)
			goals.append(np.array([center_point[0], center_point[1], center_point[2]]))
					
		for i in range(len(agents)):
			if self.parent_node.is_leader and self.parent_node.manual_control and i == 0:
				sim.setAgentPrefVelocity(agents[i], (self.parent_node.manual_velocity[0],self.parent_node.manual_velocity[1],self.parent_node.manual_velocity[2]))
			else:
				current_pos = np.array(sim.getAgentPosition(agents[i]))
				goal_vector = goals[i] - current_pos
				dist_to_goal = np.linalg.norm(goal_vector)
				
				if dist_to_goal > good_dist_to_goal:
					
					MIN_LEADER_SPEED = 0.1  # m/s
					MAX_LEADER_SPEED = 2.0  # m/s
					MAX_COEF = 0.8          # Coef when leader is slow
					MIN_COEF = 0.2 

					leader_speed = np.linalg.norm(self.vel)
            
					# 2. Calculate the ratio of how fast the leader is going within your defined range
					# Clamp the speed to ensure the ratio stays between 0.0 and 1.0
					clamped_speed = np.clip(leader_speed, MIN_LEADER_SPEED, MAX_LEADER_SPEED)
					speed_ratio = (clamped_speed - MIN_LEADER_SPEED) / (MAX_LEADER_SPEED - MIN_LEADER_SPEED)
					
					# 3. Map this ratio to your coefficient range
					# When speed_ratio is 0 (slow), coef is MAX_COEF.
					# When speed_ratio is 1 (fast), coef is MIN_COEF.
					mixture_coef = MAX_COEF - speed_ratio * (MAX_COEF - MIN_COEF)

					gain = 1.0
					desired_speed = min(maxSpeed, dist_to_goal * gain) 
					
					direction = goal_vector / dist_to_goal
					pref_velocity = desired_speed * direction 

					if not self.parent_node.is_leader or (self.parent_node.is_leader and i != 0):
						pref_velocity[0] = pref_velocity[0] * mixture_coef + self.vel[1] * (1-mixture_coef)
						pref_velocity[1] = pref_velocity[1] * mixture_coef +	 self.vel[0] * (1-mixture_coef)	
						pref_velocity[2] = pref_velocity[2] * mixture_coef - self.vel[2] * (1-mixture_coef)	

					sim.setAgentPrefVelocity(agents[i], tuple(pref_velocity))
				else:
					sim.setAgentPrefVelocity(agents[i], (0, 0, 0))
		sim.doStep()
		velocity = sim.getAgentVelocity(moving_agent)
		cons = 0.8


		if self.parent_node.is_leader:
			self.parent_node.velocity_goal = [velocity[1] * cons , velocity[0] * cons , -velocity[2] * cons ]
			#if not self.parent_node.manual_control:
				#self.parent_node.yaw = math.atan2(goal_loc.translation.x - drone_loc.x, goal_loc.translation.y - drone_loc.y)
		else:
			self.parent_node.velocity_goal = [velocity[1], velocity[0], -velocity[2] ]
			#self.parent_node.yaw = math.atan2(goal_loc.translation.x - drone_loc.x, goal_loc.translation.y - drone_loc.y)
		#self.parent_node.get_logger().info(f"Velocity Goal: {self.parent_node.velocity_goal}, Yaw: {self.parent_node.yaw}")

	def get_drone_pos(self, drone_id):
		try:
			now = rclpy.time.Time()
			drone_tf = self.parent_node.tf_buffer.lookup_transform(
				'world',
				f'{self.parent_node.px4_model}_{drone_id}/odom',
				now,
				timeout=rclpy.duration.Duration(seconds=0.5)
			)
			trans = drone_tf.transform
			return trans
		except Exception as e:
			# self.parent_node.get_logger().info(f"Could not get transform for drone_{drone_id}_base: {e}")
			return None
		
	def get_drone_to_goal(self, drone_id):
		try:
			drone_tf = self.parent_node.tf_buffer.lookup_transform(
		    	target_frame=f"{self.parent_node.px4_model}_{drone_id}/odom",       
		    	source_frame=f"{self.parent_node.px4_model}_{drone_id}/{self.parent_node.goal_frame}", 
		    	time=rclpy.time.Time(),
				timeout=rclpy.duration.Duration(seconds=0.5)
			)
			trans = drone_tf.transform
			return trans
		except Exception as e:
			# self.parent_node.get_logger().info(f"Could not get goal transform for drone_{drone_id}_base: {e}")
			return None
		
	def get_drone_absolute_goal(self, drone_id):
		try:
			now = rclpy.time.Time()
			drone_tf = self.parent_node.tf_buffer.lookup_transform(
				'world',
				f'{self.parent_node.px4_model}_{drone_id}/{self.parent_node.goal_frame}',				
				now,
				timeout=rclpy.duration.Duration(seconds=0.5)
			)
			trans = drone_tf.transform
			return trans
		except Exception as e:
			# self.parent_node.get_logger().info(f"Could not get goal transform for drone_{drone_id}_base: {e}")
			return None
	
	def get_obstacle_absolute(self):
		if (
			not hasattr(self.parent_node, 'lidar')
			or not self.parent_node.lidar.cluster_centers_cartesian
		):
			return []
		try:
			# 1. Look up the transform at the time the scan was taken
			target_frame = 'world'
			source_frame = f"{self.parent_node.px4_model}_{self.parent_node.frame_id}/odom"
			transform = self.parent_node.tf_buffer.lookup_transform(
				target_frame,
				source_frame,
				rclpy.time.Time(),#self.parent_node.get_clock().now().to_msg(),#msg.header.stamp, # Use the message timestamp for accuracy
				timeout=rclpy.duration.Duration(seconds=0.1))

			# 2. Transform each point
			transformed_centers = []
			for point in self.parent_node.lidar.cluster_centers_cartesian:
				# Create a PointStamped message for each point
				point_stamped = PointStamped()
				point_stamped.header.frame_id = source_frame
				point_stamped.header.stamp = self.parent_node.get_clock().now().to_msg()
				point_stamped.point.x = point[0]
				point_stamped.point.y = point[1]
				point_stamped.point.z = 0.0 # Assuming 2D LiDAR

				# Use the do_transform_point utility from tf2_geometry_msgs
				transformed_point = tf2_geometry_msgs.do_transform_point(point_stamped, transform)
				transformed_centers.append([transformed_point.point.x, transformed_point.point.y, transformed_point.point.z])

			# Store the final, absolute positions
			#self.absolute_cluster_centers = transformed_centers
			#self.parent_node.get_logger().info(f"Found {len(transformed_centers)} obstacles in world frame. {transformed_centers}")
			return transformed_centers
		except TransformException as e:
			self.parent_node.get_logger().error(f"Could not transform points from {source_frame} to {target_frame}: {e}")
			return []
