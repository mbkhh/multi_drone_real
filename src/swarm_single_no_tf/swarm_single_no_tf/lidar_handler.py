import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import math
from sklearn.cluster import DBSCAN
from swarm_config.config_utils import get_config

class lidarHandler():
	def __init__(self, parent_node: Node):
		self.parent_node = parent_node

		
		self.subscription = self.parent_node.create_subscription(
			LaserScan,
			f'/world/default/model/x500_depth_{self.parent_node.frame_id}/link/lidar_link/sensor/gpu_lidar/scan',  # Topic name is dynamically set
			self.scan_callback,
			10) # QoS profile depth
		self.parent_node.get_logger().info(f'Subscribed to /px4_{self.parent_node.frame_id}/scan. Waiting for data...')

		self.lidar_min_cutoff = get_config('swarm_single.lidar.lidar_min_cutoff')
		self.lidar_max_cutoff = get_config('swarm_single.lidar.lidar_max_cutoff')
		self.min_samples = get_config('swarm_single.lidar.min_samples')
		self.max_distance_samples = get_config('swarm_single.lidar.max_distance_samples')

		self.cartesian_points = None
		self.cluster_centers_cartesian = []
		self.center_ranges = []
		self.center_angles = []

	def scan_callback(self, msg):

		qx, qy, qz, qw = self.parent_node.navigation.current_pos[3:7]
		sin_roll = 2.0 * (qw * qx + qy * qz)
		cos_roll = 1.0 - 2.0 * (qx * qx + qy * qy)
		roll = math.atan2(sin_roll, cos_roll)
		sin_pitch = 2.0 * (qw * qy - qz * qx)
		pitch = math.asin(max(-1.0, min(1.0, sin_pitch)))

		angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
		ranges = np.array(msg.ranges)
		
		valid_indices = (np.isfinite(ranges)) & \
						(ranges > self.lidar_min_cutoff) & \
						(ranges < self.lidar_max_cutoff)
		valid_ranges = ranges[valid_indices]
		valid_angles = angles[valid_indices]

		if valid_ranges.size == 0:
			return # No valid points to process

		sin_gamma = np.sin(pitch) * np.cos(valid_angles) + \
								np.sin(roll) * np.cos(pitch) * np.sin(valid_angles)
			
		# Due to float precision, sin_gamma can sometimes be slightly > 1 or < -1.
		# We clip it to the valid range [-1, 1] to avoid math errors in arcsin.
		sin_gamma = np.clip(sin_gamma, -1.0, 1.0)
			
		# Calculate the tilt angle (gamma) for each beam
		gamma = np.arcsin(sin_gamma)
			
		# Correct the ranges by projecting them onto the horizontal plane
		# corrected_ranges are the true horizontal distances (d_H)
		corrected_ranges = valid_ranges * np.cos(gamma)
		
		x_points = corrected_ranges * np.sin(valid_angles)
		y_points = corrected_ranges * np.cos(valid_angles)
		self.cartesian_points = np.vstack((x_points, y_points)).T

		# 3. Apply DBSCAN clustering
		# eps: The maximum distance between two samples for one to be considered as in the neighborhood of the other.
		# min_samples: The number of samples in a neighborhood for a point to be considered as a core point.
		db = DBSCAN(eps=self.max_distance_samples, min_samples=self.min_samples).fit(self.cartesian_points)
		labels = db.labels_
		# 4. Calculate the center of each found cluster
		cluster_centers_cartesian = []
		unique_labels = set(labels)
		for label in unique_labels:
			if label == -1:
				# Label -1 is considered noise by DBSCAN, skip it
				continue
			
			# Get all points belonging to the current cluster
			points_in_cluster = self.cartesian_points[labels == label]
			
			# Calculate the mean of the points to find the center
			#center_x, center_y = np.mean(points_in_cluster, axis=0)
			distances = np.linalg.norm(points_in_cluster, axis=1)
			
			# 2. Find the index of the point with the minimum distance
			closest_point_index = np.argmin(distances)
			
			# 3. Select that point as the new cluster center
			center_x, center_y = points_in_cluster[closest_point_index]
			cluster_centers_cartesian.append((center_y, center_x))

		self.cluster_centers_cartesian = cluster_centers_cartesian

		#self.parent_node.get_logger().info(f"Cluster centers (Cartesian): {self.cluster_centers_cartesian}")

		if not cluster_centers_cartesian:
			self.absolute_cluster_centers = []
			return
		
		if cluster_centers_cartesian:
			centers_np = np.array(cluster_centers_cartesian)
			center_ranges = np.hypot(centers_np[:, 0], centers_np[:, 1])
			center_angles = np.arctan2(centers_np[:, 0], centers_np[:, 1])
			self.center_ranges = center_ranges
			self.center_angles = center_angles
