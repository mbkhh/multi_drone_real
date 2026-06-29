import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
import numpy as np
from sklearn.cluster import DBSCAN

class RadarPlotter(Node):
	def __init__(self, drone_id=1):
		super().__init__(f'radar_plotter_drone_{drone_id}')

		self.subscription = self.create_subscription(
			LaserScan,
			f'/world/default/model/x500_depth_{drone_id}/link/lidar_link/sensor/gpu_lidar/scan',  # Topic name is dynamically set
			self.scan_callback,
			10) # QoS profile depth
		self.get_logger().info(f'Subscribed to /px4_{drone_id}/scan. Waiting for data...')

		plt.ion()
		self.fig = plt.figure(figsize=(8, 8))

		self.ax = self.fig.add_subplot(111, polar=True)
		
		self.raw_points_scatter = self.ax.scatter([], [], s=5, c='blue', alpha=0.7, label='LiDAR Points')
		self.cluster_centers_scatter = self.ax.scatter([], [], s=60, c='red', zorder=3, label='Cluster Centers')


		self.setup_radar_style()

	def setup_radar_style(self):
		self.ax.set_theta_zero_location('N')  # Set 0 degrees to the top (North)
		self.ax.set_theta_direction(-1)	# Set angles to increase clockwise
		self.ax.set_rlabel_position(90)	# Move radial labels to the side
		self.ax.set_title("LiDAR Radar View", va='bottom') # va = vertical alignment
		self.ax.grid(True, linestyle='--', alpha=0.6)

	def scan_callback(self, msg):
		"""Processes LiDAR data, runs clustering, and updates the plot."""
		# 1. Get raw data and filter out invalid 'inf' values
		angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
		ranges = np.array(msg.ranges)
		
		

		valid_indices = (np.isfinite(ranges)) & \
						(ranges > 0.2) & \
						(ranges < 8)
		valid_ranges = ranges[valid_indices]
		valid_angles = angles[valid_indices]

		if valid_ranges.size == 0:
			return # No valid points to process

		# 2. Convert polar coordinates to Cartesian for clustering
		# Clustering algorithms work with Cartesian distances (x,y), not polar (angle, range)
		x_points = valid_ranges * np.sin(valid_angles)
		y_points = valid_ranges * np.cos(valid_angles)
		cartesian_points = np.vstack((x_points, y_points)).T
		print(cartesian_points)
		# 3. Apply DBSCAN clustering
		# eps: The maximum distance between two samples for one to be considered as in the neighborhood of the other.
		# min_samples: The number of samples in a neighborhood for a point to be considered as a core point.
		db = DBSCAN(eps=0.5, min_samples=2).fit(cartesian_points)
		labels = db.labels_
		print(f"DBSCAN labels: {labels}")
		# 4. Calculate the center of each found cluster
		cluster_centers_cartesian = []
		unique_labels = set(labels)
		for label in unique_labels:
			if label == -1:
				# Label -1 is considered noise by DBSCAN, skip it
				continue
			
			# Get all points belonging to the current cluster
			points_in_cluster = cartesian_points[labels == label]
			
			# Calculate the mean of the points to find the center
			distances = np.linalg.norm(points_in_cluster, axis=1)

			# 2. Find the index of the point with the minimum distance
			closest_point_index = np.argmin(distances)
			
			# 3. Select that point as the new cluster center
			center_x, center_y = points_in_cluster[closest_point_index]
			cluster_centers_cartesian.append((center_x, center_y))
		if not cluster_centers_cartesian:
			self.get_logger().info("No clusters found.")
			return

		print(f"Cluster centers (Cartesian): {cluster_centers_cartesian}")
		# 5. Update the plots
		# <<< UPDATE ALL POINTS (BLUE) >>>
		self.raw_points_scatter.set_offsets(np.vstack((valid_angles, valid_ranges)).T)
		print(f"Valid points")
		# <<< UPDATE CENTER POINTS (RED) >>>
		if cluster_centers_cartesian:
			centers_np = np.array(cluster_centers_cartesian)
			center_ranges = np.hypot(centers_np[:, 0], centers_np[:, 1])
			center_angles = np.arctan2(centers_np[:, 0], centers_np[:, 1])
			self.cluster_centers_scatter.set_offsets(np.vstack((center_angles, center_ranges)).T)
		else:
			self.cluster_centers_scatter.set_offsets(np.empty((0, 2)))
		print(f"Cluster centers (Polar): {center_ranges}, {center_angles}")
		# Adjust plot limits and redraw
		self.ax.set_rmax(np.max(valid_ranges) + 1.0)
		self.fig.canvas.draw()
		self.fig.canvas.flush_events()


def main(args=None):
	rclpy.init(args=args)
	try:
		radar_plotter = RadarPlotter(drone_id=1)
		while rclpy.ok():
			rclpy.spin_once(radar_plotter, timeout_sec=0.1)
	except KeyboardInterrupt:
		pass # Allow clean exit on Ctrl+C
	finally:
		# Cleanup
		if 'radar_plotter' in locals() and rclpy.ok():
			radar_plotter.destroy_node()
		if rclpy.ok():
			rclpy.shutdown()
		plt.ioff()
		plt.show() 


if __name__ == '__main__':
	main()