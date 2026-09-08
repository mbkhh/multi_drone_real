#!/usr/bin/env python3
"""Live 3D plot of swarm vehicle positions against the planned mission waypoints.

Subscribes to /swarm/status (one message per drone) and traces each drone's
actual trajectory and current position, overlaying the planned waypoints loaded
from the mission file (swarm_config/config/missions/<mission_name>.yaml).
"""

import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import matplotlib
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  (registers the 3d projection)

from nav_msgs.msg import Odometry
from swarm_config.config_utils import get_config


class PositionPlotter(Node):
	def __init__(self):
		super().__init__('position_plotter')

		self.declare_parameter('mission_name', 'swarm_single')
		self.mission_name = self.get_parameter('mission_name').get_parameter_value().string_value

		self.drone_count = int(get_config('swarm_sim.drone_count'))

		# How far a drone must move (m) before we record a new trajectory point.
		self.min_step = 0.05

		qos = QoSProfile(
			reliability=ReliabilityPolicy.BEST_EFFORT,
			history=HistoryPolicy.KEEP_LAST,
			depth=int(self.drone_count * 2),
		)
		# In the no-TF controller every drone publishes its own Odometry on this
		# shared topic.  The drone number is carried in child_frame_id.
		self.state_subscriber = self.create_subscription(
			Odometry, "/swarm/local_state", self._state_callback, qos
		)

		# Shared state read by the matplotlib thread.
		self.lock = threading.Lock()
		self.trajectories = {}   # drone_id -> [(x, y, z), ...]
		self.current = {}        # drone_id -> (x, y, z)
		self.goals = {}          # drone_id -> (x, y, z)

		# The station sends the leader mission from swarm_single.yaml.  Keep the
		# plotter tied to that same source instead of requiring a second
		# config/missions/<name>.yaml file.  A mission waypoint may contain a
		# fourth yaw value; plotting uses only x, y, and z.
		waypoints = get_config('swarm_single.mission.waypoints')
		if not isinstance(waypoints, list):
			self.get_logger().warning(
				"No valid swarm_single.mission.waypoints found; planned path will be empty."
			)
			waypoints = []
		self.missions = {1: [list(point[:3]) for point in waypoints
								if isinstance(point, (list, tuple)) and len(point) >= 3]}
		self.get_logger().info(
			f"Plotter started. Mission '{self.mission_name}' has {len(self.missions)} drone path(s).")

	def _state_callback(self, msg: Odometry):
		try:
			drone_id = int(msg.child_frame_id)
		except (TypeError, ValueError):
			return
		position = msg.pose.pose.position
		pos = (float(position.x), float(position.y), float(position.z))
		with self.lock:
			self.current[drone_id] = pos
			traj = self.trajectories.setdefault(drone_id, [])
			if not traj or (abs(traj[-1][0] - pos[0]) + abs(traj[-1][1] - pos[1]) + abs(traj[-1][2] - pos[2])) > self.min_step:
				traj.append(pos)


def _color(drone_id):
	cmap = plt.get_cmap('tab10')
	return cmap((drone_id - 1) % 10)


def main(args=None):
	# Use an interactive backend if one is available; fall back gracefully.
	rclpy.init(args=args)
	node = PositionPlotter()

	# Spin ROS in a background thread so matplotlib can own the main thread.
	spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
	spin_thread.start()

	fig = plt.figure(figsize=(9, 7))
	ax = fig.add_subplot(111, projection='3d')

	def update(_frame):
		with node.lock:
			missions = {k: list(v) for k, v in node.missions.items()}
			trajectories = {k: list(v) for k, v in node.trajectories.items()}
			current = dict(node.current)

		ax.clear()
		ax.set_xlabel('X (m)')
		ax.set_ylabel('Y (m)')
		ax.set_zlabel('Z (m)')
		ax.set_title(f"Swarm position vs mission '{node.mission_name}'")

		# Planned waypoints (dashed) per drone.
		for drone_id, pts in sorted(missions.items()):
			if not pts:
				continue
			arr = np.array(pts)
			c = _color(drone_id)
			ax.plot(arr[:, 0], arr[:, 1], arr[:, 2], '--', color=c, alpha=0.5)
			ax.scatter(arr[:, 0], arr[:, 1], arr[:, 2], color=c, marker='x', alpha=0.6,
			           label=f"drone {drone_id} plan")

		# Actual flown trajectories (solid) + current position marker.
		for drone_id, traj in sorted(trajectories.items()):
			if not traj:
				continue
			arr = np.array(traj)
			c = _color(drone_id)
			ax.plot(arr[:, 0], arr[:, 1], arr[:, 2], '-', color=c,
			        label=f"drone {drone_id} actual")
		for drone_id, pos in sorted(current.items()):
			ax.scatter([pos[0]], [pos[1]], [pos[2]], color=_color(drone_id), s=70, marker='o')

		handles, labels = ax.get_legend_handles_labels()
		if handles:
			ax.legend(loc='upper right', fontsize='small')

	# Keep a reference so the animation isn't garbage collected.
	_ani = FuncAnimation(fig, update, interval=200, cache_frame_data=False)

	try:
		plt.show()
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		if rclpy.ok():
			rclpy.shutdown()


if __name__ == '__main__':
	main()
