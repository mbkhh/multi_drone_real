#!/usr/bin/env python3
"""Live 3D plot of swarm positions against the configured leader mission."""

import json
import math
import os
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import matplotlib

# Some ROS installations also provide an older system Matplotlib. If a newer
# user installation is selected, keep its namespace-package tools together
# with it instead of accidentally importing the system copy of mplot3d.
import mpl_toolkits
_matplotlib_toolkits = os.path.join(
	os.path.dirname(os.path.dirname(matplotlib.__file__)), 'mpl_toolkits'
)
if (
	os.path.isdir(_matplotlib_toolkits)
	and _matplotlib_toolkits not in mpl_toolkits.__path__
):
	mpl_toolkits.__path__.insert(0, _matplotlib_toolkits)

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  (registers the 3d projection)

from nav_msgs.msg import Odometry
from std_msgs.msg import String
from swarm_config.config_utils import get_config, get_mission_waypoints
from swarm_single_no_tf_yaw.online_waypoint_generator import OnlineWaypointGenerator


def _load_planned_xyz(waypoint_file, leader_id):
	"""Load and validate the same mission rows used by the station."""
	waypoints = get_mission_waypoints(waypoint_file, leader_id)
	planned_xyz = []
	for index, point in enumerate(waypoints, start=1):
		if (
			not isinstance(point, (list, tuple))
			or len(point) not in (4, 5)
		):
			raise ValueError(
				f'Mission waypoint {index} must be [x, y, z, '
				'relative_yaw] with an optional checkpoint wait boolean.'
			)
		try:
			values = [float(value) for value in point[:4]]
		except (TypeError, ValueError) as error:
			raise ValueError(
				f'Mission waypoint {index} contains a non-numeric value.'
			) from error
		if not all(math.isfinite(value) for value in values):
			raise ValueError(
				f'Mission waypoint {index} contains a non-finite value.'
			)
		if len(point) == 5 and not isinstance(point[4], bool):
			raise ValueError(
				f'Mission waypoint {index} checkpoint wait value must be '
				'true or false.'
			)
		planned_xyz.append(values[:3])
	return planned_xyz


def _load_online_missions(parameters=None):
	"""Generate plot paths from the same planner used by online_mission."""
	if parameters is None or parameters == "":
		parameters = {}
	if isinstance(parameters, str):
		try:
			parameters = json.loads(parameters)
		except json.JSONDecodeError as error:
			raise ValueError(
				"online_config must be a JSON object of planner inputs."
			) from error
	if not isinstance(parameters, dict):
		raise ValueError("online_config must decode to a JSON object.")

	result = OnlineWaypointGenerator(None).generate(parameters)
	missions = {}
	for raw_drone_id, points in result["waypoints"].items():
		planned_xyz = []
		for index, point in enumerate(points, start=1):
			if not isinstance(point, (list, tuple)) or len(point) not in (4, 5):
				raise ValueError(
					f"Online waypoint {index} for drone {raw_drone_id} "
					"must contain x, y, z, yaw, and optional checkpoint wait."
			)
			try:
				values = [float(value) for value in point[:4]]
			except (TypeError, ValueError) as error:
				raise ValueError(
					f"Online waypoint {index} for drone {raw_drone_id} "
					"contains a non-numeric value."
			) from error
			if not all(math.isfinite(value) for value in values):
				raise ValueError(
					f"Online waypoint {index} for drone {raw_drone_id} "
					"contains a non-finite value."
			)
			if len(point) == 5 and point[4] not in (True, False, 0, 1):
				raise ValueError(
					f"Online waypoint {index} for drone {raw_drone_id} "
					"has an invalid checkpoint wait value."
			)
			planned_xyz.append(values[:3])
		missions[int(raw_drone_id)] = planned_xyz
	return missions, result["config"]


class PositionPlotter(Node):
	def __init__(self):
		super().__init__('position_plotter')

		self.declare_parameter('waypoint_file', '')
		self.declare_parameter('leader_id', 1)
		self.declare_parameter('online_mission', False)
		self.declare_parameter('online_config', '')
		requested_file = (
			self.get_parameter('waypoint_file').get_parameter_value().string_value.strip()
		)
		self.leader_id = int(
			self.get_parameter('leader_id').get_parameter_value().integer_value
		)
		online_enabled = self.get_parameter('online_mission').get_parameter_value().bool_value
		online_config = self.get_parameter('online_config').get_parameter_value().string_value.strip()
		configured_file = get_config('swarm_single.mission.waypoint_file')
		self.waypoint_file = (
			'<online planner>'
			if online_enabled
			else requested_file or configured_file or 'leader_waypoints_xyzyaw-3.txt'
		)

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
		self.command_subscriber = self.create_subscription(
			String,
			get_config('swarm_single.leader_command_topic_name'),
			self._online_command_callback,
			qos,
		)
		self.current = {}        # drone_id -> (x, y, z)
		self.goals = {}          # drone_id -> (x, y, z)

		# Offline mode reads the same installed file as the station. Online
		# mode generates the controller-ready paths, not the diagnostic sampled
		# leader array, so the plot agrees with the active mission.
		try:
			if online_enabled:
				self.missions, planner_config = _load_online_missions(
					online_config
				)
				planned_count = len(
					self.missions.get(self.leader_id, [])
				)
				self.get_logger().info(
					f'Online plot plan: {len(self.missions)} drone paths; '
					f'leader {self.leader_id} has {planned_count} waypoints '
					f'(Lx={planner_config.Lx:g}, Ly={planner_config.Ly:g}).'
				)
			else:
				planned_xyz = _load_planned_xyz(
					self.waypoint_file, self.leader_id
				)
				self.missions = (
					{self.leader_id: planned_xyz} if planned_xyz else {}
				)
				self.get_logger().info(
					f"Plotter started with '{self.waypoint_file}' for leader "
					f'{self.leader_id}: {len(planned_xyz)} waypoints.'
				)
	def _online_command_callback(self, msg: String):
		"""Refresh the plan when the station issues an online mission command."""
		try:
			command = json.loads(msg.data)
		except (TypeError, ValueError, json.JSONDecodeError) as error:
			self.get_logger().warning(
				f'Ignoring malformed mission command in plotter: {error}'
			)
			return
		if not isinstance(command, dict) or command.get('command') != 'online_mission':
			return
		try:
			missions, planner_config = _load_online_missions(
				command.get('parameters', {})
			)
		except (TypeError, ValueError, RuntimeError, OverflowError) as error:
			self.get_logger().error(
				f'Could not update online plot plan: {error}'
			)
			return
		with self.lock:
			self.missions = missions
			self.waypoint_file = '<online planner>'
		self.get_logger().info(
			f'Plotter updated from online_mission: {len(missions)} paths, '
			f'{len(missions.get(self.leader_id, []))} leader waypoints '
			f'(Lx={planner_config.Lx:g}, Ly={planner_config.Ly:g}).'
		)
		except (TypeError, ValueError, RuntimeError, OverflowError) as error:
			self.get_logger().error(
				f'Could not load planned mission: {error}'
			)
			self.missions = {}

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
		ax.set_title(f"Swarm position vs mission '{node.waypoint_file}'")

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
