import math

import numpy as np
import rclpy
import rvo23d
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped, TransformStamped
from rclpy.node import Node

from swarm_config.config_utils import get_config


class navigation:
    """Generate velocity goals from locally held state and fresh peer TFs."""

    def __init__(self, parent_node: Node):
        self.parent_node = parent_node

        self.local_velocity = [0.0, 0.0, 0.0]
        self.leader_velocity = [0.0, 0.0, 0.0]
        self.current_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]

        # Configuration is immutable while a node is running. Loading and
        # parsing YAML here once avoids doing seven disk reads on every control
        # tick, which was consuming roughly half of the 20 Hz period on battery.
        self.time_step = self._config_float(
            'swarm_single.navigation.time_step', 0.05
        )
        self.neighbor_dist = self._config_float(
            'swarm_single.navigation.neighbor_dist', 0.5
        )
        self.max_neighbors = self._config_int(
            'swarm_single.navigation.max_neighbors', 20
        )
        self.time_horizon = self._config_float(
            'swarm_single.navigation.time_horizon', 3.0
        )
        self.radius = self._config_float(
            'swarm_single.navigation.radius', 0.5
        )
        self.max_speed = self._config_float(
            'swarm_single.navigation.max_speed', 1.0
        )
        self.good_dist_to_goal = self._config_float(
            'swarm_single.navigation.good_dist_to_goal', 0.3
        )
        self.feedback_timeout = self._config_float(
            'swarm_single.navigation.feedback_timeout', 0.3
        )
        self.neighbor_transform_timeout = self._config_float(
            'swarm_single.navigation.neighbor_transform_timeout', 0.5
        )
        self.braking_acceleration = self._config_float(
            'swarm_single.navigation.braking_acceleration', 0.7
        )

        # TF timestamps can come from computers whose clocks have a constant
        # offset. Freshness is therefore based on how recently a transform's
        # source timestamp advanced according to this node's local clock.
        self._transform_cache = {}
        self._last_hold_reason = None
        self._last_hold_warning_time = None

    @staticmethod
    def _config_float(key, default):
        value = get_config(key)
        return float(default if value is None else value)

    @staticmethod
    def _config_int(key, default):
        value = get_config(key)
        return int(default if value is None else value)

    def navigate_to_goal(self):
        """Update the NED velocity goal; return False when feedback is unsafe."""
        if not self._local_position_is_fresh():
            return self._hold_for_invalid_feedback(
                'PX4 local-position feedback is missing or stale'
            )

        drone_position = np.asarray(self.current_pos[:3], dtype=float)
        if not np.all(np.isfinite(drone_position)):
            return self._hold_for_invalid_feedback(
                'PX4 local-position feedback is not finite'
            )

        if self.parent_node.is_leader and self.parent_node.manual_control:
            goal_position = None
        else:
            goal_position = self.resolve_active_goal()
            if goal_position is None:
                return self._hold_for_invalid_feedback(
                    'active goal or its parent transform is unavailable/stale'
                )

        simulator = rvo23d.PyRVOSimulator(
            self.time_step,
            self.neighbor_dist,
            self.max_neighbors,
            self.time_horizon,
            self.radius,
            self.max_speed,
        )

        moving_agent = simulator.addAgent(tuple(drone_position))
        agents = [moving_agent]
        goals = [
            np.zeros(3, dtype=float)
            if goal_position is None
            else goal_position
        ]

        if not self.parent_node.is_leader:
            for neighbor_id in self.parent_node.last_seen_neighbors:
                neighbor_pose = self.get_drone_pos(neighbor_id)
                if neighbor_pose is None:
                    return self._hold_for_invalid_feedback(
                        f'neighbor {neighbor_id} pose is unavailable/stale'
                    )
                neighbor_position = np.array([
                    neighbor_pose.translation.x,
                    neighbor_pose.translation.y,
                    neighbor_pose.translation.z,
                ])
                agent = simulator.addAgent(tuple(neighbor_position))
                agents.append(agent)
                goals.append(neighbor_position)

        for center_point in self.get_obstacle_absolute():
            obstacle_position = np.asarray(center_point, dtype=float)
            agent = simulator.addAgent(tuple(obstacle_position))
            agents.append(agent)
            goals.append(obstacle_position)

        for index, agent in enumerate(agents):
            if (
                self.parent_node.is_leader
                and self.parent_node.manual_control
                and index == 0
            ):
                preferred_velocity = tuple(self.parent_node.manual_velocity)
            else:
                current_position = np.asarray(
                    simulator.getAgentPosition(agent), dtype=float
                )
                goal_vector = goals[index] - current_position
                distance = float(np.linalg.norm(goal_vector))
                preferred_velocity = self._preferred_velocity(
                    goal_vector, distance
                )
            simulator.setAgentPrefVelocity(agent, preferred_velocity)

        simulator.doStep()
        velocity_enu = simulator.getAgentVelocity(moving_agent)
        leader_scale = 0.8 if self.parent_node.is_leader else 1.0

        # Navigation uses ENU; PX4 trajectory velocity uses NED.
        self.parent_node.velocity_goal = [
            float(velocity_enu[1]) * leader_scale,
            float(velocity_enu[0]) * leader_scale,
            -float(velocity_enu[2]) * leader_scale,
        ]
        self._last_hold_reason = None
        return True

    def _preferred_velocity(self, goal_vector, distance):
        if distance <= self.good_dist_to_goal or distance <= 0.0:
            return (0.0, 0.0, 0.0)

        # Begin braking before the stopping radius. This prevents the abrupt
        # full-speed/zero transition that magnified delayed-feedback overshoot.
        braking_distance = max(0.0, distance - self.good_dist_to_goal)
        braking_speed = math.sqrt(
            2.0 * self.braking_acceleration * braking_distance
        )
        desired_speed = min(self.max_speed, distance, braking_speed)
        direction = goal_vector / distance
        return tuple(desired_speed * direction)

    def _local_position_is_fresh(self):
        timestamp = self.parent_node.last_local_position_time
        if timestamp is None:
            return False
        age = (
            self.parent_node.get_clock().now() - timestamp
        ).nanoseconds / 1e9
        return 0.0 <= age <= self.feedback_timeout

    def _hold_for_invalid_feedback(self, reason):
        self.parent_node.velocity_goal = [0.0, 0.0, 0.0]
        now = self.parent_node.get_clock().now()
        should_warn = self._last_hold_reason != reason
        if self._last_hold_warning_time is None:
            should_warn = True
        else:
            elapsed = (
                now - self._last_hold_warning_time
            ).nanoseconds / 1e9
            should_warn = should_warn or elapsed >= 1.0
        if should_warn:
            self.parent_node.get_logger().warning(
                f'[NAVIGATION HOLD] {reason}; commanding zero velocity.'
            )
            self._last_hold_warning_time = now
        self._last_hold_reason = reason
        return False

    def resolve_active_goal(self):
        """Resolve the locally stored goal without a goal-TF DDS round trip."""
        goal_transform = self.parent_node.active_goal_transform
        if goal_transform is None:
            return None

        offset = np.array([
            goal_transform.transform.translation.x,
            goal_transform.transform.translation.y,
            goal_transform.transform.translation.z,
        ], dtype=float)
        if not np.all(np.isfinite(offset)):
            return None

        parent_frame = goal_transform.header.frame_id
        if parent_frame == 'world':
            return offset

        parent_pose = self.get_fresh_frame_pose(parent_frame)
        if parent_pose is None:
            return None

        translation = np.array([
            parent_pose.translation.x,
            parent_pose.translation.y,
            parent_pose.translation.z,
        ])
        quaternion_vector = np.array([
            parent_pose.rotation.x,
            parent_pose.rotation.y,
            parent_pose.rotation.z,
        ])
        first_cross = np.cross(quaternion_vector, offset)
        rotated_offset = offset + 2.0 * (
            parent_pose.rotation.w * first_cross
            + np.cross(quaternion_vector, first_cross)
        )
        goal = translation + rotated_offset
        return goal if np.all(np.isfinite(goal)) else None

    def get_fresh_frame_pose(self, source_frame):
        """Perform an immediate lookup and reject a TF stream that stopped."""
        try:
            stamped_transform = self.parent_node.tf_buffer.lookup_transform(
                'world', source_frame, rclpy.time.Time()
            )
        except Exception:
            return None

        now = self.parent_node.get_clock().now()
        source_stamp = (
            int(stamped_transform.header.stamp.sec),
            int(stamped_transform.header.stamp.nanosec),
        )
        cached = self._transform_cache.get(source_frame)
        if cached is None or cached['source_stamp'] != source_stamp:
            cached = {
                'source_stamp': source_stamp,
                'observed_at': now,
                'transform': stamped_transform.transform,
            }
            self._transform_cache[source_frame] = cached

        age = (now - cached['observed_at']).nanoseconds / 1e9
        if age < 0.0 or age > self.neighbor_transform_timeout:
            return None
        return cached['transform']

    def get_drone_pos(self, drone_id):
        return self.get_fresh_frame_pose(
            f'{self.parent_node.px4_model}_{drone_id}/odom'
        )

    def get_drone_to_goal(self, drone_id):
        try:
            return self.parent_node.tf_buffer.lookup_transform(
                target_frame=f'{self.parent_node.px4_model}_{drone_id}/odom',
                source_frame=(
                    f'{self.parent_node.px4_model}_{drone_id}/'
                    f'{self.parent_node.goal_frame}'
                ),
                time=rclpy.time.Time(),
            ).transform
        except Exception:
            return None

    def get_drone_absolute_goal(self, _drone_id):
        goal = self.resolve_active_goal()
        if goal is None:
            return None
        transform = TransformStamped().transform
        transform.translation.x = float(goal[0])
        transform.translation.y = float(goal[1])
        transform.translation.z = float(goal[2])
        transform.rotation.w = 1.0
        return transform

    def get_obstacle_absolute(self):
        if (
            not hasattr(self.parent_node, 'lidar')
            or not self.parent_node.lidar.cluster_centers_cartesian
        ):
            return []

        # The local pose is already available in memory; avoid another TF
        # lookup from inside the control timer.
        transform = TransformStamped()
        transform.header.frame_id = 'world'
        transform.child_frame_id = (
            f'{self.parent_node.px4_model}_{self.parent_node.frame_id}/odom'
        )
        transform.transform.translation.x = self.current_pos[0]
        transform.transform.translation.y = self.current_pos[1]
        transform.transform.translation.z = self.current_pos[2]
        transform.transform.rotation.x = self.current_pos[3]
        transform.transform.rotation.y = self.current_pos[4]
        transform.transform.rotation.z = self.current_pos[5]
        transform.transform.rotation.w = self.current_pos[6]

        transformed_centers = []
        for center_point in self.parent_node.lidar.cluster_centers_cartesian:
            point = PointStamped()
            point.header.frame_id = transform.child_frame_id
            point.point.x = float(center_point[0])
            point.point.y = float(center_point[1])
            point.point.z = 0.0
            transformed = tf2_geometry_msgs.do_transform_point(
                point, transform
            )
            transformed_centers.append([
                transformed.point.x,
                transformed.point.y,
                transformed.point.z,
            ])
        return transformed_centers
