import math

import numpy as np
import rvo23d
from rclpy.node import Node

from swarm_config.config_utils import get_config


class navigation:
    """Generate velocity goals from local PX4 state and shared peer states."""

    def __init__(self, parent_node: Node):
        self.parent_node = parent_node

        self.local_velocity = [0.0, 0.0, 0.0]
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
        self.neighbor_state_timeout = self._config_float(
            'swarm_single.navigation.neighbor_state_timeout', 0.5
        )
        self.braking_acceleration = self._config_float(
            'swarm_single.navigation.braking_acceleration', 0.7
        )

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
                    'active local goal or shared leader state is unavailable/stale'
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
                neighbor_position = self.get_drone_pos(neighbor_id)
                if neighbor_position is None:
                    return self._hold_for_invalid_feedback(
                        f'neighbor {neighbor_id} pose is unavailable/stale'
                    )
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
        """Resolve an absolute or formation goal using only local memory."""
        mode = self.parent_node.active_goal_mode
        if mode == 'absolute':
            goal = np.asarray(self.parent_node.active_goal, dtype=float)
            return goal if goal.shape == (3,) and np.all(np.isfinite(goal)) else None

        if mode != 'formation' or self.parent_node.formation_offset is None:
            return None
        leader_id = self.parent_node.leader_id
        if leader_id is None or leader_id == int(self.parent_node.frame_id):
            return None
        leader_position = self.get_drone_pos(leader_id)
        if leader_position is None:
            return None
        offset = np.asarray(self.parent_node.formation_offset, dtype=float)
        goal = leader_position + offset
        return goal if np.all(np.isfinite(goal)) else None

    def get_fresh_peer_state(self, drone_id):
        """Return peer data if it was recently received on this computer."""
        state = self.parent_node.peer_states.get(int(drone_id))
        if state is None:
            return None
        age = (
            self.parent_node.get_clock().now() - state['received_at']
        ).nanoseconds / 1e9
        if age < 0.0 or age > self.neighbor_state_timeout:
            return None
        return state

    def get_drone_pos(self, drone_id):
        state = self.get_fresh_peer_state(drone_id)
        if state is None:
            return None
        position = np.asarray(state['position'], dtype=float)
        return position if position.shape == (3,) and np.all(np.isfinite(position)) else None

    def get_obstacle_absolute(self):
        if (
            not hasattr(self.parent_node, 'lidar')
            or not self.parent_node.lidar.cluster_centers_cartesian
        ):
            return []

        transformed_centers = []
        translation = np.asarray(self.current_pos[:3], dtype=float)
        quaternion_vector = np.asarray(self.current_pos[3:6], dtype=float)
        quaternion_w = float(self.current_pos[6])
        for center_point in self.parent_node.lidar.cluster_centers_cartesian:
            point = np.asarray(
                [float(center_point[0]), float(center_point[1]), 0.0],
                dtype=float,
            )
            first_cross = np.cross(quaternion_vector, point)
            rotated = point + 2.0 * (
                quaternion_w * first_cross
                + np.cross(quaternion_vector, first_cross)
            )
            transformed_centers.append((translation + rotated).tolist())
        return transformed_centers
