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
            'swarm_single.navigation.neighbor_dist', 6.0
        )
        self.max_neighbors = self._config_int(
            'swarm_single.navigation.max_neighbors', 20
        )
        self.time_horizon = self._config_float(
            'swarm_single.navigation.time_horizon', 5.0
        )
        self.radius = self._config_float(
            'swarm_single.navigation.radius', 0.75
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
        collision_values = (
            self.neighbor_dist,
            self.radius,
            self.time_horizon,
        )
        if not all(
            math.isfinite(value) and value > 0.0
            for value in collision_values
        ):
            raise ValueError(
                'navigation neighbor_dist, radius, and time_horizon must be '
                'finite and positive.'
            )
        if self.neighbor_dist <= 2.0 * self.radius:
            raise ValueError(
                'navigation.neighbor_dist must exceed two navigation.radius '
                'values so avoidance begins before overlap.'
            )

        self._last_hold_reason = None
        self._last_hold_warning_time = None
        self._last_collision_warning_time = None

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
        drone_velocity = self._finite_vector(self.local_velocity)
        if drone_velocity is None:
            return self._hold_for_invalid_feedback(
                'PX4 local-velocity feedback is not finite'
            )

        if self.parent_node.manual_control:
            goal_position = None
        else:
            goal_position = self.resolve_active_goal()
            if goal_position is None:
                return self._hold_for_invalid_feedback(
                    'active local goal is unavailable'
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
        simulator.setAgentVelocity(moving_agent, tuple(drone_velocity))
        if self.parent_node.manual_control:
            own_preferred_velocity = self._bounded_velocity(
                self.parent_node.manual_velocity
            )
        else:
            goal_vector = goal_position - drone_position
            own_preferred_velocity = self._preferred_velocity(
                goal_vector, float(np.linalg.norm(goal_vector))
            )
        simulator.setAgentPrefVelocity(
            moving_agent, tuple(own_preferred_velocity)
        )
        peer_samples = []

        # Every vehicle has its own goal, but all vehicles remain reciprocal
        # RVO agents. Use both position and measured ENU velocity from the
        # directly exchanged /swarm/local_state sample. A simulator is rebuilt
        # each control tick, so omitting setAgentVelocity would incorrectly
        # reset every approaching aircraft to stationary on every tick.
        for neighbor_id in sorted(self.parent_node.last_seen_neighbors):
            neighbor_state = self.get_fresh_peer_state(neighbor_id)
            if neighbor_state is None:
                return self._hold_for_invalid_feedback(
                    f'neighbor {neighbor_id} pose is unavailable/stale'
                )
            neighbor_position = self._finite_vector(
                neighbor_state.get('position')
            )
            neighbor_velocity = self._finite_vector(
                neighbor_state.get('velocity')
            )
            if neighbor_position is None or neighbor_velocity is None:
                return self._hold_for_invalid_feedback(
                    f'neighbor {neighbor_id} state is not finite'
                )
            agent = simulator.addAgent(tuple(neighbor_position))
            simulator.setAgentVelocity(agent, tuple(neighbor_velocity))
            # Its exact local mission goal intentionally stays private. The
            # measured velocity is the best low-bandwidth prediction of what
            # that independently controlled drone intends to keep doing.
            simulator.setAgentPrefVelocity(
                agent, tuple(self._bounded_velocity(neighbor_velocity))
            )
            peer_samples.append(
                (int(neighbor_id), neighbor_position, neighbor_velocity)
            )

        for center_point in self.get_obstacle_absolute():
            obstacle_position = np.asarray(center_point, dtype=float)
            agent = simulator.addAgent(tuple(obstacle_position))
            simulator.setAgentVelocity(agent, (0.0, 0.0, 0.0))
            simulator.setAgentPrefVelocity(agent, (0.0, 0.0, 0.0))

        simulator.doStep()
        velocity_enu = np.asarray(
            simulator.getAgentVelocity(moving_agent), dtype=float
        )
        velocity_enu = self._apply_collision_safety(
            velocity_enu, drone_position, peer_samples
        )
        # Navigation uses ENU; PX4 trajectory velocity uses NED.
        self.parent_node.velocity_goal = [
            float(velocity_enu[1]),
            float(velocity_enu[0]),
            -float(velocity_enu[2]),
        ]
        self._last_hold_reason = None
        return True

    def _apply_collision_safety(
        self, candidate_velocity, own_position, peer_samples
    ):
        """Add a deterministic horizontal escape for predicted close passes.

        RVO remains the normal planner. This final filter protects against
        delayed samples, acceleration limits, and the installed RVO3D build's
        weak response to perfectly symmetric head-on encounters.
        """
        candidate = self._bounded_velocity(candidate_velocity)
        safety_distance = 2.0 * self.radius
        threatening_peers = []
        minimum_distance = math.inf

        for peer_id, peer_position, peer_velocity in peer_samples:
            relative_position = peer_position - own_position
            current_distance = float(np.linalg.norm(relative_position))
            if not math.isfinite(current_distance):
                continue
            minimum_distance = min(minimum_distance, current_distance)

            if current_distance <= 1e-6:
                own_id = int(self.parent_node.frame_id)
                line_of_sight = np.asarray(
                    [1.0 if own_id < peer_id else -1.0, 0.0, 0.0],
                    dtype=float,
                )
            else:
                line_of_sight = relative_position / current_distance

            # relative_position(t) = peer(t) - self(t)
            relative_velocity = peer_velocity - candidate
            speed_squared = float(np.dot(relative_velocity, relative_velocity))
            if speed_squared > 1e-9:
                closest_time = -float(
                    np.dot(relative_position, relative_velocity)
                ) / speed_squared
                closest_time = max(
                    0.0, min(self.time_horizon, closest_time)
                )
            else:
                closest_time = 0.0
            closest_offset = (
                relative_position + relative_velocity * closest_time
            )
            closest_distance = float(np.linalg.norm(closest_offset))
            predicted_threat = (
                closest_time > 0.0
                and closest_time <= self.time_horizon
                and closest_distance < safety_distance
            )
            if current_distance >= safety_distance and not predicted_threat:
                continue

            relative_closing_speed = max(
                0.0,
                float(np.dot(candidate - peer_velocity, line_of_sight)),
            )
            # Each decentralized drone normally removes half the closing
            # component. If safety volumes already overlap, remove all of it.
            avoidance_share = (
                1.0 if current_distance < safety_distance else 0.5
            )
            candidate = candidate - (
                line_of_sight
                * relative_closing_speed
                * avoidance_share
            )

            # Both drones use the same left-around-the-other rule. Because
            # their line-of-sight vectors are opposite, their global escape
            # directions are also opposite instead of choosing the same side.
            lateral = np.asarray(
                [-line_of_sight[1], line_of_sight[0], 0.0], dtype=float
            )
            lateral_norm = float(np.linalg.norm(lateral))
            if lateral_norm <= 1e-6:
                own_id = int(self.parent_node.frame_id)
                lateral = np.asarray(
                    [1.0 if own_id < peer_id else -1.0, 0.0, 0.0],
                    dtype=float,
                )
            else:
                lateral = lateral / lateral_norm

            penetration = max(
                0.0,
                (safety_distance - closest_distance) / safety_distance,
            )
            lateral_speed = self.max_speed * min(
                0.75, 0.35 + 0.4 * penetration
            )
            candidate = self._bounded_velocity(
                candidate + lateral * lateral_speed
            )
            threatening_peers.append(peer_id)

        if threatening_peers:
            self._warn_collision_avoidance(
                threatening_peers, minimum_distance
            )
        return candidate

    def _warn_collision_avoidance(self, peer_ids, minimum_distance):
        now = self.parent_node.get_clock().now()
        if self._last_collision_warning_time is not None:
            elapsed = (
                now - self._last_collision_warning_time
            ).nanoseconds / 1e9
            if 0.0 <= elapsed < 1.0:
                return
        self._last_collision_warning_time = now
        self.parent_node.get_logger().warning(
            '[COLLISION AVOIDANCE] predicted unsafe separation from peers '
            f'{sorted(set(peer_ids))}; nearest current distance is '
            f'{minimum_distance:.2f} m.'
        )

    @staticmethod
    def _finite_vector(values):
        """Return a finite three-vector or None for unsafe state data."""
        try:
            vector = np.asarray(values, dtype=float)
        except (TypeError, ValueError):
            return None
        if vector.shape != (3,) or not np.all(np.isfinite(vector)):
            return None
        return vector

    def _bounded_velocity(self, values):
        """Bound a preferred ENU velocity to the configured RVO speed."""
        vector = self._finite_vector(values)
        if vector is None:
            return np.zeros(3, dtype=float)
        speed = float(np.linalg.norm(vector))
        if speed > self.max_speed and speed > 0.0:
            vector = vector * (self.max_speed / speed)
        return vector

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
        """Resolve this drone's independent absolute goal."""
        mode = self.parent_node.active_goal_mode
        if mode == 'absolute':
            goal = np.asarray(self.parent_node.active_goal, dtype=float)
            return goal if goal.shape == (3,) and np.all(np.isfinite(goal)) else None
        return None

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
