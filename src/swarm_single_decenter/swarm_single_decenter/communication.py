"""Decentralized station communication for one independently controlled UAV."""

import json
import math

from px4_msgs.msg import VehicleStatus
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String
from swarm_config.config_utils import get_config, get_mission_waypoints
from swarm_msgs.msg import Status


COMMAND_TOPIC = '/swarm/decenter/command'
STATUS_TOPIC = '/swarm/decenter/status'
DEFAULT_WAYPOINT_FILE = 'waypoints_decentralized_36x34.yaml'


class Communication:
    """Give every drone a direct station command and status connection."""

    def __init__(self, parent_node):
        self.parent_node = parent_node
        self.drone_id = int(parent_node.frame_id)

        parent_node.declare_parameter('decenter_command_topic', COMMAND_TOPIC)
        parent_node.declare_parameter('decenter_status_topic', STATUS_TOPIC)
        self.command_topic = (
            parent_node.get_parameter('decenter_command_topic')
            .get_parameter_value().string_value
        )
        self.status_topic = (
            parent_node.get_parameter('decenter_status_topic')
            .get_parameter_value().string_value
        )

        # Keep inter-computer traffic non-blocking. Commands have a deeper
        # queue than continuously refreshed status/state traffic.
        self.command_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.status_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=max(3, int(get_config('swarm_sim.drone_count') or 1) * 2),
        )

        self.command_subscriber = parent_node.create_subscription(
            String,
            self.command_topic,
            self.command_callback,
            self.command_qos,
        )
        self.status_publisher = parent_node.create_publisher(
            Status,
            self.status_topic,
            self.status_qos,
        )

        status_interval = float(get_config('swarm_single.status_interval') or 1.0)
        self.status_timer = parent_node.create_timer(
            status_interval, self.broadcast_status
        )
        self.peer_timeout = max(
            2.0,
            float(get_config('swarm_single.broadcast_interval') or 1.5) * 4.0,
        )
        self.peer_timer = parent_node.create_timer(
            min(1.0, self.peer_timeout / 2.0), self.remove_stale_peers
        )

        parent_node.get_logger().info(
            f'Decentralized communication ready for drone {self.drone_id}: '
            f'commands={self.command_topic}, status={self.status_topic}.'
        )

    def command_callback(self, msg):
        """Execute a command only when it targets this drone or all drones."""
        try:
            command = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError) as error:
            self.parent_node.get_logger().error(
                f'Ignoring malformed decentralized command: {error}'
            )
            return
        if not isinstance(command, dict):
            self.parent_node.get_logger().error(
                'Ignoring decentralized command: JSON payload must be an object.'
            )
            return
        if not self.is_targeted(command.get('target', 'all')):
            return

        command_type = str(command.get('command', '')).strip().lower()
        self.parent_node.get_logger().info(
            f'Drone {self.drone_id} received command: {msg.data}'
        )
        try:
            self.execute_command(command_type, command)
        except (TypeError, ValueError, OverflowError) as error:
            self.parent_node.get_logger().error(
                f"Command '{command_type}' rejected: {error}"
            )

    def is_targeted(self, target):
        """Return whether a JSON target addresses this vehicle."""
        if isinstance(target, str) and target.strip().lower() in ('all', '*'):
            return True
        try:
            return int(target) == self.drone_id
        except (TypeError, ValueError):
            self.parent_node.get_logger().warning(
                f'Ignoring command with invalid target {target!r}.'
            )
            return False

    def execute_command(self, command_type, command):
        if command_type == 'arm':
            self.parent_node.request_offboard_control()
            return

        if command_type == 'takeoff':
            self.execute_takeoff(command.get('height', 3.0))
            return

        if command_type == 'land':
            self.parent_node.request_land()
            return

        if command_type == 'disarm':
            self.parent_node.request_safe_disarm()
            return

        if command_type in ('fly', 'set_goal', 'move'):
            self.execute_goal(command)
            return

        if command_type == 'yaw':
            self.parent_node.request_relative_yaw(command.get('delta_degrees'))
            return

        if command_type == 'mission':
            self.execute_mission(command)
            return

        if command_type in ('abort_mission', 'stop_mission'):
            if self.parent_node.mission_active:
                self.parent_node.abort_mission('station abort command')
            else:
                self.parent_node.get_logger().info('No active mission to abort.')
            return

        self.parent_node.get_logger().error(
            f'Unknown decentralized command {command_type!r}.'
        )

    def execute_takeoff(self, height):
        """Apply the existing explicit takeoff behavior to this drone only."""
        height = float(height)
        if not math.isfinite(height) or height <= 0.0:
            raise ValueError('takeoff height must be finite and greater than zero')
        if (
            self.parent_node.state != 'TAKEOFF'
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
            float(value) for value in self.parent_node.navigation.current_pos[:3]
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
        self.parent_node.get_logger().info(
            f'Drone {self.drone_id}: taking off {height:.2f} m.'
        )
        return True

    def execute_goal(self, command):
        values = [float(command[axis]) for axis in ('x', 'y', 'z')]
        if not all(math.isfinite(value) for value in values):
            raise ValueError('goal coordinates must be finite')
        if not bool(command.get('absolute', True)):
            base = self.parent_node.leader_goal
            values = [float(base[index]) + values[index] for index in range(3)]
        if self.parent_node.goal_callback_temp(values):
            if self.parent_node.mission_active:
                self.parent_node.abort_mission('replaced by station goal')
            if self.parent_node.state == 'TAKEOFF':
                self.parent_node.motion_enabled = True
            else:
                self.parent_node.get_logger().warning(
                    'Goal stored but motion is disabled until ARM Offboard.'
                )

    def execute_mission(self, command):
        """Load this drone's own key from the shared decentralized YAML file."""
        waypoint_file = command.get('waypoint_file', DEFAULT_WAYPOINT_FILE)
        points = get_mission_waypoints(waypoint_file, self.drone_id)
        expected_counts = command.get('waypoint_counts')
        if isinstance(expected_counts, dict):
            expected = expected_counts.get(str(self.drone_id))
            if expected is not None and int(expected) != len(points):
                raise ValueError(
                    f'local file has {len(points)} waypoints, station expected '
                    f'{int(expected)}'
                )

        relative_to_start = bool(command.get('relative_to_start', False))
        yaw_relative = bool(command.get('yaw_relative', True))
        if self.parent_node.start_mission(
            points,
            relative_to_start=relative_to_start,
            yaw_relative=yaw_relative,
        ):
            self.parent_node.get_logger().info(
                f"Drone {self.drone_id} started its {len(points)}-point mission "
                f"from '{waypoint_file}'."
            )

    def broadcast_status(self):
        """Publish this drone's own state; there is no elected leader."""
        node = self.parent_node
        msg = Status()
        msg.timestamp = int(node.get_clock().now().nanoseconds / 1000)
        msg.leader_id = self.drone_id  # Legacy field used as the reporting ID.
        msg.swarm_members = sorted(
            set(node.last_seen_neighbors.keys()) | {self.drone_id}
        )
        msg.pattern_name = 'decentralized'
        msg.leader_x = float(node.navigation.current_pos[0])
        msg.leader_y = float(node.navigation.current_pos[1])
        msg.leader_z = float(node.navigation.current_pos[2])
        msg.leader_yaw = float(node.yaw)

        goal = node.active_goal
        if goal is not None and len(goal) == 3:
            msg.goal_x = float(goal[0])
            msg.goal_y = float(goal[1])
            msg.goal_z = float(goal[2])
        msg.message = node.message
        msg.control_state = node.state
        msg.armed = (
            node.vehicle_status.arming_state
            == VehicleStatus.ARMING_STATE_ARMED
        )
        msg.offboard = (
            node.vehicle_status.nav_state
            == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        )
        msg.mission_active = node.mission_active
        msg.mission_count = len(node.mission)
        msg.mission_index = (
            node.mission_index + 1
            if node.mission_active
            else min(node.mission_index, len(node.mission))
        )
        msg.mission_state = node.mission_state
        self.status_publisher.publish(msg)
        node.message = ''

    def remove_stale_peers(self):
        """Expire membership derived from the existing shared-state stream."""
        now = self.parent_node.get_clock().now()
        stale = []
        for peer_id, last_seen in list(
            self.parent_node.last_seen_neighbors.items()
        ):
            age = (now - last_seen).nanoseconds / 1e9
            if age < 0.0 or age > self.peer_timeout:
                stale.append(peer_id)
        for peer_id in stale:
            self.parent_node.last_seen_neighbors.pop(peer_id, None)
            self.parent_node.peer_states.pop(peer_id, None)
            self.parent_node.get_logger().warning(
                f'Drone {peer_id} state timed out; removed from local awareness.'
            )
