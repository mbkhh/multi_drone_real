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
VISION_TRIGGER_TOPIC = '/swarm/vision_trigger'
VISION_DETECTION_TOPIC = '/swarm/vision_command'
VISION_EVENT_TARGET_DETECTED = 'TARGET_DETECTED'
VISION_EVENT_TARGET_DETECTED_LEGACY = 'TARGET_DETECTED_LAND'
VISION_ACTION_REPORT = 'report'
VISION_ACTION_RETURN_LAND = 'return_land'


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

        # Keep the vision-node wire contract unchanged. Every decentralized
        # controller listens for the shared detection event, while the station
        # command selects whether that event is report-only or a one-shot
        # return-and-land request.
        self.vision_trigger_publisher = parent_node.create_publisher(
            String,
            VISION_TRIGGER_TOPIC,
            10,
        )
        self.vision_detection_subscriber = parent_node.create_subscription(
            String,
            VISION_DETECTION_TOPIC,
            self.vision_detection_callback,
            10,
        )
        self.vision_detection_action = VISION_ACTION_REPORT
        self.vision_return_land_active = False
        self.vision_return_target = None

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
            self.cancel_vision_return_land('station TAKEOFF command')
            self.execute_takeoff(command.get('height', 3.0))
            return

        if command_type == 'land':
            self.cancel_vision_return_land('station LAND command')
            self.parent_node.request_land()
            return

        if command_type == 'disarm':
            self.cancel_vision_return_land('station DISARM command')
            self.parent_node.request_safe_disarm()
            return

        if command_type in ('fly', 'set_goal', 'move'):
            self.cancel_vision_return_land('station move/set_goal command')
            self.execute_goal(command)
            return

        if command_type == 'yaw':
            self.cancel_vision_return_land('station yaw command')
            self.parent_node.request_relative_yaw(command.get('delta_degrees'))
            return

        if command_type == 'mission':
            self.cancel_vision_return_land('station mission command')
            self.execute_mission(command)
            return

        if command_type == 'start_detection':
            self.execute_start_detection(command)
            return

        if command_type == 'stop_detection':
            self.execute_stop_detection()
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

    def execute_start_detection(self, command):
        """Enable unchanged swarm_vision nodes and select the response mode."""
        detection_action = str(
            command.get('on_detection', VISION_ACTION_REPORT)
        ).strip().lower()
        if detection_action not in (
            VISION_ACTION_REPORT,
            VISION_ACTION_RETURN_LAND,
        ):
            raise ValueError('on_detection must be report or return_land')
        if self.vision_return_land_active:
            raise ValueError('return-home landing is already active')

        class_ids = command.get('class_ids')
        if class_ids is None:
            trigger_payload = 'START'
        elif not isinstance(class_ids, list) or not class_ids:
            raise ValueError(
                'class_ids must be a non-empty list of COCO class IDs'
            )
        else:
            validated_ids = []
            for value in class_ids:
                try:
                    numeric_value = float(value)
                    class_id = int(numeric_value)
                except (TypeError, ValueError, OverflowError) as error:
                    raise ValueError(
                        f'invalid detection class ID {value!r}'
                    ) from error
                if (
                    not math.isfinite(numeric_value)
                    or numeric_value != class_id
                    or class_id < 0
                    or class_id > 79
                ):
                    raise ValueError(
                        'detection class IDs must be integers from 0 through 79'
                    )
                if class_id not in validated_ids:
                    validated_ids.append(class_id)
            trigger_payload = 'START:' + ','.join(
                str(class_id) for class_id in validated_ids
            )

        self.vision_detection_action = detection_action
        self.vision_return_target = None
        trigger = String()
        trigger.data = trigger_payload
        self.vision_trigger_publisher.publish(trigger)
        self.parent_node.message = (
            f'VISION STARTED: {trigger_payload}, action={detection_action}'
        )
        self.parent_node.get_logger().info(
            f'Drone {self.drone_id} published vision trigger: '
            f'{trigger_payload}; on detection={detection_action}.'
        )

    def execute_stop_detection(self):
        """Stop inference without cancelling a return already in progress."""
        trigger = String()
        trigger.data = 'STOP'
        self.vision_trigger_publisher.publish(trigger)
        if not self.vision_return_land_active:
            self.vision_detection_action = VISION_ACTION_REPORT
            self.vision_return_target = None
            self.parent_node.message = 'VISION STOPPED'
        self.parent_node.get_logger().info(
            f'Drone {self.drone_id} published vision trigger: STOP'
        )

    def vision_detection_callback(self, msg):
        """Report a shared detection and start this drone's selected response."""
        payload = msg.data.strip()
        if ':' not in payload:
            self.parent_node.get_logger().warning(
                f'Ignoring malformed vision detection: {payload!r}'
            )
            return

        source, event = (part.strip() for part in payload.split(':', 1))
        if not source or event not in (
            VISION_EVENT_TARGET_DETECTED,
            VISION_EVENT_TARGET_DETECTED_LEGACY,
        ):
            self.parent_node.get_logger().warning(
                f'Ignoring unknown vision detection: {payload!r}'
            )
            return

        report = f'VISION TARGET DETECTED by {source}'
        self.parent_node.message = report
        self.parent_node.get_logger().warning(
            f'{report}; queued for decentralized station status.'
        )
        if self.vision_detection_action == VISION_ACTION_RETURN_LAND:
            self.start_vision_return_land(source)

    def start_vision_return_land(self, source):
        """Return this drone to its configured start XY at current altitude."""
        if self.vision_return_land_active:
            return

        # The response is one-shot. STOP is harmless when several controllers
        # publish it after receiving the same shared detection event.
        self.vision_detection_action = VISION_ACTION_REPORT
        trigger = String()
        trigger.data = 'STOP'
        self.vision_trigger_publisher.publish(trigger)

        node = self.parent_node
        if (
            node.state != 'TAKEOFF'
            or node.vehicle_status.arming_state
            != VehicleStatus.ARMING_STATE_ARMED
            or node.vehicle_status.nav_state
            != VehicleStatus.NAVIGATION_STATE_OFFBOARD
        ):
            node.message = (
                f'VISION TARGET DETECTED by {source}; RETURN REJECTED: '
                f'drone {self.drone_id} is not armed in Offboard'
            )
            node.get_logger().error(node.message)
            return
        if node.manual_control:
            node.message = (
                f'VISION TARGET DETECTED by {source}; RETURN REJECTED: '
                'manual control is active'
            )
            node.get_logger().error(node.message)
            return

        safety_violation = node.safety_violation_reason()
        if safety_violation is not None:
            node.message = (
                f'VISION TARGET DETECTED by {source}; RETURN REJECTED: '
                f'{safety_violation}'
            )
            node.get_logger().error(node.message)
            return

        current = [float(value) for value in node.navigation.current_pos[:3]]
        home = [float(value) for value in node.initial_world_position[:3]]
        if not all(math.isfinite(value) for value in current):
            node.message = (
                f'VISION TARGET DETECTED by {source}; RETURN REJECTED: '
                'current position is not finite'
            )
            node.get_logger().error(node.message)
            return
        if len(home) != 3 or not all(math.isfinite(value) for value in home):
            node.message = (
                f'VISION TARGET DETECTED by {source}; RETURN REJECTED: '
                'configured home position is invalid'
            )
            node.get_logger().error(node.message)
            return
        if not node.min_goal_altitude <= current[2] <= node.max_goal_altitude:
            node.message = (
                f'VISION TARGET DETECTED by {source}; RETURN REJECTED: '
                'current height is outside the goal safety envelope'
            )
            node.get_logger().error(node.message)
            return

        if node.mission_active:
            node.abort_mission(
                'target detected; decentralized return-home landing selected'
            )

        # The configured home is trusted and may be farther away than the
        # interactive one-command goal limit. Normal navigation velocity,
        # acceleration, collision-avoidance, and safety limits remain active.
        self.vision_return_target = [home[0], home[1], current[2]]
        node.set_local_goal(self.vision_return_target)
        node.motion_enabled = True
        self.vision_return_land_active = True

        home_distance = math.hypot(
            current[0] - home[0],
            current[1] - home[1],
        )
        node.message = (
            f'VISION TARGET DETECTED by {source}; DRONE {self.drone_id} '
            f'RETURNING HOME [{home[0]:.2f}, {home[1]:.2f}] from '
            f'{home_distance:.2f} m at z={current[2]:.2f}'
        )
        node.get_logger().warning(node.message)

    def update_vision_return_land(self):
        """Start this drone's controlled landing after it reaches home."""
        if not self.vision_return_land_active:
            return

        node = self.parent_node
        if node.state != 'TAKEOFF':
            self.cancel_vision_return_land(
                f'drone {self.drone_id} left active Offboard flight'
            )
            return

        current = [float(value) for value in node.navigation.current_pos[:3]]
        if not all(math.isfinite(value) for value in current):
            return
        target = self.vision_return_target
        if (
            target is None
            or len(target) != 3
            or not all(math.isfinite(float(value)) for value in target)
        ):
            self.cancel_vision_return_land('saved return target is unavailable')
            return

        configured_tolerance = get_config('swarm_single.goal_tolerance')
        goal_tolerance = float(
            0.1 if configured_tolerance is None else configured_tolerance
        )
        tolerance = max(goal_tolerance, float(node.mission_goal_tolerance))
        if math.dist(current, target) > tolerance:
            return

        home_x, home_y, _ = node.initial_world_position
        if node.request_land():
            node.message = (
                f'VISION RETURN HOME COMPLETE: drone {self.drone_id} reached '
                f'[{home_x:.2f}, {home_y:.2f}]; controlled LAND started'
            )
            node.get_logger().warning(node.message)
        else:
            node.message = (
                f'VISION RETURN HOME COMPLETE: drone {self.drone_id} LAND '
                'request rejected'
            )
            node.get_logger().error(node.message)
        self.vision_return_land_active = False
        self.vision_return_target = None

    def cancel_vision_return_land(self, reason):
        """Cancel a pending automatic return when explicitly overridden."""
        if not self.vision_return_land_active:
            return
        self.vision_return_land_active = False
        self.vision_return_target = None
        self.vision_detection_action = VISION_ACTION_REPORT
        self.parent_node.message = f'VISION RETURN CANCELLED: {reason}'
        self.parent_node.get_logger().warning(self.parent_node.message)

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
        self.update_vision_return_land()

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
