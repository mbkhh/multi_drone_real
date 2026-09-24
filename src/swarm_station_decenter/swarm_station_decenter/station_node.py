#!/usr/bin/env python3
"""Interactive ground station for independently controlled drones."""

import json
import math
import select
import shlex
import sys

import rclpy
from rclpy.node import Node
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


class DecentralizedStationNode(Node):
    """Send targeted JSON commands and retain one status per drone."""

    STATUS_TIMEOUT = 5.0

    def __init__(self):
        super().__init__('decentralized_station')
        self.declare_parameter('command_topic', COMMAND_TOPIC)
        self.declare_parameter('status_topic', STATUS_TOPIC)
        self.command_topic = (
            self.get_parameter('command_topic').get_parameter_value().string_value
        )
        self.status_topic = (
            self.get_parameter('status_topic').get_parameter_value().string_value
        )

        command_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        status_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=max(3, int(get_config('swarm_sim.drone_count') or 1) * 2),
        )
        self.command_publisher = self.create_publisher(
            String, self.command_topic, command_qos
        )
        self.status_subscriber = self.create_subscription(
            Status, self.status_topic, self.status_callback, status_qos
        )
        self.statuses = {}
        self.status_times = {}
        self.input_timer = self.create_timer(0.1, self.check_for_input)
        self.timeout_timer = self.create_timer(1.0, self.check_timeouts)
        self.get_logger().info(
            'Decentralized station ready. Use "help" for commands.'
        )

    def status_callback(self, msg):
        # Status.leader_id is retained by the wire schema but represents the
        # reporting drone ID in decentralized mode.
        drone_id = int(msg.leader_id)
        is_new = drone_id not in self.statuses
        self.statuses[drone_id] = msg
        self.status_times[drone_id] = self.get_clock().now()
        if is_new:
            self.get_logger().info(f'Drone {drone_id} connected to station.')
        if msg.message:
            self.get_logger().info(f'Drone {drone_id}: {msg.message}')

    def check_timeouts(self):
        now = self.get_clock().now()
        for drone_id, received_at in list(self.status_times.items()):
            age = (now - received_at).nanoseconds / 1e9
            if age > self.STATUS_TIMEOUT:
                self.status_times.pop(drone_id, None)
                self.statuses.pop(drone_id, None)
                self.get_logger().warning(
                    f'Drone {drone_id} disconnected from station (status timeout).'
                )

    def show_status(self):
        """Print a current line for every connected drone."""
        if not self.statuses:
            self.get_logger().warning('No drones are currently connected.')
            return
        self.get_logger().info(
            f'Connected drones: {sorted(self.statuses)}'
        )
        for drone_id in sorted(self.statuses):
            status = self.statuses[drone_id]
            mission = (
                f'{status.mission_state} '
                f'{status.mission_index}/{status.mission_count}'
            )
            self.get_logger().info(
                f'[{drone_id}] state={status.control_state}, '
                f'armed={status.armed}, offboard={status.offboard}, '
                f'position=({status.leader_x:.2f}, {status.leader_y:.2f}, '
                f'{status.leader_z:.2f}), mission={mission}, '
                f'peers={list(status.swarm_members)}'
            )

    def publish_command(self, target, command, **values):
        payload = {'target': target, 'command': command}
        payload.update(values)
        msg = String()
        msg.data = json.dumps(payload)
        self.command_publisher.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')

    def validate_mission_file(self, target, waypoint_file):
        """Validate all selected YAML sections before sending the filename."""
        if target == 'all':
            drone_count = int(get_config('swarm_sim.drone_count') or 3)
            drone_ids = list(range(1, drone_count + 1))
        else:
            drone_ids = [int(target)]

        waypoint_counts = {}
        for drone_id in drone_ids:
            points = get_mission_waypoints(waypoint_file, drone_id)
            if not points:
                raise ValueError(f'drone {drone_id} has no waypoints')
            waypoint_counts[str(drone_id)] = len(points)
        return waypoint_counts

    @staticmethod
    def parse_target(tokens):
        """Consume an optional `all`, `ID`, or `drone ID` target prefix."""
        if not tokens:
            return 'all', tokens
        first = tokens[0].lower()
        if first == 'all':
            return 'all', tokens[1:]
        if first == 'drone':
            if len(tokens) < 2:
                raise ValueError('drone target requires an integer ID')
            return int(tokens[1]), tokens[2:]
        try:
            return int(tokens[0]), tokens[1:]
        except ValueError:
            return 'all', tokens

    def handle_command(self, command_input):
        try:
            tokens = shlex.split(command_input)
        except ValueError as error:
            self.get_logger().error(f'Invalid command quoting: {error}')
            return
        if not tokens:
            return

        local_command = tokens[0].lower()
        if local_command == 'status':
            if len(tokens) != 1:
                self.get_logger().error('Usage: status')
            else:
                self.show_status()
            return
        if local_command == 'help':
            self.print_help()
            return
        if local_command in ('exit', 'quit', 'shutdown'):
            rclpy.shutdown()
            return

        try:
            target, tokens = self.parse_target(tokens)
            if not tokens:
                raise ValueError('missing command after target')
            command = tokens[0].lower()
            arguments = tokens[1:]

            if command in ('arm', 'land', 'disarm', 'abort_mission'):
                if arguments:
                    raise ValueError(f'Usage: [all|ID] {command}')
                self.publish_command(target, command)
                return

            if command in ('start_detection', 'start_dection'):
                if target != 'all':
                    raise ValueError(
                        'Detection is swarm-wide. Use: start_detection '
                        '[report|return_land] [class_id,class_id,...]'
                    )
                if len(arguments) > 2:
                    raise ValueError(
                        'Usage: start_detection [report|return_land] '
                        '[class_id,class_id,...]'
                    )
                on_detection = 'report'
                if arguments and arguments[0].lower() in (
                    'report',
                    'return_land',
                ):
                    on_detection = arguments.pop(0).lower()
                if len(arguments) > 1:
                    raise ValueError(
                        'Put the action before the optional detection class list'
                    )
                class_ids = [32]
                if arguments:
                    try:
                        class_ids = [
                            int(value)
                            for value in arguments[0].split(',')
                            if value
                        ]
                    except ValueError as error:
                        raise ValueError(
                            'Detection class IDs must be comma-separated integers'
                        ) from error
                if not class_ids or any(
                    class_id < 0 or class_id > 79 for class_id in class_ids
                ):
                    raise ValueError(
                        'Detection class IDs must be integers from 0 through 79'
                    )
                self.publish_command(
                    'all',
                    'start_detection',
                    class_ids=class_ids,
                    on_detection=on_detection,
                )
                return

            if command == 'stop_detection':
                if target != 'all' or arguments:
                    raise ValueError('Usage: stop_detection')
                self.publish_command('all', 'stop_detection')
                return

            if command == 'takeoff':
                if len(arguments) > 1:
                    raise ValueError('Usage: [all|ID] takeoff [height]')
                configured = get_config('swarm_single.control.takeoff_height')
                height = float(
                    arguments[0]
                    if arguments
                    else (3.0 if configured is None else configured)
                )
                if not math.isfinite(height) or height <= 0.0:
                    raise ValueError('takeoff height must be finite and positive')
                self.publish_command(target, 'takeoff', height=height)
                return

            if command in ('move', 'set_goal', 'move_relative'):
                if len(arguments) != 3:
                    raise ValueError(
                        'Usage: [all|ID] move x y z, or move_relative x y z'
                    )
                coordinates = [float(value) for value in arguments]
                if not all(math.isfinite(value) for value in coordinates):
                    raise ValueError('goal coordinates must be finite')
                self.publish_command(
                    target,
                    'fly',
                    x=coordinates[0],
                    y=coordinates[1],
                    z=coordinates[2],
                    absolute=command != 'move_relative',
                )
                return

            if command == 'yaw':
                if len(arguments) != 1:
                    raise ValueError('Usage: [all|ID] yaw degrees')
                degrees = float(arguments[0])
                if not math.isfinite(degrees):
                    raise ValueError('yaw must be finite')
                self.publish_command(
                    target, 'yaw', delta_degrees=degrees
                )
                return

            if command == 'mission':
                if len(arguments) > 1:
                    raise ValueError('Usage: [all|ID] mission [waypoint_file]')
                waypoint_file = (
                    arguments[0] if arguments else DEFAULT_WAYPOINT_FILE
                )
                counts = self.validate_mission_file(target, waypoint_file)
                self.publish_command(
                    target,
                    'mission',
                    waypoint_file=waypoint_file,
                    waypoint_counts=counts,
                    relative_to_start=False,
                    yaw_relative=True,
                )
                return

            raise ValueError(f'unknown command {command!r}')
        except (TypeError, ValueError) as error:
            self.get_logger().error(str(error))

    def print_help(self):
        self.get_logger().info(
            'Commands:\n'
            '  status\n'
            '  [all|ID|drone ID] arm\n'
            '  [all|ID|drone ID] takeoff [height]\n'
            '  [all|ID|drone ID] mission [waypoint_file]\n'
            '  [all|ID|drone ID] move x y z\n'
            '  [all|ID|drone ID] move_relative x y z\n'
            '  [all|ID|drone ID] yaw degrees\n'
            '  [all|ID|drone ID] abort_mission\n'
            '  [all|ID|drone ID] land\n'
            '  [all|ID|drone ID] disarm\n'
            '  start_detection [report|return_land] [class_id,class_id,...]\n'
            '  stop_detection\n'
            'Detection commands are always swarm-wide.\n'
            'A command without a target is sent to all drones.'
        )

    def check_for_input(self):
        try:
            readable, _, _ = select.select([sys.stdin], [], [], 0.0)
        except (OSError, ValueError):
            return
        if readable:
            line = sys.stdin.readline()
            if line:
                self.handle_command(line.strip())


def main(args=None):
    rclpy.init(args=args)
    node = DecentralizedStationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
