#!/usr/bin/env python3

from collections import defaultdict, deque
from threading import Lock

import rclpy
from px4_msgs.msg import (
    BatteryStatus,
    EstimatorStatusFlags,
    FailsafeFlags,
    ManualControlSetpoint,
    OffboardControlMode,
    SensorGps,
    TrajectorySetpoint,
    VehicleAttitude,
    VehicleCommand,
    VehicleCommandAck,
    VehicleControlMode,
    VehicleGlobalPosition,
    VehicleLandDetected,
    VehicleLocalPosition,
    VehicleOdometry,
    VehicleStatus,
)
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, Header, String
from swarm_config.config_utils import get_config
from swarm_msgs.msg import FormationCommand, ManualControl, Status


class SwarmLogger(Node):
    """Collect messages by topic and report the newest sample once per interval."""

    def __init__(self):
        super().__init__('swarm_logger')

        default_drone_count = int(get_config('swarm_sim.drone_count'))
        self.declare_parameter('drone_count', default_drone_count)
        self.declare_parameter('print_interval', 1.0)

        self.drone_count = (
            self.get_parameter('drone_count').get_parameter_value().integer_value
        )
        print_interval = (
            self.get_parameter('print_interval').get_parameter_value().double_value
        )

        self._queues = defaultdict(deque)
        self._queue_lock = Lock()
        self._subscriptions = []

        # VOLATILE subscribers work with the normal PX4 best-effort telemetry
        # publishers and do not request old samples when this logger starts.
        self._px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )
        self._ros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )

        self._add_swarm_topics()
        self._add_vehicle_topics()
        self.create_timer(print_interval, self._print_and_clear_queues)

        self.get_logger().info(
            f'Watching {len(self._subscriptions)} topics for '
            f'{self.drone_count} drone(s); printing every {print_interval:.2f} s.'
        )

    def _subscribe(self, message_type, topic, qos):
        def callback(message, topic_name=topic):
            self._enqueue(topic_name, message)

        subscription = self.create_subscription(
            message_type, topic, callback, qos
        )
        self._subscriptions.append(subscription)

    def _add_swarm_topics(self):
        topics = [
            (Status, '/swarm/status'),
            (Header, '/swarm/live'),
            (String, '/swarm/command'),
            (FormationCommand, '/swarm/formation_command'),
            (ManualControl, '/manual_controller'),
            (Bool, '/swarm/takeoff_spacing_ok'),
            (String, '/swarm/takeoff_spacing'),
            # Leader-to-follower topics are relative in swarm_single and
            # currently resolve to these global names.
            (String, '/command'),
            (FormationCommand, '/formation'),
        ]
        for message_type, topic in topics:
            self._subscribe(message_type, topic, self._ros_qos)

    def _add_vehicle_topics(self):
        px4_topics = [
            (VehicleStatus, 'vehicle_status_v1'),
            (VehicleLocalPosition, 'vehicle_local_position_v1'),
            (VehicleGlobalPosition, 'vehicle_global_position'),
            (SensorGps, 'vehicle_gps_position'),
            (VehicleAttitude, 'vehicle_attitude'),
            (VehicleOdometry, 'vehicle_odometry'),
            (VehicleControlMode, 'vehicle_control_mode'),
            (VehicleLandDetected, 'vehicle_land_detected'),
            (BatteryStatus, 'battery_status_v1'),
            (VehicleCommandAck, 'vehicle_command_ack'),
            (EstimatorStatusFlags, 'estimator_status_flags'),
            (FailsafeFlags, 'failsafe_flags'),
            (ManualControlSetpoint, 'manual_control_setpoint'),
        ]
        control_topics = [
            (VehicleCommand, 'vehicle_command'),
            (TrajectorySetpoint, 'trajectory_setpoint'),
            (OffboardControlMode, 'offboard_control_mode'),
        ]

        for drone_id in range(1, self.drone_count + 1):
            namespace = f'/uav_{drone_id}/fmu'
            for message_type, topic_name in px4_topics:
                self._subscribe(
                    message_type,
                    f'{namespace}/out/{topic_name}',
                    self._px4_qos,
                )
            for message_type, topic_name in control_topics:
                self._subscribe(
                    message_type,
                    f'{namespace}/in/{topic_name}',
                    self._px4_qos,
                )

    def _enqueue(self, topic, message):
        with self._queue_lock:
            self._queues[topic].append(message)

    def _print_and_clear_queues(self):
        with self._queue_lock:
            samples = [
                (topic, len(queue), queue[-1])
                for topic, queue in self._queues.items()
                if queue
            ]
            for queue in self._queues.values():
                queue.clear()

        if not samples:
            self.get_logger().info('No messages received during this interval.')
            return

        samples.sort(key=lambda sample: sample[0])
        separator = '\n' + ('-' * 72)
        report = separator.join(
            f'{topic} | received={count}\n{last_message}'
            for topic, count, last_message in samples
        )
        self.get_logger().info(
            f'Latest topic samples ({len(samples)} active topics):\n{report}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = SwarmLogger()
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
