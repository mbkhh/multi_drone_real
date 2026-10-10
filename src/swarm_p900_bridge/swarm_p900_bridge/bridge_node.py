"""ROS 2 integration node for the selected-topic P900 bridge."""

from dataclasses import dataclass
import queue
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.serialization import deserialize_message, serialize_message

from swarm_p900_bridge.roles import (
    CODE_TO_ROLE,
    role_code,
    ROLE_STATION,
    VALID_ROLES,
)
from swarm_p900_bridge.serial_transport import SerialTransport
from swarm_p900_bridge.topic_registry import (
    routes_received_by,
    routes_sent_by,
    TOPIC_REGISTRY,
)
from swarm_p900_bridge.transport_protocol import (
    BROADCAST_NODE_ID,
    DEFAULT_MAX_PAYLOAD,
    encode_frame,
    SequenceTracker,
    StreamParser,
)


@dataclass
class BridgeCounters:
    """Counters updated from ROS and serial worker threads."""

    tx_frames: int = 0
    tx_bytes: int = 0
    serialization_errors: int = 0
    rx_frames: int = 0
    rx_bytes: int = 0
    rx_queue_dropped: int = 0
    deserialization_errors: int = 0
    wrong_destination: int = 0
    invalid_direction: int = 0


class P900BridgeNode(Node):
    """Bridge configured ROS topics according to the local station/drone role."""

    def __init__(self):
        super().__init__('p900_bridge')

        self.declare_parameter('role', ROLE_STATION)
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 230400)
        self.declare_parameter('node_id', -1)
        self.declare_parameter('destination_node_id', -1)
        self.declare_parameter('max_payload_bytes', DEFAULT_MAX_PAYLOAD)
        self.declare_parameter('diagnostics_interval', 10.0)
        self.declare_parameter('reconnect_interval', 2.0)
        self.declare_parameter('serial_tx_queue_size', 100)
        self.declare_parameter('ros_rx_queue_size', 100)

        self.role = self.get_parameter('role').value.strip().lower()
        if self.role not in VALID_ROLES:
            choices = ', '.join(sorted(VALID_ROLES))
            raise ValueError(
                f'role must be one of {choices}; received {self.role!r}'
            )

        self.serial_port = str(self.get_parameter('serial_port').value)
        self.baud_rate = int(self.get_parameter('baud_rate').value)
        requested_node_id = int(self.get_parameter('node_id').value)
        requested_destination = int(
            self.get_parameter('destination_node_id').value
        )
        self.node_id = self._resolve_node_id(requested_node_id)
        self.destination_node_id = self._resolve_destination_id(
            requested_destination
        )
        self.max_payload = int(
            self.get_parameter('max_payload_bytes').value
        )
        if not 1 <= self.max_payload <= 0xFFFFFFFF:
            raise ValueError('max_payload_bytes must be in 1..4294967295')

        diagnostics_interval = float(
            self.get_parameter('diagnostics_interval').value
        )
        if diagnostics_interval <= 0.0:
            raise ValueError('diagnostics_interval must be positive')

        self.tx_routes = routes_sent_by(self.role)
        self.rx_routes = routes_received_by(self.role)
        self._route_publishers = {}
        self._route_subscriptions = []
        self._next_sequence = {
            route.topic_id: 0 for route in self.tx_routes
        }
        self._counter_lock = threading.Lock()
        self._counters = BridgeCounters()
        self._sequence_tracker = SequenceTracker()
        self._last_sequence_warning = 0.0
        self._last_disconnected_warning = 0.0

        self._parser = StreamParser(
            max_payload=self.max_payload,
            known_topic_ids=TOPIC_REGISTRY.keys(),
        )
        ros_rx_queue_size = int(
            self.get_parameter('ros_rx_queue_size').value
        )
        if ros_rx_queue_size < 1:
            raise ValueError('ros_rx_queue_size must be positive')
        self._rx_frames = queue.Queue(maxsize=ros_rx_queue_size)
        self._serial_events = queue.SimpleQueue()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        for route in self.tx_routes:
            subscription = self.create_subscription(
                route.message_type,
                route.topic_name,
                lambda message, selected=route: self._transmit_message(
                    selected, message
                ),
                qos,
            )
            self._route_subscriptions.append(subscription)
        for route in self.rx_routes:
            self._route_publishers[route.topic_id] = self.create_publisher(
                route.message_type,
                route.topic_name,
                qos,
            )

        reconnect_interval = float(
            self.get_parameter('reconnect_interval').value
        )
        tx_queue_size = int(
            self.get_parameter('serial_tx_queue_size').value
        )
        self._serial_transport = SerialTransport(
            port=self.serial_port,
            baud_rate=self.baud_rate,
            on_bytes=self._receive_serial_bytes,
            on_status=self._queue_serial_status,
            on_tx_success=self._record_tx_success,
            reconnect_interval=reconnect_interval,
            tx_queue_size=tx_queue_size,
        )

        self._rx_timer = self.create_timer(0.01, self._process_rx_queues)
        self._diagnostics_timer = self.create_timer(
            diagnostics_interval, self._log_diagnostics
        )
        self._log_startup_configuration()
        self._serial_transport.start()

    def _resolve_node_id(self, requested):
        if requested == -1:
            return 0 if self.role == ROLE_STATION else 1
        if not 0 <= requested < BROADCAST_NODE_ID:
            raise ValueError('node_id must be in 0..254')
        return requested

    def _resolve_destination_id(self, requested):
        if requested == -1:
            return 1 if self.role == ROLE_STATION else 0
        if not 0 <= requested <= BROADCAST_NODE_ID:
            raise ValueError('destination_node_id must be in 0..255')
        return requested

    def _transmit_message(self, route, message):
        try:
            payload = bytes(serialize_message(message))
            sequence = self._next_sequence[route.topic_id]
            frame = encode_frame(
                topic_id=route.topic_id,
                source_role=role_code(self.role),
                source_node_id=self.node_id,
                destination_node_id=self.destination_node_id,
                sequence=sequence,
                payload=payload,
                max_payload=self.max_payload,
            )
        except Exception as error:
            with self._counter_lock:
                self._counters.serialization_errors += 1
            self.get_logger().error(
                f'cannot serialize {route.topic_name}: {error}'
            )
            return

        if not self._serial_transport.send(frame):
            now = time.monotonic()
            if now - self._last_disconnected_warning >= 5.0:
                self._last_disconnected_warning = now
                self.get_logger().warning(
                    f'dropped {route.topic_name}: serial link is disconnected '
                    'or its TX queue is full'
                )
            return

        self._next_sequence[route.topic_id] = (
            sequence + 1
        ) & 0xFFFFFFFF

    def _receive_serial_bytes(self, data):
        for frame in self._parser.feed(data):
            try:
                self._rx_frames.put_nowait(frame)
            except queue.Full:
                with self._counter_lock:
                    self._counters.rx_queue_dropped += 1

    def _process_rx_queues(self):
        self._drain_serial_events()
        for _ in range(100):
            try:
                frame = self._rx_frames.get_nowait()
            except queue.Empty:
                break
            self._publish_frame(frame)

    def _publish_frame(self, frame):
        if frame.destination_node_id not in (
            self.node_id,
            BROADCAST_NODE_ID,
        ):
            with self._counter_lock:
                self._counters.wrong_destination += 1
            return

        route = TOPIC_REGISTRY.get(frame.topic_id)
        sender_role = CODE_TO_ROLE.get(frame.source_role)
        if (
            route is None
            or self.role not in route.receiver_roles
            or sender_role not in route.sender_roles
        ):
            with self._counter_lock:
                self._counters.invalid_direction += 1
            return

        observation = self._sequence_tracker.observe(
            frame.source_node_id,
            frame.topic_id,
            frame.sequence,
        )
        if observation.status in ('gap', 'duplicate', 'out_of_order'):
            self._warn_sequence(frame, observation)

        try:
            message = deserialize_message(frame.payload, route.message_type)
        except Exception as error:
            with self._counter_lock:
                self._counters.deserialization_errors += 1
            self.get_logger().warning(
                f'discarded invalid {route.topic_name} payload: {error}'
            )
            return

        self._route_publishers[route.topic_id].publish(message)
        with self._counter_lock:
            self._counters.rx_frames += 1
            self._counters.rx_bytes += frame.wire_size

    def _warn_sequence(self, frame, observation):
        now = time.monotonic()
        if now - self._last_sequence_warning < 5.0:
            return
        self._last_sequence_warning = now
        detail = (
            f', missing={observation.missing}'
            if observation.missing else ''
        )
        self.get_logger().warning(
            f'P900 sequence {observation.status}: source='
            f'{frame.source_node_id}, topic={frame.topic_id}, '
            f'sequence={frame.sequence}{detail}'
        )

    def _queue_serial_status(self, level, message):
        self._serial_events.put((level, message))

    def _drain_serial_events(self):
        while True:
            try:
                level, message = self._serial_events.get_nowait()
            except queue.Empty:
                return
            logger_method = getattr(self.get_logger(), level, None)
            if logger_method is None:
                logger_method = self.get_logger().info
            logger_method(f'P900 serial: {message}')

    def _record_tx_success(self, frame_size):
        with self._counter_lock:
            self._counters.tx_frames += 1
            self._counters.tx_bytes += int(frame_size)

    def _log_startup_configuration(self):
        tx_lines = [
            f'  [{route.topic_id}] {route.topic_name}'
            for route in self.tx_routes
        ] or ['  none']
        rx_lines = [
            f'  [{route.topic_id}] {route.topic_name}'
            for route in self.rx_routes
        ] or ['  none']
        self.get_logger().info(
            '\n'.join([
                'P900 Bridge',
                f'Role: {self.role}',
                f'Node ID: {self.node_id}',
                f'Destination: {self.destination_node_id}',
                f'Serial: {self.serial_port} @ {self.baud_rate} (8N1)',
                'TX:',
                *tx_lines,
                'RX:',
                *rx_lines,
            ])
        )

    def _log_diagnostics(self):
        with self._counter_lock:
            counters = BridgeCounters(**vars(self._counters))
        parser = self._parser.stats
        sequence = self._sequence_tracker.stats
        serial_stats = self._serial_transport.stats
        self.get_logger().info(
            'P900 stats: '
            f'connected={self._serial_transport.connected} '
            f'TX={counters.tx_frames} frames/{counters.tx_bytes} bytes '
            f'TX_drop={serial_stats.dropped_tx_frames} '
            f'RX={counters.rx_frames} frames/{counters.rx_bytes} bytes '
            f'CRC={parser.crc_errors} malformed={parser.malformed_frames} '
            f'unknown_topic={parser.unknown_topic_ids} '
            f'codec_error={counters.serialization_errors}/'
            f'{counters.deserialization_errors} '
            f'missing={sequence.missing} duplicate={sequence.duplicates} '
            f'out_of_order={sequence.out_of_order} '
            f'wrong_dst={counters.wrong_destination} '
            f'bad_direction={counters.invalid_direction} '
            f'RX_queue_drop={counters.rx_queue_dropped} '
            f'serial_disconnects={serial_stats.disconnects} '
            f'serial_io_error={serial_stats.read_errors}/'
            f'{serial_stats.write_errors}'
        )

    def stop_serial(self):
        """Stop the worker even if the ROS context already shut down."""
        if hasattr(self, '_serial_transport'):
            self._serial_transport.stop()

    def destroy_node(self):
        """Stop serial I/O before destroying ROS entities."""
        self.stop_serial()
        return super().destroy_node()


def main(args=None):
    """Run the P900 bridge node."""
    rclpy.init(args=args)
    node = None
    try:
        node = P900BridgeNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.destroy_node()
            else:
                node.stop_serial()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
