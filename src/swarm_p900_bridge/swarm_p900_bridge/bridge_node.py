"""ROS 2 integration node for the selected-topic P900 bridge."""

from collections import defaultdict
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
)
from rclpy.serialization import deserialize_message, serialize_message

from swarm_p900_bridge.roles import (
    CODE_TO_ROLE,
    normalize_role,
    role_code,
    ROLE_FOLLOWER,
    ROLE_LEADER,
    ROLE_STATION,
    VALID_ROLE_INPUTS,
)
from swarm_p900_bridge.serial_transport import SerialTransport
from swarm_p900_bridge.state_codec import (
    decode_compact_state,
    encode_compact_state,
)
from swarm_p900_bridge.topic_registry import (
    CODEC_COMPACT_STATE,
    LEADER_NODE_ID,
    routes_received_by,
    routes_sent_by,
    STATION_NODE_ID,
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
    tx_queued_frames: int = 0
    tx_queued_bytes: int = 0
    serialization_errors: int = 0
    rx_frames: int = 0
    rx_bytes: int = 0
    rx_queue_dropped: int = 0
    deserialization_errors: int = 0
    received_for_me: int = 0
    received_broadcast: int = 0
    dropped_not_for_me: int = 0
    dropped_own_source: int = 0
    invalid_source: int = 0
    invalid_direction: int = 0
    loop_suppressed: int = 0
    relayed_frames: int = 0
    relay_dropped: int = 0
    state_samples_replaced: int = 0
    state_samples_dropped: int = 0


@dataclass
class TopicCounters:
    """Per-topic accepted traffic counters."""

    tx_frames: int = 0
    tx_bytes: int = 0
    rx_frames: int = 0
    rx_bytes: int = 0
    relayed_frames: int = 0
    replaced_samples: int = 0


@dataclass(frozen=True)
class PendingLatestFrame:
    """One newest-only state awaiting the 10 Hz transmission tick."""

    route: object
    payload: bytes
    source_role_code: int
    source_node_id: int
    sequence: object
    destination_node_id: int
    relayed: bool


class P900BridgeNode(Node):
    """Bridge selected ROS topics for station, leader or follower roles."""

    def __init__(self):
        super().__init__('p900_bridge')

        self.declare_parameter('role', ROLE_STATION)
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 230400)
        self.declare_parameter('node_id', -1)
        # Retained only for old point-to-point launch commands. New PMP roles
        # get each topic destination from the centralized registry.
        self.declare_parameter('destination_node_id', -1)
        self.declare_parameter('max_payload_bytes', DEFAULT_MAX_PAYLOAD)
        self.declare_parameter('diagnostics_interval', 10.0)
        self.declare_parameter('reconnect_interval', 2.0)
        self.declare_parameter('serial_tx_queue_size', 100)
        self.declare_parameter('ros_rx_queue_size', 100)

        requested_role = str(self.get_parameter('role').value)
        try:
            self.role = normalize_role(requested_role)
        except ValueError as error:
            choices = ', '.join(sorted(VALID_ROLE_INPUTS))
            raise ValueError(
                f'role must be one of {choices}; received {requested_role!r}'
            ) from error

        self.serial_port = str(self.get_parameter('serial_port').value)
        self.baud_rate = int(self.get_parameter('baud_rate').value)
        self.node_id = self._resolve_node_id(
            int(self.get_parameter('node_id').value)
        )
        self.destination_override = self._resolve_destination_override(
            int(self.get_parameter('destination_node_id').value)
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
        self._topic_counters = defaultdict(TopicCounters)
        self._sequence_tracker = SequenceTracker()
        self._last_sequence_warning = 0.0
        self._last_disconnected_warning = 0.0

        # ROS callbacks replace entries in this map. The periodic state timer
        # takes a snapshot, so intermediate high-rate samples never form a FIFO.
        self._latest_lock = threading.Lock()
        self._latest_pending = {}
        self._latest_last_sent = {}

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

        for route in self.tx_routes:
            qos = self._qos_for_route(route)
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
            qos = self._qos_for_route(route)
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
        self._latest_timer = self.create_timer(
            0.02, self._flush_latest_frames
        )
        self._diagnostics_timer = self.create_timer(
            diagnostics_interval, self._log_diagnostics
        )
        self._log_startup_configuration()
        self._serial_transport.start()

    def _resolve_node_id(self, requested):
        if requested == -1:
            defaults = {
                ROLE_STATION: STATION_NODE_ID,
                ROLE_LEADER: LEADER_NODE_ID,
                ROLE_FOLLOWER: 2,
            }
            return defaults[self.role]
        if not 0 <= requested < BROADCAST_NODE_ID:
            raise ValueError('node_id must be in 0..254')
        return requested

    @staticmethod
    def _resolve_destination_override(requested):
        if requested == -1:
            return None
        if not 0 <= requested <= BROADCAST_NODE_ID:
            raise ValueError('destination_node_id must be in 0..255')
        return requested

    @staticmethod
    def _qos_for_route(route):
        """Build the local ROS QoS profile declared by a topic route."""
        return QoSProfile(
            reliability=route.qos_reliability,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=route.qos_depth,
        )

    def _origin_destination(self, route):
        if self.destination_override is not None:
            return self.destination_override
        return route.link_for_sender(self.role).destination_node_id

    def _message_is_local_origin(self, route, message):
        """Reject a peer message that this bridge published into local ROS."""
        if not route.source_id_field:
            return True
        try:
            message_source = int(
                getattr(message, route.source_id_field)
            )
        except (AttributeError, TypeError, ValueError, OverflowError):
            with self._counter_lock:
                self._counters.serialization_errors += 1
            return False
        if message_source == self.node_id:
            return True
        with self._counter_lock:
            self._counters.loop_suppressed += 1
        return False

    def _serialize_route_message(self, route, message):
        if route.codec == CODEC_COMPACT_STATE:
            return encode_compact_state(message)
        return bytes(serialize_message(message))

    def _deserialize_route_message(self, route, frame):
        if route.codec == CODEC_COMPACT_STATE:
            return decode_compact_state(
                frame.payload,
                frame.source_node_id,
                self.get_clock().now().to_msg(),
            )
        return deserialize_message(frame.payload, route.message_type)

    def _transmit_message(self, route, message):
        if not self._message_is_local_origin(route, message):
            return
        try:
            payload = self._serialize_route_message(route, message)
        except Exception as error:
            with self._counter_lock:
                self._counters.serialization_errors += 1
            self.get_logger().error(
                f'cannot encode {route.topic_name}: {error}'
            )
            return

        if route.latest_sample:
            pending = PendingLatestFrame(
                route=route,
                payload=payload,
                source_role_code=role_code(self.role),
                source_node_id=self.node_id,
                sequence=None,
                destination_node_id=self._origin_destination(route),
                relayed=False,
            )
            self._replace_latest(pending)
            return

        self._send_payload(
            route=route,
            payload=payload,
            source_role_code=role_code(self.role),
            source_node_id=self.node_id,
            sequence=None,
            destination_node_id=self._origin_destination(route),
            relayed=False,
            message=message,
        )

    def _replace_latest(self, pending):
        key = (pending.route.topic_id, pending.source_node_id)
        with self._latest_lock:
            replaced = key in self._latest_pending
            self._latest_pending[key] = pending
        if replaced:
            with self._counter_lock:
                self._counters.state_samples_replaced += 1
                self._topic_counters[
                    pending.route.topic_id
                ].replaced_samples += 1

    def _flush_latest_frames(self):
        now = time.monotonic()
        with self._latest_lock:
            ready = []
            for key, state in tuple(self._latest_pending.items()):
                interval = 1.0 / state.route.max_tx_rate_hz
                last_sent = self._latest_last_sent.get(key)
                if last_sent is None or now - last_sent >= interval:
                    ready.append((key, state))
                    del self._latest_pending[key]
        for key, state in ready:
            sent = self._send_payload(
                route=state.route,
                payload=state.payload,
                source_role_code=state.source_role_code,
                source_node_id=state.source_node_id,
                sequence=state.sequence,
                destination_node_id=state.destination_node_id,
                relayed=state.relayed,
                message=None,
                latest=True,
            )
            if sent:
                with self._latest_lock:
                    self._latest_last_sent[key] = now

    def _send_payload(
        self,
        *,
        route,
        payload,
        source_role_code,
        source_node_id,
        sequence,
        destination_node_id,
        relayed,
        message,
        latest=False,
    ):
        own_sequence = sequence is None
        if own_sequence:
            sequence = self._next_sequence[route.topic_id]
        try:
            frame = encode_frame(
                topic_id=route.topic_id,
                source_role=source_role_code,
                source_node_id=source_node_id,
                destination_node_id=destination_node_id,
                sequence=sequence,
                payload=payload,
                max_payload=self.max_payload,
            )
        except Exception as error:
            with self._counter_lock:
                self._counters.serialization_errors += 1
                if relayed:
                    self._counters.relay_dropped += 1
            self.get_logger().error(
                f'cannot frame {route.topic_name}: {error}'
            )
            return False

        serial_replaced = False
        if latest:
            result = self._serial_transport.send_latest(
                frame,
                key=(route.topic_id, source_node_id),
            )
            accepted = result.accepted
            serial_replaced = result.replaced
        else:
            accepted = self._serial_transport.send(frame)

        if not accepted:
            with self._counter_lock:
                if latest:
                    self._counters.state_samples_dropped += 1
                if relayed:
                    self._counters.relay_dropped += 1
            self._warn_tx_drop(route)
            return False

        if own_sequence:
            self._next_sequence[route.topic_id] = (
                sequence + 1
            ) & 0xFFFFFFFF
        with self._counter_lock:
            self._counters.tx_queued_frames += 1
            self._counters.tx_queued_bytes += len(frame)
            topic = self._topic_counters[route.topic_id]
            topic.tx_frames += 1
            topic.tx_bytes += len(frame)
            if serial_replaced:
                self._counters.state_samples_replaced += 1
                topic.replaced_samples += 1
            if relayed:
                self._counters.relayed_frames += 1
                topic.relayed_frames += 1

        self._log_tx_packet(
            route,
            sequence,
            source_node_id,
            destination_node_id,
            payload,
            message,
            relayed,
        )
        return True

    def _warn_tx_drop(self, route):
        now = time.monotonic()
        if now - self._last_disconnected_warning < 5.0:
            return
        self._last_disconnected_warning = now
        self.get_logger().warning(
            f'dropped {route.topic_name}: serial link is disconnected '
            'or its TX queue is full'
        )

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
        if frame.destination_node_id == self.node_id:
            with self._counter_lock:
                self._counters.received_for_me += 1
        elif frame.destination_node_id == BROADCAST_NODE_ID:
            with self._counter_lock:
                self._counters.received_broadcast += 1
        else:
            with self._counter_lock:
                self._counters.dropped_not_for_me += 1
            return

        if frame.source_node_id == self.node_id:
            with self._counter_lock:
                self._counters.dropped_own_source += 1
            return
        if frame.source_node_id == BROADCAST_NODE_ID:
            with self._counter_lock:
                self._counters.invalid_source += 1
            return

        route = TOPIC_REGISTRY.get(frame.topic_id)
        sender_role = CODE_TO_ROLE.get(frame.source_role)
        if (
            route is None
            or sender_role is None
            or not route.accepts(
                sender_role,
                self.role,
                frame.destination_node_id,
            )
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
            message = self._deserialize_route_message(route, frame)
        except Exception as error:
            with self._counter_lock:
                self._counters.deserialization_errors += 1
            self.get_logger().warning(
                f'discarded invalid {route.topic_name} payload: {error}'
            )
            return

        if route.should_leader_relay(
            self.role,
            sender_role,
            frame.destination_node_id,
        ):
            self._relay_frame(route, frame)

        self._log_rx_packet(route, frame, message)
        self._route_publishers[route.topic_id].publish(message)
        with self._counter_lock:
            self._counters.rx_frames += 1
            self._counters.rx_bytes += frame.wire_size
            topic = self._topic_counters[route.topic_id]
            topic.rx_frames += 1
            topic.rx_bytes += frame.wire_size

    def _relay_frame(self, route, frame):
        pending = PendingLatestFrame(
            route=route,
            payload=frame.payload,
            source_role_code=frame.source_role,
            source_node_id=frame.source_node_id,
            sequence=frame.sequence,
            destination_node_id=BROADCAST_NODE_ID,
            relayed=True,
        )
        if route.latest_sample:
            self._replace_latest(pending)
            return
        self._send_payload(
            route=route,
            payload=frame.payload,
            source_role_code=frame.source_role,
            source_node_id=frame.source_node_id,
            sequence=frame.sequence,
            destination_node_id=BROADCAST_NODE_ID,
            relayed=True,
            message=None,
        )

    def _log_tx_packet(
        self,
        route,
        sequence,
        source,
        destination,
        payload,
        message,
        relayed,
    ):
        logger = (
            self.get_logger().info
            if route.log_each_packet else self.get_logger().debug
        )
        data = (
            self._message_summary(message)
            if message is not None else '<encoded relay/state>'
        )
        logger(
            f'ROS/P900 TX: topic=[{route.topic_id}] {route.topic_name}, '
            f'sequence={sequence}, source={source}, '
            f'destination={destination}, payload={len(payload)} bytes, '
            f'relay={relayed}, data={data}'
        )

    def _log_rx_packet(self, route, frame, message):
        logger = (
            self.get_logger().info
            if route.log_each_packet else self.get_logger().debug
        )
        logger(
            f'P900 RX -> ROS PUB: topic=[{frame.topic_id}] '
            f'{route.topic_name}, sequence={frame.sequence}, '
            f'source={frame.source_node_id}, '
            f'destination={frame.destination_node_id}, '
            f'payload={len(frame.payload)} bytes, '
            f'data={self._message_summary(message)}'
        )

    @staticmethod
    def _message_summary(message, max_characters=300):
        """Return a compact one-line representation for packet logging."""
        value = getattr(message, 'data', message)
        summary = repr(value).replace('\n', '\\n')
        if len(summary) > max_characters:
            summary = summary[:max_characters - 3] + '...'
        return summary

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
        tx_lines = []
        for route in self.tx_routes:
            destination = self._origin_destination(route)
            policy = (
                f', latest @ {route.max_tx_rate_hz:g} Hz'
                if route.latest_sample else ''
            )
            tx_lines.append(
                f'  [{route.topic_id}] {route.topic_name} -> '
                f'{destination}{policy}'
            )
        if not tx_lines:
            tx_lines = ['  none']
        rx_lines = [
            f'  [{route.topic_id}] {route.topic_name}'
            for route in self.rx_routes
        ] or ['  none']
        override = (
            'registry'
            if self.destination_override is None
            else f'OVERRIDE {self.destination_override}'
        )
        self.get_logger().info(
            '\n'.join([
                'P900 Bridge',
                f'Role: {self.role}',
                f'Node ID: {self.node_id}',
                f'Destination policy: {override}',
                f'Serial: {self.serial_port} @ {self.baud_rate} (8N1)',
                'TX:',
                *tx_lines,
                'RX:',
                *rx_lines,
            ])
        )
        if self.destination_override is not None:
            self.get_logger().warning(
                'destination_node_id overrides every registry route; leave it '
                'at -1 for PMP operation.'
            )

    def _log_diagnostics(self):
        with self._counter_lock:
            counters = BridgeCounters(**vars(self._counters))
            topics = {
                topic_id: TopicCounters(**vars(values))
                for topic_id, values in self._topic_counters.items()
            }
        parser = self._parser.stats
        sequence = self._sequence_tracker.stats
        serial_stats = self._serial_transport.stats
        topic_parts = []
        for topic_id in sorted(topics):
            values = topics[topic_id]
            route = TOPIC_REGISTRY[topic_id]
            topic_parts.append(
                f'[{topic_id}]{route.topic_name} '
                f'tx={values.tx_frames}/{values.tx_bytes}B '
                f'rx={values.rx_frames}/{values.rx_bytes}B '
                f'relay={values.relayed_frames} '
                f'replaced={values.replaced_samples}'
            )
        topic_summary = '; '.join(topic_parts) or 'none'
        self.get_logger().info(
            'P900 stats: '
            f'connected={self._serial_transport.connected} '
            f'TX_written={counters.tx_frames}/{counters.tx_bytes}B '
            f'TX_queued={counters.tx_queued_frames}/'
            f'{counters.tx_queued_bytes}B '
            f'TX_drop={serial_stats.dropped_tx_frames} '
            f'RX_published={counters.rx_frames}/{counters.rx_bytes}B '
            f'for_me={counters.received_for_me} '
            f'broadcast={counters.received_broadcast} '
            f'not_for_me={counters.dropped_not_for_me} '
            f'own_source={counters.dropped_own_source} '
            f'bad_source={counters.invalid_source} '
            f'bad_direction={counters.invalid_direction} '
            f'loop_suppressed={counters.loop_suppressed} '
            f'relayed={counters.relayed_frames} '
            f'relay_drop={counters.relay_dropped} '
            f'state_replaced={counters.state_samples_replaced} '
            f'state_drop={counters.state_samples_dropped} '
            f'CRC={parser.crc_errors} malformed={parser.malformed_frames} '
            f'unknown_topic={parser.unknown_topic_ids} '
            f'codec_error={counters.serialization_errors}/'
            f'{counters.deserialization_errors} '
            f'missing={sequence.missing} duplicate={sequence.duplicates} '
            f'out_of_order={sequence.out_of_order} '
            f'RX_queue_drop={counters.rx_queue_dropped} '
            f'serial_latest_replaced='
            f'{serial_stats.replaced_latest_frames} '
            f'serial_disconnects={serial_stats.disconnects} '
            f'serial_io_error={serial_stats.read_errors}/'
            f'{serial_stats.write_errors}; topics: {topic_summary}'
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
