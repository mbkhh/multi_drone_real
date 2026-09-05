#!/usr/bin/env python3

"""Bounded, correlation-friendly swarm flight diagnostic logger."""

from __future__ import annotations

import math
import os
import platform
import socket
import sys
import time
from collections import Counter, deque
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
from threading import Lock

import rclpy
from px4_msgs.msg import (
    BatteryStatus,
    FailsafeFlags,
    OffboardControlMode,
    TimesyncStatus,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleCommandAck,
    VehicleLandDetected,
    VehicleLocalPosition,
    VehicleStatus,
)
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.exceptions import ParameterUninitializedException
from rclpy.executors import (
    ExternalShutdownException,
    MultiThreadedExecutor,
)
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Header, String
from swarm_config.config_utils import get_config
from swarm_logger.background_diagnostics import WifiEventMonitor
from swarm_logger.diagnostic_trackers import (
    DDSWriterTracker,
    StampedStreamTracker,
    TransformFrameTracker,
    format_gid,
    stamp_to_nanoseconds,
)
from swarm_logger.health_collector import LocalHealthCollector
from swarm_logger.system_diagnostics import signal_label
from swarm_msgs.msg import FormationCommand, ManualControl, Status
from tf2_msgs.msg import TFMessage

try:
    # MessageInfo delivery to Python subscription callbacks was added after
    # Humble. On Rolling it exposes the DDS writer and sequence information.
    from rclpy.subscription import MessageInfo as RclpyMessageInfo
except ImportError:  # pragma: no cover - exercised on Humble systems
    RclpyMessageInfo = None


SCHEMA_VERSION = 2
_BORDER = '=' * 118
_SECTION = '-' * 118


def _format_optional(value, suffix='', precision=2):
    if value is None:
        return 'N/A'
    if isinstance(value, float):
        if not math.isfinite(value):
            return str(value)
        return f'{value:.{precision}f}{suffix}'
    return f'{value}{suffix}'


def _enum_name(value):
    return str(getattr(value, 'name', value))


def _safe_float(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return math.nan


def _vector(message, name):
    try:
        return tuple(float(value) for value in getattr(message, name))
    except (AttributeError, TypeError, ValueError):
        return ()


class SwarmLogger(Node):
    """Observe TF, DDS, PX4, Wi-Fi, and host health with bounded overhead."""

    def __init__(self):
        # Reports remain local. The launch commands redirect them to a local
        # file, so neither /rosout nor an SSH terminal carries the output.
        super().__init__('swarm_logger', enable_rosout=False)

        default_drone_count = int(get_config('swarm_sim.drone_count'))
        self.declare_parameter('drone_count', default_drone_count)
        self.declare_parameter('drone_ids', Parameter.Type.INTEGER_ARRAY)
        self.declare_parameter('local_drone_id', 0)
        self.declare_parameter('print_interval', 5.0)
        self.declare_parameter('wifi_interface', 'wlan0')
        self.declare_parameter('monitor_vehicle_topics', False)
        self.declare_parameter('monitor_tf', False)
        self.declare_parameter('tf_qos_depth', 32)
        self.declare_parameter('tf_stale_warn_age', 0.5)
        self.declare_parameter('tf_gap_warn_threshold', 0.25)
        self.declare_parameter('tf_startup_grace', 5.0)
        self.declare_parameter('max_tf_frames', 64)
        self.declare_parameter('monitor_wifi_events', True)
        self.declare_parameter('probe_timeout', 0.4)
        self.declare_parameter('controller_process_match', 'control_node')
        self.declare_parameter('monitor_microxrce_agent', True)

        self.drone_count = self._integer_parameter('drone_count')
        try:
            self.drone_ids = list(
                self.get_parameter('drone_ids')
                .get_parameter_value().integer_array_value
            )
        except ParameterUninitializedException:
            self.drone_ids = []
        self.local_drone_id = self._integer_parameter('local_drone_id')
        self.print_interval = self._double_parameter('print_interval')
        self.wifi_interface = self._string_parameter('wifi_interface')
        self.monitor_vehicle_topics = self._bool_parameter(
            'monitor_vehicle_topics'
        )
        self.monitor_tf = self._bool_parameter('monitor_tf')
        self.tf_qos_depth = self._integer_parameter('tf_qos_depth')
        self.tf_stale_warn_age = self._double_parameter(
            'tf_stale_warn_age'
        )
        self.tf_gap_warn_threshold = self._double_parameter(
            'tf_gap_warn_threshold'
        )
        self.tf_startup_grace = self._double_parameter('tf_startup_grace')
        self.max_tf_frames = self._integer_parameter('max_tf_frames')
        self.monitor_wifi_events = self._bool_parameter(
            'monitor_wifi_events'
        )
        self.probe_timeout = self._double_parameter('probe_timeout')
        self.controller_process_match = self._string_parameter(
            'controller_process_match'
        )
        self.monitor_microxrce_agent = self._bool_parameter(
            'monitor_microxrce_agent'
        )
        self._validate_parameters()

        if not self.drone_ids:
            self.drone_ids = list(range(1, self.drone_count + 1))
        if self.local_drone_id == 0 and len(self.drone_ids) == 1:
            self.local_drone_id = self.drone_ids[0]

        configured_heartbeat_period = get_config(
            'swarm_single.broadcast_interval'
        )
        self.heartbeat_period = float(configured_heartbeat_period or 1.5)
        self.heartbeat_warn_age = max(2.5 * self.heartbeat_period, 2.0)
        self.neighbor_timeout = 4.0 * self.heartbeat_period
        self.px4_model = str(get_config('swarm_sim.px4_model') or 'x500')
        self.goal_frame = str(
            get_config('swarm_single.goal_frame_name') or 'goal'
        )
        expected_ids = range(1, self.drone_count + 1)
        self._expected_odom_frames = {
            f'{self.px4_model}_{drone_id}/odom'
            for drone_id in expected_ids
        }
        self._expected_goal_frames = {
            f'{self.px4_model}_{drone_id}/{self.goal_frame}'
            for drone_id in expected_ids
        }
        self._expected_tf_frames = (
            self._expected_odom_frames | self._expected_goal_frames
        )

        self._lock = Lock()
        self._topic_trackers = {}
        self._heartbeat_trackers = {}
        self._status_leader_counts = Counter()
        self._last_status = None
        self._tf_frames = {}
        self._tf_writers = {}
        self._tf_untracked = Counter()
        self._vehicle_latest = {}
        self._vehicle_window_counts = Counter()
        self._vehicle_window_maxima = {}
        self._recent_events = deque(maxlen=100)
        self._monitored_topics = []
        self._subscriptions = []
        self._critical_vehicle_topics = set()
        self._endpoint_signatures = {}

        self._started_monotonic = time.monotonic()
        self._last_report_time = self._started_monotonic
        self._report_number = 0
        self._closed = False

        self._tf_callback_group = MutuallyExclusiveCallbackGroup()
        self._report_callback_group = MutuallyExclusiveCallbackGroup()
        self._px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self._ros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=8,
        )
        self._tf_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=self.tf_qos_depth,
        )

        self._add_swarm_topics()
        if self.monitor_vehicle_topics:
            self._add_vehicle_topics()
        if self.monitor_tf:
            tf_callback = (
                self._tf_callback_with_info
                if RclpyMessageInfo is not None
                else self._tf_callback
            )
            self._subscribe(
                TFMessage,
                '/tf',
                self._tf_qos,
                tf_callback,
                callback_group=self._tf_callback_group,
            )

        self._health_collector = LocalHealthCollector(
            self.wifi_interface,
            probe_timeout=self.probe_timeout,
            controller_process_match=self.controller_process_match,
            monitor_microxrce_agent=self.monitor_microxrce_agent,
        )
        self._health_executor = ThreadPoolExecutor(
            max_workers=1,
            thread_name_prefix='swarm-logger-health',
        )
        # Prime cumulative Wi-Fi, kernel, and process baselines immediately so
        # the first flight report covers its complete first window.
        self._health_future = self._health_executor.submit(
            self._health_collector.collect
        )
        self._health_prime = True
        self._health_prime_error = None
        self._health_prime_overruns = 0
        self._pending_report = None

        self._wifi_event_monitor = None
        if self.monitor_wifi_events:
            self._wifi_event_monitor = WifiEventMonitor(self.wifi_interface)
            self._wifi_event_monitor.start()

        self.create_timer(
            self.print_interval,
            self._report,
            callback_group=self._report_callback_group,
        )
        self.create_timer(
            0.2,
            self._flush_pending_report,
            callback_group=self._report_callback_group,
        )
        self.get_logger().info(
            'Diagnostic logger active: local_drone=%s, vehicle IDs=%s, '
            'TF=%s, MessageInfo=%s, report=%.1f s, /rosout disabled.'
            % (
                self.local_drone_id or 'unspecified',
                self.drone_ids,
                self.monitor_tf,
                RclpyMessageInfo is not None,
                self.print_interval,
            )
        )

    def _integer_parameter(self, name):
        return self.get_parameter(name).get_parameter_value().integer_value

    def _double_parameter(self, name):
        return self.get_parameter(name).get_parameter_value().double_value

    def _string_parameter(self, name):
        return self.get_parameter(name).get_parameter_value().string_value

    def _bool_parameter(self, name):
        return self.get_parameter(name).get_parameter_value().bool_value

    def _validate_parameters(self):
        if self.drone_count < 1:
            raise ValueError('drone_count must be positive')
        if self.print_interval < 1.0:
            raise ValueError('print_interval must be at least 1 second')
        if any(drone_id < 1 for drone_id in self.drone_ids):
            raise ValueError('drone_ids must contain positive IDs')
        if self.local_drone_id < 0:
            raise ValueError('local_drone_id cannot be negative')
        if self.tf_qos_depth < 1 or self.max_tf_frames < 1:
            raise ValueError('TF depth/frame limits must be positive')
        if self.tf_stale_warn_age <= 0.0:
            raise ValueError('tf_stale_warn_age must be positive')
        if self.tf_gap_warn_threshold <= 0.0:
            raise ValueError('tf_gap_warn_threshold must be positive')
        if self.tf_startup_grace < 0.0:
            raise ValueError('tf_startup_grace cannot be negative')
        if not 0.05 <= self.probe_timeout <= 5.0:
            raise ValueError('probe_timeout must be between 0.05 and 5 seconds')

    def _subscribe(
        self,
        message_type,
        topic,
        qos,
        callback=None,
        callback_group=None,
    ):
        if callback is None:

            def callback(message, topic_name=topic):
                self._generic_callback(topic_name, message)

        subscription = self.create_subscription(
            message_type,
            topic,
            callback,
            qos,
            callback_group=callback_group,
        )
        self._subscriptions.append(subscription)
        if topic not in self._monitored_topics:
            self._monitored_topics.append(topic)

    def _add_swarm_topics(self):
        self._subscribe(
            Status, '/swarm/status', self._ros_qos, self._status_callback
        )
        self._subscribe(
            Header, '/swarm/live', self._ros_qos, self._heartbeat_callback
        )
        topics = (
            (String, '/swarm/command'),
            (FormationCommand, '/swarm/formation_command'),
            (ManualControl, '/manual_controller'),
            (String, '/command'),
            (FormationCommand, '/formation'),
        )
        for message_type, topic in topics:

            def callback(message, topic_name=topic):
                self._swarm_control_callback(topic_name, message)

            self._subscribe(
                message_type, topic, self._ros_qos, callback
            )

    def _add_vehicle_topics(self):
        px4_topics = (
            (VehicleStatus, 'out/vehicle_status_v1', 'status'),
            (
                VehicleLocalPosition,
                'out/vehicle_local_position_v1',
                'local_position',
            ),
            (VehicleLandDetected, 'out/vehicle_land_detected', 'land'),
            (BatteryStatus, 'out/battery_status_v1', 'battery'),
            (FailsafeFlags, 'out/failsafe_flags', 'failsafe_flags'),
            (VehicleCommandAck, 'out/vehicle_command_ack', 'command_ack'),
            (TimesyncStatus, 'out/timesync_status', 'timesync'),
            (VehicleCommand, 'in/vehicle_command', 'vehicle_command'),
            (
                TrajectorySetpoint,
                'in/trajectory_setpoint',
                'trajectory_setpoint',
            ),
            (
                OffboardControlMode,
                'in/offboard_control_mode',
                'offboard_control_mode',
            ),
        )
        critical_kinds = {
            'local_position',
            'trajectory_setpoint',
            'offboard_control_mode',
        }
        for drone_id in self.drone_ids:
            namespace = f'/uav_{drone_id}/fmu'
            for message_type, suffix, kind in px4_topics:
                topic = f'{namespace}/{suffix}'

                def callback(
                    message,
                    topic_name=topic,
                    vehicle_id=drone_id,
                    semantic_kind=kind,
                ):
                    self._vehicle_callback(
                        topic_name, vehicle_id, semantic_kind, message
                    )

                self._subscribe(
                    message_type, topic, self._px4_qos, callback
                )
                if kind in critical_kinds:
                    self._critical_vehicle_topics.add(topic)

    @staticmethod
    def _source_stamp(message):
        header = getattr(message, 'header', None)
        if header is not None and hasattr(header, 'stamp'):
            return stamp_to_nanoseconds(header.stamp), True
        if hasattr(message, 'stamp'):
            return stamp_to_nanoseconds(message.stamp), True
        if hasattr(message, 'timestamp'):
            try:
                return int(message.timestamp) * 1000, True
            except (TypeError, ValueError):
                pass
        return None, False

    def _record_locked(
        self,
        topic,
        source_stamp_ns,
        observed_monotonic,
        observed_wall_ns=None,
    ):
        tracker = self._topic_trackers.setdefault(
            topic, StampedStreamTracker()
        )
        if source_stamp_ns is None:
            tracker.observe_arrival(observed_monotonic)
        else:
            tracker.observe(
                source_stamp_ns,
                observed_monotonic,
                observed_wall_ns,
            )

    def _generic_callback(self, topic, message):
        observed = time.monotonic()
        wall_ns = time.time_ns()
        stamp_ns, wall_clock = self._source_stamp(message)
        with self._lock:
            self._record_locked(
                topic,
                stamp_ns,
                observed,
                wall_ns if wall_clock else None,
            )

    def _event_locked(self, category, text, observed=None):
        self._recent_events.append(
            (observed if observed is not None else time.monotonic(), category, text)
        )

    def _swarm_control_callback(self, topic, message):
        observed = time.monotonic()
        wall_ns = time.time_ns()
        stamp_ns, wall_clock = self._source_stamp(message)
        if isinstance(message, String):
            summary = f'data={message.data!r}'
        elif isinstance(message, FormationCommand):
            summary = (
                f'pattern={message.pattern_name!r} spacing={message.spacing:.2f} '
                f'rotation=({message.rotation_x:.2f},{message.rotation_y:.2f},'
                f'{message.rotation_z:.2f})'
            )
        elif isinstance(message, ManualControl):
            summary = (
                f'manual={message.manual_mode} velocity=({message.vx:.2f},'
                f'{message.vy:.2f},{message.vz:.2f}) yaw={message.vyaw:.2f}'
            )
        else:
            summary = type(message).__name__
        with self._lock:
            self._record_locked(
                topic,
                stamp_ns,
                observed,
                wall_ns if wall_clock else None,
            )
            self._event_locked('command', f'{topic}: {summary}', observed)

    def _heartbeat_callback(self, message):
        observed = time.monotonic()
        wall_ns = time.time_ns()
        frame_id = str(message.frame_id).strip()
        source_stamp_ns = stamp_to_nanoseconds(message.stamp)
        with self._lock:
            # The topic-wide stream interleaves independent publisher clocks;
            # timestamp progression is meaningful only in the per-peer tracker.
            self._record_locked('/swarm/live', None, observed)
            tracker = self._heartbeat_trackers.setdefault(
                frame_id, StampedStreamTracker()
            )
            relation = tracker.observe(source_stamp_ns, observed, wall_ns)
            if not frame_id:
                self._event_locked(
                    'heartbeat', 'received empty heartbeat frame_id', observed
                )
            if relation in ('regressed', 'zero'):
                self._event_locked(
                    'heartbeat',
                    f'peer={frame_id or "<empty>"} stamp={relation}',
                    observed,
                )

    def _status_callback(self, message):
        observed = time.monotonic()
        wall_ns = time.time_ns()
        snapshot = {
            'leader_id': int(message.leader_id),
            'members': tuple(sorted(int(item) for item in message.swarm_members)),
            'state': str(message.control_state),
            'armed': bool(message.armed),
            'offboard': bool(message.offboard),
            'pattern': str(message.pattern_name),
            'spacing': float(message.spacing),
            'leader_position': (
                float(message.leader_x),
                float(message.leader_y),
                float(message.leader_z),
            ),
            'goal': (
                float(message.goal_x),
                float(message.goal_y),
                float(message.goal_z),
            ),
            'message': str(message.message),
            'mission': (
                bool(message.mission_active),
                int(message.mission_index),
                int(message.mission_count),
                str(message.mission_state),
            ),
            'observed_at': observed,
        }
        with self._lock:
            self._record_locked(
                '/swarm/status', int(message.timestamp) * 1000, observed, wall_ns
            )
            previous = self._last_status
            self._status_leader_counts[snapshot['leader_id']] += 1
            self._last_status = snapshot
            if previous is not None:
                changes = []
                for field in ('leader_id', 'members', 'state', 'armed', 'offboard'):
                    if previous.get(field) != snapshot.get(field):
                        changes.append(
                            f'{field}={previous.get(field)!r}->{snapshot.get(field)!r}'
                        )
                if changes:
                    self._event_locked(
                        'swarm_status', ', '.join(changes), observed
                    )
            if snapshot['message']:
                self._event_locked(
                    'swarm_message', snapshot['message'][:300], observed
                )

    def _tf_callback(self, message):
        self._handle_tf(message, None)

    def _tf_callback_with_info(self, message, message_info):
        self._handle_tf(message, message_info)

    def _handle_tf(self, message, message_info):
        observed = time.monotonic()
        wall_ns = time.time_ns()
        transforms = tuple(message.transforms)
        source_stamps = [
            stamp_to_nanoseconds(transform.header.stamp)
            for transform in transforms
        ]
        batch_stamp_ns = max(source_stamps, default=None)
        writer_gid = format_gid(
            getattr(message_info, 'publisher_gid', None)
        )
        publication_sequence = getattr(
            message_info, 'publication_sequence_number', 0
        )
        source_timestamp = getattr(message_info, 'source_timestamp', 0)
        received_timestamp = getattr(message_info, 'received_timestamp', 0)
        children = tuple(
            str(transform.child_frame_id).strip()
            for transform in transforms
        )

        with self._lock:
            self._record_locked('/tf', batch_stamp_ns, observed, wall_ns)
            writer = self._tf_writers.get(writer_gid)
            if writer is None and len(self._tf_writers) < 32:
                writer = DDSWriterTracker()
                self._tf_writers[writer_gid] = writer
            if writer is not None:
                writer.observe(
                    publication_sequence,
                    source_timestamp,
                    received_timestamp,
                    observed,
                    wall_ns,
                    children,
                )
            else:
                self._tf_untracked['writer_limit_messages'] += 1

            for transform in transforms:
                child = str(transform.child_frame_id).strip()
                key = child or '<empty>'
                tracker = self._tf_frames.get(key)
                if tracker is None and (
                    key in self._expected_tf_frames
                    or len(self._tf_frames) < self.max_tf_frames
                ):
                    tracker = TransformFrameTracker()
                    self._tf_frames[key] = tracker
                if tracker is None:
                    self._tf_untracked['frame_limit_transforms'] += 1
                    self._tf_untracked[f'child:{key[:80]}'] += 1
                    continue
                anomalies = tracker.observe(
                    transform, writer_gid, observed, wall_ns
                )
                for anomaly in anomalies:
                    self._event_locked(
                        'tf_anomaly', f'{key}: {anomaly}', observed
                    )

    def _vehicle_callback(self, topic, drone_id, kind, message):
        observed = time.monotonic()
        # PX4 timestamps are microseconds on the PX4 monotonic clock, not Unix
        # wall time. Their progression is useful, but wall-clock offset is not.
        try:
            stamp_ns = int(message.timestamp) * 1000
        except (AttributeError, TypeError, ValueError):
            stamp_ns = 0
        with self._lock:
            self._record_locked(topic, stamp_ns, observed)
            self._record_vehicle_semantics_locked(
                drone_id, kind, message, observed
            )

    def _record_vehicle_semantics_locked(
        self, drone_id, kind, message, observed
    ):
        key = (int(drone_id), kind)
        previous = self._vehicle_latest.get(key, {})

        if kind == 'status':
            snapshot = {
                'arming_state': int(message.arming_state),
                'nav_state': int(message.nav_state),
                'nav_intention': int(message.nav_state_user_intention),
                'failsafe': bool(message.failsafe),
                'gcs_lost': bool(message.gcs_connection_lost),
                'high_latency_lost': bool(message.high_latency_data_link_lost),
                'preflight_ok': bool(message.pre_flight_checks_pass),
                'power_input_valid': bool(message.power_input_valid),
            }
            transition_fields = (
                'arming_state', 'nav_state', 'nav_intention', 'failsafe',
                'gcs_lost', 'high_latency_lost', 'preflight_ok',
            )
        elif kind == 'failsafe_flags':
            fields = message.get_fields_and_field_types()
            active = tuple(
                sorted(
                    name
                    for name, field_type in fields.items()
                    if field_type == 'boolean'
                    and not name.startswith('mode_req_')
                    and bool(getattr(message, name))
                )
            )
            snapshot = {
                'active': active,
                'battery_warning': int(message.battery_warning),
            }
            transition_fields = ('active', 'battery_warning')
        elif kind == 'local_position':
            position = (
                float(message.x), float(message.y), float(message.z)
            )
            velocity = (
                float(message.vx), float(message.vy), float(message.vz)
            )
            snapshot = {
                'position': position,
                'velocity': velocity,
                'finite': all(
                    math.isfinite(value) for value in position + velocity
                ),
                'valid': bool(
                    message.xy_valid
                    and message.z_valid
                    and message.v_xy_valid
                    and message.v_z_valid
                ),
                'dead_reckoning': bool(message.dead_reckoning),
                'eph': float(message.eph),
                'epv': float(message.epv),
                'sample_timestamp_us': int(message.timestamp_sample),
                'reset_counters': (
                    int(message.xy_reset_counter),
                    int(message.z_reset_counter),
                    int(message.vxy_reset_counter),
                    int(message.vz_reset_counter),
                    int(message.heading_reset_counter),
                ),
            }
            transition_fields = (
                'finite', 'valid', 'dead_reckoning', 'reset_counters'
            )
            if not snapshot['finite']:
                self._vehicle_window_counts[(drone_id, 'position_nonfinite')] += 1
        elif kind == 'trajectory_setpoint':
            velocity = _vector(message, 'velocity')
            finite_velocity = bool(velocity) and all(
                math.isfinite(value) for value in velocity
            )
            speed = (
                math.sqrt(sum(value * value for value in velocity))
                if finite_velocity
                else None
            )
            is_zero = bool(speed is not None and speed < 1e-4)
            snapshot = {
                'velocity': velocity,
                'finite_velocity': finite_velocity,
                'speed': speed,
                'zero': is_zero,
                'yaw': float(message.yaw),
                'yawspeed': float(message.yawspeed),
            }
            transition_fields = ('finite_velocity', 'zero')
            if speed is not None:
                self._update_vehicle_maximum(
                    (drone_id, 'setpoint_speed'), speed
                )
            old_velocity = previous.get('velocity', ())
            if (
                finite_velocity
                and len(old_velocity) == len(velocity)
                and all(math.isfinite(value) for value in old_velocity)
            ):
                delta = math.sqrt(
                    sum(
                        (new - old) ** 2
                        for new, old in zip(velocity, old_velocity)
                    )
                )
                self._update_vehicle_maximum(
                    (drone_id, 'setpoint_velocity_step'), delta
                )
            if not finite_velocity:
                self._vehicle_window_counts[(drone_id, 'setpoint_nonfinite')] += 1
            if is_zero:
                self._vehicle_window_counts[(drone_id, 'zero_setpoints')] += 1
        elif kind == 'offboard_control_mode':
            enabled = tuple(
                name
                for name in (
                    'position', 'velocity', 'acceleration', 'attitude',
                    'body_rate', 'thrust_and_torque', 'direct_actuator',
                )
                if bool(getattr(message, name, False))
            )
            snapshot = {'enabled': enabled}
            transition_fields = ('enabled',)
        elif kind == 'battery':
            snapshot = {
                'connected': bool(message.connected),
                'remaining': float(message.remaining),
                'voltage_v': float(message.voltage_v),
                'current_a': float(message.current_a),
                'temperature': float(message.temperature),
                'warning': int(message.warning),
                'faults': int(message.faults),
            }
            transition_fields = ('connected', 'warning', 'faults')
        elif kind == 'land':
            snapshot = {
                'landed': bool(message.landed),
                'ground_contact': bool(message.ground_contact),
                'maybe_landed': bool(message.maybe_landed),
                'freefall': bool(message.freefall),
                'at_rest': bool(message.at_rest),
            }
            transition_fields = tuple(snapshot)
        elif kind == 'vehicle_command':
            snapshot = {
                'command': int(message.command),
                'target': (
                    int(message.target_system),
                    int(message.target_component),
                ),
                'params': tuple(
                    float(getattr(message, f'param{index}'))
                    for index in range(1, 8)
                ),
                'from_external': bool(message.from_external),
            }
            transition_fields = ()
            self._event_locked(
                'px4_command',
                f'drone={drone_id} command={snapshot["command"]} '
                f'target={snapshot["target"]}',
                observed,
            )
        elif kind == 'command_ack':
            snapshot = {
                'command': int(message.command),
                'result': int(message.result),
                'result_param1': int(message.result_param1),
                'result_param2': int(message.result_param2),
            }
            transition_fields = ()
            self._event_locked(
                'px4_ack',
                f'drone={drone_id} command={snapshot["command"]} '
                f'result={snapshot["result"]} params='
                f'{snapshot["result_param1"]}/{snapshot["result_param2"]}',
                observed,
            )
        elif kind == 'timesync':
            snapshot = {
                'source_protocol': int(message.source_protocol),
                'observed_offset_us': int(message.observed_offset),
                'estimated_offset_us': int(message.estimated_offset),
                'round_trip_time_us': int(message.round_trip_time),
                'remote_timestamp_us': int(message.remote_timestamp),
            }
            transition_fields = ('source_protocol',)
            self._update_vehicle_maximum(
                (drone_id, 'timesync_rtt_us'),
                abs(snapshot['round_trip_time_us']),
            )
        else:
            snapshot = {'message_type': type(message).__name__}
            transition_fields = ()

        snapshot['observed_at'] = observed
        self._vehicle_latest[key] = snapshot
        if previous:
            changes = []
            for field in transition_fields:
                if previous.get(field) != snapshot.get(field):
                    changes.append(
                        f'{field}={previous.get(field)!r}->{snapshot.get(field)!r}'
                    )
            if changes:
                self._event_locked(
                    'px4_transition',
                    f'drone={drone_id} {kind}: ' + ', '.join(changes),
                    observed,
                )

    def _update_vehicle_maximum(self, key, value):
        previous = self._vehicle_window_maxima.get(key)
        if previous is None or value > previous:
            self._vehicle_window_maxima[key] = value

    def _snapshot_ros_state(self, now):
        with self._lock:
            snapshot = {
                'topics': {
                    topic: tracker.snapshot(now)
                    for topic, tracker in self._topic_trackers.items()
                },
                'heartbeats': {
                    frame_id: tracker.snapshot(now)
                    for frame_id, tracker in self._heartbeat_trackers.items()
                },
                'leader_counts': dict(self._status_leader_counts),
                'status': (
                    None if self._last_status is None else dict(self._last_status)
                ),
                'tf_frames': {
                    child: tracker.snapshot(now)
                    for child, tracker in self._tf_frames.items()
                },
                'tf_writers': {
                    gid: tracker.snapshot(now)
                    for gid, tracker in self._tf_writers.items()
                },
                'tf_untracked': dict(self._tf_untracked),
                'vehicle_latest': {
                    key: dict(value)
                    for key, value in self._vehicle_latest.items()
                },
                'vehicle_counts': dict(self._vehicle_window_counts),
                'vehicle_maxima': dict(self._vehicle_window_maxima),
                'events': tuple(self._recent_events),
            }
            self._status_leader_counts.clear()
            self._tf_untracked.clear()
            self._vehicle_window_counts.clear()
            self._vehicle_window_maxima.clear()
            self._recent_events.clear()
        return snapshot

    @staticmethod
    def _qos_text(qos):
        if qos is None:
            return 'unknown'
        return '%s/%s/%s(depth=%s)' % (
            _enum_name(getattr(qos, 'reliability', '?')),
            _enum_name(getattr(qos, 'durability', '?')),
            _enum_name(getattr(qos, 'history', '?')),
            getattr(qos, 'depth', '?'),
        )

    def _endpoint_snapshot(self, topics):
        snapshot = {}
        for topic in topics:
            endpoints = []
            try:
                infos = self.get_publishers_info_by_topic(topic)
            except Exception as error:  # noqa: B902 - RMW diagnostic boundary
                snapshot[topic] = {
                    'error': f'{type(error).__name__}: {error}',
                    'endpoints': (),
                }
                continue
            for info in infos:
                namespace = str(getattr(info, 'node_namespace', '')).rstrip('/')
                node_name = str(getattr(info, 'node_name', 'unknown'))
                node_path = f'{namespace}/{node_name}' if namespace else node_name
                endpoints.append({
                    'gid': format_gid(getattr(info, 'endpoint_gid', None)),
                    'node': node_path,
                    'type': str(getattr(info, 'topic_type', 'unknown')),
                    'qos': self._qos_text(getattr(info, 'qos_profile', None)),
                })
            endpoints.sort(key=lambda item: (item['node'], item['gid']))
            snapshot[topic] = {'error': None, 'endpoints': tuple(endpoints)}
        return snapshot

    def _append_metadata_lines(
        self, lines, elapsed, now, report_epoch_ns, report_utc
    ):
        try:
            rmw_identifier = rclpy.get_rmw_implementation_identifier()
        except Exception as error:  # noqa: B902 - diagnostic boundary
            rmw_identifier = f'unavailable:{type(error).__name__}'
        lines.extend((
            f' identity: schema={SCHEMA_VERSION} report={self._report_number} '
            f'drone={self.local_drone_id or "unspecified"} host={socket.gethostname()}',
            f' clocks: utc={report_utc} epoch_ns={report_epoch_ns} '
            f'monotonic={now:.6f} node_uptime={now - self._started_monotonic:.2f}s '
            f'window={elapsed:.3f}s',
            f' runtime: ROS_DISTRO={os.environ.get("ROS_DISTRO", "unset")} '
            f'RMW={rmw_identifier} RMW_IMPLEMENTATION='
            f'{os.environ.get("RMW_IMPLEMENTATION", "unset")} '
            f'DOMAIN={os.environ.get("ROS_DOMAIN_ID", "0(default)")} '
            f'LOCALHOST_ONLY={os.environ.get("ROS_LOCALHOST_ONLY", "unset")}',
            f' software: python={platform.python_version()} executable={sys.executable} '
            f'message_info={RclpyMessageInfo is not None} pid={os.getpid()}',
        ))

    def _append_health_lines(self, lines, health, health_age, warnings):
        lines.append(' LOCAL HOST / POWER / SCHEDULING')
        if health is None:
            detail = 'background health collection returned no sample'
            lines.append(f'  health sample unavailable: {detail}')
            warnings.append(f'host health unavailable: {detail}')
            return
        lines.append(
            '  sample: age=%ss collection=%ss source_window=%ss host=%s '
            'kernel=%s machine=%s uptime=%ss boot_id=%s'
            % (
                _format_optional(health_age, precision=2),
                _format_optional(health['collection_duration_s'], precision=3),
                _format_optional(health['window_s'], precision=2),
                health['hostname'],
                health['kernel'],
                health['machine'],
                _format_optional(health['uptime_s'], precision=0),
                health['boot_id'],
            )
        )
        if health_age is not None and health_age > 2.0 * self.print_interval:
            warnings.append(f'background health sample stale: {health_age:.2f}s')

        cpu = health['cpu_percent']
        aggregate = cpu.get('cpu')
        per_core = sorted(
            (name, value) for name, value in cpu.items() if name != 'cpu'
        )
        hottest_core = max(per_core, key=lambda item: item[1], default=None)
        load = health['load']
        load_text = (
            'N/A' if load is None else '%.2f/%.2f/%.2f' % load
        )
        memory = health['memory']
        memory_percent = None if memory is None else memory['used_percent']
        pressure = health['pressure']
        lines.append(
            '  resources: CPU=%s%% max_core=%s load=%s CPUs=%d memory=%s%% '
            'temp=%sC PSI10(cpu/mem/io)=%s/%s/%s'
            % (
                _format_optional(aggregate),
                'N/A' if hottest_core is None else '%s=%.1f%%' % hottest_core,
                load_text,
                health['cpu_count'],
                _format_optional(memory_percent),
                _format_optional(health['temperature_c']),
                _format_optional(pressure['cpu'].get('some_avg10')),
                _format_optional(pressure['memory'].get('some_avg10')),
                _format_optional(pressure['io'].get('some_avg10')),
            )
        )
        if aggregate is not None and aggregate >= 90.0:
            warnings.append(f'host CPU saturation: {aggregate:.1f}%')
        if hottest_core is not None and hottest_core[1] >= 95.0:
            warnings.append(
                f'CPU core saturation: {hottest_core[0]}={hottest_core[1]:.1f}%'
            )
        if load is not None and load[0] > health['cpu_count']:
            warnings.append(
                f'load average exceeds CPU count: {load[0]:.2f}'
            )
        if memory_percent is not None and memory_percent >= 90.0:
            warnings.append(f'memory pressure: {memory_percent:.1f}% used')
        if (
            health['temperature_c'] is not None
            and health['temperature_c'] >= 80.0
        ):
            warnings.append(
                f'high Raspberry Pi temperature: {health["temperature_c"]:.1f}C'
            )
        for resource, values in pressure.items():
            if values.get('some_avg10', 0.0) >= 20.0:
                warnings.append(
                    f'{resource} PSI pressure={values["some_avg10"]:.1f}%'
                )

        wifi = health['wifi']
        throttled = wifi['throttled_value']
        if throttled is None:
            throttle_text = 'unavailable'
        elif throttled == 0:
            throttle_text = '0x0 (no current/historical flags)'
        else:
            throttle_text = '0x%x (%s)' % (
                throttled, ', '.join(wifi['throttled_reasons'])
            )
            warnings.append(f'Raspberry Pi throttle flags: {throttle_text}')
        lines.append(f'  Raspberry Pi throttle: {throttle_text}')

        interface = health['interface']
        metadata = interface['metadata']
        link = wifi['link']
        lines.append(
            '  interface: name=%s state=%s carrier=%s MAC=%s MTU=%s driver=%s '
            'tx_queue_len=%s'
            % (
                interface['name'], interface['state'], interface['carrier'],
                metadata.get('address', 'N/A'), metadata.get('mtu', 'N/A'),
                metadata.get('driver', 'N/A'),
                metadata.get('tx_queue_len', 'N/A'),
            )
        )
        lines.append(
            '  Wi-Fi link: probe=%s connected=%s SSID=%s BSSID=%s freq=%sMHz '
            'signal=%sdBm(%s) tx/rx=%s/%sMbit/s power_save=%s'
            % (
                link['probe_status'], link['connected'], link.get('ssid') or 'N/A',
                link.get('bssid') or 'N/A',
                _format_optional(link.get('frequency_mhz'), precision=0),
                _format_optional(link.get('signal_dbm')),
                signal_label(link.get('signal_dbm')),
                _format_optional(link.get('tx_bitrate_mbps')),
                _format_optional(link.get('rx_bitrate_mbps')),
                _format_optional(wifi['power_save']),
            )
        )
        if link['probe_status'] == 'ok' and not link['connected']:
            warnings.append('Wi-Fi is not associated according to iw')
        if wifi['power_save'] is True:
            warnings.append('Wi-Fi power saving is enabled')
        if wifi['bssid_changed']:
            warnings.append('Wi-Fi BSSID changed (roam/reassociation)')
        if link.get('signal_dbm') is not None and link['signal_dbm'] < -75.0:
            warnings.append(f'weak Wi-Fi signal: {link["signal_dbm"]:.1f}dBm')

        net_delta = interface['net_delta']
        net_window = max(health['window_s'], 1e-6)
        if net_delta:
            rx_mbps = 8.0 * net_delta.get('rx_bytes', 0) / net_window / 1e6
            tx_mbps = 8.0 * net_delta.get('tx_bytes', 0) / net_window / 1e6
            lines.append(
                '  kernel interface delta: rx/tx=%.3f/%.3fMbit/s '
                'packets=%d/%d drops=%d/%d errors=%d/%d resets=%s'
                % (
                    rx_mbps, tx_mbps,
                    net_delta.get('rx_packets', 0),
                    net_delta.get('tx_packets', 0),
                    net_delta.get('rx_dropped', 0),
                    net_delta.get('tx_dropped', 0),
                    net_delta.get('rx_errors', 0),
                    net_delta.get('tx_errors', 0),
                    interface['net_resets'] or 'none',
                )
            )
            interface_failures = sum(
                net_delta.get(name, 0)
                for name in (
                    'rx_dropped', 'tx_dropped', 'rx_errors', 'tx_errors'
                )
            )
            if interface_failures:
                warnings.append(
                    f'kernel interface drops/errors increased by {interface_failures}'
                )
        else:
            lines.append('  kernel interface delta: unavailable/first sample')

        station = wifi['station']
        station_delta = wifi['station_delta']
        ratios = wifi['station_ratios']
        lines.append(
            '  station: inactive=%sms avg_signal=%sdBm throughput=%sMbps '
            'delta_tx_packets=%s retries=%s failed=%s beacon_loss=%s '
            'rx_drop_misc=%s retry/packet=%s failed/1000=%s resets=%s'
            % (
                _format_optional(station.get('inactive_time_ms'), precision=0),
                _format_optional(station.get('signal_avg_dbm')),
                _format_optional(station.get('expected_throughput_mbps')),
                _format_optional(station_delta.get('tx_packets'), precision=0),
                _format_optional(station_delta.get('tx_retries'), precision=0),
                _format_optional(station_delta.get('tx_failed'), precision=0),
                _format_optional(station_delta.get('beacon_loss'), precision=0),
                _format_optional(station_delta.get('rx_drop_misc'), precision=0),
                _format_optional(ratios.get('retries_per_tx_packet'), precision=3),
                _format_optional(
                    ratios.get('failed_per_1000_tx_packets'), precision=2
                ),
                wifi['station_resets'] or 'none',
            )
        )
        if station_delta.get('tx_failed', 0):
            warnings.append(
                f'Wi-Fi tx_failed +{station_delta["tx_failed"]}'
            )
        if station_delta.get('beacon_loss', 0):
            warnings.append(
                f'Wi-Fi beacon_loss +{station_delta["beacon_loss"]}'
            )
        if ratios.get('retries_per_tx_packet', 0.0) > 0.2:
            warnings.append(
                'high Wi-Fi retry ratio: %.3f retries/packet'
                % ratios['retries_per_tx_packet']
            )
        if ratios.get('failed_per_1000_tx_packets', 0.0) > 1.0:
            warnings.append(
                'high Wi-Fi failure ratio: %.2f/1000 packets'
                % ratios['failed_per_1000_tx_packets']
            )

        survey = wifi['survey']
        survey_pct = wifi['survey_percentages']
        lines.append(
            '  channel survey: freq=%sMHz noise=%sdBm busy=%s%% rx=%s%% '
            'tx=%s%% own-BSS-rx=%s%% resets=%s'
            % (
                _format_optional(survey.get('frequency_mhz'), precision=0),
                _format_optional(survey.get('noise_dbm'), precision=0),
                _format_optional(survey_pct.get('busy_percent')),
                _format_optional(survey_pct.get('receive_percent')),
                _format_optional(survey_pct.get('transmit_percent')),
                _format_optional(survey_pct.get('bss_receive_percent')),
                wifi['survey_resets'] or 'none',
            )
        )
        if survey_pct.get('busy_percent', 0.0) >= 80.0:
            warnings.append(
                f'Wi-Fi channel busy {survey_pct["busy_percent"]:.1f}%'
            )

        interesting_snmp = (
            'Udp.InDatagrams', 'Udp.OutDatagrams', 'Udp.NoPorts',
            'Udp.InErrors', 'Udp.RcvbufErrors', 'Udp.SndbufErrors',
            'Ip.InDiscards', 'Ip.OutDiscards', 'Ip.InHdrErrors',
            'Ip.ReasmFails',
        )
        snmp_text = ' '.join(
            f'{name}={health["snmp_delta"].get(name, 0)}'
            for name in interesting_snmp
        )
        lines.append(
            f'  kernel UDP/IP delta: {snmp_text} '
            f'resets={health["snmp_resets"] or "none"}'
        )
        kernel_errors = sum(
            health['snmp_delta'].get(name, 0)
            for name in interesting_snmp[2:]
        )
        if kernel_errors:
            warnings.append(f'kernel UDP/IP errors/discards +{kernel_errors}')
        softnet = health['softnet_delta']
        lines.append(
            '  softnet delta: processed=%s dropped=%s time_squeeze=%s '
            'resets=%s; sockets=%s'
            % (
                softnet.get('processed', 'N/A'),
                softnet.get('dropped', 'N/A'),
                softnet.get('time_squeeze', 'N/A'),
                health['softnet_resets'] or 'none',
                ' '.join(
                    f'{name}={value}'
                    for name, value in sorted(health['sockstat'].items())
                    if name in ('sockets.used', 'UDP.inuse', 'UDP.mem')
                ) or 'N/A',
            )
        )
        if softnet.get('dropped', 0) or softnet.get('time_squeeze', 0):
            warnings.append(
                'kernel softnet pressure: dropped=%d time_squeeze=%d'
                % (
                    softnet.get('dropped', 0),
                    softnet.get('time_squeeze', 0),
                )
            )

        probe_text = ', '.join(
            '%s=%s(%.3fs%s)' % (
                name,
                result['status'],
                result['duration_s'],
                '' if result['returncode'] is None
                else f',rc={result["returncode"]}',
            )
            for name, result in sorted(wifi['probes'].items())
        )
        lines.append(f'  external probe status: {probe_text}')

        lines.append('  relevant processes:')
        labels_seen = Counter()
        for process in health['processes']:
            label = process['label']
            labels_seen[label] += 1
            delta = process['delta']
            lines.append(
                '   - %-16s pid=%-7s state=%s CPU=%s%% RSS=%.1fMiB '
                'threads=%s core=%s runqueue=%s%% nvcsw=%s wchan=%s RMW=%s '
                'cmd=%s'
                % (
                    label,
                    process['pid'],
                    process['state'],
                    _format_optional(delta.get('cpu_percent_one_core')),
                    process.get('rss_bytes', 0) / 1048576.0,
                    process.get('num_threads', 'N/A'),
                    process.get('processor', 'N/A'),
                    _format_optional(
                        delta.get('runqueue_percent_one_core')
                    ),
                    _format_optional(
                        delta.get('nonvoluntary_context_switches'),
                        precision=0,
                    ),
                    process.get('wchan', 'N/A'),
                    process.get('rmw_libraries') or 'not-mapped',
                    str(process.get('cmdline', ''))[:160],
                )
            )
            if delta.get('cpu_percent_one_core', 0.0) >= 90.0:
                warnings.append(
                    f'{label} pid={process["pid"]} CPU='
                    f'{delta["cpu_percent_one_core"]:.1f}% of one core'
                )
            if delta.get('runqueue_percent_one_core', 0.0) >= 25.0:
                warnings.append(
                    f'{label} pid={process["pid"]} run-queue delay='
                    f'{delta["runqueue_percent_one_core"]:.1f}%'
                )
            if delta.get('restarted'):
                warnings.append(f'{label} pid={process["pid"]} restarted')
        if not labels_seen['controller']:
            warnings.append(
                f'controller process matching {self.controller_process_match!r} not found'
            )
        if self.monitor_microxrce_agent and not labels_seen['microxrce_agent']:
            warnings.append('Micro XRCE Agent process not found')

    def _append_endpoint_lines(self, lines, endpoints, warnings):
        lines.append(' DDS PUBLISHER ENDPOINTS / QOS')
        for topic, data in endpoints.items():
            if data['error']:
                lines.append(f'  {topic}: query failed: {data["error"]}')
                warnings.append(f'DDS endpoint query failed for {topic}')
                continue
            endpoint_items = data['endpoints']
            signature = tuple(
                (item['gid'], item['node'], item['qos'])
                for item in endpoint_items
            )
            previous = self._endpoint_signatures.get(topic)
            changed = previous is not None and previous != signature
            self._endpoint_signatures[topic] = signature
            lines.append(
                f'  {topic}: publishers={len(endpoint_items)} '
                f'changed={changed}'
            )
            for item in endpoint_items:
                lines.append(
                    f'   - gid={item["gid"]} node={item["node"]} '
                    f'qos={item["qos"]}'
                )
            if changed:
                warnings.append(f'DDS publisher endpoint set changed for {topic}')
        live_count = len(endpoints['/swarm/live']['endpoints'])
        status_count = len(endpoints['/swarm/status']['endpoints'])
        if live_count < self.drone_count:
            warnings.append(
                f'DDS graph sees {live_count}/{self.drone_count} heartbeat publishers'
            )
        if status_count > 1:
            warnings.append(
                f'SPLIT-BRAIN evidence: {status_count} status publishers'
            )

    def _append_heartbeat_status_lines(
        self, lines, snapshot, elapsed, now, warnings
    ):
        lines.append(' SWARM HEARTBEATS / STATUS')
        seen_ids = set(snapshot['heartbeats'])
        expected_ids = {str(item) for item in range(1, self.drone_count + 1)}
        for frame_id in sorted(expected_ids | seen_ids):
            data = snapshot['heartbeats'].get(frame_id)
            if data is None:
                lines.append(f'  heartbeat peer={frame_id}: never received')
                warnings.append(f'heartbeat {frame_id} never received')
                continue
            lines.append(
                '  heartbeat peer=%-8s rate=%6.2fHz age=%ss max_gap=%ss '
                'advance_age=%ss repeated=%d regress=%d zero=%d '
                'clock(source-local)=%ss excess_delay(last/max)=%s/%ss'
                % (
                    frame_id or '<empty>', data['count'] / elapsed,
                    _format_optional(data['arrival_age']),
                    _format_optional(data['max_arrival_gap'], precision=3),
                    _format_optional(data['advance_age']),
                    data['repeated'], data['regressed'], data['zero_stamps'],
                    _format_optional(data['source_minus_local_s'], precision=3),
                    _format_optional(data['last_excess_delay_s'], precision=3),
                    _format_optional(data['max_excess_delay_s'], precision=3),
                )
            )
            if frame_id in expected_ids and (
                data['arrival_age'] is None
                or data['arrival_age'] > self.heartbeat_warn_age
            ):
                warnings.append(
                    f'heartbeat {frame_id} missing/stale age='
                    f'{_format_optional(data["arrival_age"])}s'
                )
            if (
                data['max_arrival_gap'] is not None
                and data['max_arrival_gap'] > 2.5 * self.heartbeat_period
            ):
                warnings.append(
                    f'heartbeat {frame_id} max gap '
                    f'{data["max_arrival_gap"]:.3f}s '
                    f'(neighbor timeout={self.neighbor_timeout:.1f}s)'
                )
            if data['regressed']:
                warnings.append(
                    f'heartbeat {frame_id} source timestamp regressed '
                    f'{data["regressed"]} time(s)'
                )
            offset = data['source_minus_local_s']
            if offset is not None and abs(offset) > 0.25:
                warnings.append(
                    f'peer {frame_id} clock offset approximately {offset:+.3f}s'
                )

        leader_counts = snapshot['leader_counts']
        if len(leader_counts) > 1:
            warnings.append(
                f'SPLIT-BRAIN/status leader changes: {leader_counts}'
            )
        status = snapshot['status']
        if status is None:
            lines.append('  swarm status: never received')
            warnings.append('swarm status never received')
            return
        age = max(0.0, now - status['observed_at'])
        lines.append(
            '  status: age=%.2fs leader=%d members=%s state=%s armed=%s '
            'offboard=%s pattern=%s spacing=%.2f leader_pos=%s goal=%s '
            'leader_counts=%s mission=%s message=%r'
            % (
                age, status['leader_id'], status['members'], status['state'],
                status['armed'], status['offboard'], status['pattern'],
                status['spacing'], status['leader_position'], status['goal'],
                leader_counts, status['mission'], status['message'],
            )
        )
        if age > max(2.5, 2.5 * self.print_interval):
            warnings.append(f'swarm status stale: age={age:.2f}s')

    @staticmethod
    def _writer_names(endpoints):
        return {
            item['gid']: item['node']
            for item in endpoints.get('/tf', {}).get('endpoints', ())
        }

    @staticmethod
    def _find_tf_cycles(frame_data):
        parents = {
            child: data.get('parent')
            for child, data in frame_data.items()
            if child and data.get('parent')
        }
        cycles = set()
        for start in parents:
            order = []
            positions = {}
            current = start
            while current in parents:
                if current in positions:
                    cycle = order[positions[current]:]
                    if cycle:
                        rotations = [
                            tuple(cycle[index:] + cycle[:index])
                            for index in range(len(cycle))
                        ]
                        cycles.add(min(rotations))
                    break
                positions[current] = len(order)
                order.append(current)
                current = parents[current]
        return tuple(sorted(cycles))

    def _append_tf_lines(
        self, lines, snapshot, endpoints, elapsed, now, warnings
    ):
        lines.append(' TF TIMING / CONTENT / DDS DELIVERY')
        if not self.monitor_tf:
            lines.append('  disabled (set monitor_tf:=true for a flight test)')
            return
        frame_data = snapshot['tf_frames']
        writer_names = self._writer_names(endpoints)
        flight_active = bool(
            snapshot['status']
            and (
                snapshot['status']['armed']
                or snapshot['status']['offboard']
                or snapshot['status']['state'] != 'IDLE'
            )
        )
        startup_complete = now - self._started_monotonic >= self.tf_startup_grace
        all_frames = sorted(self._expected_tf_frames | set(frame_data))
        if not all_frames:
            lines.append('  no transform frames observed')
        for child in all_frames:
            data = frame_data.get(child)
            expected_type = (
                'odom' if child in self._expected_odom_frames
                else 'goal' if child in self._expected_goal_frames
                else 'other'
            )
            if data is None:
                lines.append(
                    f'  frame={child} expected={expected_type}: never received'
                )
                should_warn = startup_complete and (
                    expected_type == 'odom'
                    or (expected_type == 'goal' and flight_active)
                )
                if should_warn:
                    warnings.append(f'expected TF frame never received: {child}')
                continue
            writer = data['last_writer'] or 'unknown'
            writer_display = writer_names.get(writer, '<unmapped>')
            gap_ago = (
                None if data['max_arrival_gap_at'] is None
                else max(0.0, now - data['max_arrival_gap_at'])
            )
            lines.append(
                '  frame=%-42s parent=%-24s rate=%6.2fHz arrival_age=%ss '
                'advance_age=%ss gap(arrival/advance)=%s/%ss gap_ago=%ss '
                'repeat/regress/zero=%d/%d/%d writer=%s(%s) authorities=%d'
                % (
                    child, data['parent'] or '<empty>',
                    data['count'] / elapsed,
                    _format_optional(data['arrival_age']),
                    _format_optional(data['advance_age']),
                    _format_optional(data['max_arrival_gap'], precision=3),
                    _format_optional(data['max_advance_gap'], precision=3),
                    _format_optional(gap_ago), data['repeated'],
                    data['regressed'], data['zero_stamps'], writer,
                    writer_display, data['writer_count'],
                )
            )
            lines.append(
                '    content: parent_changes=%d writer_changes=%d '
                'invalid(empty/self/nonfinite/zero_q/nonunit_q)=%d/%d/%d/%d/%d '
                'max_step(position/angular)=%s m/%s rad max_implied_speed=%s m/s '
                'max_source_step=%ss '
                'clock(source-local)=%ss excess_delay(last/max)=%s/%ss'
                % (
                    data['parent_changes'], data['writer_changes'],
                    data['empty_frame_ids'], data['self_edges'],
                    data['nonfinite'], data['zero_quaternions'],
                    data['nonunit_quaternions'],
                    _format_optional(data['max_position_step'], precision=3),
                    _format_optional(data['max_angular_step'], precision=3),
                    _format_optional(data['max_linear_speed'], precision=2),
                    _format_optional(data['max_source_step'], precision=3),
                    _format_optional(data['source_minus_local_s'], precision=3),
                    _format_optional(data['last_excess_delay_s'], precision=3),
                    _format_optional(data['max_excess_delay_s'], precision=3),
                )
            )
            expected_active = expected_type == 'odom' or (
                expected_type == 'goal' and flight_active
            )
            if startup_complete and expected_active:
                if (
                    data['arrival_age'] is None
                    or data['arrival_age'] > self.tf_stale_warn_age
                ):
                    warnings.append(
                        f'TF transport/publisher arrival stale {child}: '
                        f'{_format_optional(data["arrival_age"])}s'
                    )
                if (
                    data['advance_age'] is None
                    or data['advance_age'] > self.tf_stale_warn_age
                ):
                    warnings.append(
                        f'TF source timestamp stale {child}: '
                        f'{_format_optional(data["advance_age"])}s'
                    )
            if (
                data['max_arrival_gap'] is not None
                and data['max_arrival_gap'] > self.tf_gap_warn_threshold
            ):
                warnings.append(
                    f'TF arrival gap {child}: {data["max_arrival_gap"]:.3f}s'
                )
            if data['repeated']:
                warnings.append(
                    f'TF repeated source stamp {child}: {data["repeated"]}'
                )
            if data['regressed']:
                warnings.append(
                    f'TF source stamp regressed {child}: {data["regressed"]}'
                )
            if (
                data['max_source_step'] is not None
                and data['max_source_step'] > self.tf_gap_warn_threshold
            ):
                warnings.append(
                    f'TF publisher source-time step {child}: '
                    f'{data["max_source_step"]:.3f}s'
                )
            if (
                data['max_excess_delay_s'] is not None
                and data['max_excess_delay_s'] > 0.1
            ):
                warnings.append(
                    f'TF variable delivery delay {child}: '
                    f'{data["max_excess_delay_s"]:.3f}s above baseline'
                )
            if (
                expected_type == 'odom'
                and data['max_position_step'] is not None
                and data['max_position_step'] > 2.0
            ):
                warnings.append(
                    f'TF position jump {child}: '
                    f'{data["max_position_step"]:.3f}m'
                )
            if (
                expected_type == 'odom'
                and data['max_linear_speed'] is not None
                and data['max_linear_speed'] > 5.0
            ):
                warnings.append(
                    f'TF implausible speed {child}: '
                    f'{data["max_linear_speed"]:.2f}m/s'
                )
            if data['writer_count'] > 1:
                warnings.append(
                    f'TF multiple writer authorities for {child}: '
                    f'{data["writer_count"]}'
                )
            invalid_count = sum(
                data[name]
                for name in (
                    'empty_frame_ids', 'self_edges', 'nonfinite',
                    'zero_quaternions', 'nonunit_quaternions',
                )
            )
            if invalid_count:
                warnings.append(
                    f'TF invalid content {child}: {invalid_count} sample(s)'
                )

        lines.append('  TF DDS writers (MessageInfo observer):')
        for gid, data in sorted(snapshot['tf_writers'].items()):
            name = writer_names.get(gid, '<unmapped>')
            lines.append(
                '   - gid=%s node=%s rate=%.2fHz messages_by_batch=%s '
                'missing_seq=%d duplicate_seq=%d regressed_seq=%d '
                'middleware_delay_max=%ss callback_queue_max=%ss '
                'children=%s'
                % (
                    gid, name, data['count'] / elapsed, data['message_sizes'],
                    data['missing_sequences'], data['sequence_duplicates'],
                    data['sequence_regressions'],
                    _format_optional(data['max_middleware_delay_s'], precision=4),
                    _format_optional(data['max_callback_queue_delay_s'], precision=4),
                    data['children'],
                )
            )
            if data['missing_sequences']:
                warnings.append(
                    f'TF DDS writer {gid} observer missed '
                    f'{data["missing_sequences"]} sequence(s)'
                )
            if data['sequence_regressions']:
                warnings.append(
                    f'TF DDS writer {gid} sequence regressed '
                    f'{data["sequence_regressions"]} time(s)'
                )
            if data['max_callback_queue_delay_s'] is not None and (
                data['max_callback_queue_delay_s'] > self.tf_gap_warn_threshold
            ):
                warnings.append(
                    f'logger TF callback queue delay '
                    f'{data["max_callback_queue_delay_s"]:.3f}s writer={gid}'
                )
        if RclpyMessageInfo is None:
            lines.append(
                '   - MessageInfo unavailable in this rclpy; writer identity '
                'and DDS sequence loss cannot be measured on this host'
            )
        if snapshot['tf_untracked']:
            lines.append(f'  bounded/untracked TF data: {snapshot["tf_untracked"]}')
            warnings.append('TF diagnostic frame/writer bound was reached')
        cycles = self._find_tf_cycles(frame_data)
        lines.append(f'  TF cycles detected: {cycles or "none"}')
        if cycles:
            warnings.append(f'TF graph cycle(s): {cycles}')

    def _append_vehicle_lines(self, lines, snapshot, now, warnings):
        lines.append(' LOCAL PX4 SEMANTICS')
        if not self.monitor_vehicle_topics:
            lines.append('  disabled (set monitor_vehicle_topics:=true)')
            return
        latest = snapshot['vehicle_latest']
        for drone_id in self.drone_ids:
            lines.append(f'  drone={drone_id}:')
            for kind in (
                'status', 'failsafe_flags', 'local_position',
                'trajectory_setpoint', 'offboard_control_mode', 'battery',
                'land', 'timesync', 'vehicle_command', 'command_ack',
            ):
                data = latest.get((drone_id, kind))
                if data is None:
                    lines.append(f'   - {kind}: never received')
                    continue
                age = max(0.0, now - data['observed_at'])
                payload = {
                    key: value
                    for key, value in data.items()
                    if key != 'observed_at'
                }
                lines.append(f'   - {kind}: age={age:.2f}s {payload}')
                if kind == 'status' and data.get('failsafe'):
                    warnings.append(f'PX4 drone {drone_id} reports failsafe')
                if kind == 'failsafe_flags' and (
                    data.get('active') or data.get('battery_warning', 0)
                ):
                    warnings.append(
                        f'PX4 drone {drone_id} failsafe flags: '
                        f'{data.get("active")} battery_warning='
                        f'{data.get("battery_warning")}'
                    )
                if kind == 'local_position' and (
                    not data.get('finite')
                    or not data.get('valid')
                    or data.get('dead_reckoning')
                ):
                    warnings.append(
                        f'PX4 drone {drone_id} local position unhealthy: '
                        f'finite={data.get("finite")} valid={data.get("valid")} '
                        f'dead_reckoning={data.get("dead_reckoning")}'
                    )
        counts = snapshot['vehicle_counts']
        maxima = snapshot['vehicle_maxima']
        lines.append(f'  PX4 window counters: {counts or "none"}')
        lines.append(f'  PX4 window maxima: {maxima or "none"}')
        for drone_id in self.drone_ids:
            if counts.get((drone_id, 'setpoint_nonfinite'), 0):
                warnings.append(
                    f'drone {drone_id} emitted non-finite velocity setpoint(s)'
                )
            if counts.get((drone_id, 'position_nonfinite'), 0):
                warnings.append(
                    f'drone {drone_id} received non-finite local position(s)'
                )

    def _append_topic_lines(
        self, lines, snapshot, elapsed, now, warnings
    ):
        lines.append(' SUBSCRIPTION RECEIVE TIMING')
        for topic in sorted(self._monitored_topics):
            data = snapshot['topics'].get(topic)
            if data is None:
                lines.append(f'  {topic}: never received')
                continue
            gap_ago = (
                None if data['max_arrival_gap_at'] is None
                else max(0.0, now - data['max_arrival_gap_at'])
            )
            lines.append(
                '  %-58s rate=%7.2fHz age=%ss max_gap=%ss gap_ago=%ss '
                'stamp_advance_age=%ss repeat/regress/zero=%d/%d/%d'
                % (
                    topic, data['count'] / elapsed,
                    _format_optional(data['arrival_age']),
                    _format_optional(data['max_arrival_gap'], precision=3),
                    _format_optional(gap_ago),
                    _format_optional(data['advance_age']), data['repeated'],
                    data['regressed'], data['zero_stamps'],
                )
            )
            if topic in self._critical_vehicle_topics:
                if (
                    data['arrival_age'] is not None
                    and data['arrival_age'] > self.tf_gap_warn_threshold
                ):
                    warnings.append(
                        f'critical PX4 topic stale {topic}: '
                        f'{data["arrival_age"]:.3f}s'
                    )
                if (
                    data['max_arrival_gap'] is not None
                    and data['max_arrival_gap'] > self.tf_gap_warn_threshold
                ):
                    warnings.append(
                        f'critical PX4 topic gap {topic}: '
                        f'{data["max_arrival_gap"]:.3f}s'
                    )

    def _append_event_lines(self, lines, snapshot, wifi_events, now, warnings):
        lines.append(' TRANSITIONS / ANOMALIES / WIFI EVENTS')
        if snapshot['events']:
            for observed, category, text in snapshot['events'][-40:]:
                lines.append(
                    f'  - {max(0.0, now - observed):7.3f}s ago '
                    f'[{category}] {text}'
                )
        else:
            lines.append('  ROS/PX4 events: none in this window')
        if wifi_events is None:
            lines.append('  iw event monitor: disabled')
            return
        lines.append(
            f'  iw event monitor: status={wifi_events["status"]} '
            f'counts={wifi_events["counts"]}'
        )
        for observed, category, text in wifi_events['events'][-20:]:
            lines.append(
                f'   - {max(0.0, now - observed):7.3f}s ago '
                f'[{category}] {text}'
            )
        disruptive = sum(
            wifi_events['counts'].get(name, 0)
            for name in ('disconnect', 'deauth', 'disassoc', 'roam')
        )
        if disruptive:
            warnings.append(
                f'nl80211 recorded {disruptive} disconnect/deauth/disassoc/roam events'
            )

    def _report(self):
        """Capture a ROS window and start its matching health sample."""
        started = time.monotonic()
        if self._pending_report is not None:
            self._pending_report['overruns'] += 1
            return
        if self._health_prime:
            if self._health_future is not None and not self._health_future.done():
                self._health_prime_overruns += 1
                return
            try:
                if self._health_future is not None:
                    self._health_future.result()
            except Exception as error:  # noqa: B902 - diagnostic boundary
                self._health_prime_error = (
                    f'{type(error).__name__}: {error}'
                )
            self._health_future = None
            self._health_prime = False
        elapsed = max(1e-6, started - self._last_report_time)
        self._last_report_time = started
        self._report_number += 1
        snapshot = self._snapshot_ros_state(started)
        endpoints = self._endpoint_snapshot(
            ('/swarm/live', '/swarm/status', '/tf')
        )
        wifi_events = (
            None if self._wifi_event_monitor is None
            else self._wifi_event_monitor.snapshot(clear=True)
        )
        self._health_future = self._health_executor.submit(
            self._health_collector.collect
        )
        self._pending_report = {
            'started': started,
            'capture_duration': time.monotonic() - started,
            'epoch_ns': time.time_ns(),
            'utc': datetime.now(timezone.utc).isoformat(timespec='milliseconds'),
            'elapsed': elapsed,
            'snapshot': snapshot,
            'endpoints': endpoints,
            'wifi_events': wifi_events,
            'overruns': 0,
            'prime_error': self._health_prime_error,
            'prime_overruns': self._health_prime_overruns,
        }
        self._health_prime_error = None
        self._health_prime_overruns = 0

    def _flush_pending_report(self):
        """Emit a captured report only after its background probes finish."""
        pending = self._pending_report
        future = self._health_future
        if pending is None or future is None or not future.done():
            return
        health = None
        health_error = None
        try:
            health = future.result()
        except Exception as error:  # noqa: B902 - diagnostic boundary
            health_error = f'{type(error).__name__}: {error}'
        self._pending_report = None
        self._health_future = None
        self._emit_report(pending, health, health_error)

    def _emit_report(self, pending, health, health_error):
        """Format one already-captured diagnostic window."""
        started = pending['started']
        elapsed = pending['elapsed']
        snapshot = pending['snapshot']
        endpoints = pending['endpoints']
        wifi_events = pending['wifi_events']
        health_age = (
            None
            if health is None
            else max(
                0.0,
                time.monotonic() - health['sample_started_monotonic'],
            )
        )
        warnings = []
        if elapsed > self.print_interval + max(0.5, 0.25 * self.print_interval):
            warnings.append(f'logger report timer delayed: {elapsed:.3f}s')
        if pending['overruns']:
            warnings.append(
                f'health collection overran {pending["overruns"]} report period(s)'
            )
        if pending['prime_overruns']:
            warnings.append(
                'health baseline priming overran '
                f'{pending["prime_overruns"]} report period(s)'
            )
        if pending['prime_error']:
            warnings.append(
                f'health baseline priming failed: {pending["prime_error"]}'
            )
        if health_error:
            warnings.append(f'host health collection failed: {health_error}')

        lines = ['', _BORDER, 'SWARM FLIGHT DIAGNOSTIC REPORT', _BORDER]
        self._append_metadata_lines(
            lines,
            elapsed,
            started,
            pending['epoch_ns'],
            pending['utc'],
        )
        lines.append(_SECTION)
        self._append_health_lines(lines, health, health_age, warnings)
        lines.append(_SECTION)
        self._append_endpoint_lines(lines, endpoints, warnings)
        lines.append(_SECTION)
        self._append_heartbeat_status_lines(
            lines, snapshot, elapsed, started, warnings
        )
        lines.append(_SECTION)
        self._append_tf_lines(
            lines, snapshot, endpoints, elapsed, started, warnings
        )
        lines.append(_SECTION)
        self._append_vehicle_lines(lines, snapshot, started, warnings)
        lines.append(_SECTION)
        self._append_topic_lines(
            lines, snapshot, elapsed, started, warnings
        )
        lines.append(_SECTION)
        self._append_event_lines(
            lines, snapshot, wifi_events, started, warnings
        )
        lines.append(_SECTION)
        if warnings:
            lines.append(f' WARNINGS ({len(warnings)}):')
            lines.extend(f'  ! {warning}' for warning in warnings)
        else:
            lines.append(' WARNINGS: none observed in this reporting window')
        format_duration = time.monotonic() - health['sample_monotonic'] if health else 0.0
        lines.append(
            ' logger capture=%0.4fs background_health=%ss '
            'health_to_emit=%0.4fs'
            % (
                pending['capture_duration'],
                _format_optional(
                    None if health is None else health['collection_duration_s'],
                    precision=4,
                ),
                format_duration,
            )
        )
        lines.append(_BORDER)

        # Intentionally use local stdout rather than ROS logging. commands.txt
        # redirects this to explicitly selected local storage on each aircraft.
        print('\n'.join(lines), flush=True)

    def close_diagnostics(self):
        """Stop only background resources owned by this logger."""
        if self._closed:
            return
        self._closed = True
        if self._wifi_event_monitor is not None:
            self._wifi_event_monitor.stop()
        if self._health_future is not None:
            self._health_future.cancel()
        self._health_executor.shutdown(wait=False, cancel_futures=True)


def main(args=None):
    rclpy.init(args=args)
    node = SwarmLogger()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close_diagnostics()
        try:
            executor.remove_node(node)
            executor.shutdown(timeout_sec=1.0)
            node.destroy_node()
        except (RuntimeError, ValueError):
            # The ROS context may already have destroyed entities on SIGTERM.
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
