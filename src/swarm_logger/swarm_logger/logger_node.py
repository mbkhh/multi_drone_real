#!/usr/bin/env python3

"""Low-overhead swarm, Raspberry Pi, and Wi-Fi health logger."""

from __future__ import annotations

import os
import shutil
import subprocess
import time
from collections import Counter
from threading import Lock

import rclpy
from px4_msgs.msg import (
    BatteryStatus,
    FailsafeFlags,
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLandDetected,
    VehicleLocalPosition,
    VehicleStatus,
)
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.exceptions import ParameterUninitializedException
from rclpy.executors import ExternalShutdownException
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Header, String
from swarm_config.config_utils import get_config
from swarm_logger.system_diagnostics import (
    counter_deltas,
    cpu_usage_percent,
    decode_throttled_state,
    parse_iw_link,
    parse_iw_station_dump,
    parse_throttled_output,
    read_cpu_times,
    read_interface_state,
    read_load_average,
    read_memory,
    read_net_counters,
    read_process_rss,
    read_temperature,
    signal_label,
)
from swarm_msgs.msg import FormationCommand, ManualControl, Status
from tf2_msgs.msg import TFMessage


def _format_optional(value, suffix='', precision=1):
    if value is None:
        return 'N/A'
    if isinstance(value, float):
        return f'{value:.{precision}f}{suffix}'
    return f'{value}{suffix}'


class SwarmLogger(Node):
    """Report data freshness and host health without retaining messages."""

    def __init__(self):
        # Keep diagnostic reports off /rosout so the logger cannot worsen the
        # wireless DDS traffic that it is trying to measure.
        super().__init__('swarm_logger', enable_rosout=False)

        default_drone_count = int(get_config('swarm_sim.drone_count'))
        self.declare_parameter('drone_count', default_drone_count)
        self.declare_parameter('drone_ids', Parameter.Type.INTEGER_ARRAY)
        self.declare_parameter('print_interval', 5.0)
        self.declare_parameter('wifi_interface', 'wlan0')
        self.declare_parameter('monitor_vehicle_topics', False)
        self.declare_parameter('monitor_tf', False)

        self.drone_count = (
            self.get_parameter('drone_count').get_parameter_value().integer_value
        )
        try:
            self.drone_ids = list(
                self.get_parameter('drone_ids')
                .get_parameter_value().integer_array_value
            )
        except ParameterUninitializedException:
            self.drone_ids = []
        self.print_interval = (
            self.get_parameter('print_interval')
            .get_parameter_value().double_value
        )
        self.wifi_interface = (
            self.get_parameter('wifi_interface')
            .get_parameter_value().string_value
        )
        self.monitor_vehicle_topics = (
            self.get_parameter('monitor_vehicle_topics')
            .get_parameter_value().bool_value
        )
        self.monitor_tf = (
            self.get_parameter('monitor_tf').get_parameter_value().bool_value
        )

        if self.drone_count < 1:
            raise ValueError('drone_count must be positive')
        if self.print_interval < 1.0:
            raise ValueError(
                'print_interval must be at least 1 second to keep diagnostics cheap'
            )
        if not self.drone_ids:
            self.drone_ids = list(range(1, self.drone_count + 1))
        if any(drone_id < 1 for drone_id in self.drone_ids):
            raise ValueError('drone_ids must contain positive IDs')

        configured_heartbeat_period = get_config(
            'swarm_single.broadcast_interval'
        )
        self.heartbeat_period = float(configured_heartbeat_period or 1.5)
        self.heartbeat_warn_age = max(2.5 * self.heartbeat_period, 2.0)

        self._lock = Lock()
        self._topic_counts = Counter()
        self._topic_last_seen = {}
        self._topic_max_gap = {}
        self._heartbeat_counts = Counter()
        self._heartbeat_last_seen = {}
        self._heartbeat_max_gap = {}
        self._status_leader_counts = Counter()
        self._last_status = None
        self._tf_frame_counts = Counter()
        self._tf_frame_last_seen = {}
        self._tf_frame_max_gap = {}
        self._tf_frame_last_source_stamp = {}
        self._tf_frame_repeated_stamps = Counter()
        self._monitored_topics = []
        self._subscriptions = []

        self._last_report_time = time.monotonic()
        self._previous_cpu_times = read_cpu_times()
        self._previous_net_counters = read_net_counters(self.wifi_interface)
        self._previous_station_counters = None
        self._previous_bssid = None

        # Diagnostic subscriptions never need retransmission or stale samples.
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
            depth=5,
        )

        self._add_swarm_topics()
        if self.monitor_vehicle_topics:
            self._add_vehicle_topics()
        if self.monitor_tf:
            self._subscribe(
                TFMessage, '/tf', self._ros_qos, self._tf_callback
            )

        self.create_timer(self.print_interval, self._report)
        self.get_logger().info(
            'Diagnostic logger active: report interval %.1f s, vehicle IDs %s, '
            'Wi-Fi interface %s, /rosout disabled.'
            % (self.print_interval, self.drone_ids, self.wifi_interface)
        )

    def _subscribe(self, message_type, topic, qos, callback=None):
        if callback is None:
            def callback(message, topic_name=topic):
                self._record(topic_name, message)
        subscription = self.create_subscription(
            message_type, topic, callback, qos
        )
        self._subscriptions.append(subscription)
        self._monitored_topics.append(topic)

    def _add_swarm_topics(self):
        self._subscribe(
            Status,
            '/swarm/status',
            self._ros_qos,
            self._status_callback,
        )
        self._subscribe(
            Header,
            '/swarm/live',
            self._ros_qos,
            self._heartbeat_callback,
        )
        topics = (
            (String, '/swarm/command'),
            (FormationCommand, '/swarm/formation_command'),
            (ManualControl, '/manual_controller'),
            (String, '/command'),
            (FormationCommand, '/formation'),
        )
        for message_type, topic in topics:
            self._subscribe(message_type, topic, self._ros_qos)

    def _add_vehicle_topics(self):
        px4_topics = (
            (VehicleStatus, 'out/vehicle_status_v1'),
            (VehicleLocalPosition, 'out/vehicle_local_position_v1'),
            (VehicleLandDetected, 'out/vehicle_land_detected'),
            (BatteryStatus, 'out/battery_status_v1'),
            (FailsafeFlags, 'out/failsafe_flags'),
            (VehicleCommand, 'in/vehicle_command'),
            (TrajectorySetpoint, 'in/trajectory_setpoint'),
            (OffboardControlMode, 'in/offboard_control_mode'),
        )
        for drone_id in self.drone_ids:
            namespace = f'/uav_{drone_id}/fmu'
            for message_type, suffix in px4_topics:
                self._subscribe(
                    message_type,
                    f'{namespace}/{suffix}',
                    self._px4_qos,
                )

    def _record(self, topic, _message=None):
        observed_at = time.monotonic()
        with self._lock:
            previous = self._topic_last_seen.get(topic)
            if previous is not None:
                self._topic_max_gap[topic] = max(
                    self._topic_max_gap.get(topic, 0.0),
                    observed_at - previous,
                )
            self._topic_counts[topic] += 1
            self._topic_last_seen[topic] = observed_at

    def _heartbeat_callback(self, message):
        observed_at = time.monotonic()
        frame_id = str(message.frame_id).strip()
        with self._lock:
            topic_previous = self._topic_last_seen.get('/swarm/live')
            if topic_previous is not None:
                self._topic_max_gap['/swarm/live'] = max(
                    self._topic_max_gap.get('/swarm/live', 0.0),
                    observed_at - topic_previous,
                )
            peer_previous = self._heartbeat_last_seen.get(frame_id)
            if peer_previous is not None:
                self._heartbeat_max_gap[frame_id] = max(
                    self._heartbeat_max_gap.get(frame_id, 0.0),
                    observed_at - peer_previous,
                )
            self._topic_counts['/swarm/live'] += 1
            self._topic_last_seen['/swarm/live'] = observed_at
            self._heartbeat_counts[frame_id] += 1
            self._heartbeat_last_seen[frame_id] = observed_at

    def _status_callback(self, message):
        observed_at = time.monotonic()
        snapshot = {
            'leader_id': int(message.leader_id),
            'members': tuple(sorted(int(item) for item in message.swarm_members)),
            'state': str(message.control_state),
            'armed': bool(message.armed),
            'offboard': bool(message.offboard),
            'observed_at': observed_at,
        }
        with self._lock:
            previous = self._topic_last_seen.get('/swarm/status')
            if previous is not None:
                self._topic_max_gap['/swarm/status'] = max(
                    self._topic_max_gap.get('/swarm/status', 0.0),
                    observed_at - previous,
                )
            self._topic_counts['/swarm/status'] += 1
            self._topic_last_seen['/swarm/status'] = observed_at
            self._status_leader_counts[snapshot['leader_id']] += 1
            self._last_status = snapshot

    def _tf_callback(self, message):
        observed_at = time.monotonic()
        with self._lock:
            topic_previous = self._topic_last_seen.get('/tf')
            if topic_previous is not None:
                self._topic_max_gap['/tf'] = max(
                    self._topic_max_gap.get('/tf', 0.0),
                    observed_at - topic_previous,
                )
            self._topic_counts['/tf'] += 1
            self._topic_last_seen['/tf'] = observed_at

            for transform in message.transforms:
                child_frame = str(transform.child_frame_id)
                previous = self._tf_frame_last_seen.get(child_frame)
                if previous is not None:
                    self._tf_frame_max_gap[child_frame] = max(
                        self._tf_frame_max_gap.get(child_frame, 0.0),
                        observed_at - previous,
                    )
                source_stamp = (
                    int(transform.header.stamp.sec),
                    int(transform.header.stamp.nanosec),
                )
                if self._tf_frame_last_source_stamp.get(child_frame) == source_stamp:
                    self._tf_frame_repeated_stamps[child_frame] += 1
                self._tf_frame_last_source_stamp[child_frame] = source_stamp
                self._tf_frame_counts[child_frame] += 1
                self._tf_frame_last_seen[child_frame] = observed_at

    def _snapshot_ros_state(self):
        with self._lock:
            snapshot = {
                'counts': dict(self._topic_counts),
                'last_seen': dict(self._topic_last_seen),
                'max_gap': dict(self._topic_max_gap),
                'heartbeat_counts': dict(self._heartbeat_counts),
                'heartbeat_last_seen': dict(self._heartbeat_last_seen),
                'heartbeat_max_gap': dict(self._heartbeat_max_gap),
                'status_leader_counts': dict(self._status_leader_counts),
                'last_status': self._last_status,
                'tf_frame_counts': dict(self._tf_frame_counts),
                'tf_frame_last_seen': dict(self._tf_frame_last_seen),
                'tf_frame_max_gap': dict(self._tf_frame_max_gap),
                'tf_frame_repeated_stamps': dict(
                    self._tf_frame_repeated_stamps
                ),
            }
            self._topic_counts.clear()
            self._topic_max_gap.clear()
            self._heartbeat_counts.clear()
            self._heartbeat_max_gap.clear()
            self._status_leader_counts.clear()
            self._tf_frame_counts.clear()
            self._tf_frame_max_gap.clear()
            self._tf_frame_repeated_stamps.clear()
        return snapshot

    @staticmethod
    def _run_optional(command, timeout=0.4):
        if not command or shutil.which(command[0]) is None:
            return None
        try:
            completed = subprocess.run(
                command,
                capture_output=True,
                text=True,
                timeout=timeout,
                check=False,
            )
        except (OSError, subprocess.SubprocessError):
            return None
        if completed.returncode != 0:
            return None
        return completed.stdout.strip()

    def _read_throttled(self):
        output = self._run_optional(['vcgencmd', 'get_throttled'])
        value = parse_throttled_output(output or '')
        return value, decode_throttled_state(value)

    def _read_wifi(self):
        link_output = self._run_optional(
            ['iw', 'dev', self.wifi_interface, 'link']
        )
        station_output = self._run_optional(
            ['iw', 'dev', self.wifi_interface, 'station', 'dump']
        )
        power_output = self._run_optional(
            ['iw', 'dev', self.wifi_interface, 'get', 'power_save']
        )
        link = parse_iw_link(link_output or '')
        link['supported'] = link_output is not None
        station = parse_iw_station_dump(station_output or '')
        if power_output is None:
            power_save = None
        elif 'Power save: on' in power_output:
            power_save = True
        elif 'Power save: off' in power_output:
            power_save = False
        else:
            power_save = None
        return link, station, power_save

    def _read_host_health(self, elapsed):
        current_cpu = read_cpu_times()
        cpu_percent = cpu_usage_percent(
            self._previous_cpu_times, current_cpu
        )
        self._previous_cpu_times = current_cpu

        current_net = read_net_counters(self.wifi_interface)
        net_delta = counter_deltas(
            self._previous_net_counters, current_net
        )
        self._previous_net_counters = current_net

        link, station, power_save = self._read_wifi()
        station_delta = counter_deltas(
            self._previous_station_counters, station
        )
        self._previous_station_counters = station or None
        throttled_value, throttled_reasons = self._read_throttled()

        bssid_changed = (
            self._previous_bssid is not None
            and link['bssid'] is not None
            and link['bssid'] != self._previous_bssid
        )
        if link['bssid'] is not None:
            self._previous_bssid = link['bssid']

        return {
            'cpu_percent': cpu_percent,
            'load': read_load_average(),
            'cpu_count': os.cpu_count() or 1,
            'memory': read_memory(),
            'process_rss': read_process_rss(),
            'temperature_c': read_temperature(),
            'throttled_value': throttled_value,
            'throttled_reasons': throttled_reasons,
            'interface_state': read_interface_state(self.wifi_interface),
            'net_delta': net_delta,
            'net_elapsed': elapsed,
            'wifi_link': link,
            'wifi_station_delta': station_delta,
            'power_save': power_save,
            'bssid_changed': bssid_changed,
        }

    def _append_host_lines(self, lines, health, warnings):
        load = health['load']
        if load is None:
            load_text = 'N/A'
        else:
            load_text = '%.2f/%.2f/%.2f (CPUs=%d)' % (
                load[0], load[1], load[2], health['cpu_count']
            )
        memory = health['memory']
        memory_text = (
            'N/A'
            if memory is None
            else f"{memory['used_percent']:.1f}% used"
        )
        rss = health['process_rss']
        rss_text = 'N/A' if rss is None else f'{rss / 1048576.0:.1f} MiB'
        lines.append(
            ' Host: CPU=%s, load=%s, memory=%s, logger_RSS=%s, temp=%s'
            % (
                _format_optional(health['cpu_percent'], '%'),
                load_text,
                memory_text,
                rss_text,
                _format_optional(health['temperature_c'], ' C'),
            )
        )

        throttled_value = health['throttled_value']
        if throttled_value is None:
            throttle_text = 'unavailable (vcgencmd missing or failed)'
        elif throttled_value == 0:
            throttle_text = '0x0 (no current or historical throttle flags)'
        else:
            throttle_text = '0x%x: %s' % (
                throttled_value,
                '; '.join(health['throttled_reasons']),
            )
            warnings.append(f'Raspberry Pi throttle flags: {throttle_text}')
        lines.append(f' Power/thermal flags: {throttle_text}')

        link = health['wifi_link']
        signal = link['signal_dbm']
        lines.append(
            ' Wi-Fi: state=%s, connected=%s, SSID=%s, BSSID=%s, '
            'freq=%s MHz, signal=%s dBm (%s), tx/rx=%s/%s Mbit/s, '
            'power_save=%s'
            % (
                health['interface_state'],
                link['connected'] if link['supported'] else 'N/A',
                link['ssid'] or 'N/A',
                link['bssid'] or 'N/A',
                _format_optional(link['frequency_mhz'], precision=0),
                _format_optional(signal),
                signal_label(signal),
                _format_optional(link['tx_bitrate_mbps']),
                _format_optional(link['rx_bitrate_mbps']),
                _format_optional(health['power_save']),
            )
        )
        if link['supported'] and not link['connected']:
            warnings.append('Wi-Fi is not associated according to iw')
        if health['power_save'] is True:
            warnings.append('Wi-Fi power saving is enabled (latency risk)')
        if health['bssid_changed']:
            warnings.append('Wi-Fi BSSID changed since the previous report')

        elapsed = health['net_elapsed']
        net_delta = health['net_delta']
        if net_delta and elapsed > 0.0:
            rx_mbps = 8.0 * net_delta.get('rx_bytes', 0) / elapsed / 1e6
            tx_mbps = 8.0 * net_delta.get('tx_bytes', 0) / elapsed / 1e6
            lines.append(
                ' Interface delta: rx=%.3f Mbit/s, tx=%.3f Mbit/s, '
                'drops rx/tx=%d/%d, errors rx/tx=%d/%d'
                % (
                    rx_mbps,
                    tx_mbps,
                    net_delta.get('rx_dropped', 0),
                    net_delta.get('tx_dropped', 0),
                    net_delta.get('rx_errors', 0),
                    net_delta.get('tx_errors', 0),
                )
            )
            drop_error_count = sum(
                net_delta.get(name, 0)
                for name in (
                    'rx_dropped',
                    'tx_dropped',
                    'rx_errors',
                    'tx_errors',
                )
            )
            if drop_error_count:
                warnings.append(
                    f'Kernel interface drops/errors increased by {drop_error_count}'
                )
        else:
            lines.append(' Interface delta: unavailable')

        retry_delta = health['wifi_station_delta']
        if retry_delta:
            retry_text = ', '.join(
                f'{name}={value}' for name, value in sorted(retry_delta.items())
            )
            lines.append(f' Wi-Fi driver counter delta: {retry_text}')
            if retry_delta.get('tx_failed', 0) or retry_delta.get(
                'beacon_loss', 0
            ):
                warnings.append(f'Wi-Fi failures increased: {retry_text}')

    def _append_ros_lines(self, lines, snapshot, elapsed, now, warnings):
        live_publishers = self.count_publishers('/swarm/live')
        live_subscribers = self.count_subscribers('/swarm/live')
        status_publishers = self.count_publishers('/swarm/status')
        tf_publishers = self.count_publishers('/tf')
        tf_subscribers = self.count_subscribers('/tf')
        lines.append(
            ' DDS graph: /swarm/live pub/sub=%d/%d, /swarm/status pub=%d, '
            '/tf pub/sub=%d/%d'
            % (
                live_publishers,
                live_subscribers,
                status_publishers,
                tf_publishers,
                tf_subscribers,
            )
        )
        if live_publishers < self.drone_count:
            warnings.append(
                f'DDS graph sees {live_publishers}/{self.drone_count} heartbeat publishers'
            )
        if status_publishers > 1:
            warnings.append(
                f'SPLIT-BRAIN: {status_publishers} /swarm/status publishers exist'
            )

        lines.append(' Heartbeats by frame_id:')
        expected_ids = [str(item) for item in range(1, self.drone_count + 1)]
        seen_ids = set(snapshot['heartbeat_last_seen'])
        for frame_id in sorted(set(expected_ids) | seen_ids):
            count = snapshot['heartbeat_counts'].get(frame_id, 0)
            last_seen = snapshot['heartbeat_last_seen'].get(frame_id)
            max_gap = snapshot['heartbeat_max_gap'].get(frame_id)
            age = None if last_seen is None else max(0.0, now - last_seen)
            lines.append(
                '  - id=%s rate=%.2f Hz age=%s s max_gap=%s s'
                % (
                    frame_id or '<empty>',
                    count / elapsed if elapsed > 0.0 else 0.0,
                    _format_optional(age, precision=2),
                    _format_optional(max_gap, precision=3),
                )
            )
            if frame_id in expected_ids and (
                age is None or age > self.heartbeat_warn_age
            ):
                warnings.append(
                    f'Heartbeat {frame_id} missing/stale: age={_format_optional(age)} s'
                )

        leader_counts = snapshot['status_leader_counts']
        if len(leader_counts) > 1:
            warnings.append(
                f'SPLIT-BRAIN: status messages reported leaders {sorted(leader_counts)}'
            )
        status = snapshot['last_status']
        if status is None:
            lines.append(' Last swarm status: never received')
        else:
            status_age = max(0.0, now - status['observed_at'])
            lines.append(
                ' Last swarm status: leader=%d members=%s state=%s armed=%s '
                'offboard=%s age=%.2f s'
                % (
                    status['leader_id'],
                    status['members'],
                    status['state'],
                    status['armed'],
                    status['offboard'],
                    status_age,
                )
            )

        if self.monitor_tf:
            lines.append(' TF frames (observer mode):')
            tf_frames = sorted(snapshot['tf_frame_last_seen'])
            if not tf_frames:
                lines.append('  - no transforms received')
            for child_frame in tf_frames:
                count = snapshot['tf_frame_counts'].get(child_frame, 0)
                last_seen = snapshot['tf_frame_last_seen'][child_frame]
                age = max(0.0, now - last_seen)
                max_gap = snapshot['tf_frame_max_gap'].get(child_frame)
                repeated = snapshot['tf_frame_repeated_stamps'].get(
                    child_frame, 0
                )
                lines.append(
                    '  - %-45s %7.2f Hz age=%s s max_gap=%s s '
                    'repeated_stamp=%d'
                    % (
                        child_frame,
                        count / elapsed if elapsed > 0.0 else 0.0,
                        _format_optional(age, precision=2),
                        _format_optional(max_gap, precision=3),
                        repeated,
                    )
                )

        lines.append(' Monitored topic receive rates/freshness:')
        for topic in sorted(self._monitored_topics):
            count = snapshot['counts'].get(topic, 0)
            last_seen = snapshot['last_seen'].get(topic)
            max_gap = snapshot['max_gap'].get(topic)
            age = None if last_seen is None else max(0.0, now - last_seen)
            lines.append(
                '  - %-55s %7.2f Hz  age=%s s  max_gap=%s s'
                % (
                    topic,
                    count / elapsed if elapsed > 0.0 else 0.0,
                    _format_optional(age, precision=2),
                    _format_optional(max_gap, precision=3),
                )
            )

    def _report(self):
        now = time.monotonic()
        elapsed = now - self._last_report_time
        self._last_report_time = now
        snapshot = self._snapshot_ros_state()
        health = self._read_host_health(elapsed)
        warnings = []

        if elapsed > self.print_interval + max(0.5, 0.25 * self.print_interval):
            warnings.append(
                f'Logger timer was delayed: interval={elapsed:.3f} s'
            )

        border = '=' * 100
        lines = [
            '',
            border,
            f'SWARM/COMPANION HEALTH REPORT (window={elapsed:.2f} s)',
            border,
        ]
        self._append_host_lines(lines, health, warnings)
        lines.append('-' * 100)
        self._append_ros_lines(lines, snapshot, elapsed, now, warnings)
        lines.append('-' * 100)
        if warnings:
            lines.append(' WARNINGS:')
            lines.extend(f'  ! {warning}' for warning in warnings)
        else:
            lines.append(' WARNINGS: none observed in this reporting window')
        lines.append(border)
        report = '\n'.join(lines)

        # /rosout is disabled for this node, so reports remain local.
        if warnings:
            self.get_logger().warning(report)
        else:
            self.get_logger().info(report)


def main(args=None):
    rclpy.init(args=args)
    node = SwarmLogger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except (RuntimeError, ValueError):
            # The context may already have destroyed entities during SIGTERM.
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
