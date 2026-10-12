"""One-shot, read-only health snapshot used by the full_status command."""

from __future__ import annotations

import math
import os
import shutil
import socket
import subprocess
import time

from px4_msgs.msg import VehicleStatus


IMPORTANT_SERVICES = (
    'swarm-xrce-agent.service',
    'swarm-p900-bridge.service',
    'swarm-control.service',
    'swarm-vision.service',
)

FULL_STATUS_RESPONSE_KIND = 'full_status_response'
FULL_STATUS_REPORT_KIND = 'full_status_report'
FULL_STATUS_SCHEMA_VERSION = 1


def valid_request_id(value):
    """Accept a short printable identifier safe to carry in diagnostic JSON."""
    if not isinstance(value, str) or not 1 <= len(value) <= 64:
        return False
    return all(character.isalnum() or character in '._:-' for character in value)


def count_ros_topics(topic_names, drone_id):
    """Count all topics and topics belonging to this drone's UAV namespace."""
    names = {str(name) for name in topic_names}
    marker = f'uav_{int(drone_id)}'
    local_names = [name for name in names if marker in name.lower()]
    return {
        'total': len(names),
        'uav_marker': marker.upper(),
        'uav_count': len(local_names),
        'uav_fmu_count': sum('/fmu/' in name.lower() for name in local_names),
    }


def query_service_states(service_names=IMPORTANT_SERVICES, runner=subprocess.run):
    """Return systemd active states using one bounded subprocess call."""
    names = tuple(service_names)
    states = {name: 'query_error' for name in names}
    error = ''
    try:
        result = runner(
            ['systemctl', 'is-active', *names],
            capture_output=True,
            text=True,
            timeout=2.0,
            check=False,
        )
        lines = [line.strip() for line in result.stdout.splitlines()]
        for index, name in enumerate(names):
            if index < len(lines) and lines[index]:
                states[name] = lines[index]
        if len(lines) < len(names):
            error = (result.stderr or 'incomplete systemctl response').strip()
    except (OSError, subprocess.SubprocessError) as exc:
        error = str(exc)
    return states, error[:200]


def _finite_float(value):
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return result if math.isfinite(result) else None


def _ros_age_seconds(node, timestamp):
    if timestamp is None:
        return None
    try:
        age = (node.get_clock().now() - timestamp).nanoseconds / 1e9
    except (AttributeError, TypeError):
        return None
    return round(max(0.0, float(age)), 3)


def _read_memory_megabytes():
    values = {}
    try:
        with open('/proc/meminfo', encoding='utf-8') as stream:
            for line in stream:
                key, raw_value = line.split(':', 1)
                if key in ('MemTotal', 'MemAvailable'):
                    values[key] = int(raw_value.split()[0]) / 1024.0
    except (OSError, ValueError, IndexError):
        pass
    return {
        'total_mb': round(values.get('MemTotal', 0.0), 1),
        'available_mb': round(values.get('MemAvailable', 0.0), 1),
    }


def _read_cpu_temperature():
    try:
        with open(
            '/sys/class/thermal/thermal_zone0/temp', encoding='utf-8'
        ) as stream:
            value = float(stream.read().strip()) / 1000.0
        return round(value, 1) if math.isfinite(value) else None
    except (OSError, ValueError):
        return None


def _read_uptime_seconds():
    try:
        with open('/proc/uptime', encoding='utf-8') as stream:
            return round(float(stream.read().split()[0]), 1)
    except (OSError, ValueError, IndexError):
        return None


def collect_full_status(node):
    """Collect a compact diagnostic report without changing controller state."""
    drone_id = int(node.frame_id)
    errors = []

    try:
        topic_names = [name for name, _types in node.get_topic_names_and_types()]
    except Exception as exc:  # ROS graph implementations can fail transiently.
        topic_names = []
        errors.append(f'ROS graph query failed: {exc}'[:200])

    service_states, service_error = query_service_states()
    if service_error:
        errors.append(f'systemd query: {service_error}'[:200])

    try:
        load_1m = round(float(os.getloadavg()[0]), 2)
    except (AttributeError, OSError):
        load_1m = None

    try:
        disk = shutil.disk_usage(os.getcwd())
        disk_report = {
            'total_mb': round(disk.total / (1024 * 1024), 1),
            'free_mb': round(disk.free / (1024 * 1024), 1),
        }
    except OSError:
        disk_report = {'total_mb': 0.0, 'free_mb': 0.0}

    vehicle_status = getattr(node, 'vehicle_status', None)
    failsafe_flags = getattr(node, 'failsafe_flags', None)
    arming_state = getattr(vehicle_status, 'arming_state', None)
    nav_state = getattr(vehicle_status, 'nav_state', None)

    navigation = getattr(node, 'navigation', None)
    position = list(getattr(navigation, 'current_pos', (None, None, None)))[:3]
    while len(position) < 3:
        position.append(None)
    position = [_finite_float(value) for value in position]

    yaw = _finite_float(getattr(node, 'current_yaw_enu', None))
    if yaw is None:
        yaw = _finite_float(getattr(node, 'yaw', None))

    neighbors = sorted(
        int(value) for value in getattr(node, 'last_seen_neighbors', {}).keys()
    )
    mission = getattr(node, 'mission', ())

    return {
        'schema': FULL_STATUS_SCHEMA_VERSION,
        'drone_id': drone_id,
        'hostname': socket.gethostname(),
        'generated_unix_ms': int(time.time() * 1000),
        'services': service_states,
        'ros_topics': count_ros_topics(topic_names, drone_id),
        'control': {
            'state': str(getattr(node, 'state', 'UNKNOWN')),
            'is_leader': bool(getattr(node, 'is_leader', False)),
            'leader_id': getattr(node, 'leader_id', None),
            'armed': arming_state == VehicleStatus.ARMING_STATE_ARMED,
            'offboard': nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD,
            'arming_state': arming_state,
            'nav_state': nav_state,
            'preflight_ok': bool(
                getattr(vehicle_status, 'pre_flight_checks_pass', False)
            ),
            'failsafe': bool(getattr(vehicle_status, 'failsafe', False)),
            'manual_signal_lost': bool(
                getattr(failsafe_flags, 'manual_control_signal_lost', False)
            ),
            'local_position_invalid': bool(
                getattr(failsafe_flags, 'local_position_invalid', False)
            ),
            'local_velocity_invalid': bool(
                getattr(failsafe_flags, 'local_velocity_invalid', False)
            ),
            'motion_enabled': bool(getattr(node, 'motion_enabled', False)),
        },
        'mission': {
            'active': bool(getattr(node, 'mission_active', False)),
            'state': str(getattr(node, 'mission_state', 'UNKNOWN')),
            'index': int(getattr(node, 'mission_index', 0)),
            'count': len(mission),
        },
        'swarm': {
            'neighbors': neighbors,
            'neighbor_count': len(neighbors),
            'peer_state_count': len(getattr(node, 'peer_states', {})),
        },
        'pose': {
            'position': position,
            'yaw_deg': None if yaw is None else round(math.degrees(yaw), 2),
        },
        'telemetry_age_s': {
            'vehicle_status': _ros_age_seconds(
                node, getattr(node, 'last_vehicle_status_time', None)
            ),
            'local_position': _ros_age_seconds(
                node, getattr(node, 'last_local_position_time', None)
            ),
            'failsafe_flags': _ros_age_seconds(
                node, getattr(node, 'last_failsafe_flags_time', None)
            ),
        },
        'system': {
            'uptime_s': _read_uptime_seconds(),
            'load_1m': load_1m,
            'cpu_temp_c': _read_cpu_temperature(),
            'memory': _read_memory_megabytes(),
            'disk': disk_report,
        },
        'errors': errors,
    }
