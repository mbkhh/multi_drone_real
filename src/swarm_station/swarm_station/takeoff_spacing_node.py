#!/usr/bin/env python3

import itertools
import json
import math

import rclpy
from px4_msgs.msg import VehicleGlobalPosition
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, String
from swarm_config.config_utils import get_config


EARTH_RADIUS_M = 6378137.0


def relative_position_enu_m(first, second):
    """Return second minus first as a short-distance ENU vector."""
    mean_latitude = math.radians((first['lat'] + second['lat']) / 2.0)
    north = EARTH_RADIUS_M * math.radians(second['lat'] - first['lat'])
    east = (
        EARTH_RADIUS_M
        * math.cos(mean_latitude)
        * math.radians(second['lon'] - first['lon'])
    )
    first_altitude = first.get('alt')
    second_altitude = second.get('alt')
    up = None
    if first_altitude is not None and second_altitude is not None:
        up = second_altitude - first_altitude
    return east, north, up


def horizontal_distance_m(first, second):
    """Return the horizontal magnitude of the relative ENU position."""
    east, north, _up = relative_position_enu_m(first, second)
    return math.hypot(east, north)


def evaluate_spacing(positions, required_ids, origin_offsets, tolerance):
    """Compare measured pair distances with the configured takeoff layout."""
    results = []
    okay = True
    for first_id, second_id in itertools.combinations(required_ids, 2):
        first = positions[first_id]
        second = positions[second_id]
        east, north, up = relative_position_enu_m(first, second)
        actual = math.hypot(east, north)
        first_origin = origin_offsets[first_id]
        second_origin = origin_offsets[second_id]
        expected = math.hypot(
            second_origin[0] - first_origin[0],
            second_origin[1] - first_origin[1],
        )
        error = abs(actual - expected)
        pair_okay = error <= tolerance
        okay = okay and pair_okay
        results.append(
            {
                'pair': f'{first_id}-{second_id}',
                'relative_enu_m': {
                    'east': east,
                    'north': north,
                    'up': up,
                },
                'actual_m': actual,
                'expected_m': expected,
                'error_m': error,
                'okay': pair_okay,
            }
        )
    return okay, results


class TakeoffSpacingNode(Node):
    """Report whether all drones match the configured physical pad layout."""

    def __init__(self):
        super().__init__('takeoff_spacing_node')

        configured_ids = get_config(
            'swarm_single.group.required_drone_ids'
        )
        if not isinstance(configured_ids, list) or len(configured_ids) < 2:
            raise ValueError(
                'takeoff spacing requires at least two group.required_drone_ids'
            )
        self.required_ids = sorted({int(value) for value in configured_ids})
        self.origin_offsets = self._load_origin_offsets()
        self.distance_tolerance = self._config_float(
            'swarm_single.group.takeoff_spacing_check.distance_tolerance',
            0.5,
        )
        self.position_timeout = self._config_float(
            'swarm_single.group.takeoff_spacing_check.position_timeout',
            2.5,
        )
        self.max_eph = self._config_float(
            'swarm_single.group.takeoff_spacing_check.max_eph', 1.0
        )
        check_interval = self._config_float(
            'swarm_single.group.takeoff_spacing_check.check_interval', 1.0
        )
        if self.distance_tolerance < 0.0:
            raise ValueError(
                'takeoff spacing distance_tolerance cannot be negative'
            )
        if self.position_timeout <= 0.0 or check_interval <= 0.0:
            raise ValueError('takeoff spacing time values must be positive')

        self.positions = {}
        self._position_subscriptions = []
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        for drone_id in self.required_ids:
            subscription = self.create_subscription(
                VehicleGlobalPosition,
                f'/uav_{drone_id}/fmu/out/vehicle_global_position',
                lambda msg, vehicle_id=drone_id: self._position_callback(
                    vehicle_id, msg
                ),
                px4_qos,
            )
            self._position_subscriptions.append(subscription)

        self.okay_publisher = self.create_publisher(
            Bool, '/swarm/takeoff_spacing_ok', 10
        )
        self.status_publisher = self.create_publisher(
            String, '/swarm/takeoff_spacing', 10
        )
        self.create_timer(check_interval, self._check_spacing)
        self.get_logger().info(
            f'Checking takeoff spacing for drones {self.required_ids}; '
            f'tolerance=+/-{self.distance_tolerance:.2f} m, '
            f'max_eph={self.max_eph:.2f} m.'
        )

    @staticmethod
    def _config_float(key, default):
        value = get_config(key)
        return float(default if value is None else value)

    def _load_origin_offsets(self):
        configured = get_config(
            'swarm_single.group.shared_frame.origin_offsets'
        )
        if not isinstance(configured, dict):
            raise ValueError('group.shared_frame.origin_offsets is missing')

        offsets = {}
        for drone_id in self.required_ids:
            value = configured.get(drone_id)
            if value is None:
                value = configured.get(str(drone_id))
            if not isinstance(value, (list, tuple)) or len(value) != 3:
                raise ValueError(
                    f'origin offset for drone {drone_id} must be [east,north,up]'
                )
            offset = [float(component) for component in value]
            if not all(math.isfinite(component) for component in offset):
                raise ValueError(
                    f'origin offset for drone {drone_id} must be finite'
                )
            offsets[drone_id] = offset
        return offsets

    def _position_callback(self, drone_id, msg):
        valid = (
            msg.lat_lon_valid
            and not msg.dead_reckoning
            and all(
                math.isfinite(float(value))
                for value in (msg.lat, msg.lon, msg.eph)
            )
            and float(msg.eph) <= self.max_eph
        )
        self.positions[drone_id] = {
            'lat': float(msg.lat),
            'lon': float(msg.lon),
            'alt': (
                float(msg.alt)
                if msg.alt_valid and math.isfinite(float(msg.alt))
                else None
            ),
            'eph': float(msg.eph),
            'valid': valid,
            'received': self.get_clock().now(),
        }

    def _check_spacing(self):
        now = self.get_clock().now()
        unavailable = []
        usable_positions = {}
        for drone_id in self.required_ids:
            position = self.positions.get(drone_id)
            if position is None:
                unavailable.append(f'drone {drone_id}: no global position')
                continue
            age = (now - position['received']).nanoseconds / 1e9
            if age > self.position_timeout:
                unavailable.append(
                    f'drone {drone_id}: position stale ({age:.1f} s)'
                )
            elif not position['valid']:
                unavailable.append(
                    f'drone {drone_id}: invalid/inaccurate global position '
                    f'(eph={position["eph"]:.2f} m)'
                )
            else:
                usable_positions[drone_id] = position

        if unavailable:
            self._publish_result(False, '; '.join(unavailable), [])
            return

        okay, pair_results = evaluate_spacing(
            usable_positions,
            self.required_ids,
            self.origin_offsets,
            self.distance_tolerance,
        )
        pair_messages = []
        for result in pair_results:
            relative = result['relative_enu_m']
            up_text = (
                f'{relative["up"]:+.2f}'
                if relative['up'] is not None
                else 'n/a'
            )
            pair_messages.append(
                f'{result["pair"]} relative ENU '
                f'[E={relative["east"]:+.2f}, '
                f'N={relative["north"]:+.2f}, U={up_text}] m; '
                f'horizontal={result["actual_m"]:.2f} m '
                f'(expected {result["expected_m"]:.2f} +/- '
                f'{self.distance_tolerance:.2f} m)'
            )
        pair_text = '; '.join(pair_messages)
        self._publish_result(okay, pair_text, pair_results)

    def _publish_result(self, okay, detail, pairs):
        verdict = 'OK' if okay else 'NOT OK'
        message = f'TAKEOFF SPACING {verdict}: {detail}'
        if okay:
            self.get_logger().info(message)
        else:
            self.get_logger().warning(message)

        okay_msg = Bool()
        okay_msg.data = bool(okay)
        self.okay_publisher.publish(okay_msg)

        status_msg = String()
        status_msg.data = json.dumps(
            {'okay': bool(okay), 'message': message, 'pairs': pairs}
        )
        self.status_publisher.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TakeoffSpacingNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
