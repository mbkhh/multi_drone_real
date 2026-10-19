"""ROS 2 node exposing SIYI ZR10 gimbal control and status topics."""

import json
import math
import struct
import time
from typing import Any, Dict

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from .commands import (
    attitude_payload,
    CMD_CAMERA_FUNCTION,
    CMD_CAMERA_SYSTEM_INFO,
    CMD_FIRMWARE_VERSION,
    CMD_GIMBAL_ATTITUDE,
    CMD_GIMBAL_MODE,
    CMD_GIMBAL_ROTATION,
    CMD_ONE_KEY_CENTERING,
    CMD_SET_GIMBAL_ATTITUDE,
    decode_response,
    mode_payload,
    speed_payload,
)
from .udp_transport import SiyiUdpTransport


class GimbalNode(Node):
    """Control a SIYI ZR10 over UDP while publishing compact JSON status."""

    def __init__(self) -> None:
        super().__init__('gimbal_node')

        self.declare_parameter('camera_ip', '192.168.144.25')
        self.declare_parameter('camera_port', 37260)
        self.declare_parameter('local_port', 0)
        self.declare_parameter('control_topic', '/swarm/gimbal/control')
        self.declare_parameter('status_topic', '/swarm/gimbal/status')
        self.declare_parameter('status_rate_hz', 5.0)
        self.declare_parameter('attitude_poll_rate_hz', 5.0)
        self.declare_parameter('connection_timeout_sec', 2.0)
        self.declare_parameter('speed_command_timeout_sec', 0.5)

        self.camera_ip = str(self.get_parameter('camera_ip').value)
        self.camera_port = int(self.get_parameter('camera_port').value)
        local_port = int(self.get_parameter('local_port').value)
        control_topic = str(self.get_parameter('control_topic').value)
        status_topic = str(self.get_parameter('status_topic').value)
        status_rate = self._positive_parameter('status_rate_hz')
        attitude_rate = self._positive_parameter('attitude_poll_rate_hz')
        self.connection_timeout = self._positive_parameter(
            'connection_timeout_sec'
        )
        self.speed_timeout = float(
            self.get_parameter('speed_command_timeout_sec').value
        )
        if not math.isfinite(self.speed_timeout) or self.speed_timeout < 0.0:
            raise ValueError('speed_command_timeout_sec must be finite and >= 0')

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.status_publisher = self.create_publisher(String, status_topic, qos)
        self.control_subscription = self.create_subscription(
            String, control_topic, self._control_callback, qos
        )

        self.transport = SiyiUdpTransport(
            self.camera_ip, self.camera_port, local_port
        )
        self.transport.connect()

        self._attitude: Dict[str, Any] = {}
        self._camera: Dict[str, Any] = {}
        self._firmware: Dict[str, Any] = {}
        self._last_ack: Dict[str, Any] = {}
        self._last_feedback = ''
        self._last_rx_monotonic = 0.0
        self._last_command = ''
        self._last_command_error = ''
        self._decoded_frames = 0
        self._decode_errors = 0
        self._speed_deadline = 0.0
        self._speed_active = False
        self._last_info_request = 0.0
        self._last_firmware_request = 0.0

        self.rx_timer = self.create_timer(0.01, self._receive_timer)
        self.poll_timer = self.create_timer(
            1.0 / attitude_rate, self._poll_timer
        )
        self.status_timer = self.create_timer(
            1.0 / status_rate, self._publish_status
        )
        self.watchdog_timer = self.create_timer(0.05, self._watchdog_timer)

        self.get_logger().info(
            'SIYI ZR10 gimbal node started\n'
            f'  UDP: {self.camera_ip}:{self.camera_port}\n'
            f'  Control: {control_topic} (std_msgs/msg/String JSON)\n'
            f'  Status: {status_topic} (std_msgs/msg/String JSON)\n'
            f'  Attitude poll: {attitude_rate:.1f} Hz\n'
            f'  Speed watchdog: {self.speed_timeout:.2f} s'
        )

    def _positive_parameter(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f'{name} must be finite and > 0')
        return value

    def _send(
        self, command_id: int, payload: bytes = b'', need_ack: bool = True
    ) -> bool:
        try:
            self.transport.send_command(command_id, payload, need_ack)
            return True
        except (OSError, RuntimeError) as exc:
            self._last_command_error = str(exc)
            self.get_logger().error(f'Could not send SIYI command: {exc}')
            return False

    def _poll_timer(self) -> None:
        now = time.monotonic()
        self._send(CMD_GIMBAL_ATTITUDE)
        if now - self._last_info_request >= 1.0:
            self._send(CMD_CAMERA_SYSTEM_INFO)
            self._send(CMD_GIMBAL_MODE)
            self._last_info_request = now
        if now - self._last_firmware_request >= 30.0:
            self._send(CMD_FIRMWARE_VERSION)
            self._last_firmware_request = now

    def _receive_timer(self) -> None:
        for frame in self.transport.receive_available():
            self._decoded_frames += 1
            self._last_rx_monotonic = time.monotonic()
            try:
                update = decode_response(frame)
            except (ValueError, struct.error) as exc:
                self._decode_errors += 1
                self.get_logger().warning(
                    f'Ignored malformed SIYI response 0x{frame.command_id:02X}: '
                    f'{exc}'
                )
                continue
            self._apply_update(update)

    def _apply_update(self, update: Dict[str, Any]) -> None:
        if 'attitude' in update:
            self._attitude.update(update['attitude'])
        if 'camera' in update:
            self._camera.update(update['camera'])
        if 'firmware' in update:
            self._firmware.update(update['firmware'])
        if 'ack' in update:
            self._last_ack = dict(update['ack'])
            self._last_ack['stamp'] = self.get_clock().now().nanoseconds / 1e9
        if 'function_feedback' in update:
            self._last_feedback = str(update['function_feedback'])

    def _control_callback(self, message: String) -> None:
        try:
            request = self._parse_control(message.data)
            command = str(request.get('command', '')).strip().lower()
            if not command:
                raise ValueError('missing command')

            if command in ('speed', 'move'):
                payload = speed_payload(
                    request.get('yaw', 0), request.get('pitch', 0)
                )
                moving = payload != b'\x00\x00'
                if self._send(CMD_GIMBAL_ROTATION, payload):
                    self._speed_active = moving
                    self._speed_deadline = (
                        time.monotonic() + self.speed_timeout
                        if moving and self.speed_timeout > 0.0
                        else 0.0
                    )

            elif command == 'stop':
                if self._send(CMD_GIMBAL_ROTATION, speed_payload(0, 0)):
                    self._speed_active = False
                    self._speed_deadline = 0.0

            elif command in ('angle', 'set_angle'):
                payload = attitude_payload(
                    request.get('yaw_deg', request.get('yaw')),
                    request.get('pitch_deg', request.get('pitch')),
                )
                self._send(CMD_SET_GIMBAL_ATTITUDE, payload)

            elif command == 'center':
                self._send(CMD_ONE_KEY_CENTERING, b'\x01')

            elif command in ('down', 'downward'):
                self._send(CMD_ONE_KEY_CENTERING, b'\x02')

            elif command == 'mode':
                # Camera function 3/4/5 selects lock/follow/FPV mode.
                self._send(
                    CMD_CAMERA_FUNCTION,
                    mode_payload(request.get('mode', '')),
                    need_ack=False,
                )

            elif command in ('query', 'status'):
                self._send(CMD_GIMBAL_ATTITUDE)
                self._send(CMD_CAMERA_SYSTEM_INFO)
                self._send(CMD_GIMBAL_MODE)

            else:
                raise ValueError(
                    'unknown command; use speed, stop, angle, center, down, '
                    'mode, or query'
                )

            self._last_command = command
            self._last_command_error = ''
            self.get_logger().info(f'Gimbal command accepted: {command}')
        except (ValueError, TypeError, json.JSONDecodeError) as exc:
            self._last_command_error = str(exc)
            self.get_logger().error(f'Rejected gimbal control message: {exc}')

    @staticmethod
    def _parse_control(text: str) -> Dict[str, Any]:
        stripped = text.strip()
        if not stripped:
            raise ValueError('empty control message')
        if stripped.startswith('{'):
            parsed = json.loads(stripped)
            if not isinstance(parsed, dict):
                raise ValueError('control JSON must be an object')
            return parsed
        return {'command': stripped}

    def _watchdog_timer(self) -> None:
        if (
            self._speed_active
            and self._speed_deadline > 0.0
            and time.monotonic() >= self._speed_deadline
        ):
            self._send(CMD_GIMBAL_ROTATION, speed_payload(0, 0))
            self._speed_active = False
            self._speed_deadline = 0.0
            self.get_logger().warning(
                'Gimbal speed watchdog stopped motion; refresh speed commands '
                'to move continuously.'
            )

    def _publish_status(self) -> None:
        now_monotonic = time.monotonic()
        age = (
            now_monotonic - self._last_rx_monotonic
            if self._last_rx_monotonic > 0.0
            else None
        )
        status = {
            'stamp': self.get_clock().now().nanoseconds / 1e9,
            'connected': age is not None and age <= self.connection_timeout,
            'endpoint': f'{self.camera_ip}:{self.camera_port}',
            'last_rx_age_sec': round(age, 3) if age is not None else None,
            'attitude': self._attitude or None,
            'camera': self._camera or None,
            'firmware': self._firmware or None,
            'last_ack': self._last_ack or None,
            'last_function_feedback': self._last_feedback or None,
            'last_command': self._last_command or None,
            'last_command_error': self._last_command_error or None,
            'speed_watchdog_active': self._speed_active,
            'transport': {
                'tx_packets': self.transport.tx_packets,
                'tx_bytes': self.transport.tx_bytes,
                'rx_packets': self.transport.rx_packets,
                'rx_bytes': self.transport.rx_bytes,
                'decoded_frames': self._decoded_frames,
                'crc_errors': self.transport.parser.crc_errors,
                'malformed_frames': (
                    self.transport.parser.malformed_frames
                    + self._decode_errors
                ),
                'discarded_bytes': self.transport.parser.discarded_bytes,
                'socket_errors': self.transport.socket_errors,
                'last_socket_error': self.transport.last_error or None,
            },
        }
        output = String()
        output.data = json.dumps(status, separators=(',', ':'), allow_nan=False)
        self.status_publisher.publish(output)

    def shutdown(self) -> None:
        """Stop any rate motion and close the UDP socket."""
        if self.transport.is_open:
            self._send(CMD_GIMBAL_ROTATION, speed_payload(0, 0))
            self.transport.close()


def main(args=None) -> None:
    """Run the gimbal node."""
    rclpy.init(args=args)
    node = GimbalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
