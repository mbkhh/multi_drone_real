"""Non-blocking UDP transport for SIYI gimbal commands."""

import socket
from typing import List, Optional

from .protocol import encode_frame, FrameParser, SiyiFrame


class SiyiUdpTransport:
    """Small non-blocking SIYI UDP client suitable for ROS timer polling."""

    def __init__(self, camera_ip: str, camera_port: int, local_port: int = 0) -> None:
        self.camera_ip = camera_ip
        self.camera_port = camera_port
        self.local_port = local_port
        self.parser = FrameParser()
        self._socket: Optional[socket.socket] = None
        self._sequence = 0
        self.tx_packets = 0
        self.tx_bytes = 0
        self.rx_packets = 0
        self.rx_bytes = 0
        self.socket_errors = 0
        self.last_error = ''

    def connect(self) -> None:
        """Create and connect the UDP socket."""
        self.close()
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if self.local_port:
            sock.bind(('0.0.0.0', self.local_port))
        sock.connect((self.camera_ip, self.camera_port))
        sock.setblocking(False)
        self._socket = sock
        self.parser.reset()
        self.last_error = ''

    def close(self) -> None:
        """Close the UDP socket if it is open."""
        if self._socket is not None:
            self._socket.close()
            self._socket = None

    @property
    def is_open(self) -> bool:
        return self._socket is not None

    def send_command(
        self, command_id: int, payload: bytes = b'', need_ack: bool = True
    ) -> int:
        """Send one command and return the sequence number assigned to it."""
        if self._socket is None:
            raise RuntimeError('UDP transport is not open')
        sequence = self._sequence
        self._sequence = (self._sequence + 1) & 0xFFFF
        packet = encode_frame(command_id, payload, sequence, need_ack)
        try:
            sent = self._socket.send(packet)
        except OSError as exc:
            self.socket_errors += 1
            self.last_error = str(exc)
            raise
        self.tx_packets += 1
        self.tx_bytes += sent
        return sequence

    def receive_available(self, limit: int = 100) -> List[SiyiFrame]:
        """Read all currently available datagrams without blocking."""
        if self._socket is None:
            return []
        frames: List[SiyiFrame] = []
        for _ in range(limit):
            try:
                data = self._socket.recv(65535)
            except BlockingIOError:
                break
            except OSError as exc:
                self.socket_errors += 1
                self.last_error = str(exc)
                break
            if not data:
                break
            self.rx_packets += 1
            self.rx_bytes += len(data)
            frames.extend(self.parser.feed(data))
        return frames
