"""Non-blocking, reconnecting serial I/O worker for a transparent radio."""

from dataclasses import dataclass
import queue
import threading

import serial


class _WriteFailure(Exception):
    """Internal marker distinguishing serial TX failures from RX failures."""


@dataclass
class SerialStats:
    """Cumulative serial worker counters."""

    connections: int = 0
    disconnects: int = 0
    tx_frames: int = 0
    tx_bytes: int = 0
    rx_bytes: int = 0
    dropped_tx_frames: int = 0
    read_errors: int = 0
    write_errors: int = 0


class SerialTransport:
    """Own the serial port in one worker so ROS callbacks never block on I/O."""

    def __init__(
        self,
        *,
        port,
        baud_rate,
        on_bytes,
        on_status=None,
        on_tx_success=None,
        reconnect_interval=2.0,
        read_timeout=0.05,
        write_timeout=1.0,
        read_size=4096,
        tx_queue_size=100,
    ):
        if not port:
            raise ValueError('serial port must not be empty')
        if int(baud_rate) <= 0:
            raise ValueError('baud_rate must be positive')
        if reconnect_interval < 0.0:
            raise ValueError('reconnect_interval must be non-negative')
        if read_timeout <= 0.0 or write_timeout <= 0.0:
            raise ValueError('serial timeouts must be positive')
        if read_size < 1 or tx_queue_size < 1:
            raise ValueError('serial read and queue sizes must be positive')

        self.port = str(port)
        self.baud_rate = int(baud_rate)
        self.on_bytes = on_bytes
        self.on_status = on_status
        self.on_tx_success = on_tx_success
        self.reconnect_interval = float(reconnect_interval)
        self.read_timeout = float(read_timeout)
        self.write_timeout = float(write_timeout)
        self.read_size = int(read_size)

        self.stats = SerialStats()
        self._tx_queue = queue.Queue(maxsize=int(tx_queue_size))
        self._stop_event = threading.Event()
        self._connected_event = threading.Event()
        self._thread = None
        self._serial = None
        self._serial_lock = threading.Lock()

    @property
    def connected(self):
        """Return whether the worker currently owns an open serial port."""
        return self._connected_event.is_set()

    def start(self):
        """Start the serial worker once."""
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._run,
            name='p900-serial',
            daemon=True,
        )
        self._thread.start()

    def stop(self, join_timeout=2.0):
        """Request shutdown and close the serial port."""
        self._stop_event.set()
        thread = self._thread
        if thread is not None:
            try:
                thread.join(timeout=join_timeout)
            except KeyboardInterrupt:
                # ros2 launch may forward a second SIGINT while the node is
                # already cleaning up after the first one.
                pass
        self._close_serial(notify=False)

    def send(self, frame):
        """
        Queue one frame without blocking the caller.

        Frames are rejected while disconnected so a flight command cannot sit
        in a queue and execute unexpectedly after a later reconnection.
        """
        if not isinstance(frame, (bytes, bytearray, memoryview)):
            raise TypeError('serial frame must be bytes-like')
        if not self.connected:
            self.stats.dropped_tx_frames += 1
            return False
        try:
            self._tx_queue.put_nowait(bytes(frame))
        except queue.Full:
            self.stats.dropped_tx_frames += 1
            return False
        return True

    def _run(self):
        while not self._stop_event.is_set():
            if not self.connected:
                try:
                    self._open_serial()
                except (OSError, ValueError, serial.SerialException) as error:
                    self._emit_status(
                        'error',
                        f'cannot open {self.port}: {error}; retrying',
                    )
                    self._stop_event.wait(self.reconnect_interval)
                    continue

            try:
                self._write_one_queued_frame()
                self._read_available_bytes()
            except _WriteFailure as error:
                self.stats.write_errors += 1
                self._disconnect(f'serial write error: {error}')
            except (OSError, serial.SerialException) as error:
                self.stats.read_errors += 1
                self._disconnect(f'serial I/O error: {error}')
            except Exception as error:  # Protect the worker from callbacks.
                self._emit_status('error', f'serial callback error: {error}')

        self._close_serial(notify=False)

    def _open_serial(self):
        serial_arguments = {
            'baudrate': self.baud_rate,
            'bytesize': serial.EIGHTBITS,
            'parity': serial.PARITY_NONE,
            'stopbits': serial.STOPBITS_ONE,
            'timeout': self.read_timeout,
            'write_timeout': self.write_timeout,
            'xonxoff': False,
            'rtscts': False,
            'dsrdtr': False,
        }
        if '://' in self.port:
            device = serial.serial_for_url(self.port, **serial_arguments)
        else:
            device = serial.Serial(port=self.port, **serial_arguments)
        device.reset_input_buffer()
        with self._serial_lock:
            self._serial = device
        self._connected_event.set()
        self.stats.connections += 1
        self._emit_status(
            'info', f'connected to {self.port} @ {self.baud_rate}'
        )

    def _write_one_queued_frame(self):
        try:
            frame = self._tx_queue.get_nowait()
        except queue.Empty:
            return

        device = self._get_serial()
        offset = 0
        try:
            while offset < len(frame):
                written = device.write(frame[offset:])
                if written is None or written <= 0:
                    raise serial.SerialTimeoutException(
                        'serial write made no progress'
                    )
                offset += written
        except (OSError, serial.SerialException) as error:
            self.stats.dropped_tx_frames += 1
            raise _WriteFailure(str(error)) from error

        self.stats.tx_frames += 1
        self.stats.tx_bytes += len(frame)
        if self.on_tx_success is not None:
            self.on_tx_success(len(frame))

    def _read_available_bytes(self):
        device = self._get_serial()
        waiting = getattr(device, 'in_waiting', 0)
        amount = min(self.read_size, waiting) if waiting else 1
        data = device.read(amount)
        if not data:
            return
        self.stats.rx_bytes += len(data)
        self.on_bytes(data)

    def _get_serial(self):
        with self._serial_lock:
            device = self._serial
        if device is None:
            raise serial.SerialException('serial device is not open')
        return device

    def _disconnect(self, reason):
        self.stats.disconnects += 1
        self._emit_status('error', reason)
        self._close_serial(notify=False)
        self._discard_queued_frames()
        self._stop_event.wait(self.reconnect_interval)

    def _discard_queued_frames(self):
        while True:
            try:
                self._tx_queue.get_nowait()
            except queue.Empty:
                return
            self.stats.dropped_tx_frames += 1

    def _close_serial(self, notify=True):
        with self._serial_lock:
            device = self._serial
            self._serial = None
        was_connected = self._connected_event.is_set()
        self._connected_event.clear()
        if device is not None:
            try:
                device.close()
            except (OSError, serial.SerialException):
                pass
        if notify and was_connected:
            self._emit_status('warning', 'serial port closed')

    def _emit_status(self, level, message):
        if self.on_status is not None:
            self.on_status(level, message)
