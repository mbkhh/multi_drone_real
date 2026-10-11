"""Small pyserial loopback test for the background serial worker."""

import threading
import time

import pytest


pytest.importorskip('serial')

from swarm_p900_bridge.serial_transport import SerialTransport  # noqa: E402


def test_loopback_serial_worker_sends_and_receives_without_ros_callbacks():
    connected = threading.Event()
    received = bytearray()
    received_event = threading.Event()
    transmitted = []

    def on_bytes(data):
        received.extend(data)
        if bytes(received) == b'loopback-frame':
            received_event.set()

    def on_status(level, message):
        if level == 'info' and message.startswith('connected'):
            connected.set()

    transport = SerialTransport(
        port='loop://',
        baud_rate=230400,
        on_bytes=on_bytes,
        on_status=on_status,
        on_tx_success=transmitted.append,
        reconnect_interval=0.05,
        read_timeout=0.01,
    )
    transport.start()
    try:
        assert connected.wait(2.0)
        assert transport.send(b'loopback-frame')
        assert received_event.wait(2.0)
        deadline = time.monotonic() + 1.0
        while not transmitted and time.monotonic() < deadline:
            time.sleep(0.01)
        assert transmitted == [len(b'loopback-frame')]
    finally:
        transport.stop()

    assert not transport.connected


def test_latest_frame_replaces_stale_frame_instead_of_queueing_it():
    transport = SerialTransport(
        port='loop://',
        baud_rate=230400,
        on_bytes=lambda data: None,
    )
    # Exercise the newest-only storage without racing the worker thread.
    transport._connected_event.set()

    first = transport.send_latest(b'state-A', key=(7, 2))
    second = transport.send_latest(b'state-E', key=(7, 2))

    assert first.accepted and not first.replaced
    assert second.accepted and second.replaced
    assert list(transport._latest_frames.values()) == [b'state-E']
    assert transport.stats.replaced_latest_frames == 1
    transport._connected_event.clear()
