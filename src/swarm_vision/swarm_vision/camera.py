"""OpenCV camera-source helpers shared by the vision nodes."""

import os
import time
from urllib.parse import urlsplit, urlunsplit

import cv2


def resolve_camera_source(camera_source, camera_index):
    """Prefer an explicit stream/device string, otherwise use a camera index."""
    source = str(camera_source).strip()
    if source:
        return source
    return int(camera_index)


def describe_camera_source(source):
    """Return a log-safe description without exposing URL credentials."""
    if not isinstance(source, str):
        return f'camera index {source}'

    parsed = urlsplit(source)
    if parsed.username is None:
        return source

    hostname = parsed.hostname or ''
    if parsed.port is not None:
        hostname = f'{hostname}:{parsed.port}'
    safe_netloc = f'{parsed.username}:***@{hostname}'
    return urlunsplit(
        (parsed.scheme, safe_netloc, parsed.path, parsed.query, parsed.fragment)
    )


def _is_rtsp_source(source):
    return isinstance(source, str) and source.lower().startswith('rtsp://')


def build_rtsp_gstreamer_pipeline(source):
    """Build a low-latency RTSP pipeline that keeps the session alive."""
    escaped_source = source.replace('\\', '\\\\').replace('"', '\\"')
    return (
        f'rtspsrc location="{escaped_source}" protocols=tcp latency=100 '
        'drop-on-latency=true do-rtsp-keep-alive=true '
        'tcp-timeout=5000000 ! '
        'rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! '
        'video/x-raw,format=BGR ! '
        'appsink drop=true max-buffers=1 sync=false'
    )


class ResilientCamera:
    """
    VideoCapture-compatible camera with bounded RTSP reconnection.

    The ZR10 RTSP server can leave a TCP socket established after video data
    stops.  GStreamer's RTSP keepalive prevents its normal session timeout;
    its TCP timeout plus this wrapper also recover from a real link outage.
    """

    def __init__(self, source, width=640, height=480):
        self.source = source
        self.width = int(width)
        self.height = int(height)
        self.is_rtsp = _is_rtsp_source(source)
        self.capture = None
        self.backend = ''
        self.reconnect_count = 0
        self._next_reconnect_time = 0.0
        if not self._connect(initial=True):
            raise RuntimeError(
                f'Could not open {describe_camera_source(source)}. Set '
                'camera_source to an RTSP/video URL or camera_index to a '
                'valid /dev/video device.'
            )

    def _open_gstreamer(self):
        pipeline = build_rtsp_gstreamer_pipeline(self.source)
        capture = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        return capture

    def _open_fallback(self):
        if self.is_rtsp:
            # Used only on machines whose OpenCV lacks GStreamer. Limit I/O
            # blocking so the reconnect loop can still recover.
            os.environ.setdefault(
                'OPENCV_FFMPEG_CAPTURE_OPTIONS',
                'rtsp_transport;tcp|stimeout;5000000',
            )
            capture = cv2.VideoCapture()
            if hasattr(cv2, 'CAP_PROP_OPEN_TIMEOUT_MSEC'):
                capture.set(cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 5000)
            if hasattr(cv2, 'CAP_PROP_READ_TIMEOUT_MSEC'):
                capture.set(cv2.CAP_PROP_READ_TIMEOUT_MSEC, 5000)
            capture.open(self.source, cv2.CAP_FFMPEG)
        else:
            capture = cv2.VideoCapture(self.source)
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return capture

    def _connect(self, initial=False):
        self._release_capture()
        candidates = []
        if self.is_rtsp:
            candidates.append(('gstreamer', self._open_gstreamer))
        candidates.append(('opencv', self._open_fallback))

        for backend, opener in candidates:
            capture = opener()
            if capture.isOpened():
                self.capture = capture
                self.backend = backend
                if not initial:
                    self.reconnect_count += 1
                    print(
                        'Vision camera reconnected: '
                        f'{describe_camera_source(self.source)} '
                        f'via {backend} (count={self.reconnect_count})',
                        flush=True,
                    )
                return True
            capture.release()

        self.capture = None
        self.backend = ''
        self._next_reconnect_time = time.monotonic() + 1.0
        return False

    def _release_capture(self):
        if self.capture is not None:
            self.capture.release()
            self.capture = None

    def _mark_disconnected(self):
        if self.capture is not None:
            print(
                'Vision camera stopped delivering frames; reconnecting to '
                f'{describe_camera_source(self.source)}',
                flush=True,
            )
        self._release_capture()
        self.backend = ''
        self._next_reconnect_time = time.monotonic() + 1.0

    def isOpened(self):
        if self.capture is not None and self.capture.isOpened():
            return True
        if time.monotonic() < self._next_reconnect_time:
            return False
        return self._connect()

    def read(self):
        if not self.isOpened():
            return False, None
        success, frame = self.capture.read()
        if not success:
            self._mark_disconnected()
            return False, None
        return True, frame

    def grab(self):
        if not self.isOpened():
            return False
        success = self.capture.grab()
        if not success:
            self._mark_disconnected()
        return success

    def release(self):
        self._release_capture()
        self.backend = ''


def open_camera(source, width=640, height=480):
    """Open a local camera or a resilient, latest-frame RTSP reader."""
    return ResilientCamera(source, width=width, height=height)
