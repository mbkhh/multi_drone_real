"""OpenCV camera-source helpers shared by the vision nodes."""

import os
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


def open_camera(source, width=640, height=480):
    """Open a local camera index or an OpenCV-supported network stream."""
    if isinstance(source, str) and source.lower().startswith('rtsp://'):
        # TCP is more reliable than UDP on the shared airborne network. Honor
        # an explicit operator setting when one is already present.
        os.environ.setdefault(
            'OPENCV_FFMPEG_CAPTURE_OPTIONS',
            'rtsp_transport;tcp',
        )

    camera = cv2.VideoCapture(source)
    # Backends are allowed to ignore this, but supported backends retain only
    # the newest frame instead of growing a stale-frame queue.
    camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not isinstance(source, str):
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, int(width))
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, int(height))

    if not camera.isOpened():
        camera.release()
        raise RuntimeError(
            f'Could not open {describe_camera_source(source)}. Set '
            'camera_source to an RTSP/video URL or camera_index to a valid '
            '/dev/video device.'
        )
    return camera
