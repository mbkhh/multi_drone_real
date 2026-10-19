from swarm_vision.camera import (
    describe_camera_source,
    resolve_camera_source,
)


def test_explicit_camera_source_overrides_index():
    source = 'rtsp://192.168.144.25:8554/main.264'

    assert resolve_camera_source(source, 3) == source


def test_empty_camera_source_uses_integer_index():
    assert resolve_camera_source('', 2) == 2


def test_camera_description_redacts_url_password():
    description = describe_camera_source(
        'rtsp://operator:secret@camera.local:8554/main.264'
    )

    assert description == 'rtsp://operator:***@camera.local:8554/main.264'
    assert 'secret' not in description
