from pathlib import Path

import numpy as np
import pytest

from swarm_vision.detector import resolve_model_path, YoloDetector


def test_workspace_model_runs_cpu_inference():
    model_path = resolve_model_path()
    detector = YoloDetector(
        model_path=model_path,
        input_size=320,
        inference_threads=1,
    )
    frame = np.zeros((480, 640, 3), dtype=np.uint8)

    detected, annotated = detector.detect(
        frame,
        selected_classes=[32],
    )

    assert model_path.name == 'yolo11n.onnx'
    assert not detected
    assert annotated.shape == frame.shape


def test_explicit_missing_model_is_rejected(tmp_path):
    missing = Path(tmp_path) / 'missing.onnx'

    with pytest.raises(FileNotFoundError):
        resolve_model_path(str(missing))


def test_invalid_input_size_is_rejected():
    with pytest.raises(ValueError, match='multiple of 32'):
        YoloDetector(resolve_model_path(), input_size=300)


def test_widescreen_frame_is_letterboxed_without_stretching():
    detector = YoloDetector.__new__(YoloDetector)
    detector.input_size = 640
    frame = np.zeros((720, 1280, 3), dtype=np.uint8)

    tensor, scale, pad_x, pad_y = detector._prepare_input(frame)

    assert tensor.shape == (1, 3, 640, 640)
    assert scale == pytest.approx(0.5)
    assert pad_x == 0
    assert pad_y == 140
    assert tensor.dtype == np.float32
    assert tensor[0, 0, 0, 0] == pytest.approx(114.0 / 255.0)
    assert tensor[0, 0, pad_y, 0] == 0.0
