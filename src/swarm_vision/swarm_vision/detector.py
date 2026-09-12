"""Shared YOLO ONNX inference and portable path helpers."""

import os
from pathlib import Path
from typing import Iterable, Optional

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
import cv2
import numpy as np
import onnxruntime as ort


COCO_CLASSES = {
    0: 'person', 1: 'bicycle', 2: 'car', 3: 'motorcycle', 4: 'airplane',
    5: 'bus', 6: 'train', 7: 'truck', 8: 'boat', 9: 'traffic light',
    10: 'fire hydrant', 11: 'stop sign', 12: 'parking meter', 13: 'bench',
    14: 'bird', 15: 'cat', 16: 'dog', 17: 'horse', 18: 'sheep',
    19: 'cow', 20: 'elephant', 21: 'bear', 22: 'zebra', 23: 'giraffe',
    24: 'backpack', 25: 'umbrella', 26: 'handbag', 27: 'tie',
    28: 'suitcase', 29: 'frisbee', 30: 'skis', 31: 'snowboard',
    32: 'sports ball', 33: 'kite', 34: 'baseball bat',
    35: 'baseball glove', 36: 'skateboard', 37: 'surfboard',
    38: 'tennis racket', 39: 'bottle', 40: 'wine glass', 41: 'cup',
    42: 'fork', 43: 'knife', 44: 'spoon', 45: 'bowl', 46: 'banana',
    47: 'apple', 48: 'sandwich', 49: 'orange', 50: 'broccoli',
    51: 'carrot', 52: 'hot dog', 53: 'pizza', 54: 'donut', 55: 'cake',
    56: 'chair', 57: 'couch', 58: 'potted plant', 59: 'bed',
    60: 'dining table', 61: 'toilet', 62: 'tv', 63: 'laptop',
    64: 'mouse', 65: 'remote', 66: 'keyboard', 67: 'cell phone',
    68: 'microwave', 69: 'oven', 70: 'toaster', 71: 'sink',
    72: 'refrigerator', 73: 'book', 74: 'clock', 75: 'vase',
    76: 'scissors', 77: 'teddy bear', 78: 'hair drier', 79: 'toothbrush',
}

_COLORS = np.random.default_rng(42).integers(
    50, 255, size=(len(COCO_CLASSES), 3), dtype=np.uint8
)


def resolve_model_path(requested_path: str = '') -> Path:
    """Resolve an explicit, installed, or workspace-root YOLO model path."""
    explicit = requested_path.strip() or os.environ.get(
        'SWARM_VISION_MODEL', ''
    ).strip()
    if explicit:
        path = Path(explicit).expanduser().resolve()
        if not path.is_file():
            raise FileNotFoundError(f'YOLO model does not exist: {path}')
        return path

    candidates = []
    try:
        candidates.append(
            Path(get_package_share_directory('swarm_vision'))
            / 'models'
            / 'yolo11n.onnx'
        )
    except Exception:
        pass
    try:
        package_prefix = Path(get_package_prefix('swarm_vision')).resolve()
        candidates.append(package_prefix.parents[1] / 'yolo11n.onnx')
    except Exception:
        pass

    # Source-tree fallback: <workspace>/src/swarm_vision/swarm_vision/file.py.
    candidates.append(Path(__file__).resolve().parents[3] / 'yolo11n.onnx')
    candidates.append(Path.cwd() / 'yolo11n.onnx')

    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()
    searched = ', '.join(str(path) for path in candidates)
    raise FileNotFoundError(
        f'Could not find yolo11n.onnx. Searched: {searched}'
    )


def resolve_output_directory(requested_path: str = '') -> Path:
    """Create a persistent, user-writable vision output directory."""
    path = (
        Path(requested_path).expanduser()
        if requested_path.strip()
        else Path.home() / 'swarm_flight_logs' / 'vision'
    )
    path = path.resolve()
    path.mkdir(parents=True, exist_ok=True)
    return path


class YoloDetector:
    """Small CPU-only YOLO11 ONNX detector shared by both ROS nodes."""

    def __init__(
        self,
        model_path: Path,
        input_size: int = 320,
        confidence: float = 0.45,
        nms_threshold: float = 0.45,
        inference_threads: int = 1,
    ):
        if input_size <= 0 or input_size % 32 != 0:
            raise ValueError('input_size must be a positive multiple of 32.')
        if not 0.0 < confidence <= 1.0:
            raise ValueError('confidence must be in the range (0, 1].')
        if not 0.0 < nms_threshold <= 1.0:
            raise ValueError('nms_threshold must be in the range (0, 1].')

        options = ort.SessionOptions()
        options.intra_op_num_threads = max(1, int(inference_threads))
        options.inter_op_num_threads = 1
        options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
        options.graph_optimization_level = (
            ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        )
        self.session = ort.InferenceSession(
            str(model_path),
            sess_options=options,
            providers=['CPUExecutionProvider'],
        )
        self.input_name = self.session.get_inputs()[0].name
        self.input_size = int(input_size)
        self.confidence = float(confidence)
        self.nms_threshold = float(nms_threshold)

    def detect(
        self,
        frame: np.ndarray,
        selected_classes: Optional[Iterable[int]] = None,
        detect_all: bool = False,
    ):
        """Return detection state and an annotated copy of one BGR frame."""
        if frame is None or frame.ndim != 3 or frame.shape[2] != 3:
            raise ValueError('Expected a BGR image with three channels.')

        height, width = frame.shape[:2]
        resized = cv2.resize(frame, (self.input_size, self.input_size))
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        tensor = np.transpose(rgb.astype(np.float32) / 255.0, (2, 0, 1))
        tensor = np.expand_dims(tensor, axis=0)
        output = np.squeeze(
            self.session.run(None, {self.input_name: tensor})[0]
        )
        if output.ndim != 2:
            raise RuntimeError(
                f'Unexpected YOLO output dimensions: {output.shape}'
            )
        # Ultralytics detection exports are normally [84, anchors].
        predictions = output.T if output.shape[0] < output.shape[1] else output
        if predictions.shape[1] < 5:
            raise RuntimeError(
                f'Unexpected YOLO detection shape: {predictions.shape}'
            )

        scores = predictions[:, 4:]
        class_ids = np.argmax(scores, axis=1)
        confidences = np.max(scores, axis=1)
        allowed = {int(value) for value in (selected_classes or [])}
        scale_x = width / self.input_size
        scale_y = height / self.input_size
        boxes = []
        kept_confidences = []
        kept_class_ids = []

        for index, confidence in enumerate(confidences):
            class_id = int(class_ids[index])
            if confidence <= self.confidence:
                continue
            if not detect_all and class_id not in allowed:
                continue
            center_x, center_y, box_width, box_height = predictions[index, :4]
            x = int((center_x - box_width / 2.0) * scale_x)
            y = int((center_y - box_height / 2.0) * scale_y)
            w = int(box_width * scale_x)
            h = int(box_height * scale_y)
            boxes.append([x, y, w, h])
            kept_confidences.append(float(confidence))
            kept_class_ids.append(class_id)

        annotated = frame.copy()
        if not boxes:
            return False, annotated
        indices = cv2.dnn.NMSBoxes(
            boxes,
            kept_confidences,
            score_threshold=self.confidence,
            nms_threshold=self.nms_threshold,
        )
        selected_indices = np.asarray(indices).reshape(-1)
        for index in selected_indices:
            index = int(index)
            x, y, w, h = boxes[index]
            class_id = kept_class_ids[index]
            confidence = kept_confidences[index]
            color = tuple(
                int(value) for value in _COLORS[class_id % len(_COLORS)]
            )
            cv2.rectangle(annotated, (x, y), (x + w, y + h), color, 2)
            label = (
                f'{COCO_CLASSES.get(class_id, f"ID:{class_id}")} '
                f'{confidence:.0%}'
            )
            (text_width, text_height), _ = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1
            )
            text_y = max(y, text_height + 5)
            cv2.rectangle(
                annotated,
                (x, text_y - text_height - 4),
                (x + text_width + 4, text_y + 2),
                color,
                -1,
            )
            cv2.putText(
                annotated,
                label,
                (x + 2, text_y - 2),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1,
            )
        return bool(selected_indices.size), annotated
