"""Streamless ROS 2 YOLO node intended for onboard flight use."""

from datetime import datetime
import threading
import time

import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from swarm_vision.detector import (
    COCO_CLASSES,
    resolve_model_path,
    resolve_output_directory,
    YoloDetector,
)


class FlightVisionNode(Node):
    """Run triggered object detection without a video web stream."""

    def __init__(self):
        super().__init__('flight_vision_node')
        self.declare_parameter('uav_id', 'UAV_1')
        self.declare_parameter('model_path', '')
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('camera_width', 640)
        self.declare_parameter('camera_height', 480)
        self.declare_parameter('input_size', 320)
        self.declare_parameter('confidence', 0.45)
        self.declare_parameter('nms_threshold', 0.45)
        self.declare_parameter('inference_threads', 1)
        self.declare_parameter('output_directory', '')
        self.declare_parameter('snapshot_interval', 1.0)
        self.declare_parameter('command_interval', 1.0)

        self.uav_id = self.get_parameter('uav_id').value
        model_path = resolve_model_path(self.get_parameter('model_path').value)
        output_root = resolve_output_directory(
            self.get_parameter('output_directory').value
        )
        self.save_directory = output_root / 'detections'
        self.save_directory.mkdir(parents=True, exist_ok=True)
        self.snapshot_interval = max(
            0.0, float(self.get_parameter('snapshot_interval').value)
        )
        self.command_interval = max(
            0.0, float(self.get_parameter('command_interval').value)
        )

        self.detector = YoloDetector(
            model_path=model_path,
            input_size=int(self.get_parameter('input_size').value),
            confidence=float(self.get_parameter('confidence').value),
            nms_threshold=float(self.get_parameter('nms_threshold').value),
            inference_threads=int(
                self.get_parameter('inference_threads').value
            ),
        )

        camera_index = int(self.get_parameter('camera_index').value)
        self.camera = cv2.VideoCapture(camera_index)
        self.camera.set(
            cv2.CAP_PROP_FRAME_WIDTH,
            int(self.get_parameter('camera_width').value),
        )
        self.camera.set(
            cv2.CAP_PROP_FRAME_HEIGHT,
            int(self.get_parameter('camera_height').value),
        )
        if not self.camera.isOpened():
            self.camera.release()
            raise RuntimeError(
                f'Could not open camera index {camera_index}. Set '
                'camera_index to the correct /dev/video device.'
            )

        self.command_publisher = self.create_publisher(
            String, '/swarm/vision_command', 10
        )
        self.trigger_subscription = self.create_subscription(
            String,
            '/swarm/vision_trigger',
            self.trigger_callback,
            10,
        )
        self.active_classes = [32]
        self.detection_enabled = False
        self.last_saved_time = 0.0
        self.last_command_time = 0.0
        self.stop_event = threading.Event()
        self.worker = threading.Thread(
            target=self.vision_loop,
            name='swarm-vision-inference',
            daemon=True,
        )
        self.worker.start()
        self.get_logger().info(
            f'Flight vision online for {self.uav_id}; model={model_path}, '
            f'camera={camera_index}, output={self.save_directory}'
        )

    def trigger_callback(self, msg):
        """Enable selected COCO classes with START[:ids], or stop detection."""
        payload = msg.data.strip().upper()
        if payload.startswith('START'):
            if ':' in payload:
                requested = []
                for value in payload.split(':', 1)[1].split(','):
                    value = value.strip()
                    if not value:
                        continue
                    try:
                        class_id = int(value)
                    except ValueError:
                        self.get_logger().warning(
                            f'Ignoring invalid class ID: {value}'
                        )
                        continue
                    if class_id in COCO_CLASSES:
                        requested.append(class_id)
                    else:
                        self.get_logger().warning(
                            f'Ignoring unknown COCO class ID: {class_id}'
                        )
                if requested:
                    self.active_classes = requested
            self.detection_enabled = True
            names = [COCO_CLASSES[value] for value in self.active_classes]
            self.get_logger().info(
                f'Detection active for {self.active_classes} ({names}).'
            )
        elif payload in ('STOP', 'DISABLE'):
            self.detection_enabled = False
            self.get_logger().info('Detection stopped.')
        else:
            self.get_logger().warning(
                'Unknown vision trigger. Use START, START:32,0,29, or STOP.'
            )

    def publish_detection_command(self):
        """Publish at a bounded rate while the selected target remains visible."""
        now = time.monotonic()
        if now - self.last_command_time < self.command_interval:
            return
        self.last_command_time = now
        message = String()
        message.data = f'{self.uav_id}:TARGET_DETECTED_LAND'
        self.command_publisher.publish(message)
        self.get_logger().info(f'Published vision command: {message.data}')

    def vision_loop(self):
        """Read frames and run inference outside the ROS executor thread."""
        while not self.stop_event.is_set():
            if not self.detection_enabled:
                self.stop_event.wait(0.08)
                continue
            success, frame = self.camera.read()
            if not success:
                self.stop_event.wait(0.05)
                continue
            try:
                detected, annotated = self.detector.detect(
                    frame, selected_classes=self.active_classes
                )
            except Exception as error:
                self.get_logger().error(f'Vision inference failed: {error}')
                self.detection_enabled = False
                continue
            if not detected:
                continue

            self.publish_detection_command()
            now = time.monotonic()
            if now - self.last_saved_time < self.snapshot_interval:
                continue
            self.last_saved_time = now
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            path = self.save_directory / f'detection_{timestamp}.jpg'
            cv2.imwrite(str(path), annotated)

    def close(self):
        """Stop the worker and release the camera deterministically."""
        self.stop_event.set()
        if self.worker.is_alive():
            self.worker.join(timeout=2.0)
        self.camera.release()


def main(args=None):
    """Run the streamless flight vision node."""
    rclpy.init(args=args)
    node = None
    failure = None
    try:
        node = FlightVisionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        failure = error
        if node is not None:
            node.get_logger().fatal(str(error))
        else:
            print(f'Failed to start flight vision: {error}')
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if failure is not None:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
