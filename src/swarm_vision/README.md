# Swarm vision

This package provides two CPU-only YOLO11 ONNX ROS 2 nodes:

- `flight_vision_node`: triggered, streamless detection for onboard use;
- `vision_node`: detection, recording, and a Flask MJPEG web interface.

Both use the workspace-root `yolo11n.onnx` model. The model is also copied to
the installed package share directory during a build. Outputs default to the
persistent, user-writable `~/swarm_flight_logs/vision` directory.

## Isolated installation

Do not install the vision requirements globally or use the workspace's large
top-level `requirements.txt`. From the workspace root, run:

```bash
python3 -m venv --system-site-packages .venv-swarm-vision
.venv-swarm-vision/bin/pip install -r src/swarm_vision/requirements.txt
source /opt/ros/rolling/setup.bash
colcon build --packages-select swarm_vision
source install/setup.bash
```

The local environment reuses Ubuntu's ROS 2, NumPy, and OpenCV packages. Only
Flask, ONNX Runtime CPU, and their small Python dependencies are installed in
`.venv-swarm-vision`. The supplied runner disables `~/.local` Python packages
for the vision process to avoid version conflicts.

## Run

List cameras first:

```bash
ls -l /dev/video*
```

Start the lightweight flight node:

```bash
./run_swarm_vision.sh flight --ros-args \
  -p uav_id:=UAV_1 \
  -p camera_index:=0
```

Enable sports-ball detection (COCO class 32), or stop it:

```bash
ros2 topic pub --once /swarm/vision_trigger std_msgs/msg/String \
  "{data: 'START:32'}"
ros2 topic pub --once /swarm/vision_trigger std_msgs/msg/String \
  "{data: 'STOP'}"
```

When `swarm_single_no_tf_yaw` and the station are running, use the station
commands instead of publishing these topics manually:

```text
start_detection
start_detection 32
start_detection 0,2,32
stop_detection
```

The station sends the request to the elected leader. The leader publishes the
corresponding `START:<class IDs>` or `STOP` trigger. A detection is published
as `UAV_ID:TARGET_DETECTED` on `/swarm/vision_command`; the leader puts a
report in its existing `/swarm/status` message, and the station prints it as
`Message from leader: VISION TARGET DETECTED by UAV_ID`.

This integration is report-only. A detection never changes a goal or flight
state and never sends LAND, RTL, arm, disarm, or PX4 commands. The leader also
accepts the older `TARGET_DETECTED_LAND` event name only so different drones
can be upgraded without interpreting it as a landing request.

Both vision nodes wait for a `START` trigger before running inference. The web
node can still show and record its camera stream while detection is stopped.

Start the web node:

```bash
./run_swarm_vision.sh web --ros-args \
  -p uav_id:=UAV_1 \
  -p camera_index:=0 \
  -p web_host:=0.0.0.0 \
  -p web_port:=5000
```

Open `http://DRONE_IP:5000/` from another machine. The development web server
has no authentication, so expose it only on a trusted flight network.

Useful parameters shared by both nodes:

- `model_path`: explicit ONNX path; empty auto-discovers `yolo11n.onnx`;
- `camera_index`: OpenCV camera number;
- `input_size`: inference size, default `320` (must be a multiple of 32);
- `confidence`: detection threshold, default `0.45`;
- `nms_threshold`: non-maximum suppression threshold, default `0.45`;
- `inference_threads`: ONNX CPU threads, default `1` to protect flight control;
- `output_directory`: persistent output root;
- `command_interval`: minimum seconds between detection commands, default `1`.

The supplied model metadata identifies it as an Ultralytics AGPL-3.0 model.
Check the applicable Ultralytics license before distributing the model or a
system containing it.
