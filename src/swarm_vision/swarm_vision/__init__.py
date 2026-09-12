"""ROS 2 swarm vision package."""

import os


# This runs before detector imports the ONNX Runtime native module.
os.environ.setdefault('ORT_DISABLE_TELEMETRY', '1')
