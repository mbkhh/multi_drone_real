# swarm_gimbal

`swarm_gimbal` exposes a SIYI ZR10 gimbal as two ROS 2 topics. It talks to
the camera directly through the SIYI UDP SDK protocol; it does not modify the
flight controller, station, vision, or swarm-control nodes.

The package is self-contained and does not require a third-party Python SIYI
package. Its framing and command fields follow the SIYI Gimbal SDK protocol.

## Topics

- `/swarm/gimbal/control` (`std_msgs/msg/String`): commands, normally JSON.
- `/swarm/gimbal/status` (`std_msgs/msg/String`): periodic JSON status.

Status contains the last gimbal yaw/pitch/roll, angular rates, mode, recording
state, mounting direction, firmware versions, connection age, last command
acknowledgement, and UDP/protocol counters. `connected` becomes false when no
valid response has arrived within `connection_timeout_sec`.

## Build and run

```bash
cd ~/multi_drone_real
source /opt/ros/humble/setup.bash
colcon build --packages-select swarm_gimbal
source install/setup.bash
ros2 launch swarm_gimbal gimbal.launch.py \
  camera_ip:=192.168.144.25 \
  camera_port:=37260
```

On a Rolling system, source `/opt/ros/rolling/setup.bash` instead. The Jetson
must have an IP route to `192.168.144.25`; video and control use the same
camera network but different ports.

Watch status:

```bash
ros2 topic echo /swarm/gimbal/status std_msgs/msg/String
```

## Commands

Center the gimbal:

```bash
ros2 topic pub --once /swarm/gimbal/control std_msgs/msg/String \
  "data: '{\"command\":\"center\"}'"
```

Move to an absolute ZR10 angle:

```bash
ros2 topic pub --once /swarm/gimbal/control std_msgs/msg/String \
  "data: '{\"command\":\"angle\",\"yaw_deg\":30.0,\"pitch_deg\":-45.0}'"
```

The ZR10 limits enforced by the node are yaw `-135..135` degrees and pitch
`-90..25` degrees.

Speed control uses integers from `-100..100`. A nonzero speed command is
automatically stopped after 0.5 seconds by default. Publish repeatedly to keep
moving; this prevents a lost publisher from leaving the gimbal rotating.

```bash
ros2 topic pub -r 5 /swarm/gimbal/control std_msgs/msg/String \
  "data: '{\"command\":\"speed\",\"yaw\":30,\"pitch\":0}'"

ros2 topic pub --once /swarm/gimbal/control std_msgs/msg/String \
  "data: '{\"command\":\"stop\"}'"
```

Other commands:

```json
{"command":"down"}
{"command":"mode","mode":"lock"}
{"command":"mode","mode":"follow"}
{"command":"mode","mode":"fpv"}
{"command":"query"}
```

For continuous speed motion, set `speed_command_timeout_sec:=0.0` only when a
higher-level controller provides its own watchdog.

## Parameters

| Parameter | Default | Meaning |
|---|---:|---|
| `camera_ip` | `192.168.144.25` | ZR10 SDK address |
| `camera_port` | `37260` | ZR10 SDK UDP port |
| `local_port` | `0` | Local UDP port (`0` = automatic) |
| `control_topic` | `/swarm/gimbal/control` | ROS command topic |
| `status_topic` | `/swarm/gimbal/status` | ROS status topic |
| `status_rate_hz` | `5.0` | Status publishing rate |
| `attitude_poll_rate_hz` | `5.0` | Gimbal attitude query rate |
| `connection_timeout_sec` | `2.0` | No-response disconnect threshold |
| `speed_command_timeout_sec` | `0.5` | Automatic rate-command stop time |

