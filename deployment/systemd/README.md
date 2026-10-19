# Swarm drone automatic startup

This deployment starts the four companion-computer processes with `systemd`:

1. `swarm-xrce-agent.service` — PX4 Micro XRCE Agent;
2. `swarm-p900-bridge.service` — selected-topic P900 bridge;
3. `swarm-control.service` — `swarm_single_no_tf_yaw` controller; and
4. `swarm-vision.service` — triggered onboard vision.

`swarm-drone.target` starts and stops all four. Each process has independent
logs and automatic restart. No service sends an arm, takeoff, mission or land
command; those still require an explicit station command.

The XRCE Agent intentionally starts in a clean standalone environment and does
**not** source ROS. Sourcing ROS before the `/usr/local` Agent can mix Fast DDS
libraries and has previously caused `stack smashing detected` when PX4 connects.

## 1. Prepare each Raspberry Pi

Perform these steps separately on Drone 1, Drone 2 and Drone 3.

Update and build the existing workspace:

```bash
cd /home/mbkh/multi_drone_real
git pull
source /opt/ros/rolling/setup.sh
colcon build --packages-up-to \
  swarm_p900_bridge swarm_single_no_tf_yaw swarm_vision
source install/setup.sh
```

The existing vision environment must already be available:

```bash
test -x /home/mbkh/multi_drone_real/.venv-swarm-vision/bin/python
```

Verify the Micro XRCE Agent location and all devices:

```bash
command -v MicroXRCEAgent
ls -l /dev/ttyAMA0
ls -l /dev/serial/by-id/
ls -l /dev/video0
```

The expected Agent path is `/usr/local/bin/MicroXRCEAgent`. Record the P900's
complete `/dev/serial/by-id/...` path. It is safer than `/dev/ttyUSB0`, whose
number can change after a reconnect.

Ensure the service account can access serial devices and the camera:

```bash
sudo usermod -aG dialout,video mbkh
```

If group membership was just changed, reboot once before continuing.

## 2. Install the systemd files

Run on each Pi:

```bash
cd /home/mbkh/multi_drone_real
sudo ./deployment/systemd/install.sh \
  --user mbkh \
  --workspace /home/mbkh/multi_drone_real
```

The installer copies the runner to `/usr/local/libexec/swarm-drone-run`, puts
the units in `/etc/systemd/system`, creates `/etc/swarm-drone.conf` only when it
does not already exist, and reloads systemd. It deliberately does not start or
enable the flight stack.

## 3. Configure each drone

First get the P900 device name on that Pi:

```bash
ls -l /dev/serial/by-id/
```

Then edit:

```bash
sudoedit /etc/swarm-drone.conf
```

Keep these common settings on all drones:

```ini
SWARM_WORKSPACE=/home/mbkh/multi_drone_real
ROS_SETUP=/opt/ros/rolling/setup.sh
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ROS_DOMAIN_ID=0

P900_SERIAL_PORT=/dev/serial/by-id/REPLACE_WITH_THE_REAL_DEVICE
P900_BAUD_RATE=230400
P900_DIAGNOSTICS_INTERVAL=10.0

XRCE_AGENT_BIN=/usr/local/bin/MicroXRCEAgent
XRCE_SERIAL_PORT=/dev/ttyAMA0
XRCE_BAUD_RATE=921600

VISION_CAMERA_SOURCE=
VISION_CAMERA_INDEX=0
VISION_INPUT_SIZE=640
VISION_INFERENCE_THREADS=1
```

For a SIYI ZR10 network stream, set the source instead of relying on a local
`/dev/video` device:

```ini
VISION_CAMERA_SOURCE=rtsp://192.168.144.25:8554/main.264
```

The companion computer must have a route to the camera subnet. Verify it with
`ip route get 192.168.144.25` and test TCP port 8554 before starting vision.

Use the following identity values.

### Drone 1 — leader / P900 Master

```ini
DRONE_ID=1
UAV_ID=UAV_1
P900_ROLE=leader
P900_NODE_ID=1
```

### Drone 2 — follower / P900 Remote ID 2

```ini
DRONE_ID=2
UAV_ID=UAV_2
P900_ROLE=follower
P900_NODE_ID=2
```

### Drone 3 — follower / P900 Remote ID 3

```ini
DRONE_ID=3
UAV_ID=UAV_3
P900_ROLE=follower
P900_NODE_ID=3
```

`P900_NODE_ID` is the bridge's application node ID and must match the planned
radio/node identity. Future followers use their own positive drone/node ID and
the same `P900_ROLE=follower` service.

Keep `ROS_DOMAIN_ID=0` unless PX4's `UXRCE_DDS_DOM_ID` is deliberately changed
to the same value. Choosing an arbitrary different ROS domain can hide all PX4
topics from the controller.

### Keep the P900 test isolated from ordinary DDS networking

These services do not change Wi-Fi or DDS networking. If the station and all
Pis remain on the same LAN in the same ROS domain, ROS 2 can deliver the
transported topics directly over Wi-Fi, bypassing P900 and potentially feeding
a republished topic back into another bridge. That is the same reason the
three-radio laptop test required isolated ROS domains.

For a P900-only flight, keep the ROS graphs isolated using the networking plan
you have bench-tested (for example no cross-machine Wi-Fi DDS). If you choose
separate domain IDs while retaining Wi-Fi/SSH, PX4's `UXRCE_DDS_DOM_ID` must be
changed to the matching domain on each aircraft as well. Do not change only
the service's `ROS_DOMAIN_ID`, because that disconnects the controller from
PX4 telemetry.

## 4. Safe first start

Remove propellers for the first boot test. Confirm no old launcher or manually
started duplicate processes are running:

```bash
pgrep -af 'MicroXRCEAgent|swarm_p900_bridge|swarm_single_no_tf_yaw|swarm_vision'
```

Start the four services without enabling boot startup yet:

```bash
sudo systemctl start swarm-drone.target
```

Inspect all services:

```bash
systemctl --no-pager --full status \
  swarm-xrce-agent.service \
  swarm-p900-bridge.service \
  swarm-control.service \
  swarm-vision.service
```

Expected results:

- Agent: serial transport is running and PX4 establishes a session;
- P900: `connected` to the configured serial path and shows the correct role;
- control: the correct `control_node_no_tf_yaw_dN` initializes and remains
  idle/disarmed until an explicit station ARM command;
- vision: the camera opens and `Flight vision online for UAV_N` is logged.

Follow individual logs in separate terminals:

```bash
journalctl -fu swarm-xrce-agent.service
journalctl -fu swarm-p900-bridge.service
journalctl -fu swarm-control.service
journalctl -fu swarm-vision.service
```

Show logs from the current boot without following:

```bash
journalctl -b --no-pager \
  -u swarm-xrce-agent.service \
  -u swarm-p900-bridge.service \
  -u swarm-control.service \
  -u swarm-vision.service
```

Stop the complete test stack:

```bash
sudo systemctl stop swarm-drone.target
```

Verify all four services are inactive and that the drone remained disarmed:

```bash
systemctl is-active \
  swarm-xrce-agent.service \
  swarm-p900-bridge.service \
  swarm-control.service \
  swarm-vision.service
```

## 5. Enable startup after the bench test

Only after all four services pass the propeller-off test:

```bash
sudo systemctl enable swarm-drone.target
sudo reboot
```

After reboot, reconnect over SSH and verify:

```bash
systemctl is-enabled swarm-drone.target
systemctl is-active swarm-drone.target
systemctl --no-pager --full status \
  swarm-xrce-agent.service \
  swarm-p900-bridge.service \
  swarm-control.service \
  swarm-vision.service
```

Also verify through the station that the expected drone is visible, is
disarmed, and has not entered Offboard by itself.

## Normal operation

Start or stop the complete stack:

```bash
sudo systemctl start swarm-drone.target
sudo systemctl stop swarm-drone.target
sudo systemctl restart swarm-drone.target
```

Restart only one component:

```bash
sudo systemctl restart swarm-p900-bridge.service
```

Disable automatic startup and stop the stack:

```bash
sudo systemctl disable --now swarm-drone.target
```

Do not stop or restart the target while an aircraft is flying. Land and disarm
first. A controller or XRCE restart during flight interrupts Offboard/PX4 data
and must be treated as a tested failsafe event, not a normal operation.

## Updating the software

Land and disarm before updating. Then:

```bash
sudo systemctl stop swarm-drone.target
cd /home/mbkh/multi_drone_real
git pull
source /opt/ros/rolling/setup.sh
colcon build --packages-up-to \
  swarm_p900_bridge swarm_single_no_tf_yaw swarm_vision
```

If anything under `deployment/systemd` changed, reinstall the units/runner:

```bash
sudo ./deployment/systemd/install.sh \
  --user mbkh \
  --workspace /home/mbkh/multi_drone_real
```

The installer preserves the existing `/etc/swarm-drone.conf`. Start the target
and inspect the logs before the next flight.

## Troubleshooting

Show the most recent failure for one component:

```bash
journalctl -u swarm-p900-bridge.service -n 100 --no-pager
```

If a serial service repeatedly restarts, check the configured path and whether
another process owns it:

```bash
sudo fuser -v /dev/ttyAMA0
sudo fuser -v /dev/serial/by-id/YOUR_P900_DEVICE
```

If `MicroXRCEAgent` reports `stack smashing detected`, confirm it is being
started by `swarm-xrce-agent.service` and not by a shell/script that sourced
ROS. The supplied Agent service explicitly clears ROS and library-path
variables.

If the vision service fails, verify the virtual environment and camera:

```bash
test -x /home/mbkh/multi_drone_real/.venv-swarm-vision/bin/python
ls -l /dev/video0
journalctl -u swarm-vision.service -n 100 --no-pager
```

If desired, make journal logs survive reboot (at the cost of SD-card writes):

```bash
sudo mkdir -p /var/log/journal
sudo systemd-tmpfiles --create --prefix /var/log/journal
sudo systemctl restart systemd-journald
```
