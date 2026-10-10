# Swarm P900 bridge

`swarm_p900_bridge` transports selected ROS 2 topics over a Holybro/Microhard
P900 transparent serial link. It does not replace DDS networking, create an IP
link, or configure the radio with AT commands. The existing station and flight
nodes continue to publish and subscribe to their normal ROS topics.

The current Point-to-Point version transports three routes:

| Topic ID | ROS topic | ROS type | Direction |
|---:|---|---|---|
| 1 | `/swarm/command` | `std_msgs/msg/String` | station -> drone |
| 2 | `/swarm/formation_command` | `swarm_msgs/msg/FormationCommand` | station -> drone |
| 3 | `/swarm/status` | `swarm_msgs/msg/Status` | drone -> station |

The station bridge subscribes to `/swarm/command` and
`/swarm/formation_command`, serializes each complete ROS message with ROS 2
CDR serialization, frames it, and writes it to the P900. The drone bridge
validates each frame, deserializes it, and republishes it on the original topic.
In the reverse direction, the drone bridge subscribes to `/swarm/status`,
sends it over the radio, and the station bridge republishes it locally. The
role directions in the registry ensure that a bridge never subscribes to a
topic it republishes, preventing feedback loops.

If DDS discovery still connects the station computer directly to the drone
computer over Wi-Fi, a transported message can also take that direct path and
arrive twice. When P900 must be the only inter-computer path, isolate DDS
between the two sides: for example, run the station nodes and station bridge in
one `ROS_DOMAIN_ID`, and run the drone/swarm nodes and drone bridge in another.
All drones may share the same drone-side domain. This is ordinary local ROS
isolation; DDS traffic is never sent through the P900.

## Radio assumptions

The radios must already be configured and linked. This package does not send
radio configuration commands. Expected serial settings are:

- 230400 baud by default;
- 8 data bits, no parity, 1 stop bit (8N1);
- no RTS/CTS, DSR/DTR, or software flow control;
- transparent serial operation.

The serial device is a ROS parameter and is not required to be
`/dev/ttyUSB0`.

## Build

Install the Ubuntu pyserial dependency if it is not already present:

```bash
sudo apt install python3-serial
```

Then build from the workspace root:

```bash
source /opt/ros/rolling/setup.bash
colcon build --packages-select swarm_p900_bridge
source install/setup.bash
```

The user running the bridge needs permission to open the serial device. On a
typical Ubuntu installation that means membership in `dialout`; log out and
back in after changing group membership.

## Launch

Station, node 0, sending to drone 1:

```bash
ros2 launch swarm_p900_bridge p900_bridge.launch.py \
  role:=station \
  serial_port:=/dev/ttyUSB0 \
  baud_rate:=230400 \
  node_id:=0 \
  destination_node_id:=1
```

Drone 1, receiving from station 0:

```bash
ros2 launch swarm_p900_bridge p900_bridge.launch.py \
  role:=drone \
  serial_port:=/dev/ttyUSB0 \
  baud_rate:=230400 \
  node_id:=1 \
  destination_node_id:=0
```

The executable can also be run directly:

```bash
ros2 run swarm_p900_bridge bridge_node --ros-args \
  -p role:=station -p serial_port:=/dev/ttyUSB0 \
  -p baud_rate:=230400 -p node_id:=0 -p destination_node_id:=1
```

## Parameters

| Parameter | Default | Meaning |
|---|---:|---|
| `role` | `station` | `station` subscribes/TX; `drone` receives/publishes. |
| `serial_port` | `/dev/ttyUSB0` | Serial device or a pyserial URL such as `loop://`. |
| `baud_rate` | `230400` | Serial baud rate. |
| `node_id` | `-1` | Source ID. `-1` selects 0 for station or 1 for drone. |
| `destination_node_id` | `-1` | Destination. `-1` selects the P2P peer; 255 is broadcast. |
| `max_payload_bytes` | `65536` | Maximum accepted serialized ROS payload. |
| `diagnostics_interval` | `10.0` | Seconds between counter summaries. |
| `reconnect_interval` | `2.0` | Delay before reopening a failed serial device. |
| `serial_tx_queue_size` | `100` | Bounded background serial TX queue. |
| `ros_rx_queue_size` | `100` | Bounded serial-to-ROS handoff queue. |

Serial I/O runs in a dedicated worker. ROS callbacks only serialize and enqueue
frames, so they do not wait for serial reads or writes. A message is discarded
if the port is disconnected or the bounded queue is full; messages are not held
for later delivery after reconnection.

## Transport frame

All integer fields use network byte order (big-endian):

| Field | Size | Description |
|---|---:|---|
| Magic | 4 bytes | `P9R2`, used for stream resynchronization |
| Version | 1 byte | Protocol version, currently 1 |
| Topic ID | 2 bytes | Registry ID selecting command, formation, or status |
| Source role | 1 byte | 0 station, 1 drone |
| Source node ID | 1 byte | 0..254 |
| Destination node ID | 1 byte | 0..254 unicast, 255 broadcast |
| Sequence | 4 bytes | Per-topic unsigned sequence number |
| Payload length | 4 bytes | Serialized ROS payload byte count |
| Payload | variable | ROS 2 serialized message |
| CRC32 | 4 bytes | CRC32 over header and payload |

The incremental parser accepts partial reads and multiple frames per read. It
searches for the magic marker after garbage or corruption, checks length before
processing a payload, validates CRC32, and discards unknown topics safely.
Sequence gaps, duplicates, and out-of-order packets are counted and logged but
do not cause a valid message to be rejected.

## Adding another selected topic

Add one `TopicRoute` entry to
`swarm_p900_bridge/topic_registry.py`. The entry defines all of the following
in one place:

- unique numeric topic ID;
- ROS topic name;
- ROS message class;
- roles allowed to transmit it;
- roles allowed to receive and publish it.
- local ROS QoS reliability and history depth.

The bridge creates ROS subscriptions and publishers from the registry. Message
fields are never copied manually; ROS serialization handles the registered
type. A new custom message package must also be added as a dependency in
`package.xml`.

## Safe Point-to-Point ground test

For the first test, leave the flight controller node stopped and run only the
bridges plus `ros2 topic echo`. Different ROS domain IDs prove the message used
the radio rather than DDS over Wi-Fi while SSH can remain connected.

On the drone computer:

```bash
export ROS_DOMAIN_ID=41
source /opt/ros/rolling/setup.bash
source ~/multi_drone_real/install/setup.bash
ros2 launch swarm_p900_bridge p900_bridge.launch.py \
  role:=drone serial_port:=/dev/ttyUSB0 baud_rate:=230400 \
  node_id:=1 destination_node_id:=0
```

In a second drone terminal:

```bash
export ROS_DOMAIN_ID=41
source /opt/ros/rolling/setup.bash
source ~/multi_drone_real/install/setup.bash
ros2 topic echo /swarm/command std_msgs/msg/String
```

On the station computer:

```bash
export ROS_DOMAIN_ID=40
source /opt/ros/rolling/setup.bash
source ~/mbkh/multi_drone_real/install/setup.bash
ros2 launch swarm_p900_bridge p900_bridge.launch.py \
  role:=station serial_port:=/dev/ttyUSB0 baud_rate:=230400 \
  node_id:=0 destination_node_id:=1
```

Publish a harmless test string from another station terminal using the same
domain:

```bash
export ROS_DOMAIN_ID=40
source /opt/ros/rolling/setup.bash
source ~/mbkh/multi_drone_real/install/setup.bash
ros2 topic pub --once /swarm/command std_msgs/msg/String \
  "{data: 'P900_LINK_TEST'}"
```

The drone echo must print exactly `P900_LINK_TEST`. The station startup table
must list command/formation under TX and status under RX; the drone table must
show the inverse directions. After this isolated test, use the same ROS domain
computer and start the normal station/control software. Ensure no `p900_test`
script still has the serial port open when starting the bridge.

## Tests

Run the framing/parser and pyserial loopback tests with:

```bash
source /opt/ros/rolling/setup.bash
source install/setup.bash
colcon test --packages-select swarm_p900_bridge --event-handlers console_direct+
colcon test-result --verbose
```
