# Swarm P900 bridge

`swarm_p900_bridge` transports selected ROS 2 topics over a
Holybro/Microhard P900 transparent serial link. It does not replace DDS,
create an IP link, modify flight-control nodes, or send radio AT commands.

The application topology is leader-centric Point-to-Multipoint:

```text
station 100 (Remote)  <->  leader 1 (Master)  <->  followers 2..254 (Remote)
```

Node ID `255` is reserved for application broadcast. It is not a P900 radio
configuration address. Frames are accepted only for the local node ID or 255,
and the topic registry performs an additional role/direction check.

## Topic registry

| ID | ROS topic | ROS type | Origin -> destination | Leader relay |
|---:|---|---|---|---|
| 1 | `/swarm/command` | `std_msgs/msg/String` | station -> leader 1 | no |
| 2 | `/swarm/formation_command` | `swarm_msgs/msg/FormationCommand` | station -> leader 1 | no |
| 3 | `/swarm/status` | `swarm_msgs/msg/Status` | leader -> station 100 | no |
| 4 | `/command` | `std_msgs/msg/String` | leader -> broadcast 255 | no |
| 5 | `/formation` | `swarm_msgs/msg/FormationCommand` | leader -> broadcast 255 | no |
| 6 | `/swarm/live` | `std_msgs/msg/Header` | leader -> 255; follower -> leader 1 | follower frames -> 255 |
| 7 | `/swarm/local_state` | `nav_msgs/msg/Odometry` | leader -> 255; follower -> leader 1 | follower frames -> 255 |
| 8 | `/swarm/vision_trigger` | `std_msgs/msg/String` | leader -> broadcast 255 | no |
| 9 | `/swarm/vision_command` | `std_msgs/msg/String` | follower -> leader 1 | no |

The old `role:=drone` launch value remains an alias for `role:=leader`. Its
wire role code is still 1, preserving the existing topic 1-3 frame behavior.

## Compact local state

The ROS-facing topic remains `nav_msgs/msg/Odometry`. Only its P900 payload is
compact. Topic 7 carries seven network-order float32 values:

```text
position x/y/z, linear velocity x/y/z, ENU yaw
```

The payload is exactly 28 bytes and the complete P900 application frame is 50
bytes. The receiver reconstructs normal Odometry with:

- receiver ROS time;
- `header.frame_id = "world"`;
- `child_frame_id` from the P900 source node ID;
- a yaw-only quaternion;
- zero angular velocity and covariance.

State transmission is capped at 10 Hz per source. Both the ROS-side pending
slot and serial worker are newest-only: an unsent state is replaced instead of
forming a stale FIFO. A leader relays the already-encoded follower payload,
preserving its original role, node ID and sequence number.

Feedback loops are prevented in three layers:

1. an origin follower drops a returned frame whose P900 source is its own ID;
2. the registry allows follower broadcast state only as a leader relay;
3. bridge-injected `/swarm/live` and `/swarm/local_state` ROS messages contain
   the peer ID, so a bridge transmits only messages whose embedded source ID
   equals its own node ID.

## Radio assumptions

- P900 radios are already configured for PMP externally;
- 230400 baud by default;
- 8 data bits, no parity, 1 stop bit;
- no RTS/CTS, DSR/DTR, or software flow control;
- transparent serial operation.

## Build and test

```bash
source /opt/ros/rolling/setup.bash
cd ~/multi_drone_real
colcon build --packages-select swarm_p900_bridge
source install/setup.bash
colcon test --packages-select swarm_p900_bridge \
  --event-handlers console_direct+
colcon test-result --verbose
```

The bridge user must be able to open `/dev/ttyUSB0`, normally through the
`dialout` group. Only one process may own each radio serial device.

## Launch

Leave `destination_node_id` at `-1` for PMP; the registry selects a destination
for every topic.

Station:

```bash
ros2 launch swarm_p900_bridge p900_bridge.launch.py \
  role:=station node_id:=100 \
  serial_port:=/dev/ttyUSB0 baud_rate:=230400
```

Leader:

```bash
ros2 launch swarm_p900_bridge p900_bridge.launch.py \
  role:=leader node_id:=1 \
  serial_port:=/dev/ttyUSB0 baud_rate:=230400
```

Follower 1:

```bash
ros2 launch swarm_p900_bridge p900_bridge.launch.py \
  role:=follower node_id:=2 \
  serial_port:=/dev/ttyUSB0 baud_rate:=230400
```

Future followers use the same command with `node_id:=3`, `node_id:=4`, etc.

## Parameters

| Parameter | Default | Meaning |
|---|---:|---|
| `role` | `station` | `station`, `leader`, `follower`; legacy `drone` means leader |
| `serial_port` | `/dev/ttyUSB0` | Serial device or pyserial URL |
| `baud_rate` | `230400` | Serial baud rate |
| `node_id` | `-1` | `-1` selects station=100, leader=1, follower=2 |
| `destination_node_id` | `-1` | Legacy global override; keep `-1` for PMP |
| `max_payload_bytes` | `65536` | Maximum accepted frame payload |
| `diagnostics_interval` | `10.0` | Seconds between statistics reports |
| `reconnect_interval` | `2.0` | Delay before reopening serial |
| `serial_tx_queue_size` | `100` | Queue for event/command frames |
| `ros_rx_queue_size` | `100` | Serial-thread to ROS-thread queue |

## Transport frame

The existing version-1 frame is unchanged. All integers use network byte
order:

| Field | Size |
|---|---:|
| Magic `P9R2` | 4 bytes |
| Protocol version | 1 byte |
| Topic ID | 2 bytes |
| Source role | 1 byte |
| Source node ID | 1 byte |
| Destination node ID | 1 byte |
| Sequence number | 4 bytes |
| Payload length | 4 bytes |
| Payload | variable |
| CRC32 | 4 bytes |

Role codes are station=0, leader=1, follower=2. The parser handles partial
reads, multiple frames per read, garbage resynchronization, maximum-length
validation, CRC rejection and unknown topics.

## Diagnostics

The periodic `P900 stats` line includes written/queued TX frames and bytes,
published RX frames and bytes, received unicast/broadcast, not-for-me and
own-source drops, invalid paths, suppressed ROS feedback, relay counts,
replaced/dropped state samples, CRC/malformed/unknown frames, sequence gaps,
duplicates/out-of-order frames, queue drops and serial errors. A per-topic
summary follows it.

High-rate liveness/state packets use DEBUG logging. Commands and other
event-driven traffic retain an INFO packet summary.

## Isolated bench testing

To prove traffic uses P900 rather than Wi-Fi DDS, put the local ROS graph on
each computer in a different `ROS_DOMAIN_ID`. The bridge transports serialized
messages without requiring matching domains.

Suggested IDs:

```text
station=40, leader=41, follower 2=42, follower 3=43
```

Start each bridge and use `ros2 topic pub`/`echo` in the corresponding local
domain. Do not run flight-control nodes during the initial command test.

To test relay with a second follower, publish Odometry on follower 2 with
`child_frame_id: "2"`; echo `/swarm/local_state` on leader and follower 3.
Follower 3 must receive a message with child ID 2, the leader relay counter
must increase, and follower 2 must increment `own_source` without publishing a
duplicate received copy. The current three-radio setup (station, leader and
one follower) can verify upstream reception and origin suppression; proving
delivery to another follower physically requires a second follower radio.

## Adding another topic

Add one route to `swarm_p900_bridge/topic_registry.py`. The centralized entry
defines topic ID, ROS name/type, role links, destination, QoS, codec, relay
policy, rate policy and source-identity field. Ordinary messages use ROS CDR.
Only `/swarm/local_state` currently uses the compact codec.
