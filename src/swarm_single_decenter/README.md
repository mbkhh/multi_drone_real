# swarm_single_decenter

This package is an isolated copy of the proven TF-free/yaw controller for
decentralized flight. It does not elect a leader and does not use formation
goals. Each drone:

- subscribes directly to `/swarm/decenter/command`;
- filters commands by `target` (`all` or its numeric ID);
- publishes its own status on `/swarm/decenter/status`;
- publishes and receives peer poses on the existing best-effort
  `/swarm/local_state` stream for reciprocal collision avoidance;
- loads only its own numeric section from
  `waypoints_decentralized_36x34.yaml`.

The YAML positions are absolute common-ENU coordinates and its yaw values are
relative PX4/NED yaw changes. ARM calibration still anchors every vehicle to
`swarm_single.real_world.initial_positions` from `swarm_single.yaml.dist`.

The ordinary interactive goal limit remains 10 m. Prevalidated missions use a
separate `max_mission_leg_distance` ROS parameter (50 m by default) because
the supplied decentralized file contains a 34 m leg.

Collision avoidance reads the shared `swarm_single.navigation` configuration:

- `neighbor_dist`: `6.0` m;
- `radius`: `0.75` m per drone (`1.5` m combined separation);
- `time_horizon`: `5.0` s.

The controller supplies RVO with each drone's measured ENU velocity as well as
its position. A final predicted-closest-approach filter removes closing speed
and gives the pair opposite horizontal escape directions if a close pass is
still predicted. It logs `[COLLISION AVOIDANCE]` when that backup activates.

## Build

```bash
colcon build --packages-select swarm_config swarm_msgs \
  swarm_single_decenter swarm_station_decenter
source install/setup.bash
```

## Real vehicles

Run one controller on each vehicle with the correct ID:

```bash
ros2 run swarm_single_decenter control_node --ros-args -p frame_id:=1
ros2 run swarm_single_decenter control_node --ros-args -p frame_id:=2
ros2 run swarm_single_decenter control_node --ros-args -p frame_id:=3
```

Do not pass the SITL safety overrides during real flight.

Run the station on the ground computer:

```bash
ros2 run swarm_station_decenter station
```

## Simulation

The new launch file starts three configured SITL vehicles, the decentralized
controllers, and the decentralized station without changing `swarm_sim`:

```bash
ros2 launch swarm_single_decenter fullsim_decenter.launch.py
```

The launch file deliberately enables the same SITL-only safety bypasses as the
existing simulator. They are not controller defaults and do not apply to the
real-vehicle commands above.
