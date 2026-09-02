# swarm_single_no_tf

This package is a copy of `swarm_single` whose control path does not use ROS
TF. Each drone:

- reads its own PX4 `VehicleLocalPosition`;
- converts PX4 NED position and velocity to the configured common ENU world;
- publishes common-ENU position, orientation, and velocity as
  `nav_msgs/msg/Odometry` on `/swarm/local_state`;
- caches peer state using local message receipt time;
- stores absolute goals only in its own process; and
- resolves follower goals as `leader position + rotated formation offset`.

The drone ID is carried in `Odometry.child_frame_id`. The header frame is
always `world`. No active goal is published for other drones to consume; the
legacy goal fields in `swarm_msgs/Status` are left at their default values.

Run one instance on each companion computer:

```bash
ros2 run swarm_single_no_tf control_node --ros-args \
  -p frame_id:=1 \
  -p use_configured_world_origin:=true \
  -p require_manual_control_signal:=true \
  -p simulation_disable_safety_checks:=false
```

Use the correct `frame_id` on every vehicle. Do not run `swarm_single` and
`swarm_single_no_tf` for the same drone: both publish to the same PX4 command
and setpoint topics.

## Missions and yaw

The station's `mission` command reads `swarm_single.mission.waypoints` from
`swarm_single.yaml`. Each waypoint is normally `[x, y, z]`; an optional fourth
value supplies the leader yaw in PX4/NED degrees. The controller converts this
value to radians for PX4, for example:

```yaml
mission:
  relative_to_start: true
  waypoints:
    - [0.0, 0.0, 1.0, 0.0]
    - [2.0, 0.0, 0.0, 90.0]
    - [0.0, 2.0, 0.0, 180.0]
```

Only the leader executes these yaw values. Its measured common-world ENU
orientation is already carried by the quaternion in `/swarm/local_state`.
Followers extract yaw only from the elected leader's odometry and rotate their
immutable base offset by that angle. There is no separate leader-yaw topic.
Consequently, a 30-degree leader turn rotates every follower's world position
30 degrees around the leader while preserving its radius and bearing in the
leader-relative frame. Select a `circle` formation when every follower must
stay exactly `spacing` metres from the leader.

### Limiting yaw speed

The fourth mission value is a target heading, not an instantaneous command.
The controller slews the PX4 yaw setpoint at most
`swarm_single.control.max_yaw_rate_deg_s` degrees per second. Set this in your
active `swarm_single.yaml`; for example, `20.0` limits a 90-degree change to
at least 4.5 seconds:

```yaml
control:
  max_yaw_rate_deg_s: 20.0
```

This limits the commanded setpoint. The vehicle's measured yaw can still turn
more slowly due to PX4 tuning or airframe limits, and followers use the
leader's measured yaw so their formation rotation remains synchronized.

For an in-flight relative turn, the station also accepts `move yaw=<degrees>`:

```text
move yaw=20
move yaw=-20
```

The command uses the leader's measured current PX4/NED heading as its starting
point. It changes only the leader's yaw target; the yaw-rate limiter above
controls how quickly the target is sent to PX4. Followers do not receive a
separate yaw command—they react to the leader orientation in
`/swarm/local_state`.

## Simulation

Build the simulator and TF-free controller, source the workspace, then launch
the dedicated simulation file:

```bash
colcon build --packages-select swarm_config swarm_msgs px4_msgs \
  swarm_station swarm_single_no_tf swarm_sim
source install/setup.bash
ros2 launch swarm_sim fullsim_no_tf.launch.py
```
