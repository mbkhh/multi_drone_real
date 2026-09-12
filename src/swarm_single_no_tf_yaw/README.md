# swarm_single_no_tf_yaw

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
ros2 run swarm_single_no_tf_yaw control_node --ros-args \
  -p frame_id:=1 \
  -p use_configured_world_origin:=true \
  -p require_manual_control_signal:=true \
  -p simulation_disable_safety_checks:=false
```

Use the correct `frame_id` on every vehicle. Do not run `swarm_single` and
`swarm_single_no_tf_yaw` for the same drone: both publish to the same PX4 command
and setpoint topics.

## Missions and yaw

The station's `mission` command validates the configured
`leader_waypoints_xyzyaw-3.txt` file. It sends only the filename and mode to the
leader, and the leader loads the same installed file locally. This avoids
sending the complete waypoint array across the swarm Wi-Fi link.

The file is YAML-formatted. Its top-level key must match the elected leader ID,
and every point is `[absolute ENU x, absolute ENU y, absolute ENU z,
relative PX4/NED yaw degrees, checkpoint wait]`. The fifth value is optional
for compatibility with older files and defaults to `false`:

```yaml
1:
  - [0.0, 0.0, 2.0, 0.0, false]
  - [2.0, 0.0, 2.0, -20.0, true]
  - [2.0, 2.0, 2.0, 0.0, false]
```

Each point is activated through the same absolute-goal handler as a station
`set_goal x y z` command. Its yaw value is applied like `move yaw=<degrees>`
The first yaw delta starts from the leader's measured mission-start heading;
later deltas accumulate from the preceding mission yaw target so small tracking
errors cannot distort the generated path. The mission moves to the next point
only after both position and yaw are within tolerance for the configured dwell
time. When the fifth value is `true`, the normal dwell is replaced by
`swarm_single.mission.checkpoint_delay` (five seconds by default). This uses
the existing mission timer and does not block Offboard heartbeat or setpoint
publication.

Only the leader executes the file. Followers receive a small mission-start
notification and continue resolving their own goals from the leader's measured
position, measured yaw, and their formation offset. There is no separate
leader-yaw topic.

The flight sequence remains explicit; the `mission` command does not arm,
take off, or select a formation automatically:

```text
arm
status
takeoff
set_formation square spacing=5 rotation_z=0
mission
```

Wait for ARM/Offboard confirmation before takeoff, for all aircraft to reach
takeoff height before setting the formation, and for the formation to settle
before starting the mission. `mission another_file.txt` can select a different
installed waypoint file when required.

### Limiting yaw speed

The fourth mission value is a relative heading change. The controller slews
the resulting PX4 yaw target at most
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

## Vision reporting

Run `swarm_vision` on the companion computer that has the camera. The station
can then control detection through the elected leader:

```text
start_detection
start_detection 32
start_detection 0,2,32
stop_detection
```

With no class list, the station requests COCO class 32 (`sports ball`). The
leader forwards the request on `/swarm/vision_trigger` and listens for the
vision node on `/swarm/vision_command`. Detection reports reuse the existing
`Status.message` field, so no additional leader-to-station DDS topic or custom
message is introduced. The station prints a report such as:

```text
Message from leader: VISION TARGET DETECTED by UAV_1
```

Vision is report-only in this version. Its callback does not call landing,
RTL, goal, arming, disarming, Offboard, or PX4 command methods.

## Simulation

Build the simulator and TF-free controller, source the workspace, then launch
the dedicated simulation file:

```bash
colcon build --packages-select swarm_config swarm_msgs px4_msgs \
  swarm_station swarm_single_no_tf_yaw swarm_sim
source install/setup.bash
ros2 launch swarm_sim fullsim_no_tf_yaw.launch.py
```
