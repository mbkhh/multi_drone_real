# swarm_single_no_tf

This package is a copy of `swarm_single` whose control path does not use ROS
TF. Each drone:

- reads its own PX4 `VehicleLocalPosition`;
- converts PX4 NED position and velocity to the configured common ENU world;
- publishes `nav_msgs/msg/Odometry` on `/swarm/local_state`;
- caches peer state using local message receipt time;
- stores absolute goals only in its own process; and
- resolves follower goals as `leader position + formation offset`.

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

## Simulation

Build the simulator and TF-free controller, source the workspace, then launch
the dedicated simulation file:

```bash
colcon build --packages-select swarm_config swarm_msgs px4_msgs \
  swarm_station swarm_single_no_tf swarm_sim
source install/setup.bash
ros2 launch swarm_sim fullsim_no_tf.launch.py
```
