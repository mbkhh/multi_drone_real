# Swarm diagnostic logger

`swarm_logger` reports per-sender heartbeat freshness, observed topic rates and
maximum gaps, DDS endpoint counts, Raspberry Pi throttle flags, CPU/memory and
temperature, and Wi-Fi association/counter health.

Reports are kept off `/rosout` so the diagnostic output does not consume the
wireless network. The node stores counters and timestamps rather than retaining
complete ROS messages.

## Recommended real-flight use

Run one logger on each Raspberry Pi. Keep high-rate vehicle subscriptions local
to that Pi by selecting only its drone ID:

```bash
ros2 run swarm_logger logger --ros-args \
  -p drone_count:=3 \
  -p drone_ids:="[1]" \
  -p monitor_vehicle_topics:=true \
  -p print_interval:=5.0 \
  -p wifi_interface:=wlan0
```

Use `[2]` and `[3]` on the other vehicles. By default,
`monitor_vehicle_topics` is false; the logger then observes only low-rate swarm
coordination topics and local host/network health.

Monitoring all vehicle IDs from every Raspberry Pi is not recommended because
each DDS reader can increase cross-network delivery and alter the system under
test. A ground-station observer can select all IDs during a propeller-off test:

```bash
ros2 run swarm_logger logger --ros-args \
  -p drone_count:=3 \
  -p drone_ids:="[1, 2, 3]" \
  -p monitor_vehicle_topics:=true \
  -p monitor_tf:=true
```

`monitor_tf` is deliberately false by default. When enabled, the report splits
`/tf` by child frame and records receive rate, freshness, maximum arrival gap,
and repeated source timestamps. Enabling it adds another DDS reader, so use it
only for controlled ground diagnosis rather than routine flight.

The external commands `iw` and `vcgencmd` are optional. Missing or unsupported
probes are reported as unavailable and do not stop the logger.
