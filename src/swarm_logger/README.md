# Swarm diagnostic logger

`swarm_logger` reports per-sender heartbeat freshness, observed topic rates and
maximum gaps, DDS endpoint counts, Raspberry Pi throttle flags, CPU/memory and
temperature, Wi-Fi association/counter health, and optional detailed TF
transport diagnostics.

Reports are kept off `/rosout` so the diagnostic output does not consume the
wireless network. The node stores counters and timestamps rather than retaining
complete ROS messages.

## Lightweight no-TF flight mode

Use this mode for normal flight tests with either `swarm_single_no_tf` or
`swarm_single_no_tf_yaw`. Run one logger on each Pi, changing the local ID and
filename for that vehicle. First create the persistent directory once on every
drone:

```bash
mkdir -p "$HOME/swarm_flight_logs"
```

For example, the first `swarm_single_no_tf` test on drone 1 is:

```bash
ros2 run swarm_logger logger --ros-args \
  -p drone_count:=3 \
  -p local_drone_id:=1 \
  -p drone_ids:="[1]" \
  -p monitor_vehicle_topics:=false \
  -p monitor_tf:=false \
  -p monitor_wifi_events:=true \
  -p print_interval:=10.0 \
  -p wifi_interface:=wlan0 \
  -r __node:=swarm_logger_d1 \
  > "$HOME/swarm_flight_logs/no_tf_test1_d1.log" 2>&1
```

This keeps the passive heartbeat, topology/status, command, DDS endpoint,
Wi-Fi, kernel UDP, controller-process, and host-health diagnostics. It avoids
the TF reader, PX4 topic readers, and ROS network logging. Only one diagnostic
report is written to persistent storage every 10 seconds. The files survive a
normal shutdown and reboot. Ready-to-copy, separately named commands for all
three drones and both controller packages are in `commands.txt`.

After landing, keep the logger running for at least 12 seconds so its last full
reporting window is written. Stop it with `Ctrl-C`, flush the filesystem, and
then perform a normal shutdown:

```bash
sync
sudo shutdown -h now
```

Do not remove power until the operating system has finished shutting down.
After reboot, the files remain in `$HOME/swarm_flight_logs` and can be copied to
the analysis computer.

## Comprehensive three-drone test mode

Run one logger on each Raspberry Pi. Set `local_drone_id` and `drone_ids` to the
ID of that Pi so high-rate PX4 subscriptions remain local. Enable TF monitoring
on all three receivers so the same writer can be compared at each drone. The
drone-1 command is:

```bash
ros2 run swarm_logger logger --ros-args \
  -p drone_count:=3 \
  -p local_drone_id:=1 \
  -p drone_ids:="[1]" \
  -p monitor_vehicle_topics:=true \
  -p monitor_tf:=true \
  -p monitor_wifi_events:=true \
  -p print_interval:=5.0 \
  -p wifi_interface:=wlan0 \
  -r __node:=swarm_logger_d1 \
  > /dev/shm/swarm_logger_d1.log 2>&1
```

Use the corresponding ID and filename on drones 2 and 3. Both stdout and stderr
go to the Pi's local memory-backed filesystem, so reports are not streamed
through SSH or written to the SD card during flight. Each launch truncates the
previous file.
`/dev/shm` is volatile and is erased by reboot; check `df -h /dev/shm` before a
test and retrieve the log after landing.

Do not use `tail -f` over Wi-Fi during the test. After all motors are stopped,
copy the logs to the ground computer, for example:

```bash
scp pi@DRONE1:/dev/shm/swarm_logger_d1.log ./drone1_logger.txt
scp pi@DRONE2:/dev/shm/swarm_logger_d2.log ./drone2_logger.txt
scp pi@DRONE3:/dev/shm/swarm_logger_d3.log ./drone3_logger.txt
```

Replace `pi` and the hostnames with the actual SSH user and addresses. Copy the
files before rebooting any Pi.

## Keeping the logger alive

Shell redirection prevents report traffic over SSH, but it does not by itself
keep the process alive when an SSH session closes. Start the command in `tmux`,
then detach with `Ctrl-b d`:

```bash
tmux new-session -s swarm_logger
# Run this drone's command from commands.txt, then press Ctrl-b d.
tmux attach-session -t swarm_logger
```

For repeatable tests, a systemd service is more reliable. The following is a
template; replace `USER`, `WORKSPACE`, `ROS_DISTRO`, and `N` for each Pi:

```ini
[Unit]
Description=Swarm diagnostic logger for drone N
Wants=network-online.target
After=network-online.target

[Service]
Type=simple
User=USER
WorkingDirectory=WORKSPACE
Environment=PYTHONUNBUFFERED=1
ExecStart=/bin/bash -lc 'source /opt/ros/ROS_DISTRO/setup.bash && source WORKSPACE/install/setup.bash && exec ros2 run swarm_logger logger --ros-args -p drone_count:=3 -p local_drone_id:=N -p drone_ids:=[N] -p monitor_vehicle_topics:=true -p monitor_tf:=true -p monitor_wifi_events:=true -p print_interval:=5.0 -p wifi_interface:=wlan0 -r __node:=swarm_logger_dN'
StandardOutput=append:/dev/shm/swarm_logger_dN.log
StandardError=append:/dev/shm/swarm_logger_dN.log
Restart=on-failure
RestartSec=2
KillSignal=SIGINT

[Install]
WantedBy=multi-user.target
```

Install it as `/etc/systemd/system/swarm-logger.service`, then use
`systemctl daemon-reload`, `systemctl enable --now swarm-logger`, and
`systemctl status swarm-logger`. Stop it with `systemctl stop swarm-logger`
before collecting the final log. System-level installation and control require
administrator privileges.

## TF diagnostics

When `monitor_tf` is enabled, the logger reports both per-frame and per-writer
behavior. Per-frame data includes receive rate and freshness, arrival and source
stamp gaps, repeated/backward/zero stamps, parent or writer changes, transform
validity, and motion jumps. Per-writer data uses ROS middleware message metadata
when the installed ROS version provides it: publisher identity, sequence gaps,
source/receive timing, and callback queue delay. `/tf` endpoint node names and
QoS policies are also recorded.

These measurements help distinguish three cases: a writer that stopped
publishing, samples published but missing at one receiver, and samples received
by middleware but delayed before the Python callback ran. Sequence metadata is
middleware- and ROS-version-dependent; the report states when it is unavailable.

Use the evidence together rather than treating every stale warning as a Wi-Fi
failure:

- TF arrival and source-stamp progression both stop, while Wi-Fi failures,
  beacon loss, reassociation events, or UDP receive-buffer errors increase:
  suspect the wireless path.
- TF messages keep arriving but the frame source stamp repeats or stops
  advancing: suspect the transform publisher or controller scheduling.
- One receiver records DDS sequence gaps for a writer while the other receivers
  do not: suspect delivery loss to that receiver.
- Callback-queue delay, controller run-queue delay, CPU/PSI pressure, and PX4
  topic gaps rise together: suspect companion-computer scheduling/load.
- Publisher endpoint identities or TF authorities change: check for a process
  restart, duplicate broadcaster, or DDS rediscovery event.

The logger deliberately does not run ping, iperf, or other active network-load
tests during flight. Swarm heartbeats and TF delivery provide passive path
measurements without adding probe packets to the radio channel.

## Observer effect

`monitor_tf` adds one DDS `/tf` reader on every Pi. `monitor_vehicle_topics`
also adds readers, but the singleton `drone_ids` value keeps those high-rate PX4
subscriptions local to that vehicle. The logger uses bounded, best-effort
diagnostic subscriptions and keeps `/rosout` disabled, but it still consumes
some CPU, memory, DDS discovery traffic, and TF delivery work. Treat this as a
comprehensive diagnosis mode and record that it was enabled when comparing with
earlier flights.

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
`/tf` by frame and writer and records transport and transform integrity metrics.
Enabling it adds another DDS reader, so use it for targeted diagnosis rather
than routine operation once the fault has been resolved.

The external commands `iw` and `vcgencmd` are optional. Missing or unsupported
probes are reported as unavailable and do not stop the logger.
