# swarm_flight_launcher

A laptop GUI that replaces the ten terminals used during a three-drone real
test. It starts the local `swarm_station` and connects to each companion
computer over SSH to run:

1. `MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600`;
2. `swarm_single_no_tf_yaw control_node` with the correct drone ID; and
3. the existing `run_swarm_vision.sh flight` command with the matching UAV ID.

The flight controller and vision packages are not modified by this package.

## Build and run

On the laptop:

```bash
cd ~/mbkh/multi_drone_real
source /opt/ros/rolling/setup.bash
colcon build --packages-select swarm_flight_launcher
source install/setup.bash
ros2 run swarm_flight_launcher launcher
```

Or, after building, run the workspace helper:

```bash
./run_swarm_launcher.sh
```

The laptop needs Tk and Paramiko. They are already available on the development
laptop. If another laptop reports a missing module, install only the Ubuntu
packages (not global pip packages):

```bash
sudo apt install python3-tk python3-paramiko
```

The drones do not need this GUI package or Paramiko; they only need the
existing workspace and SSH server.

## First use

For each drone row:

- enter its IP address or host name;
- enter the SSH username and password, or leave the password empty to use an
  existing SSH key;
- verify the remote workspace, normally `~/multi_drone_real`;
- select Agent, Control, and/or Vision; and
- press **Connect** or **Start checked**.

Unknown SSH host keys are rejected by default. For a known new Raspberry Pi,
temporarily enable **Trust a new SSH host key on first connection**, connect,
and verify the displayed SHA-256 fingerprint. Disable that option afterward.

Press **START SELECTED FLIGHT STACK** to start the local station and all checked
components on every enabled drone. Components start in Agent, Control, Vision
order. The launcher checks for a matching existing process and refuses to
start an obvious duplicate.

## Recommended operating sequence

1. Start the GUI with `./run_swarm_launcher.sh`.
2. Fill each drone row. Drone 1 defaults to `10.112.89.180`, user `mbkh`, and
   workspace `~/multi_drone_real`. Enter its password in the Password box.
3. Use **Connect** to test SSH. This is optional because a start button also
   connects automatically. Wait for `CONNECTED SHA256:...` in **SSH state**.
4. The **Use** box controls whether the large global start button includes that
   drone. The **Agent**, **Control**, and **Vision** boxes select which programs
   are launched on it.
5. For a single drone, press **Start checked** on that row. To launch the whole
   selected system, press **START SELECTED FLIGHT STACK**. That also starts the
   laptop station.
6. Inspect the matching lower tab. A healthy remote tab first shows the exact
   command, remote host/workspace, both sourced setup files, and a `RUNNING`
   state. It must not immediately show `EXITED`.
7. Use the **Station command** field and **Send** button exactly like typing in
   the old station terminal. A cautious flight sequence is `status`, `arm`,
   verify all vehicles, `takeoff`, verify altitude, then formation or mission
   commands.
8. After landing and disarming, use **STOP ALL MANAGED PROCESSES** and close the
   GUI.

The launcher automatically runs these setup steps before every remote Control
or Vision command:

```bash
cd ~/multi_drone_real
unset ROS_DOMAIN_ID
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/rolling/setup.sh
source install/setup.sh
```

The standalone Micro XRCE Agent intentionally does **not** source ROS or the
workspace. It uses the matching Fast DDS and Micro XRCE libraries installed
under `/usr/local`. Sourcing ROS first can mix `/opt/ros/rolling` Fast DDS with
the `/usr/local` Micro XRCE libraries and abort with `stack smashing detected`
when PX4 establishes its session.

The ROS setup path and remote workspace can be changed in the GUI. Passwords
are kept in memory only. Press **Save non-secret settings** to preserve IPs,
users, workspaces, ports, and checkbox selections.

## Output and station commands

The lower notebook contains ten live output views:

- Station;
- D1 Agent, Control, Vision;
- D2 Agent, Control, Vision; and
- D3 Agent, Control, Vision.

Use the input below the notebook to send commands such as `status`, `arm`,
`takeoff`, `set_formation`, or `online_mission` to the station. The GUI limits
each visible view to 5,000 lines, while complete output is saved persistently
under:

```text
~/swarm_launcher_logs/YYYYMMDD_HHMMSS/
```

Saved configuration is stored at
`~/.config/swarm_flight_launcher/config.json`. Passwords are deliberately
removed before this file is written and must be entered again after restarting
the GUI. The **Copy Drone 1 login to Drone 2/3** button reduces repeated entry
when all Pis use the same account.

## Stopping safely

The GUI records the process group of every process it starts. Stop sends
SIGINT first so ROS nodes and the camera can close cleanly, then escalates to
SIGTERM only if needed. It does not kill unrelated processes.

Do not stop the controller or XRCE agent while an aircraft is flying unless
the pilot is ready to take over and PX4 failsafe behavior has been tested.
Land and disarm all aircraft before using **Stop drone**, **STOP ALL MANAGED
PROCESSES**, or closing the GUI.

## Optional SSH-key setup

Passwords work, but SSH keys avoid typing three passwords every test:

```bash
ssh-keygen -t ed25519
ssh-copy-id USER@DRONE_1_IP
ssh-copy-id USER@DRONE_2_IP
ssh-copy-id USER@DRONE_3_IP
```

After confirming normal `ssh USER@IP` access, leave the GUI password fields
empty.
