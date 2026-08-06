# ROS 2 Drone Swarm Simulation Project

This project provides a complete simulation environment for controlling a swarm of drones using ROS 2, PX4-Autopilot, and Gazebo. It features advanced capabilities such as dynamic formation control, intelligent obstacle avoidance using RVO2/3D and Machine Learning, and a robust, decentralized architecture for swarm management.

---

## 🚀 Overview

The system is designed with modularity and extensibility in mind, separating concerns into distinct ROS 2 packages:
-   **`swarm_config`**: Centralized configuration management.
-   **`swarm_msgs`**: Custom message, service, and action definitions.
-   **`swarm_sim`**: Simulation setup and launch files.
-   **`swarm_single`**: Core control logic for each individual drone.
-   **`swarm_station`**: Ground control station for user interaction and command dispatch.

---

## ✅ Prerequisites

Before you begin, ensure you have the following installed on your system:

-   **Ubuntu 22.04 LTS**
-   **ROS 2 Humble Hawksbill**: Follow the official installation guide [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).
-   **Gazebo Fortress**: This is the recommended simulator and is often installed as part of the full ROS 2 "desktop" installation.
-   **PX4-Autopilot Toolchain**: Follow the official "Development Environment on Linux" guide from the PX4 documentation [here](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html).
-   **Python 3.10+** with `pip` and `venv`.
-   **Colcon**: The standard ROS 2 build tool (`sudo apt install python3-colcon-common-extensions`).
-   **Git**

---

## 🛠️ Installation & Setup

Follow these steps to set up the project on your local machine.

**1. Clone the Project Repository**
```bash
git clone <your-repository-url>
cd <your-project-directory>
```

**2. Set Up PX4-Autopilot**
This project requires a local clone of the PX4-Autopilot repository to run the SITL (Software-in-the-Loop) simulation.

```bash
# Clone the PX4 repository into your home directory
cd ~
git clone [https://github.com/PX4/PX4-Autopilot.git](https://github.com/PX4/PX4-Autopilot.git) --recursive
cd PX4-Autopilot
# Checkout the stable release this project was tested with
git checkout v1.14.0
```
*Note: If you clone PX4 to a different directory, you will need to update the path in the configuration files (see Configuration section below).*

**3. Set Up Python Virtual Environment**
It is highly recommended to use a virtual environment to manage Python dependencies and avoid conflicts with system packages.

```bash
# Navigate back to your project's root directory
cd /path/to/your-project-directory

# Create a virtual environment
python3 -m venv venv

# Activate the virtual environment
source venv/bin/activate
```
*You must activate the virtual environment (`source venv/bin/activate`) in every new terminal you use for this project.*

**4. Install Python Dependencies**
Install all required Python packages using the provided `requirements.txt` file.

```bash
pip install -r requirements.txt
```

**5. Build the ROS 2 Workspace**
Use `colcon` to build all the custom packages in the workspace.

```bash
# From the root of your project directory
colcon build
```

**6. Source the Workspace**
After the build is complete, you need to source the setup script to make the ROS 2 packages available in your environment.

```bash
source install/setup.bash
```
*This command must also be run in every new terminal, after activating the Python virtual environment.*

---

## ⚙️ Configuration

This project uses a flexible configuration system to allow for easy customization without creating version control conflicts.

**The `.dist` Workflow:**

-   The repository contains default configuration files with a `.yaml.dist` extension (e.g., `swarm_sim.yaml.dist`). **Do NOT edit these files directly.**
-   To create your own local configuration, make a copy of the `.dist` file and remove the `.dist` extension. For example:
    ```bash
    cp src/swarm_config/config/swarm_sim.yaml.dist src/swarm_config/config/swarm_sim.yaml
    ```
-   The system will automatically prioritize your local `.yaml` file over the default `.dist` file.
-   All `*.yaml` files are ignored by Git, so your local changes will not interfere with the work of other team members.

**Common Configuration:**
The most common parameter you'll need to change is the path to your PX4-Autopilot directory.
1.  Create your local config file: `cp src/swarm_config/config/swarm_sim.yaml.dist src/swarm_config/config/swarm_sim.yaml`
2.  Open `src/swarm_config/config/swarm_sim.yaml` and edit the `path_parameters.px4_path` to match the location where you cloned PX4.

---

## ▶️ Running the Simulation

There are three primary ways to launch the simulation, depending on your goal.

**1. Interactive Mode (`fullsim.launch.py`)**
This is the recommended mode for interactive testing and manual control. It launches the full simulation and automatically opens a **new terminal** for the Ground Control Station.

```bash
# Make sure you have sourced your workspace first!
ros2 launch swarm_sim fullsim.launch.py
```

**2. Automated Scenario Mode (`run_scenario.launch.py`)**
This mode is designed for running repeatable, automated missions defined in YAML files. It is perfect for testing and official competition runs.

```bash
# To run a scenario named "step1.yaml"
ros2 launch swarm_sim run_scenario.launch.py scenario:=step1
```
You can create your own scenario files in `src/swarm_config/config/scenarios/`.

**3. Base Mode for Development (`sim.launch.py`)**
This launches the simulation environment and all drone nodes but does **not** start the station. This is useful for developers who want to run the station manually with debugging tools.

```bash
# In Terminal 1: Launch the simulation
ros2 launch swarm_sim sim.launch.py

# In Terminal 2: Manually launch the station
# (Remember to source the workspace in this terminal too)
ros2 run swarm_station station
```

---

## ⌨️ Station Command Reference

Once the station node is running, you can issue the following commands:

| Command | Description | Example |
| :--- | :--- | :--- |
| `status` | Get the current status of the swarm. | `status` |
| `arm` | Arm all drones for flight. | `arm` |
| `set_goal` | Send the swarm to an absolute world coordinate. | `set_goal 10.0 5.0 4.0` |
| `move` | Move the swarm relative to its current position. | `move y=10.0 z=-1.0` |
| `mission` | Run the configured real-world waypoint mission after Offboard is confirmed. | `mission` |
| `set_formation`| Change the swarm's formation. | `set_formation square spacing=4.0` |
| `manual` | Enter manual control mode (use keyboard/joystick). | `manual` |
| `manual off` | Exit manual control mode. | `manual off` |
| `exit` | Shut down the station node. | `exit` |

### Real-world waypoint mission

Configure `mission.waypoints` in `swarm_single.yaml`. By default, each point is
an ENU offset from the vehicle position when `mission` is accepted: X is east,
Y is north, and positive Z is up. The shipped default takes off to 2 m and flies
a 2 m square before returning above its starting point.

Run `arm`, wait for `status` to show `state=TAKEOFF`, `armed=True`, and
`offboard=True`, then run `mission`. The mission command never arms the vehicle
automatically. At the final waypoint it remains armed in Offboard with a zero
velocity setpoint; use `land` explicitly when ready. `move`, manual control,
`land`, an RC mode change, stale telemetry, or the mission timeout aborts the
mission.

### RC takeover from Offboard (PX4 v1.14)

RC takeover is handled by PX4, so it remains available if the laptop network
connection is lost. In QGroundControl, open **Vehicle Setup > Flight Modes** and
map a dedicated transmitter switch position to **Position** mode. When PX4
accepts that mode change, the ROS controller immediately aborts any active
mission, stops its Offboard heartbeat/setpoints, and latches in `PILOT_CONTROL`.
It will not enter Offboard again until a new explicit station `arm` command.

For the PX4 v1.14 release used by this project, verify these parameters on the
flight controller:

- `COM_RC_IN_MODE` must allow the RC transmitter (`0` for RC-only operation;
  do not use `4`, which disables stick input).
- Set `COM_RC_OVERRIDE=3` to retain automatic-mode override and additionally
  enable stick override during Offboard. `2` enables Offboard override only.
- Keep `COM_RC_STICK_OV=30` initially; this is the percentage stick movement
  that triggers the override. Reduce it only after controlled testing.
- Set `COM_OBL_RC_ACT=0` so PX4 selects Position mode if the RPi stops supplying
  the Offboard heartbeat. `COM_OF_LOSS_T` controls that heartbeat-loss delay.

Test the switch and stick override in QGroundControl with propellers removed,
then repeat at low altitude in a clear area. The dedicated mode switch is the
primary takeover method; stick override is a second path.
