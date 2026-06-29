# ROS2 Swarm Simulation Package (swarm_sim)

This package serves as the main entry point for launching a multi-drone swarm simulation using ROS2, Gazebo, and the PX4 Autopilot. It is responsible for orchestrating all the necessary components, including the simulation environment, drone models, communication bridges, and individual drone control nodes.

## Overview

The primary goal of `swarm_sim` is to create a complete, ready-to-use simulation environment for developing and testing multi-robot algorithms. When launched, this package handles the following tasks:

1.  **Starts Gazebo:** It launches the Gazebo simulator with a specified world file.
2.  **Spawns Drones:** It dynamically spawns a configurable number of drone models (e.g., PX4-powered X500s) into the Gazebo world at specified starting positions.
3.  **Launches Micro-XRCE Agents:** For each drone, it starts a dedicated `MicroXRCEAgent` instance on a unique UDP port. This agent is crucial for bridging the communication between the PX4 flight controller (running inside the simulation) and the ROS2 network.
4.  **Runs Gazebo Bridge:** It initiates the `ros_gz_bridge`, which translates and relays topics between Gazebo (for physics and sensor data) and ROS2 (for control and perception). This includes bridging topics like odometry, sensor readings, and commands.
5.  **Initiates Control Nodes:** It calls the `swarm_single` package to launch a dedicated, uniquely-named control node for each drone. This ensures that every drone in the swarm has its own independent control logic running in the ROS2 ecosystem.

By managing these components, `swarm_sim` provides a scalable and organized foundation for complex swarm robotics research.

## Prerequisites

Before launching this package, ensure you have a complete ROS2 and PX4 simulation environment set up. This includes:

* **ROS2 Humble/Iron/Jazzy:** (Specify your ROS2 version)
* **Gazebo Garden/Fortress:** (Specify your Gazebo version)
* **PX4 Autopilot:** Cloned from the official repository and built for Gazebo simulation.
* **Micro-XRCE-DDS-Agent:** Installed and available in your environment.
* **ros_gz_bridge:** The ROS to Gazebo bridge package.
* **x500_d435i_description:** Or the relevant URDF/SDF description package for your drone model.
* **swarm_single:** The companion package that contains the control logic for a single drone.

## Installation

1.  Clone this package into the `src` directory of your ROS2 workspace:
    ```bash
    cd ~/ros2_ws/src
    git clone <your_repository_url>/swarm_sim.git
    ```

2.  Build the workspace:
    ```bash
    cd ~/ros2_ws
    colcon build --symlink-install
    ```

3.  Source the workspace to make the launch files available:
    ```bash
    source install/setup.bash
    ```

## Usage

The primary way to use this package is through its main launch file. You can configure the simulation using launch arguments.

### Launching the Simulation

To start a simulation with a default configuration (e.g., 3 drones in the `empty.world`), run:

```bash
ros2 launch swarm_sim swarm_launch.py
```
### Configurable Launch Arguments

You can customize the simulation by passing arguments on the command line.

* `world`: The name of the world file to load. Worlds should be located in the `swarm_sim/worlds` directory.
* `drone_count`: The number of drones to spawn in the simulation.
* `px4_model`: The specific PX4 SITL model to use (e.g., `x500_d435i`).

**Example:** Launch a simulation with 5 drones in a custom world named `forest.world`:

```bash
ros2 launch swarm_sim swarm_launch.py drone_count:=5 world:=forest
```

## Package Structure

`swarm_sim/
├── launch/
│   └── swarm_launch.py       # Main launch file to orchestrate everything
├── package.xml               # Package manifest
└── README.md                 # This README file`

* `launch/`: Contains the Python launch files that define how the simulation components are started and connected.

## How It Works

The `swarm_launch.py` file is the heart of this package. It uses a Python loop based on the `drone_count` argument. In each iteration, it programmatically generates the necessary nodes and configurations for a single drone, ensuring that all ports, topics, and node names are unique to avoid conflicts. This makes the simulation easily scalable by simply changing a launch argument.