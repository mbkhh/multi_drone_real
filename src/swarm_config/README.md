# Swarm Configuration Package (`swarm_config`)

This package is the **single source of truth** for all configuration parameters used across the drone swarm project. Centralizing the configuration here ensures that all nodes use consistent values and makes the system easier to manage and tune.

## Configuration Strategy: Default and Local Settings

The project uses a system that allows for both default and custom local configurations.

* **Default Configs (`*.yaml.dist`):** These are the default template files that come with the project. The simulation will run out-of-the-box using the values in these files.

* **Local Configs (`*.yaml`):** If you need to override the default settings (for example, to change a file path), you can create your own local `.yaml` file. The system will automatically use your local file if it exists, otherwise it will use the default `.dist` file.

## How to Set Up a Local Configuration

To create a local configuration with your own custom settings, follow these steps:

1.  In the `src/swarm_config/config` directory of your workspace, copy the default template to create your local config file. For example:
    ```bash
    cp swarm_sim.yaml.dist swarm_sim.yaml
    ```

2.  Open and edit your new local `swarm_sim.yaml` file with your specific settings:
    ```yaml
    # inside swarm_sim.yaml
    path_parameters:
      # Change this to your local path
      px4_path: '/home/your_username/path/to/PX4-Autopilot' 
        ...
    ```

## How to Access Configuration Variables in Code

A centralized Python utility function is provided to easily access any configuration variable from any other node in the workspace. This function automatically handles the logic of checking for a local file before falling back to the default.

**To use it, import the function and call it as follows:**

```python
# In any of your Python nodes (e.g., in swarm_single or swarm_planner)

from swarm_config.config_utils import get_config

# Get a specific variable using its file name and dot-separated key path
px4_path = get_config('swarm_sim.path_parameters.px4_path')