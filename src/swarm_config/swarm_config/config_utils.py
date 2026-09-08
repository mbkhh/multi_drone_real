import os
import yaml
from ament_index_python.packages import get_package_share_directory

def get_config(key_path):
	pkg_swarm_config = get_package_share_directory('swarm_config')

	# Determine which config file to use (local override or default)
	keys = key_path.split('.')
	config_file_name = keys[0]
	keys = keys[1:]
	default_config_file = os.path.join(pkg_swarm_config, 'config', f"{config_file_name}.yaml.dist")
	local_config_file = os.path.join(pkg_swarm_config, 'config', f"{config_file_name}.yaml")

	if os.path.exists(local_config_file):
		config_path_to_use = local_config_file
	elif os.path.exists(default_config_file):
		config_path_to_use = default_config_file
	else:
		raise ValueError(f"Error: Neither '{local_config_file}' nor '{default_config_file}' found.")

	try:
		with open(config_path_to_use, 'r') as f:
			config_data = yaml.safe_load(f)
	except Exception as e:
		raise ValueError(f"Error loading YAML file {config_path_to_use}: {e}")

	value = config_data
	try:
		for key in keys:
			value = value[key]
		return value
	except (KeyError, TypeError):
		return None

def get_scenario(scenario_name: str):
    try:
        pkg_share_path = get_package_share_directory('swarm_config')

        scenario_file = os.path.join(pkg_share_path, 'config', 'scenarios', f"{scenario_name}.yaml")

        with open(scenario_file, 'r') as f:
            data = yaml.safe_load(f)

        scenario_list = data.get('scenario_steps', [])
        
        if not isinstance(scenario_list, list) or not all(isinstance(step, list) and len(step) == 2 for step in scenario_list):
            
            print(f"Error: Scenario file '{scenario_file}' has an invalid format.")
            return None

        return scenario_list

    except FileNotFoundError:
        # In a ROS node, you would use self.get_logger().error()
        print(f"Error: Scenario file for '{scenario_name}' not found at '{scenario_file}'")
        return None
    except yaml.YAMLError as e:
        # In a ROS node, you would use self.get_logger().error()
        print(f"Error parsing YAML file '{scenario_name}': {e}")
        return None


def get_mission_waypoints(file_name: str, leader_id: int):
    """Load one leader's waypoint list from the installed config directory."""
    if not isinstance(file_name, str) or not file_name.strip():
        raise ValueError('Mission waypoint filename must be a non-empty string.')

    file_name = file_name.strip()
    if os.path.basename(file_name) != file_name:
        raise ValueError(
            'Mission waypoint filename must not contain a directory path.'
        )

    try:
        leader_id = int(leader_id)
    except (TypeError, ValueError) as error:
        raise ValueError('Mission leader ID must be an integer.') from error

    config_directory = os.path.join(
        get_package_share_directory('swarm_config'), 'config'
    )
    waypoint_path = os.path.join(config_directory, file_name)
    if not os.path.isfile(waypoint_path):
        raise ValueError(
            f"Mission waypoint file '{file_name}' is not installed."
        )

    try:
        with open(waypoint_path, 'r', encoding='utf-8') as waypoint_file:
            data = yaml.safe_load(waypoint_file)
    except (OSError, yaml.YAMLError) as error:
        raise ValueError(
            f"Error loading mission waypoint file '{file_name}': {error}"
        ) from error

    if not isinstance(data, dict):
        raise ValueError(
            f"Mission waypoint file '{file_name}' must contain a leader-ID map."
        )

    points = data.get(leader_id)
    if points is None:
        points = data.get(str(leader_id))
    if not isinstance(points, list) or not points:
        raise ValueError(
            f"Mission waypoint file '{file_name}' has no waypoints for "
            f'leader {leader_id}.'
        )
    return points
