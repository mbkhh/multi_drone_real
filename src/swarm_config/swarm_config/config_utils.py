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
