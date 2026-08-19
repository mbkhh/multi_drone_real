import os
import math
from launch import LaunchDescription
from launch.actions import LogInfo, ExecuteProcess, TimerAction
from launch_ros.actions import Node
from swarm_config.config_utils import get_config

def generate_spiral_position(instance_id: int, spawn_spacing: int, ):
	space = get_config('swarm_sim.spawn_spacing')
	if instance_id == 1: return 0, 0
	k = math.ceil((math.sqrt(instance_id) - 1) / 2)
	side_length = 2 * k
	prev_ring_max_id = (2 * (k - 1) + 1)**2
	pos_in_ring = instance_id - prev_ring_max_id
	side_index = math.floor((pos_in_ring - 1) / side_length)
	pos_on_side = (pos_in_ring - 1) % side_length
	if side_index == 0: x, y = k, pos_on_side - (k - 1)
	elif side_index == 1: x, y = (k - 1) - pos_on_side, k
	elif side_index == 2: x, y = -k, (k - 1) - pos_on_side
	elif side_index == 3: x, y= pos_on_side - (k - 1), -k
	return x * spawn_spacing, y * spawn_spacing

def get_configured_start_position(instance_id: int):
	"""Return this drone's configured ENU [x, y, z] start position."""
	key = f'swarm_single.real_world.initial_positions.{instance_id}'
	position = get_config(key)
	if not isinstance(position, (list, tuple)) or len(position) != 3:
		raise ValueError(f"Configuration '{key}' must contain ENU [x, y, z].")
	try:
		position = tuple(float(value) for value in position)
	except (TypeError, ValueError) as error:
		raise ValueError(f"Configuration '{key}' must be numeric.") from error
	if not all(math.isfinite(value) for value in position):
		raise ValueError(f"Configuration '{key}' must contain finite values.")
	return position

def generate_launch_description():
	# --- Define crucial paths and variables ---
	px4_autopilot_dir = os.path.expanduser(get_config('swarm_sim.path_parameters.px4_path'))
	px4_executable_path = os.path.join(px4_autopilot_dir, "build/px4_sitl_default/bin/px4")

	drone_count = get_config('swarm_sim.drone_count')  # The number of drones in your swarm
	px4_model = get_config('swarm_sim.px4_model')
	frame_id = get_config('swarm_sim.frame_id')
	spawn_spacing = get_config('swarm_sim.spawn_spacing')

	xrce_agent_process = ExecuteProcess(
		cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
		output='log'
	)

	odom_bridge_args = [
    	f"/model/{px4_model}_{i+1}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry" 
    	for i in range(drone_count)
	]

	lidar_bridge_args = [
    	f"/world/default/model/{px4_model}_{i+1}/link/lidar_link/sensor/gpu_lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"
    	for i in range(drone_count)
	]

	all_bridge_args = odom_bridge_args + lidar_bridge_args

	gz_bridge = Node(
    	package='ros_gz_bridge',
    	executable='parameter_bridge',
    	arguments=all_bridge_args, 
    	output='log',
	)

	ld = LaunchDescription([
		xrce_agent_process,
		gz_bridge, 
	])


	for i in range(drone_count):
		instance_id = i+1

		x, y, z = get_configured_start_position(instance_id)

		# Add the PX4-specific variables for this instance
		px4_env = os.environ.copy()
		px4_env['PX4_SYS_ID'] = str(instance_id)
		px4_env['PX4_SYS_AUTOSTART'] = frame_id
		px4_env['PX4_GZ_MODEL'] = px4_model
		px4_env['PX4_GZ_MODEL_POSE'] = f"{x},{y},{z + 0.1},0,0,0.9"
		px4_env['PX4_PARAM_COM_AUTHR_SYS_ID'] = "255"
		px4_env['UXRCE_DDS_NS'] = f"px4_{str(instance_id)}"

		if not instance_id==1:
			px4_env['HEADLESS'] = '1'

		# Launch PX4 SITL for each drone
		px4_sitl_process = ExecuteProcess(
			cmd=[px4_executable_path, '-i', str(instance_id)],
			cwd=px4_autopilot_dir,
			output='own_log',
			env=px4_env
		)
		pose_to_tf_node = Node(
			package='swarm_sim',
			executable='tf_publisher',
			parameters=[{
				'frame_id': str(instance_id),
				'px4_model': px4_model,
			}]
		)
		single_control_node = Node(
			package='swarm_single',
			executable='control_node',
			name=f'control_node_d{str(instance_id)}',
			parameters=[{
				'frame_id': instance_id,
				'use_configured_world_origin': False,
				# SITL has no physical RC receiver. Real launches do not set
				# this override and retain the safe default (True).
				'require_manual_control_signal': False,
				# Never set this parameter in a real-drone launch.
				'simulation_disable_safety_checks': True,
			}]
		)

		staggered_launch = TimerAction(period=float(i * 3.0), actions=[px4_sitl_process, pose_to_tf_node, single_control_node])
		ld.add_action(staggered_launch)

	ld.add_action(TimerAction(period=float(drone_count * 3.0 + 0.5), actions=[LogInfo(msg="Simulator is ready...")]))
	return ld
