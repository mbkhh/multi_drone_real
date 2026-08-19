import os
import math
from launch import LaunchDescription
from launch.actions import LogInfo, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessIO
from launch_ros.actions import Node
from swarm_config.config_utils import get_config

def generate_spiral_position(instance_id: int, spawn_spacing: int):
    # This function remains unchanged
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
    px4_param_executable_path = os.path.join(
        px4_autopilot_dir, "build/px4_sitl_default/bin/px4-param"
    )

    drone_count = get_config('swarm_sim.drone_count')
    px4_model = get_config('swarm_sim.px4_model')
    frame_id = get_config('swarm_sim.frame_id')
    spawn_spacing = get_config('swarm_sim.spawn_spacing')

    # --- Processes with output redirected to logs ---
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
    
    # --- ADDED: Station Node for Interactive Control ---
    # This node needs to interact with the user via the console.
    # 'output="screen"' directs its print statements to your terminal.
    # 'emulate_tty=True' is crucial for allowing the node to read keyboard input (sys.stdin).
    station_node = Node(
        # !! IMPORTANT: Verify these names match your station package and executable !!
        package='swarm_station',    # The ROS 2 package name for your station
        executable='station',       # The name of your station's Python executable
        name='station_node',
        output='screen',
        emulate_tty=True,
        prefix='gnome-terminal --', 
    )

    # Add the non-drone-specific nodes to the launch description
    ld = LaunchDescription([
        xrce_agent_process,
        gz_bridge, 
        station_node, # Add the station node to be launched
    ])

    # --- Drone-specific nodes loop ---
    for i in range(drone_count):
        instance_id = i + 1
        x, y, z = get_configured_start_position(instance_id)

        px4_env = os.environ.copy()
        px4_env['PX4_SYS_ID'] = str(instance_id)
        px4_env['PX4_SYS_AUTOSTART'] = frame_id
        px4_env['PX4_GZ_MODEL'] = px4_model
        px4_env['PX4_GZ_MODEL_POSE'] = f"{x},{y},{z + 0.1},0,0,0.0"
        px4_env['PX4_UXRCE_DDS_NS'] = f"uav_{str(instance_id)}"

        if not instance_id == 1:
            px4_env['HEADLESS'] = '1'

        px4_sitl_process = ExecuteProcess(
            cmd=[px4_executable_path, '-i', str(instance_id)],
            cwd=px4_autopilot_dir,
            output='own_log', # Output is sent to a dedicated log file
            env=px4_env
        )

        # PX4 1.17's x500 airframe sets NAV_DLL_ACT=2 during its startup,
        # making a MAVLink GCS connection mandatory for arming. This project
        # uses a ROS station in SITL, so apply the simulation-only override
        # after the airframe has loaded. The PX4_PARAM_* environment mechanism
        # runs too early and is overwritten by the airframe default.
        disable_sitl_gcs_requirement = ExecuteProcess(
            cmd=[
                px4_param_executable_path,
                '--instance', str(instance_id),
                'set', 'NAV_DLL_ACT', '0',
            ],
            output='log',
        )

        startup_watch = {'output': bytearray(), 'override_started': False}

        def apply_sitl_override_after_startup(
            event,
            state=startup_watch,
            override_action=disable_sitl_gcs_requirement,
        ):
            state['output'].extend(event.text)
            if len(state['output']) > 4096:
                del state['output'][:-4096]
            if (
                not state['override_started']
                and b'Startup script returned successfully' in state['output']
            ):
                state['override_started'] = True
                return override_action
            return None
        
        single_control_node = Node(
            package='swarm_single',
            executable='control_node',
            name=f'control_node_d{str(instance_id)}',
            parameters=[{
                'frame_id': instance_id,
                # Use PX4 local position as the only odom TF source. Each SITL
                # estimator has its own local origin, so anchor it at the
                # matching Gazebo spawn point to create one shared ENU world.
                'use_configured_world_origin': True,
                'initial_world_position': [x, y, z],
                # SITL has no physical RC receiver. Real launches omit this
                # override and retain the controller's safe default (True).
                'require_manual_control_signal': False,
                # Explicit SITL-only bypass. The controller default is False,
                # so real launches keep their safety checks unchanged.
                'simulation_disable_safety_checks': True,
            }],
            output='log' # MODIFIED: Suppress output
        )

        # Do not launch the legacy Gazebo odom-to-TF node here: the controller
        # already publishes the same child frame from PX4 local position.
        # Running both creates competing TF authorities and invalid navigation.
        staggered_launch = TimerAction(
            period=float(i * 3.0),
            actions=[px4_sitl_process, single_control_node],
        )
        ld.add_action(staggered_launch)
        ld.add_action(RegisterEventHandler(OnProcessIO(
            target_action=px4_sitl_process,
            on_stdout=apply_sitl_override_after_startup,
        )))

    ld.add_action(TimerAction(period=float(drone_count * 3.0 + 0.5), actions=[LogInfo(msg="Simulator is ready...")]))
    return ld
