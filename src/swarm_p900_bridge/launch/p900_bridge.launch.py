"""Launch one station, leader or follower P900 selected-topic bridge."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Declare bridge arguments and start the bridge node."""
    role = LaunchConfiguration('role')
    serial_port = LaunchConfiguration('serial_port')
    baud_rate = LaunchConfiguration('baud_rate')
    node_id = LaunchConfiguration('node_id')
    destination_node_id = LaunchConfiguration('destination_node_id')
    diagnostics_interval = LaunchConfiguration('diagnostics_interval')

    return LaunchDescription([
        DeclareLaunchArgument(
            'role',
            default_value='station',
            description='Bridge role: station, leader or follower.',
        ),
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='P900 serial device or pyserial URL.',
        ),
        DeclareLaunchArgument(
            'baud_rate',
            default_value='230400',
            description='P900 serial baud rate.',
        ),
        DeclareLaunchArgument(
            'node_id',
            default_value='-1',
            description=(
                '0..254; -1 selects station=100, leader=1, follower=2.'
            ),
        ),
        DeclareLaunchArgument(
            'destination_node_id',
            default_value='-1',
            description=(
                'Legacy global override; use -1 for registry-driven PMP.'
            ),
        ),
        DeclareLaunchArgument(
            'diagnostics_interval',
            default_value='10.0',
            description='Seconds between bridge counter summaries.',
        ),
        Node(
            package='swarm_p900_bridge',
            executable='bridge_node',
            name='p900_bridge',
            output='screen',
            parameters=[{
                'role': ParameterValue(role, value_type=str),
                'serial_port': ParameterValue(
                    serial_port, value_type=str
                ),
                'baud_rate': ParameterValue(baud_rate, value_type=int),
                'node_id': ParameterValue(node_id, value_type=int),
                'destination_node_id': ParameterValue(
                    destination_node_id, value_type=int
                ),
                'diagnostics_interval': ParameterValue(
                    diagnostics_interval, value_type=float
                ),
            }],
        ),
    ])
