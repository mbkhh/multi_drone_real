"""Launch the SIYI ZR10 ROS 2 gimbal node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Declare network, topic, polling, and safety arguments."""
    camera_ip = LaunchConfiguration('camera_ip')
    camera_port = LaunchConfiguration('camera_port')
    control_topic = LaunchConfiguration('control_topic')
    status_topic = LaunchConfiguration('status_topic')
    status_rate_hz = LaunchConfiguration('status_rate_hz')
    attitude_poll_rate_hz = LaunchConfiguration('attitude_poll_rate_hz')
    speed_command_timeout_sec = LaunchConfiguration(
        'speed_command_timeout_sec'
    )
    zoom_command_timeout_sec = LaunchConfiguration(
        'zoom_command_timeout_sec'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_ip',
            default_value='192.168.144.25',
            description='SIYI camera/gimbal IPv4 address.',
        ),
        DeclareLaunchArgument(
            'camera_port',
            default_value='37260',
            description='SIYI SDK UDP control port.',
        ),
        DeclareLaunchArgument(
            'control_topic',
            default_value='/swarm/gimbal/control',
            description='JSON control topic (std_msgs/msg/String).',
        ),
        DeclareLaunchArgument(
            'status_topic',
            default_value='/swarm/gimbal/status',
            description='JSON status topic (std_msgs/msg/String).',
        ),
        DeclareLaunchArgument(
            'status_rate_hz',
            default_value='5.0',
            description='ROS status publication rate.',
        ),
        DeclareLaunchArgument(
            'attitude_poll_rate_hz',
            default_value='5.0',
            description='ZR10 attitude query rate.',
        ),
        DeclareLaunchArgument(
            'speed_command_timeout_sec',
            default_value='0.5',
            description=(
                'Auto-stop timeout for nonzero speed control; 0 disables it.'
            ),
        ),
        DeclareLaunchArgument(
            'zoom_command_timeout_sec',
            default_value='0.5',
            description=(
                'Auto-stop timeout for manual zoom; 0 disables it.'
            ),
        ),
        Node(
            package='swarm_gimbal',
            executable='gimbal_node',
            name='gimbal_node',
            output='screen',
            parameters=[{
                'camera_ip': ParameterValue(camera_ip, value_type=str),
                'camera_port': ParameterValue(camera_port, value_type=int),
                'control_topic': ParameterValue(control_topic, value_type=str),
                'status_topic': ParameterValue(status_topic, value_type=str),
                'status_rate_hz': ParameterValue(
                    status_rate_hz, value_type=float
                ),
                'attitude_poll_rate_hz': ParameterValue(
                    attitude_poll_rate_hz, value_type=float
                ),
                'speed_command_timeout_sec': ParameterValue(
                    speed_command_timeout_sec, value_type=float
                ),
                'zoom_command_timeout_sec': ParameterValue(
                    zoom_command_timeout_sec, value_type=float
                ),
            }],
        ),
    ])
