"""Small shared-state subscriber used for manual package testing."""

import math
import rclpy

from nav_msgs.msg import Odometry
from rclpy.node import Node


class DroneStateMonitor(Node):

    def __init__(self):

        super().__init__('drone_state_monitor')

        self.subscription = self.create_subscription(
            Odometry,
            '/swarm/local_state',
            self.state_callback,
            10
        )


    def state_callback(self, msg):

        position = msg.pose.pose.position

        # Extract quaternion
        q = msg.pose.pose.orientation

        # Convert quaternion -> yaw
        yaw_rad = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
    

        self.get_logger().info(
            f'Drone {msg.child_frame_id} position: '
            f'x={position.x:.2f}, '
            f'y={position.y:.2f}, '
            f'z={position.z:.2f}, '
            f'yaw={yaw_rad:.2f} rad'
        )


def main():

    rclpy.init()

    node = DroneStateMonitor()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()