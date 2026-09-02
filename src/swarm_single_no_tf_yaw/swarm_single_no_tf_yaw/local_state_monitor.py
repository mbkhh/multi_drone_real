"""Small shared-state subscriber used for manual package testing."""

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class DroneStateMonitor(Node):
    def __init__(self):
        super().__init__('drone_state_monitor')
        self.subscription = self.create_subscription(
            Odometry, '/swarm/local_state', self.state_callback, 10
        )

    def state_callback(self, msg):
        position = msg.pose.pose.position
        self.get_logger().info(
            f'Drone {msg.child_frame_id} position: '
            f'x={position.x:.2f}, y={position.y:.2f}, z={position.z:.2f}'
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
