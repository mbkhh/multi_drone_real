"""Small shared-state publisher used for manual package testing."""

import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class DroneStateSimulator(Node):
    def __init__(self, num_drones=3):
        super().__init__('drone_state_simulator')
        self.publisher = self.create_publisher(
            Odometry, '/swarm/local_state', 10
        )
        self.num_drones = num_drones
        self.start_time = time.monotonic()
        self.timer = self.create_timer(0.1, self.publish_states)

    def publish_states(self):
        elapsed = time.monotonic() - self.start_time
        for index in range(self.num_drones):
            msg = Odometry()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'world'
            msg.child_frame_id = str(index + 1)
            angle = elapsed * 0.1 + index * 2.0 * math.pi / self.num_drones
            msg.pose.pose.position.x = 2.0 * math.cos(angle)
            msg.pose.pose.position.y = 2.0 * math.sin(angle)
            msg.pose.pose.position.z = 1.0
            msg.pose.pose.orientation.w = 1.0
            self.publisher.publish(msg)


def main():
    rclpy.init()
    node = DroneStateSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
