import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math
import time

class DroneSwarmSimulator(Node):
    def __init__(self, num_drones=3):
        super().__init__('drone_swarm_simulator')
        self.broadcaster = TransformBroadcaster(self)
        self.num_drones = num_drones
        self.timer = self.create_timer(0.1, self.broadcast_transforms)  # 10 Hz
        self.start_time = time.time()

    def broadcast_transforms(self):
        current_time = time.time() - self.start_time
        for i in range(self.num_drones):
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'world'
            t.child_frame_id = f'drone_{i}_base'

            angle = i * 2 * math.pi / self.num_drones
            radius = 2.0  # meters
            t.transform.translation.x = radius * math.cos(angle)
            t.transform.translation.y = radius * math.sin(angle)
            t.transform.translation.z = 1.0  # constant height

            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0

            self.broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = DroneSwarmSimulator(num_drones=3)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
