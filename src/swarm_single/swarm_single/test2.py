import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
import math
import time
from vector3 import Vector3

class DroneSwarmSimulator2(Node):
    def __init__(self):
        super().__init__('drone_swarm_simulator2')
        self.broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.1, self.broadcast_transforms)  # 10 Hz
        self.test = self.create_timer(1.0, self.get_state)  # 1 Hz
        self.start_time = time.time()
        self.neighbor_ids = [0,1,2]
        self.goal_pos = Vector3(0,0,5)
        self.cur_pos = Vector3(0,0,0)

    def broadcast_transforms(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = f'drone_3_base'

        t.transform.translation.x = self.cur_pos.x
        t.transform.translation.y = self.cur_pos.y
        t.transform.translation.z = self.cur_pos.z

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.broadcaster.sendTransform(t)
        
    def get_state(self):
        for i in range(3):
            self.get_drone_pos(i)

    def get_drone_pos(self, drone_id):
        try:
            now = rclpy.time.Time()
            drone_tf = self.tf_buffer.lookup_transform(
                'world',
                f'drone_{drone_id}_base',
                now,
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            trans = drone_tf.transform.translation
            self.get_logger().info(
                f"Drone {drone_id} position: x={trans.x:.2f}, y={trans.y:.2f}, z={trans.z:.2f}"
            )
        except Exception as e:
            self.get_logger().warn(f"Could not get transform for drone_{drone_id}_base: {e}")

def main():
    rclpy.init()
    node = DroneSwarmSimulator2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
