#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class OdomToTfNode(Node):
	def __init__(self):
		super().__init__('odom_to_tf_broadcaster')

		# Initialize the transform broadcaster
		self.tf_broadcaster = TransformBroadcaster(self)
		self.declare_parameter('frame_id', '1')
		self.declare_parameter('px4_model', 'x500')
		frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
		px4_model = self.get_parameter('px4_model').get_parameter_value().string_value
		self.namespace = f"{px4_model}_{frame_id}"
		# Subscribe to the odometry topic
	

		self.subscription = self.create_subscription(Odometry, f"/model/{px4_model}_{frame_id}/odometry", self.odom_callback, 10)

	def odom_callback(self, msg: Odometry):
		"""
		Callback function to handle incoming odometry messages and publish the transform.
		"""
		t = TransformStamped()

		# Read message content and assign it to corresponding tf variables
		t.header.stamp = self.get_clock().now().to_msg()
		t.header.frame_id = 'world'
		t.child_frame_id = f'{self.namespace}/odom'

		# The transform is the pose from the odometry message
		t.transform.translation.x = msg.pose.pose.position.x
		t.transform.translation.y = msg.pose.pose.position.y
		t.transform.translation.z = msg.pose.pose.position.z


		yaw = math.atan2(2.0 * (msg.pose.pose.orientation.w * msg.pose.pose.orientation.z + msg.pose.pose.orientation.x * msg.pose.pose.orientation.y), 1.0 - 2.0 * (msg.pose.pose.orientation.y * msg.pose.pose.orientation.y + msg.pose.pose.orientation.z * msg.pose.pose.orientation.z))
		t.transform.rotation.x = 0.0
		t.transform.rotation.y = 0.0
		t.transform.rotation.z = math.sin(yaw * 0.5)
		t.transform.rotation.w = math.cos(yaw * 0.5)

		# Send the transformation
		self.tf_broadcaster.sendTransform(t)

def main(args=None):
	rclpy.init(args=args)
	node = OdomToTfNode()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
	main()