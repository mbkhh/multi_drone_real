#!/usr/bin/env python3


import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from swarm_msgs.action import Fly
from geometry_msgs.msg import Vector3

class StationActionClient(Node):
	"""The Ground Station's Action Client."""

	def __init__(self):
		super().__init__('station_action_client')
		self._action_client = ActionClient(self, Fly, '/fly_to')

	def send_goal(self, x, y, z):
		self.get_logger().info('Waiting for action server...')
		self._action_client.wait_for_server()

		goal_msg = Fly.Goal()
		goal_msg.goal = Vector3(x=float(x), y=float(y), z=float(z))

		self.get_logger().info('Sending goal request...')
		self._send_goal_future = self._action_client.send_goal_async(
			goal_msg,
			feedback_callback=self.feedback_callback)
		
		self._send_goal_future.add_done_callback(self.goal_response_callback)

	def goal_response_callback(self, future):
		goal_handle = future.result()
		if not goal_handle.accepted:
			self.get_logger().error('Goal rejected by the server!')
			return

		self.get_logger().info('Goal accepted by the server. Waiting for result...')

		self._get_result_future = goal_handle.get_result_async()
		self._get_result_future.add_done_callback(self.get_result_callback)

	def get_result_callback(self, future):
		result = future.result().result
		self.get_logger().info(f'Final Result: {result.result}')
		rclpy.shutdown() # Shutdown after getting the result

	def feedback_callback(self, feedback_msg):
		progress = feedback_msg.feedback.state
		self.get_logger().info(f'Received feedback: Progress = {progress:.1f}%')


def main(args=None):
	rclpy.init(args=args)
	action_client = StationActionClient()
	
	# Send a goal to fly to a target position
	action_client.send_goal(5.0, 0.0, -5.0)
	
	rclpy.spin(action_client)

if __name__ == '__main__':
	main()
