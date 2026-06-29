import math
from abc import ABC, abstractmethod
from typing import Dict, List, Optional, Any
from swarm_single.vector3 import Vector3

class FormationPattern(ABC):
	def __init__(self, spacing: float = 2.0, **kwargs):
		self.spacing: float = spacing
		self.follower_ids: List[int] = []

	def set_followers(self, follower_ids: List[int]): self.follower_ids = follower_ids

	@abstractmethod
	def calculate_positions(self, ) -> Dict[int, Vector3]:
		pass

class CirclePattern(FormationPattern):
	def __init__(self, time: float, spacing: float = 2.0, **kwargs):
		super(CirclePattern, self).__init__(spacing, **kwargs)
		self.time = time

	def calculate_positions(self, ) -> Dict[int, Vector3]:
		radius = self.spacing * (1 + len(self.follower_ids) // 8)
		angle_step = 2 * math.pi / len(self.follower_ids)
		positions = {}
		for drone in self.follower_ids:
			angle = self.follower_ids.index(drone) * angle_step * self.time
			positions[drone] = Vector3(radius * math.cos(angle), radius * math.sin(angle), 0)
		return positions


		
class SwarmForamation():
	
	def __init__(self, ):
		self.queue = []

	def position_change_callback(self, ):
		positions = []
		for move in self.queue:
			positions.append(move.calculate_positions())

if __name__ == '__main__':
	cp = CirclePattern(time = 2.1, spacing=3.0)
	cp.set_followers([1,2,3,4,5,6])
	print(cp.calculate_positions())
	print()