import numpy as np
import rclpy
from abc import ABC, abstractmethod
from typing import Dict, List, Any

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

from scipy.spatial.transform import Rotation

from swarm_config.config_utils import get_config

class FormationPattern(ABC):
    """Abstract Base Class for all formation patterns."""
    @abstractmethod
    def calculate_position(self, drone_id: int, follower_ids: List[int], spacing: float, **kwargs) -> np.ndarray:
        """
        Calculates the position for a single drone within a formation.

        Args:
            drone_id: The ID of the drone for which to calculate the position.
            follower_ids: A sorted list of all follower drone IDs.
            spacing: The base distance between drones.
            **kwargs: Pattern-specific arguments (e.g., line_type).

        Returns:
            A numpy array representing the [x, y, z] offset from the leader.
        """
        pass


class LinePattern(FormationPattern):
    def calculate_position(self, drone_id: int, follower_ids: List[int], spacing: float, **kwargs) -> np.ndarray:
        line_type = kwargs.get("line_type", "X").upper()
        
        try:
            # The drone's index in the line (0, 1, 2...)
            idx = follower_ids.index(drone_id) + 1
        except ValueError:
            return np.array([0.0, 0.0, 0.0]) # Should not happen if list is correct

        if line_type == "X":  return np.array([-spacing * idx, 0, 0])
        if line_type == "Y":  return np.array([0, -spacing * idx, 0])
        if line_type == "Z":  return np.array([0, 0, spacing * idx])
        if line_type == "XY": return np.array([-spacing * idx, -spacing * idx, 0])
        return np.array([0.0, 0.0, 0.0])


class SquarePattern(FormationPattern):
    def calculate_position(self, drone_id: int, follower_ids: List[int], spacing: float, **kwargs) -> np.ndarray:
        num = follower_ids.index(drone_id) + 2
        k = np.ceil((np.sqrt(num) - 1) / 2)
        side_length = 2 * k
        prev_ring_max_id = (2 * (k - 1) + 1)**2
        pos_in_ring = num - prev_ring_max_id
        side_index = np.floor((pos_in_ring - 1) / side_length)
        pos_on_side = (pos_in_ring - 1) % side_length
        x, y = 0, 0
        if side_index == 0: x, y = k, pos_on_side - (k - 1)
        elif side_index == 1: x, y = (k - 1) - pos_on_side, k
        elif side_index == 2: x, y = -k, (k - 1) - pos_on_side
        elif side_index == 3: x, y= pos_on_side - (k - 1), -k
        return np.array([x * spacing, y * spacing, 0])


class CirclePattern(FormationPattern):
    def calculate_position(self, drone_id: int, follower_ids: List[int], spacing: float, **kwargs) -> np.ndarray:
        num_followers = len(follower_ids)
        if num_followers == 0:
            return np.array([0.0, 0.0, 0.0])
            
        try:
            idx = follower_ids.index(drone_id)
        except ValueError:
            return np.array([0.0, 0.0, 0.0])

        radius = spacing * (1 + num_followers // 8)
        angle_step = 2 * np.pi / num_followers
        angle = idx * angle_step
        
        return np.array([radius * np.cos(angle), radius * np.sin(angle), 0])

class TrianglePattern(FormationPattern):

    def calculate_position(self, drone_id: int, follower_ids: List[int], spacing: float, **kwargs) -> np.ndarray:
        idx = follower_ids.index(drone_id)+1
        row = int(np.floor((-1 + np.sqrt(1 + 8 * idx)) / 2))
        drones_in_prev_rows = int(row * (row + 1) / 2)
        pos_in_row = idx - drones_in_prev_rows
        x = -(row + 1) * (spacing * np.sqrt(3) / 2)
        y = (pos_in_row - row / 2.0) * spacing
        return np.array([x, y, 0.0])

class PatternController:
    PATTERN_REGISTRY: Dict[str, FormationPattern] = {
        "line": LinePattern(),
        "square": SquarePattern(),
        "circle": CirclePattern(),
        "triangle": TrianglePattern(),
    }

    def __init__(self, parent_node: rclpy.node.Node):
        
        self.parent = parent_node

        self.animation_resolution = get_config('swarm_single.formation.animation_resolution')

        self.pattern_name = None
        self.spacing = 3.0
        self.rotation_x = 0.0
        self.rotation_y = 0.0
        self.rotation_z = 0.0   

        self.rotation_x_speed = 0.0
        self.rotation_y_speed = 0.0
        self.rotation_z_speed = 0.0
        self.animation_start_time = 0

        self.animation_timer = None #self.parent_node.create_timer(0.1, self.run_animation)

    def start_animation(self, start_time, speed_x, speed_y, speed_z):
        self.parent.get_logger().info(f"Start performing animation")
        self.parent.get_logger().info(f"time is {int(self.parent.get_clock().now().nanoseconds / 1000000)}")
        self.animation_start_time = start_time
        if self.animation_timer is not None:
            self.animation_timer.cancel()
        self.animation_timer = self.parent.create_timer(self.animation_resolution, self.run_animation)
        if(speed_x != 0):
            self.rotation_x_speed = 360/(speed_x*1000)
        else:
            self.rotation_x_speed = 0.0
        if(speed_y != 0):
            self.rotation_y_speed = 360/(speed_y*1000)
        else:
            self.rotation_y_speed = 0.0
        if(speed_z != 0):
            self.rotation_z_speed = 360/(speed_z*1000)
        else:
            self.rotation_z_speed = 0.0

    def run_animation(self):
        current_time = int(self.parent.get_clock().now().nanoseconds / 1000000)
        differ = current_time - self.animation_start_time
        self.rotation_x = self.rotation_x + (differ * self.rotation_x_speed)
        self.rotation_y = self.rotation_y + (differ * self.rotation_y_speed)
        self.rotation_z = self.rotation_z + (differ * self.rotation_z_speed)
        self.animation_start_time = current_time
        self.refresh_pattern()

    def stop_animation(self):
        if self.animation_timer is None:
            self.parent.get_logger().info("Animation is already stopped")
            return
        self.animation_timer.cancel()
        self.animation_timer = None 
        self.parent.get_logger().info(f"Stop performing animation")

    def refresh_pattern(self):
        if self.pattern_name != None:
            self.set_pattern(self.pattern_name, self.spacing, self.rotation_x, self.rotation_y, self.rotation_z)
    def set_pattern(self, pattern_name: str, spacing: float = 3.0, rotation_x: float = 0.0, rotation_y: float = 0.0, rotation_z: float = 0.0, **kwargs: Any) -> None:
        self.pattern_name = pattern_name
        self.spacing = spacing
        self.rotation_x = rotation_x
        self.rotation_y = rotation_y
        self.rotation_z = rotation_z

        pattern_name = pattern_name.lower()
        if pattern_name not in self.PATTERN_REGISTRY:
            self.parent.get_logger().error(f"Unknown pattern name: '{pattern_name}'.")
            return
        pattern = self.PATTERN_REGISTRY[pattern_name]
        my_id = int(self.parent.frame_id)
        leader_id = self.parent.leader_id
        all_drone_ids = list(self.parent.last_seen_neighbors.keys())
        all_drone_ids.append(my_id)

        offset = np.array([0.0, 0.0, 0.0])
        parent_frame = f'{self.parent.px4_model}_{my_id}/odom' 

        if my_id == leader_id:
            self.parent.get_logger().info("I am the leader. Setting goal to hold position.")
            return
        else:
            parent_frame = f'{self.parent.px4_model}_{leader_id}/odom'
            
            follower_ids = sorted([_id for _id in all_drone_ids if _id != leader_id])
            
            offset = pattern.calculate_position(my_id, follower_ids, spacing, **kwargs)

            rotation = Rotation.from_euler(
                'xyz', 
                [rotation_x, rotation_y, rotation_z], 
                degrees=True
            )
            offset = rotation.apply(offset)
            self.parent.get_logger().info(f"I am a follower from followers {follower_ids}. New offset from leader: {offset}")
        self._publish_goal_transform(offset, parent_frame)

    def _publish_goal_transform(self, offset: np.ndarray, parent_frame: str):
        """Publishes the goal as a static TF transform."""
        goal_broadcaster = StaticTransformBroadcaster(self.parent)
        t = TransformStamped()
        t.header.stamp = self.parent.get_clock().now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id = f'{self.parent.px4_model}_{self.parent.frame_id}/{self.parent.goal_frame}'

        t.transform.translation.x = offset[0]
        t.transform.translation.y = offset[1]
        t.transform.translation.z = offset[2]
        t.transform.rotation.w = 1.0  # No change in orientation

        goal_broadcaster.sendTransform(t)
        self.parent.get_logger().info(f"Published goal transform for drone {self.parent.frame_id} {offset}")
