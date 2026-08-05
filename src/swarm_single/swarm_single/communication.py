import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from std_msgs.msg import String
from swarm_msgs.msg import Status, ManualControl, FormationCommand
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from swarm_msgs.action import Fly
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus
from swarm_config.config_utils import get_config
import math
import json
import numpy as np
import asyncio
from rclpy.callback_groups import ReentrantCallbackGroup

class Communication():
    def __init__(self, parent_node: Node):
        self.parent_node = parent_node
        
        self.formation_topic_name = get_config('swarm_single.formation_topic_name')
        self.command_topic_name = get_config('swarm_single.command_topic_name')
        self.liveness_topic_name = get_config('swarm_single.liveness_topic_name')
        self.manual_control_topic_name = get_config('swarm_single.manual_control_topic_name')
        self.leader_formation_topic_name = get_config('swarm_single.formation_command_topic_name')
        self.leader_command_topic_name = get_config('swarm_single.leader_command_topic_name')
        
        self.broadcast_interval = get_config('swarm_single.broadcast_interval')
        self.status_interval = get_config('swarm_single.status_interval')
        self.neighbor_timeout = self.broadcast_interval * 4
        self.default_formation_pattern = str(
            get_config('swarm_single.group.formation.pattern') or 'square'
        )
        self.default_formation_spacing = float(
            get_config('swarm_single.group.formation.spacing') or 4.0
        )
        self.group_config_signature = json.dumps(
            {
                'required_drone_ids': self.parent_node.required_drone_ids,
                'shared_frame_mode': self.parent_node.shared_frame_mode,
                'origin_offsets': get_config(
                    'swarm_single.group.shared_frame.origin_offsets'
                ),
                'reference_drone_id': (
                    self.parent_node.shared_frame_reference_id
                ),
                'formation_pattern': self.default_formation_pattern,
                'formation_spacing': self.default_formation_spacing,
            },
            sort_keys=True,
            separators=(',', ':'),
        )


        self.GOAL_TOLERANCE = get_config('swarm_single.goal_tolerance')

        self.liveness_topic = f"/swarm/{self.liveness_topic_name}"
        self.liveness_publisher = self.parent_node.create_publisher(Header, self.liveness_topic, 10)
        self.liveness_subscriber = self.parent_node.create_subscription(Header, self.liveness_topic, self._neighbor_liveness_callback, 10)
        self.liveness_broadcast_timer = self.parent_node.create_timer(self.broadcast_interval, self._broadcast_liveness)
        self.timeout_timer = self.parent_node.create_timer(self.neighbor_timeout, self._check_neighbor_timeouts)
        
        self.qos_profile_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=max(10, int(get_config('swarm_sim.drone_count')) * 2)
        )

        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.leader_velocity_subscriber = None
        self.formation_subscriber = self.parent_node.create_subscription(FormationCommand, self.formation_topic_name, self.formation_command_callback, 10)
        self.formation_publisher = None


        self.command_subscriber = self.parent_node.create_subscription(String, self.command_topic_name, self.command_callback, 10)
        self.command_publisher = None

        self.status_publisher = None
        self.leader_formation_subscriber = None
        self.leader_command_subscriber = None
        self.manual_control_subscriber = None
        self.status_timer = None

        # Every drone publishes its own readiness. The leader uses only fresh
        # reports from the explicitly configured member IDs before it permits
        # arming or movement.
        self.drone_statuses = {}
        self.drone_status_topic = '/swarm/drone_status'
        self.drone_status_publisher = self.parent_node.create_publisher(
            String, self.drone_status_topic, self.qos_profile_reliable
        )
        self.drone_status_subscriber = self.parent_node.create_subscription(
            String,
            self.drone_status_topic,
            self._drone_status_callback,
            self.qos_profile_reliable,
        )
        self.drone_status_timer = self.parent_node.create_timer(
            min(0.5, float(self.status_interval)),
            self._broadcast_drone_status,
        )
        self.group_monitor_timer = self.parent_node.create_timer(
            0.25, self._monitor_group_motion
        )
        self.pending_group_motion_reason = None
        self.pending_group_motion_start = None
        self.follower_group_start_time = None

        # --- Action Server State --- ### NEW ###
        self._action_goal_handle = None
        self._action_target_pos = None
        self._goal_check_timer = None
        
        self.referendum_conducting()
        self.parent_node.get_logger().info("Communication module initialized. Listening for swarm and station heartbeat...")

    def manual_control_handler(self, msg: ManualControl):
        """Handles manual control commands from the user."""
        #self.parent_node.get_logger().info(f"Manual Control: {msg.vx}, {msg.vy}, {msg.vz}, {msg.vyaw}")
        if msg.manual_mode:
            if (
                not self.parent_node.group_motion_active
                and not self.begin_group_motion('manual control')
            ):
                return
            if self.parent_node.mission_active:
                self.parent_node.abort_mission('manual control requested')
            self.parent_node.manual_control = True
            if (
                self.parent_node.state == "TAKEOFF"
                and self.pending_group_motion_reason is None
            ):
                self.parent_node.motion_enabled = True
            yaw = self.parent_node.yaw + msg.vyaw/30.0
            if yaw > math.pi:
                yaw -= 2 * math.pi
            elif yaw < -math.pi:
                yaw += 2 * math.pi
            self.parent_node.manual_velocity = [msg.vy, msg.vx, msg.vz]
            self.parent_node.yaw = yaw
        else:
            self.parent_node.manual_control = False
            self.parent_node.manual_velocity = [0.0, 0.0, 0.0]
            self.parent_node.get_logger().info(f"Turning manual mode off")
            
    def _broadcast_liveness(self):
        msg = Header()
        msg.stamp = self.parent_node.get_clock().now().to_msg()
        msg.frame_id = self.parent_node.frame_id
        self.liveness_publisher.publish(msg)

    def _own_drone_status(self):
        telemetry_fresh = self.parent_node.telemetry_is_fresh()
        shared_frame_ready = self.parent_node.shared_frame_is_ready()
        failsafe_flags = self.parent_node.failsafe_flags
        prearm_ready = (
            telemetry_fresh
            and shared_frame_ready
            and not self.parent_node.vehicle_status.failsafe
            and not failsafe_flags.manual_control_signal_lost
            and not failsafe_flags.local_position_invalid
            and not failsafe_flags.local_velocity_invalid
        )
        return {
            'drone_id': int(self.parent_node.drone_id),
            'leader_id': int(self.parent_node.leader_id),
            'control_state': str(self.parent_node.state),
            'armed': (
                self.parent_node.vehicle_status.arming_state
                == VehicleStatus.ARMING_STATE_ARMED
            ),
            'offboard': (
                self.parent_node.vehicle_status.nav_state
                == VehicleStatus.NAVIGATION_STATE_OFFBOARD
            ),
            'telemetry_fresh': telemetry_fresh,
            'shared_frame_ready': shared_frame_ready,
            'prearm_ready': prearm_ready,
            'group_motion_active': self.parent_node.group_motion_active,
            'group_config_signature': self.group_config_signature,
        }

    def _broadcast_drone_status(self):
        msg = String()
        msg.data = json.dumps(self._own_drone_status())
        self.drone_status_publisher.publish(msg)

    def _drone_status_callback(self, msg: String):
        try:
            status = json.loads(msg.data)
            drone_id = int(status['drone_id'])
        except (json.JSONDecodeError, KeyError, TypeError, ValueError):
            self.parent_node.get_logger().warning(
                'Ignoring malformed /swarm/drone_status payload.'
            )
            return
        if drone_id == int(self.parent_node.drone_id):
            return
        self.drone_statuses[drone_id] = (
            status,
            self.parent_node.get_clock().now(),
        )

    def group_readiness(self, require_offboard):
        """Return a fail-closed aggregate readiness snapshot."""
        if not self.parent_node.group_enabled:
            own_id = int(self.parent_node.drone_id)
            return {
                'ready': True,
                'ready_members': [own_id],
                'missing_members': [],
                'state': 'single-drone mode',
            }

        now = self.parent_node.get_clock().now()
        actual_members = set(self.parent_node.last_seen_neighbors)
        actual_members.add(int(self.parent_node.drone_id))
        expected_members = set(self.parent_node.required_drone_ids)
        missing = expected_members - actual_members
        unexpected = actual_members - expected_members
        ready_members = []
        unready_reasons = []

        for drone_id in sorted(expected_members):
            if drone_id == int(self.parent_node.drone_id):
                status = self._own_drone_status()
            else:
                record = self.drone_statuses.get(drone_id)
                if record is None:
                    missing.add(drone_id)
                    continue
                status, received_time = record
                age = (now - received_time).nanoseconds / 1e9
                if age > self.parent_node.group_status_timeout:
                    missing.add(drone_id)
                    continue

            if int(status.get('leader_id', -1)) != int(
                self.parent_node.leader_id
            ):
                unready_reasons.append(
                    f'drone {drone_id} has a different leader'
                )
                continue
            if status.get('group_config_signature') != (
                self.group_config_signature
            ):
                unready_reasons.append(
                    f'drone {drone_id} has different group configuration'
                )
                continue

            if require_offboard:
                status_ready = (
                    bool(status.get('telemetry_fresh'))
                    and bool(status.get('shared_frame_ready'))
                    and bool(status.get('armed'))
                    and bool(status.get('offboard'))
                    and status.get('control_state') == 'TAKEOFF'
                )
            else:
                status_ready = bool(status.get('prearm_ready'))

            if status_ready:
                ready_members.append(drone_id)
            else:
                phase = 'flight' if require_offboard else 'pre-arm'
                unready_reasons.append(f'drone {drone_id} is not {phase} ready')

        reasons = []
        if missing:
            reasons.append(f'missing/stale drones {sorted(missing)}')
        if unexpected:
            reasons.append(f'unexpected drones {sorted(unexpected)}')
        reasons.extend(unready_reasons)
        ready = not reasons and len(ready_members) == len(expected_members)
        return {
            'ready': ready,
            'ready_members': ready_members,
            'missing_members': sorted(missing),
            'state': 'READY' if ready else '; '.join(reasons),
        }

    def broadcast_swarm_command(self, command, **fields):
        if self.command_publisher is None:
            return
        payload = {'command': command}
        payload.update(fields)
        msg = String()
        msg.data = json.dumps(payload)
        self.command_publisher.publish(msg)

    def begin_group_motion(self, reason):
        if self.parent_node.state != 'TAKEOFF':
            self.parent_node.message = (
                'GROUP MOVE REJECTED: leader is not in armed Offboard hold'
            )
            self.parent_node.get_logger().error(self.parent_node.message)
            return False
        readiness = self.group_readiness(require_offboard=True)
        if not readiness['ready']:
            self.parent_node.message = (
                f'GROUP MOVE REJECTED: {readiness["state"]}'
            )
            self.parent_node.get_logger().error(self.parent_node.message)
            return False
        self.parent_node.group_motion_active = True
        if (
            self.parent_node.group_enabled
            and len(self.parent_node.required_drone_ids) > 1
        ):
            self.parent_node.motion_enabled = False
            self.pending_group_motion_reason = str(reason)
            self.pending_group_motion_start = (
                self.parent_node.get_clock().now()
            )
        else:
            self.parent_node.motion_enabled = True
            self.pending_group_motion_reason = None
            self.pending_group_motion_start = None
        self.broadcast_swarm_command('group_motion_start', reason=reason)
        return True

    def broadcast_group_stop(self, reason):
        self.pending_group_motion_reason = None
        self.pending_group_motion_start = None
        self.broadcast_swarm_command('group_stop', reason=str(reason))

    def _group_motion_acknowledged(self):
        now = self.parent_node.get_clock().now()
        for drone_id in self.parent_node.required_drone_ids:
            if drone_id == int(self.parent_node.drone_id):
                status = self._own_drone_status()
            else:
                record = self.drone_statuses.get(drone_id)
                if record is None:
                    return False
                status, received_time = record
                age = (now - received_time).nanoseconds / 1e9
                if age > self.parent_node.group_status_timeout:
                    return False
            if not bool(status.get('group_motion_active')):
                return False
        return True

    def _monitor_group_motion(self):
        if not self.parent_node.group_motion_active:
            return
        if not self.parent_node.is_leader:
            leader_status = self.drone_statuses.get(
                int(self.parent_node.leader_id)
            )
            if leader_status is not None:
                status, received_time = leader_status
                age = (
                    self.parent_node.get_clock().now() - received_time
                ).nanoseconds / 1e9
                if (
                    age <= self.parent_node.group_status_timeout
                    and bool(status.get('group_motion_active'))
                    and bool(status.get('offboard'))
                    and status.get('control_state') == 'TAKEOFF'
                ):
                    self.follower_group_start_time = None
                    return

            if self.follower_group_start_time is not None:
                grace = (
                    self.parent_node.get_clock().now()
                    - self.follower_group_start_time
                ).nanoseconds / 1e9
                if grace <= self.parent_node.group_status_timeout:
                    return
            self.parent_node.stop_group_motion(
                'leader group status is inactive or stale'
            )
            return

        readiness = self.group_readiness(require_offboard=True)
        if not readiness['ready']:
            reason = f'group readiness lost: {readiness["state"]}'
            self.parent_node.stop_group_motion(reason)
            self.broadcast_group_stop(reason)
            return
        if self.pending_group_motion_reason is None:
            if not self._group_motion_acknowledged():
                reason = 'a member left coordinated group motion'
                self.parent_node.stop_group_motion(reason)
                self.broadcast_group_stop(reason)
            return
        if self._group_motion_acknowledged():
            reason = self.pending_group_motion_reason
            self.pending_group_motion_reason = None
            self.pending_group_motion_start = None
            self.parent_node.motion_enabled = True
            self.parent_node.get_logger().info(
                f'All followers acknowledged {reason}; leader motion enabled.'
            )
            return

        elapsed = (
            self.parent_node.get_clock().now()
            - self.pending_group_motion_start
        ).nanoseconds / 1e9
        if elapsed > self.parent_node.group_status_timeout:
            reason = 'follower group-motion acknowledgement timed out'
            self.parent_node.stop_group_motion(reason)
            self.broadcast_group_stop(reason)
            return
        # Re-publish while waiting so a command lost during DDS discovery does
        # not leave the group permanently split.
        self.broadcast_swarm_command(
            'group_motion_start', reason=self.pending_group_motion_reason
        )

    def _broadcast_status(self):
        self.parent_node.get_logger().info("publishing status")

        msg = Status()
        msg.timestamp = int(self.parent_node.get_clock().now().nanoseconds / 1000)
        msg.swarm_members = sorted(
            list(self.parent_node.last_seen_neighbors.keys())
            + [int(self.parent_node.frame_id)]
        )
        msg.leader_id       = self.parent_node.leader_id
        msg.pattern_name    = str(self.parent_node.formation.pattern_name)
        msg.spacing         = float(self.parent_node.formation.spacing)
        msg.rotation_x      = float(self.parent_node.formation.rotation_x)
        msg.rotation_y      = float(self.parent_node.formation.rotation_y)
        msg.rotation_z      = float(self.parent_node.formation.rotation_z)
        msg.leader_x        = float(self.parent_node.navigation.current_pos[0])
        msg.leader_y        = float(self.parent_node.navigation.current_pos[1])
        msg.leader_z        = float(self.parent_node.navigation.current_pos[2])
        msg.goal_x          = float(self.parent_node.leader_goal[0])
        msg.goal_y          = float(self.parent_node.leader_goal[1])
        msg.goal_z          = float(self.parent_node.leader_goal[2])
        msg.message         = self.parent_node.message
        msg.control_state   = self.parent_node.state
        msg.armed           = (
            self.parent_node.vehicle_status.arming_state
            == VehicleStatus.ARMING_STATE_ARMED
        )
        msg.offboard        = (
            self.parent_node.vehicle_status.nav_state
            == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        )
        prearm = self.group_readiness(require_offboard=False)
        flight = self.group_readiness(require_offboard=True)
        msg.group_prearm_ready = prearm['ready']
        msg.group_ready = flight['ready']
        msg.group_motion_active = self.parent_node.group_motion_active
        msg.ready_members = flight['ready_members']
        msg.missing_members = flight['missing_members']
        msg.group_prearm_state = prearm['state']
        msg.group_state = flight['state']
        msg.mission_active  = self.parent_node.mission_active
        msg.mission_count   = len(self.parent_node.mission)
        if self.parent_node.mission_active:
            msg.mission_index = self.parent_node.mission_index + 1
        else:
            msg.mission_index = min(
                self.parent_node.mission_index,
                len(self.parent_node.mission),
            )
        msg.mission_state   = self.parent_node.mission_state
        self.parent_node.message = "" 
        self.status_publisher.publish(msg)

    def _neighbor_liveness_callback(self, msg: Header):
        count = len(list(self.parent_node.last_seen_neighbors.keys()))
        if not msg.frame_id==self.parent_node.frame_id: 
            self.parent_node.last_seen_neighbors[int(msg.frame_id)] = self.parent_node.get_clock().now()
        if not count==len(list(self.parent_node.last_seen_neighbors.keys())): 
            self.parent_node.get_logger().info(f"Connection established with neighbor {msg.frame_id}! Swarm peers count: {len(self.parent_node.last_seen_neighbors)}")
            self.referendum_conducting()

    def _check_neighbor_timeouts(self):
        now = self.parent_node.get_clock().now()
        timed_out_neighbors = set()
        for neighbor_id, last_seen_time in self.parent_node.last_seen_neighbors.items():
            if (now - last_seen_time).nanoseconds / 1e9 > self.neighbor_timeout:
                timed_out_neighbors.add(neighbor_id)
        
        for neighbor_id in timed_out_neighbors:
            self.parent_node.get_logger().info(f"Connection lost with neighbor {neighbor_id}! Removing from swarm topology.")
            del self.parent_node.last_seen_neighbors[neighbor_id]
        
        if bool(timed_out_neighbors):
            if self.parent_node.group_motion_active:
                reason = f'lost swarm neighbors {sorted(timed_out_neighbors)}'
                self.parent_node.stop_group_motion(reason)
                if self.parent_node.is_leader:
                    self.broadcast_group_stop(reason)
            self.referendum_conducting()

    def referendum_conducting(self):
        swarm_members = list(self.parent_node.last_seen_neighbors.keys()) + [int(self.parent_node.frame_id)]
        leader_id = min(swarm_members)
        expected_members = set(self.parent_node.required_drone_ids)
        topology_complete = set(swarm_members) == expected_members
        if self.parent_node.group_enabled and not topology_complete:
            self.parent_node.leader_id = leader_id
            if self.parent_node.group_motion_active:
                reason = (
                    f'group topology changed: present={sorted(swarm_members)}, '
                    f'required={sorted(expected_members)}'
                )
                self.parent_node.stop_group_motion(reason)
                if self.parent_node.is_leader:
                    self.broadcast_group_stop(reason)
            if self.parent_node.is_leader:
                self.step_down_as_leader()
            if self.parent_node.formation.pattern_name is None:
                self.parent_node.formation.set_pattern(
                    self.default_formation_pattern,
                    self.default_formation_spacing,
                )
            else:
                self.parent_node.formation.refresh_pattern()
            self.parent_node.get_logger().warning(
                f'Group topology incomplete: present={sorted(swarm_members)}, '
                f'required={sorted(expected_members)}. Station commands remain '
                'disabled.'
            )
            return
        leader_changed = leader_id != self.parent_node.leader_id
        if leader_changed:
            self.parent_node.leader_id = leader_id

            self.parent_node.get_logger().info(f"Topology update: This is drone {self.parent_node.frame_id} and leader is {leader_id}")

        if leader_changed or self.leader_velocity_subscriber is None:
            if self.leader_velocity_subscriber is not None:
                self.parent_node.destroy_subscription(
                    self.leader_velocity_subscriber
                )
            self.leader_velocity_subscriber =self.parent_node.create_subscription(
            VehicleLocalPosition, f"/uav_{leader_id}/fmu/out/vehicle_local_position_v1", self.vehicle_local_position_callback, self.qos_profile)
        if leader_id == int(self.parent_node.frame_id) and not self.parent_node.is_leader:
            self.become_leader()
        elif leader_id != int(self.parent_node.frame_id) and self.parent_node.is_leader:
            self.step_down_as_leader()
        if self.parent_node.formation.pattern_name is None:
            self.parent_node.formation.set_pattern(
                self.default_formation_pattern,
                self.default_formation_spacing,
            )
        else:
            self.parent_node.formation.refresh_pattern()

    def become_leader(self, ):
        self.parent_node.get_logger().info('Transitioning to LEADER role. Establishing Station connection...')
        self.parent_node.goal_callback_temp( [self.parent_node.navigation.current_pos[0], self.parent_node.navigation.current_pos[1], self.parent_node.navigation.current_pos[2]])
        self.parent_node.is_leader = True
        self.status_publisher = self.parent_node.create_publisher(Status, "/swarm/status",self.qos_profile_reliable)
        self.manual_control_subscriber = self.parent_node.create_subscription(ManualControl, self.manual_control_topic_name, self.manual_control_handler, 10)
        self.status_timer = self.parent_node.create_timer(self.status_interval, self._broadcast_status)
        self._fly_action = ActionServer(self.parent_node, Fly, '/fly_to', execute_callback=self.execute_callback,callback_group=ReentrantCallbackGroup(),goal_callback=self.goal_callback,cancel_callback=self.cancel_callback)

        self.formation_publisher = self.parent_node.create_publisher(FormationCommand, self.formation_topic_name, 10)
        self.command_publisher = self.parent_node.create_publisher(String, self.command_topic_name, 10)
        self.parent_node.formation.set_pattern(
            self.default_formation_pattern,
            self.default_formation_spacing,
        )
        self.send_formation_command(
            self.default_formation_pattern,
            self.default_formation_spacing,
        )
        if (self.formation_subscriber is not None):
            self.parent_node.destroy_subscription(self.formation_subscriber)
            self.parent_node.destroy_subscription(self.command_subscriber)

        self.formation_subscriber = None
        self.command_subscriber = None

        self.leader_formation_subscriber = self.parent_node.create_subscription(FormationCommand, self.leader_formation_topic_name, self.formation_leader_callback, 10)
        self.leader_command_subscriber = self.parent_node.create_subscription(String, self.leader_command_topic_name, self.command_leader_callback, 10)
    
    def step_down_as_leader(self, ):
        self.parent_node.get_logger().info('Stepping down as leader. Reverting to FOLLOWER role.')
        self.parent_node.is_leader = False
        self.parent_node.destroy_publisher(self.status_publisher)
        self.parent_node.destroy_subscription(self.manual_control_subscriber)
        self.parent_node.destroy_subscription(self.leader_formation_subscriber)
        self.parent_node.destroy_subscription(self.leader_command_subscriber)
        self.status_timer.cancel()
        self.parent_node.destroy_publisher(self.formation_publisher)
        self.parent_node.destroy_publisher(self.command_publisher)
        self.formation_subscriber = self.parent_node.create_subscription(FormationCommand, self.formation_topic_name, self.formation_command_callback, 10)
        self.command_subscriber = self.parent_node.create_subscription(String, self.command_topic_name, self.command_callback, 10)
        self._fly_action.destroy()
        self._fly_action = None
        self.formation_publisher = None
        self.command_publisher = None
        self.leader_formation_subscriber = None
        self.leader_command_subscriber = None
        self.status_publisher = None
        self.manual_control_subscriber = None
        self.status_timer = None

    def command_callback(self, msg: String):
        self.parent_node.get_logger().info(f"Follower received command payload: {msg.data}")
        try:
            cmd = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.parent_node.get_logger().error(f"JSON Decode Error: {e}")
            return
        command_type = cmd.get("command")
        if command_type == "arm":
            self.parent_node.request_offboard_control()
            self.parent_node.get_logger().info("Received ARM command.")

        elif command_type in ('group_motion_start', 'mission'):
            # Followers track their formation transform; they never execute
            # the leader's absolute mission coordinates.
            if (
                self.parent_node.state == "TAKEOFF"
                and self.parent_node.telemetry_is_fresh()
                and self.parent_node.shared_frame_is_ready()
            ):
                self.parent_node.group_motion_active = True
                self.parent_node.motion_enabled = True
                self.follower_group_start_time = (
                    self.parent_node.get_clock().now()
                )
                self.parent_node.get_logger().info(
                    'Follower group tracking enabled.'
                )
            else:
                self.parent_node.stop_group_motion(
                    'group start ignored because this follower is not ready'
                )

        elif command_type == 'group_stop':
            self.follower_group_start_time = None
            self.parent_node.stop_group_motion(
                cmd.get('reason', 'leader requested group hold')
            )

        elif command_type == 'land':
            self.follower_group_start_time = None
            self.parent_node.stop_group_motion('group LAND requested')
            self.parent_node.request_land()

        elif command_type == "start_animation":
            start_time = int(cmd.get("start_time"))

            speed_x = float(cmd.get("speed_x"))
            speed_y = float(cmd.get("speed_y"))
            speed_z = float(cmd.get("speed_z"))

            self.parent_node.formation.start_animation(start_time=start_time, speed_x= speed_x, speed_y= speed_y, speed_z= speed_z)
        elif command_type == "stop_animation":
            self.parent_node.formation.stop_animation()
        elif command_type == "leader_is_dead":
            del self.parent_node.last_seen_neighbors[self.parent_node.leader_id]
            self.referendum_conducting()


    def command_leader_callback(self, msg: String):
        self.parent_node.get_logger().info(f"Leader received Ground Station command: {msg.data}")
        try:
            cmd = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.parent_node.get_logger().error(f"JSON Decode Error: {e}")
            return
        command_type = cmd.get("command")
        if command_type == "arm":
            readiness = self.group_readiness(require_offboard=False)
            if not readiness['ready']:
                self.parent_node.message = (
                    f'GROUP ARM REJECTED: {readiness["state"]}'
                )
                self.parent_node.get_logger().error(self.parent_node.message)
                return
            self.parent_node.request_offboard_control()
            if self.parent_node.state in ('ARMING', 'TAKEOFF'):
                self.broadcast_swarm_command('arm')
                self.parent_node.get_logger().info(
                    "Leader: publishing ARM command."
                )
        elif command_type == "fly":
            readiness = self.group_readiness(require_offboard=True)
            if not readiness['ready']:
                self.parent_node.message = (
                    f'GROUP MOVE REJECTED: {readiness["state"]}'
                )
                self.parent_node.get_logger().error(self.parent_node.message)
                return
            x = float(cmd.get("x"))
            y = float(cmd.get("y"))
            z = float(cmd.get("z"))
            absolute = cmd.get("absolute", True)

            if absolute:
                goal_accepted = self.parent_node.goal_callback_temp([x, y, z])
            else:
                goal_accepted = self.parent_node.goal_callback_temp([self.parent_node.leader_goal[0] + x, self.parent_node.leader_goal[1] + y, self.parent_node.leader_goal[2] + z])
            if goal_accepted and self.parent_node.state == "TAKEOFF":
                if self.parent_node.mission_active:
                    self.parent_node.abort_mission(
                        'replaced by a station move/set_goal command'
                    )
                self.begin_group_motion('station move/set_goal')
            elif goal_accepted:
                self.parent_node.get_logger().warning(
                    "Goal stored but motion is disabled: explicitly ARM Offboard first."
                )
                
        elif command_type == "disarm_leader":
            self.parent_node.request_safe_disarm()
        elif command_type == "land":
            self.pending_group_motion_reason = None
            self.pending_group_motion_start = None
            self.broadcast_swarm_command('land')
            self.parent_node.request_land()
        elif command_type == "start_animation":
            start_time = int(self.parent_node.get_clock().now().nanoseconds / 1000000)

            speed_x = float(cmd.get("speed_x"))
            speed_y = float(cmd.get("speed_y"))
            speed_z = float(cmd.get("speed_z"))

            #self.parent_node.formation.start_animation()
            command = {
                "command": "start_animation",
                "start_time": start_time,
                "speed_x": speed_x,
                "speed_y": speed_y,
                "speed_z": speed_z
            }
            msg = String()
            msg.data = json.dumps(command)
            self.command_publisher.publish(msg)

            self.parent_node.get_logger().info(f"Leader: Publishing animation command.")
        elif command_type == "stop_animation":
            out_msg = String()
            out_msg.data = json.dumps({"command": "stop_animation"})
            self.command_publisher.publish(out_msg)
            self.parent_node.get_logger().info("Leader: Publishing stop animation command.")
        elif command_type == 'mission':
            mission = cmd.get("points")
            relative_to_start = cmd.get("relative_to_start", True)
            readiness = self.group_readiness(require_offboard=True)
            if not readiness['ready']:
                self.parent_node.message = (
                    f'GROUP MISSION REJECTED: {readiness["state"]}'
                )
                self.parent_node.get_logger().error(self.parent_node.message)
                return
            if self.parent_node.start_mission(
                mission,
                relative_to_start=relative_to_start,
            ):
                self.parent_node.get_logger().info(
                    f"Leader accepted mission with {len(mission)} waypoints."
                )
                if not self.begin_group_motion('mission'):
                    self.parent_node.abort_mission(
                        'group readiness lost before coordinated start'
                    )

            
    def send_mission(self):
        command = {
            "command": "mission",
            "points": self.parent_node.mission,
            "relative_to_start": False,
        }
        msg = String()
        msg.data = json.dumps(command)
        self.command_publisher.publish(msg)

    def vehicle_local_position_callback(self, vehicle_local_position):
        self.parent_node.navigation.vel = [vehicle_local_position.vx, vehicle_local_position.vy, vehicle_local_position.vz ]

    def formation_leader_callback(self, msg: FormationCommand):
        if self.parent_node.group_motion_active:
            self.parent_node.get_logger().error(
                'Formation change rejected while group motion is active.'
            )
            return
        self.parent_node.formation.set_pattern(
            pattern_name=msg.pattern_name,
            spacing=msg.spacing,
            rotation_x=msg.rotation_x,
            rotation_y=msg.rotation_y,
            rotation_z=msg.rotation_z
        )
        self.send_formation_command(msg.pattern_name, msg.spacing, rotation_x=msg.rotation_x, rotation_y=msg.rotation_y, rotation_z=msg.rotation_z)
    
    def send_formation_command(self, name, spacing, rotation_x=0.0, rotation_y=0.0, rotation_z=0.0, line_type="X"):
        """Populates and publishes a FormationCommand message."""
        if self.formation_publisher:
            msg = FormationCommand()
            msg.pattern_name = name
            msg.spacing = float(spacing)
            msg.rotation_x = float(rotation_x)
            msg.rotation_y = float(rotation_y)
            msg.rotation_z = float(rotation_z)
            
            self.formation_publisher.publish(msg)
            self.parent_node.get_logger().info(f"Published formation command: '{name}' with spacing {spacing}")

    def formation_command_callback(self, msg: FormationCommand):
        self.parent_node.get_logger().info(
            f"Received new formation command: '{msg.pattern_name}' with spacing {msg.spacing}"
        )
        self.parent_node.formation.set_pattern(
            pattern_name=msg.pattern_name,
            spacing=msg.spacing,
            rotation_x=msg.rotation_x,
            rotation_y=msg.rotation_y,
            rotation_z=msg.rotation_z
        )

    def goal_callback(self, goal_request):
        """Accept or reject a client request to start an action."""
        self.parent_node.get_logger().info(f"Received goal request: {goal_request.goal}")
        if self.parent_node.state != "TAKEOFF":
            self.parent_node.get_logger().warning(
                'Fly action rejected: Offboard is not active.'
            )
            return GoalResponse.REJECT
        readiness = self.group_readiness(require_offboard=True)
        if not readiness['ready']:
            self.parent_node.get_logger().warning(
                f'Fly action rejected: {readiness["state"]}'
            )
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT
    def cancel_callback(self, goal_handle):
        self.parent_node.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT
    
    async def execute_callback(self, goal_handle):
        goal = goal_handle.request.goal
        absolute = goal_handle.request.absolute
        self.parent_node.get_logger().info(f"Executing fly_to goal: [{goal.x}, {goal.y}, {goal.z}]")


        # Send goal to drone
        if absolute:
            goal_accepted = self.parent_node.goal_callback_temp(
                [goal.x, goal.y, goal.z]
            )
        else:
            goal_accepted = self.parent_node.goal_callback_temp(
                [
                    self.parent_node.leader_goal[0] + goal.x,
                    self.parent_node.leader_goal[1] + goal.y,
                    self.parent_node.leader_goal[2] + goal.z,
                ]
            )
        if not goal_accepted:
            goal_handle.abort()
            result_msg = Fly.Result()
            result_msg.result = 'Rejected by safety limits'
            return result_msg
        if self.parent_node.state == "TAKEOFF":
            if not self.begin_group_motion('fly action'):
                goal_handle.abort()
                result_msg = Fly.Result()
                result_msg.result = 'Group readiness lost before movement'
                return result_msg
        

        # Create feedback message
        feedback_msg = Fly.Feedback()
        result_msg = Fly.Result()

        # Feedback loop
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.parent_node.get_logger().info('Goal canceled by client')
                self.parent_node.stop_group_motion('fly action cancelled')
                self.broadcast_group_stop('fly action cancelled')
                goal_handle.canceled()
                result_msg.result = 'Cancelled'
                return result_msg

            # Get current position
            current_pos_vec = np.array([self.parent_node.navigation.current_pos[0], self.parent_node.navigation.current_pos[1], self.parent_node.navigation.current_pos[2]])
        
            distance = np.linalg.norm([goal.x, goal.y, goal.z]- current_pos_vec)

            feedback_msg.distance_remaining = float(distance)
            goal_handle.publish_feedback(feedback_msg)

            self.parent_node.get_logger().info(f'Distance remaining: {distance:.2f}')

            if distance < self.GOAL_TOLERANCE:  # Reached the goal
                self.parent_node.get_logger().info("Goal reached!")
                break

            await asyncio.sleep(1.0)  # Non-blocking sleep
            #time.sleep(1)
        goal_handle.succeed()
        result_msg.result = 'Reached'
        return result_msg
