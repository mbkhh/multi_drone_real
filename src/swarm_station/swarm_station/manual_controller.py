from rclpy.node import Node
from geometry_msgs.msg import Twist
from swarm_msgs.msg import ManualControl
import pygame
from pynput import keyboard
import threading
import time

class ManualController():
    def __init__(self,parent_node: Node):
        self.parent_node = parent_node

        self.publisher = self.parent_node.create_publisher(ManualControl, '/manual_controller', 10)
        self.timer = self.parent_node.create_timer(0.05, self.publish_cmd)  # 20 Hz

        pygame.init()
        pygame.joystick.init()
        self.clock = pygame.time.Clock()

        self.vel = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}
        self.input_mode = 'none'  # 'joystick' or 'keyboard'
        self.max_speed = 1.0  

        self.z_axis_offset = 0.0  # will be auto-detected
        self.deadzone = 0.1
        self.calibrated = False

        self.pressed_keys = set()
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

        # Launch input thread
        self.input_thread = threading.Thread(target=self.input_loop, daemon=True)
        self.input_thread.start()

        self.manual_mode = False

    def on_press(self, key):
        try:
            self.pressed_keys.add(key.char.lower())
        except AttributeError:
            self.pressed_keys.add(key)

    def on_release(self,key):
        try:
            self.pressed_keys.discard(key.char.lower())
        except AttributeError:
            self.pressed_keys.discard(key)

    def input_loop(self):
        joystick = None

        while True:
            pygame.event.pump()

            if pygame.joystick.get_count() > 0:
                if joystick is None:
                    joystick = pygame.joystick.Joystick(0)
                    joystick.init()
                    print(f"Joystick detected: {joystick.get_name()}")
            else: 
                joystick = None
            
            joy_active = False
            key_active = False

            if 'w' in self.pressed_keys:
                self.vel['x'] = 1.0
                key_active = True
            elif 's' in self.pressed_keys:
                self.vel['x'] = -1.0
                key_active = True
            else:
                self.vel['x'] = 0.0

            if 'a' in self.pressed_keys:
                self.vel['y'] = 1.0
                key_active = True
            elif 'd' in self.pressed_keys:
                self.vel['y'] = -1.0
                key_active = True
            else:
                self.vel['y'] = 0.0

            if keyboard.Key.up in self.pressed_keys:
                self.vel['z'] = 1.0
                key_active = True
            elif keyboard.Key.down in self.pressed_keys:
                self.vel['z'] = -1.0
                key_active = True
            else:
                self.vel['z'] = 0.0

            if keyboard.Key.left in self.pressed_keys:
                self.vel['yaw'] = 1.0
                key_active = True
            elif keyboard.Key.right in self.pressed_keys:
                self.vel['yaw'] = -1.0
                key_active = True
            else:
                self.vel['yaw'] = 0.0

            if joystick:
                ax0 = joystick.get_axis(0)  # left stick horizontal (Y)
                ax1 = -joystick.get_axis(1)  # left stick vertical (X)
                ax2 = joystick.get_axis(3)  # right stick horizontal (Yaw)
                ax3_raw = -joystick.get_axis(4)  # right stick vertical (Z)
                
                if not self.calibrated:
                    self.z_axis_offset = ax3_raw  # Save baseline at startup
                    print(f"Calibrated Z axis offset: {self.z_axis_offset:.3f}")
                    self.calibrated = True

                ax3 = ax3_raw - self.z_axis_offset  # Remove unwanted shift

                # Apply deadzone filtering
                def deadzone(val, threshold):
                    return val if abs(val) > threshold else 0.0

                ax0 = deadzone(ax0, self.deadzone)
                ax1 = deadzone(ax1, self.deadzone)
                ax2 = deadzone(ax2, self.deadzone)
                ax3 = deadzone(ax3, self.deadzone)


                A = joystick.get_button(0)
                B = joystick.get_button(1)
                X = joystick.get_button(2)
                Y = joystick.get_button(3)

                BACK = joystick.get_button(6)
                START = joystick.get_button(7)
                GUIDE = joystick.get_button(8)
                LEFT_STICK = joystick.get_button(9)
                RIGHT_STICK = joystick.get_button(10)
                

                RB = joystick.get_button(5)  # Right Bumper (Z+)
                LB = joystick.get_button(4)  # Left Bumper (Z-)

                hat_x, hat_y = joystick.get_hat(0)

                if A:
                    self.parent_node.send_arm_command()

                if Y:
                    self.parent_node.send_formation(
                        'square',
                        spacing=3.0,
                        rotation_x=0.0,
                        rotation_y=0.0,
                        rotation_z=0.0
                    )
                elif X:
                    self.parent_node.send_formation(
                        'triangle',
                        spacing=3.0,
                        rotation_x=0.0,
                        rotation_y=0.0,
                        rotation_z=0.0
                    )

                if hat_y==1:
                    self.manual_mode = True
                elif hat_y==-1:
                    if self.manual_mode == True:
                        self.publish_cmd(True)
                    self.manual_mode = False

                if RB:
                    self.max_speed = self.max_speed + 0.1
                elif LB:        
                    self.max_speed = max(0.1, self.max_speed - 0.1)
                # If there's significant joystick movement
                if abs(ax0) > 0.1 or abs(ax1) > 0.1 or abs(ax2) > 0.1 or abs(ax3) > 0.1:
                    self.vel['y'] = ax1* self.max_speed
                    self.vel['x'] = ax0* self.max_speed
                    self.vel['yaw'] = ax2
                    self.vel['z'] = ax3* self.max_speed
                    joy_active = True

            # Update control source
            if joy_active:
                self.input_mode = 'joystick'
            elif key_active:
                self.input_mode = 'keyboard'

            self.clock.tick(20)

    def publish_cmd(self, send_manual_off = False):
        # Publish the velocity command
        if send_manual_off:
            manual_msg = ManualControl()
            manual_msg.timestamp = self.parent_node.get_clock().now().to_msg().sec
            manual_msg.vx = 0.0
            manual_msg.vy = 0.0   
            manual_msg.vz = 0.0
            manual_msg.vyaw = 0.0
            manual_msg.manual_mode = False
            self.publisher.publish(manual_msg)
            self.parent_node.get_logger().info("Turning manual mode off.")
        if not self.manual_mode:
            return
        manual_msg = ManualControl()
        manual_msg.timestamp = self.parent_node.get_clock().now().to_msg().sec
        manual_msg.vx = self.vel['x']
        manual_msg.vy = self.vel['y']   
        manual_msg.vz = self.vel['z']
        manual_msg.vyaw = self.vel['yaw']
        manual_msg.manual_mode = True
        self.publisher.publish(manual_msg)
        # self.parent_node.get_logger().info(
        #     f"[{self.input_mode.upper()}] x: {self.vel['x']:.2f}, y: {self.vel['y']:.2f}, z: {self.vel['z']:.2f}, yaw: {self.vel['yaw']:.2f}",
        #     throttle_duration_sec=1.0
        # )
