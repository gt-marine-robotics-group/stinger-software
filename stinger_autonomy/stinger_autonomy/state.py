'''
Stinger Finite State Machine
'''
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np
from enum import Enum
from stinger_msgs.msg import Gate

class State(Enum):
    Searching = 0
    Approaching = 1
    Passing_Through = 2
    PassedThrough = 3

class StateMachine(Node):
    def __init__(self):
        super().__init__("State_Machine")

        self.image_width = 1280
        self.hfov = 1.09956
        self.angular_correction_factor = 1.0

        self.previous_state = None
        self.state = State.Searching

        self.timer = self.create_timer(0.1, self.state_machine_callback)
        
        self.pre_push_time = None # time difference from last push and now calls correct orientation state

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.gate_pos_sub = self.create_subscription(Gate, '/stinger/gate_location', self.gate_callback, 10)

        self.current_gate_pos : Gate = Gate()
        self.get_logger().info("State Machine node initialized!")
    
    def gate_callback(self, msg: Gate):
        self.current_gate_pos = msg

    def calculate_gate_fov_bound(self, left_gate_x, right_gate_x):
        distance = 1
        if left_gate_x is not None and right_gate_x is not None:
            distance = (right_gate_x - left_gate_x) / self.image_width
        return distance
    
    def state_debugger(self):
        if self.previous_state != self.state:
            self.get_logger().info(f"Now at: {self.state.name}")
            self.previous_state = self.state

    def state_machine_callback(self):
        self.state_debugger()
        cmd_vel = Twist()
        match self.state:
            case State.Searching:
                cmd_vel = self.search()
            case State.Approaching:
                cmd_vel = self.approach()
            case State.Passing_Through:
                cmd_vel = self.pass_through()
            case State.PassedThrough:
                self.complete()
            case _:
                pass
        if cmd_vel is None:
            return
        self.cmd_vel_pub.publish(cmd_vel)

    def search(self):
        cmd_vel = Twist()
        left_gate_location = self.current_gate_pos.red_x
        right_gate_location = self.current_gate_pos.green_x

        self.get_logger().info(f"Red: {left_gate_location}, Green: {right_gate_location}")

        if left_gate_location < 0 or right_gate_location < 0:
            return
        
        deg_per_pixel = self.hfov / self.image_width

        mid_x = (right_gate_location + left_gate_location) / 2
        mid_x_img = self.image_width // 2
        diff_mid = mid_x_img - mid_x

        '''
        If we are far too left, want to turn right, this will be a negative value
        If we are far too right, want to turn left, this will be a positive value
        '''
        turn_angle = diff_mid * deg_per_pixel

        self.get_logger().info(f"Turn angle: {turn_angle}")

        if abs(cmd_vel.angular.z) > 0.1:
            cmd_vel.angular.z = np.sign(cmd_vel.angular.z) * 0.1

        cmd_vel.angular.z = np.clip(self.angular_correction_factor * turn_angle, -0.1, 0.1)
        if abs(turn_angle) < 0.05:
            self.state = State.Approaching
        return cmd_vel
    
    def approach(self):
        cmd_vel = Twist()
        left_gate_location = self.current_gate_pos.red_x
        right_gate_location = self.current_gate_pos.green_x

        if left_gate_location is None or right_gate_location is None:
            return
        
        deg_per_pixel = self.hfov / self.image_width

        mid_x = (right_gate_location + left_gate_location) / 2
        mid_x_img = self.image_width // 2
        diff_mid = mid_x_img - mid_x
        turn_angle = diff_mid * deg_per_pixel

        cmd_vel.angular.z = self.angular_correction_factor * turn_angle
        if abs(cmd_vel.angular.z) > 0.1:
            cmd_vel.angular.z = np.sign(cmd_vel.angular.z) * 0.1
        cmd_vel.linear.x = 0.1
        gate_fov_bound = self.calculate_gate_fov_bound(left_gate_location, right_gate_location)
        
        self.get_logger().info(f"gate_fov_bound: {gate_fov_bound}")

        # to be between gate is to be done with the job

        if gate_fov_bound > 0.8:
            self.pre_push_time = self.get_clock().now()
            self.state = State.Passing_Through

        return cmd_vel
    
    def pass_through(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.1
        if (self.get_clock().now() - self.pre_push_time).nanoseconds // 1e9 > 20:
            self.state = State.PassedThrough
        self.get_logger().info(f"{(self.get_clock().now() - self.pre_push_time).nanoseconds // 1e9}")
        return cmd_vel
    
    def complete(self):
        self.get_logger().info("Gate successfully passed!")
        self.destroy_node()

rclpy.init()
state_machine = StateMachine()
rclpy.spin(state_machine)
state_machine.destroy_node()
rclpy.shutdown()