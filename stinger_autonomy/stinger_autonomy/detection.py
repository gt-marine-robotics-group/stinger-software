'''
Camera specs: 
width: 1280
height: 720
30fps

ABS
Length Approx.6cm 2.36in
Pixel 1million pixels
Photosensitive chip: OV9726(1/6.5“)
Field of view: 63°No Distortion
Output:USB2.0
'''
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
from enum import Enum
from sensor_msgs.msg import Imu
import tf_transformations

class State(Enum):
    Searching = 0
    Approaching = 1
    Push = 2
    Orientation_Correction = 3
    Passing_Through = 4
    PassedThrough = 5

class GateTask(Node):
    def __init__(self):
        super().__init__("Gate_Task")

        self.image_width = 1280
        self.hfov = 1.09956
        self.angular_correction_factor = 1.0

        self.previous_state = None
        self.state = State.Searching

        self.timer = self.create_timer(0.1, self.state_machine_callback)
        
        self.pre_push_time = None # time difference from last push and now calls correct orientation state
        self.gate_offset = 0.0 # angle offset
        self.starting_quat = None

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.imu_result = Imu()
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()
        self.hsv = np.array([])
        self.get_logger().info("Vision node initialized! Let there be light. You can see now.")

    def image_callback(self, msg):
        """Process the camera feed to detect red, green, and yellow buoys."""
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    def imu_callback(self, msg):
        self.imu_result = msg
    
    def find_circles(self, mask):
        """Find circular contours in a binary mask."""
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detected = []

        for cnt in contours:
            (x, y), radius = cv2.minEnclosingCircle(cnt)
            if 40 < radius:  # Filter small objects (radius greater than 40 pixels rn, tune as you go)
                detected.append((int(x), int(y), int(radius)))

        detected_sorted = sorted(detected, key=lambda x: x[2], reverse=True)
        return detected_sorted

    def gate_detection_cv(self):
        # Define color ranges for red, green, and yellow in HSV
        red_lower = np.array([0, 120, 70])
        red_upper = np.array([10, 255, 255])
        green_lower = np.array([60, 80, 80])
        green_upper = np.array([80, 255, 255])

        if len(self.hsv) == 0:
            self.get_logger().info("Waitng for frame")
            return [], []

        # Create masks
        red_mask = cv2.inRange(self.hsv, red_lower, red_upper)
        green_mask = cv2.inRange(self.hsv, green_lower, green_upper)

        # Find contours for each color
        red_buoy = self.find_circles(red_mask)
        green_buoy = self.find_circles(green_mask)

        return red_buoy, green_buoy
    
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
            case State.Push:
                cmd_vel = self.push()
            case State.Orientation_Correction:
                cmd_vel = self.correct_orientation()
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
        left_gate_location, right_gate_location = self.gate_detection_cv()

        self.get_logger().info(f"{left_gate_location}")
        self.get_logger().info(f"{right_gate_location}")

        if len(left_gate_location)==0 or len(right_gate_location)==0:
            return
        
        deg_per_pixel = self.hfov / self.image_width

        left_gate_x = left_gate_location[0][0]
        right_gate_x = right_gate_location[0][0]

        mid_x = (right_gate_x + left_gate_x) / 2
        mid_x_img = self.image_width // 2
        diff_mid = mid_x_img - mid_x

        # If we are far too left, want to turn right, this will be a negative value
        # If we are far too right, want to turn left, this will be a positive value
        turn_angle = diff_mid * deg_per_pixel

        self.get_logger().info(f"{turn_angle}")

        if abs(cmd_vel.angular.z) > 0.1:
            cmd_vel.angular.z = np.sign(cmd_vel.angular.z) * 0.1

        if abs(turn_angle) < 0.1:
            self.state = State.Approaching
        return cmd_vel
    
    def approach(self):
        cmd_vel = Twist()
        left_gate_location, right_gate_location = self.gate_detection_cv()

        if len(left_gate_location)==0 or len(right_gate_location)==0:
            return
        
        deg_per_pixel = self.hfov / self.image_width
        left_gate_x = left_gate_location[0][0]
        right_gate_x = right_gate_location[0][0]

        mid_x = (right_gate_x + left_gate_x) / 2
        mid_x_img = self.image_width // 2
        diff_mid = mid_x_img - mid_x
        turn_angle = diff_mid * deg_per_pixel

        #TODO, THIS MIGHT BE CURSED
        self.gate_offset = turn_angle

        cmd_vel.angular.z = self.angular_correction_factor * turn_angle
        if abs(cmd_vel.angular.z) > 0.1:
            cmd_vel.angular.z = np.sign(cmd_vel.angular.z) * 0.1
        cmd_vel.linear.x = 0.1
        gate_fov_bound = self.calculate_gate_fov_bound(left_gate_x, right_gate_x)
        
        self.get_logger().info(f"gate_fov_bound: {gate_fov_bound}")

        # to be between gate is to be done with the job
        if gate_fov_bound > 0.95:
            self.pre_push_time = self.get_clock().now()
            self.state = State.Passing_Through

        # The idea here, is that the closer we get to the gates, they will move closer towards the bounds of our FOV
        # if gate_fov_bound > 0.7:
        #     self.state = State.Push
        #     self.pre_push_time = self.get_clock().now()

        return cmd_vel
    
    # def push(self):
    #     # First a little push to get closer to the center of the gate
    #     cmd_vel = Twist()
    #     cmd_vel.linear.x = 0.5
    #     if (self.get_clock().now() - self.pre_push_time).nanoseconds // 1e9 > 2:
    #         self.state = State.Orientation_Correction
    #     self.get_logger().info(f"{(self.get_clock().now() - self.pre_push_time).nanoseconds // 1e9}")
    #     return cmd_vel
    
    # def correct_orientation(self):
    #     # First give a little push
    #     cmd_vel = Twist()
    #     quat = self.imu_result.orientation
    #     current_quat = [quat.x, quat.y, quat.z, quat.w]
    #     if self.starting_quat is None:
    #         self.starting_quat = current_quat
    #     quat_error = tf_transformations.quaternion_multiply(
    #         tf_transformations.quaternion_inverse(self.starting_quat),
    #         current_quat
    #     )
    #     _, _, yaw_change = tf_transformations.euler_from_quaternion(quat_error)
    #     desired_change = -self.gate_offset
    #     self.get_logger().info(f"{yaw_change} {desired_change}")
    #     cmd_vel.angular.z = (desired_change - yaw_change) * self.angular_correction_factor
    #     if abs(yaw_change - desired_change) < 0.17:
    #         self.state = State.Passing_Through
    #         self.pre_push_time = self.get_clock().now()
    #     return cmd_vel
    
    def pass_through(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.1
        if (self.get_clock().now() - self.pre_push_time).nanoseconds // 1e9 > 2:
            self.state = State.PassedThrough
        self.get_logger().info(f"{(self.get_clock().now() - self.pre_push_time).nanoseconds // 1e9}")
        return cmd_vel
    
    def complete(self):
        self.get_logger().info("Gate successfully passed!")
        self.destroy_node()

rclpy.init()
gate_task = GateTask()
rclpy.spin(gate_task)
gate_task.destroy_node()
rclpy.shutdown()