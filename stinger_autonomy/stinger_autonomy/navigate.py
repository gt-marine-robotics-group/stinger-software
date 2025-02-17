'''
This node sends thrust commands to the motors, avoid obstacles, and navigate to the goal position via waypointing

See the stinger manual for a complete guide of task details.

Search for TODO to complete this node.
'''

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from std_msgs.msg import Float64, Bool
import math

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')

        # Subscribers
        self.create_subscription(Odometry, '/localization/odometry', self.state_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.create_subscription(Point, '/gate_waypoint', self.gate_callback, 10)
        self.create_subscription(Bool, '/goal_found', self.goal_callback, 10)  # Vision node signals goal detection

        # Publishers for motor commands
        self.port_motor_publisher = self.create_publisher(Float64, '/thrusters/left/thrust', 10)
        self.stbd_motor_publisher = self.create_publisher(Float64, '/thrusters/right/thrust', 10)

        # TODO PID Controller gains
        self.kp_linear = None
        self.kp_angular = None

        # State variables
        self.current_position = (0.0, 0.0)
        self.current_orientation = 0.0  # Yaw in radians
        self.gate_position = None  # Updated using camera
        self.obstacle_detected = False
        self.goal_reached = False  # Updated using camera

        self.get_logger().info("Navigation node initialized.")

    def state_callback(self, msg: Odometry):
        """Extract current position and orientation from Odometry message."""
        self.current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        orientation_q = msg.pose.pose.orientation
        _, _, self.current_orientation = self.euler_from_quaternion(
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        )

        if not self.goal_reached:
            self.navigate()

    # TODO change the thershold if needed
    def lidar_callback(self, msg: LaserScan):
        """Detect obstacles within a certain distance threshold."""
        min_distance = min(msg.ranges)
        self.obstacle_detected = min_distance < 1.5

    def gate_callback(self, msg: Point):
        """Update gate position dynamically from vision node."""
        self.gate_position = (msg.x, msg.y)
        self.get_logger().info(f"Updated gate position: {self.gate_position}")

    def goal_callback(self, msg: Bool):
        """Stop the tug when the vision node detects the goal (yellow buoy)."""
        if msg.data:  # only triggers when msg is True
            self.goal_reached = True
            self.stop_tug()
            self.get_logger().info("Goal reached! Stopping navigation.")

    def navigate(self):
        """Navigate toward the detected gate midpoint while avoiding obstacles."""
        if self.gate_position is None:
            self.get_logger().info("Waiting for gate position...")
            return

        target_x, target_y = self.gate_position
        current_x, current_y = self.current_position

        # TODO Compute errors
        distance_error = None
        target_angle = None

        # TODO Normalize angular error to [-pi, pi]
        angular_error = None

        if self.obstacle_detected:
            self.avoid_obstacle()
            return

        # TODO Compute control signals
        linear_thrust = None
        angular_thrust = None

        # Apply thrust based on under-actuated dynamics
        port_thrust = max(min(linear_thrust - angular_thrust, 100.0), -100.0)
        stbd_thrust = max(min(linear_thrust + angular_thrust, 100.0), -100.0)

        self.send_motor_commands(port_thrust, stbd_thrust)

    def avoid_obstacle(self):
        """Rotate starboard to avoid obstacle dynamically."""
        self.get_logger().info("Obstacle detected, taking avoidance action...")
        self.send_motor_commands(-50.0, 50.0)  # Rotate in place

    def stop_tug(self):
        """Stop the tug when the goal is reached."""
        self.send_motor_commands(0.0, 0.0)

    def send_motor_commands(self, port_thrust, stbd_thrust):
        """Send motor commands to the thrusters."""
        self.port_motor_publisher.publish(Float64(data=port_thrust))
        self.stbd_motor_publisher.publish(Float64(data=stbd_thrust))

    @staticmethod
    def euler_from_quaternion(x, y, z, w):
        """Convert quaternion to Euler angles (roll, pitch, yaw)."""
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return 0.0, 0.0, yaw

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
