import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import math

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        # Subscribers
        self.create_subscription(Odometry, '/odometry/filtered', self.state_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Publishers for motor commands
        self.port_motor_publisher = self.create_publisher(Float64, '/thrusters/left/thrust', 10)
        self.stbd_motor_publisher = self.create_publisher(Float64, '/thrusters/right/thrust', 10)

        # TODO; Tune PID Controller gains
        self.kp_linear = 1.0
        self.kp_angular = 2.5

        # State variables
        self.current_position = (0.0, 0.0)
        self.current_orientation = 0.0  # Yaw in radians
        self.obstacle_detected = False

        self.get_logger().info("Navigation node initialized.")

    def state_callback(self, msg: Odometry):
        """Extract current position and orientation from Odometry message."""
        self.current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        orientation_q = msg.pose.pose.orientation
        _, _, self.current_orientation = self.euler_from_quaternion(
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        )

    def lidar_callback(self, msg: LaserScan):
        """Detect obstacles within a certain distance threshold."""
        min_distance = min(msg.ranges)
        self.obstacle_detected = min_distance < 0.8

    def cmd_vel_callback(self, msg: Twist):
        """Process velocity commands and convert to motor thrust."""
        linear_thrust = self.kp_linear * msg.linear.x
        angular_thrust = self.kp_angular * msg.angular.z

        port_thrust = max(min(linear_thrust - angular_thrust, 100.0), -100.0)
        stbd_thrust = max(min(linear_thrust + angular_thrust, 100.0), -100.0)

        self.get_logger().info(f"port thrust: {port_thrust} || stbd thrust: {stbd_thrust}")
        self.send_motor_commands(port_thrust, stbd_thrust)

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
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
