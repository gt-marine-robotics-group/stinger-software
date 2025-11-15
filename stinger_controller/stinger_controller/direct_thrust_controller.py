import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class DirectThrustController(Node):
    def __init__(self):
        super().__init__('direct_thrust_controller')
        
        self.wrench_pub = self.create_publisher(
            WrenchStamped,
            '/cmd_wrench',
            10
        )

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # TUNE THESE
        self.surge_gain = 0.3
        self.yaw_gain = 0.1

    def cmd_vel_callback(self, msg: Twist):
        wrench = WrenchStamped()
        wrench.header.stamp = self.get_clock().now().to_msg()
        wrench.wrench.force.x = self.surge_gain * msg.linear.x
        wrench.wrench.torque.z = self.yaw_gain * msg.angular.z
        self.wrench_pub.publish(wrench)


def main(args=None):
    rclpy.init(args=args)
    node = DirectThrustController()
    rclpy.spin(node)
