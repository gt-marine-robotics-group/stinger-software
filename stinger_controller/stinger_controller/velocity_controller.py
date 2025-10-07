import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class VelocityController(Node):
    def __init__(self):
        super().__init__('velocity_controller')
        
        # TODO: 5.1.a Velocity Controller Setup
        ### STUDENT CODE HERE

        ### END STUDENT CODE

        # Mock Values, Tune These
        self.Kp_surge = 1
        self.Ki_surge = 0
        self.Kd_surge = 0

        self.Kp_yaw = 1
        self.Ki_yaw = 0
        self.Kd_yaw = 0
        
        # TODO: 5.1.g Controller Tuning
        # You should be overriding the above values!
        ### STUDENT CODE HERE

        ### END STUDENT CODE

        self.cmd_vel = Twist()
        self.prev_time = None
        self.prev_error_surge = 0
        self.prev_error_yaw = 0
        self.I_surge_total = 0
        self.I_yaw_total = 0

    def cmd_vel_callback(self, msg: Twist):
        self.cmd_vel = msg

    def odometry_callback(self, msg: Odometry):
        if not self.prev_time:
            self.prev_time = self.get_clock().now()
            return

        output_force = WrenchStamped()
        output_force.header = msg.header
        dt = (self.get_clock().now() - self.prev_time).nanoseconds / 1e9
        self.prev_time = self.get_clock().now()

        error_surge = 0
        # TODO: 5.1.b Error Calculation
        ### STUDENT CODE HERE

        ### END STUDENT CODE

        P_surge = 0
        # TODO: 5.1.c Proportional Calculation
        ### STUDENT CODE HERE

        ### END STUDENT CODE
        
        I_surge = 0
        # TODO: 5.1.d Integral Calculation
        ### STUDENT CODE HERE

        ### END STUDENT CODE

        D_surge = 0
        # TODO: 5.1.e Derivative Calculation
        ### STUDENT CODE HERE

        ### END STUDENT CODE

        control_surge = P_surge + I_surge + D_surge
        output_force.wrench.force.x = control_surge

        # TODO: 5.1.f Yaw Control
        ### STUDENT CODE HERE

        ### END STUDENT CODE

        self.wrench_pub.publish(output_force)

        # Ignore this line, only for autograder purposes
        return error_surge, P_surge, I_surge, D_surge, control_yaw, output_force


def main(args=None):
    rclpy.init(args=args)
    node = VelocityController()
    rclpy.spin(node)
