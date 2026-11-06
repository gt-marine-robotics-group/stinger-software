#!/usr/bin/env python3
"""
Robust WASD Keyboard Teleop Node for Stinger Tugboat.
Publishes directly to motor thrust topics:
  - /thrusters/left/thrust  (std_msgs/Float64)
  - /thrusters/right/thrust (std_msgs/Float64)
  - /stinger/thruster_port/cmd_thrust (std_msgs/Float64)
  - /stinger/thruster_stbd/cmd_thrust (std_msgs/Float64)
"""

import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

HELP_MSG = """
======================================================
  STINGER TUG - WASD KEYBOARD TELEOP
======================================================
  Controls:
    [W] / [Up Arrow]    : Increase Forward Speed
    [S] / [Down Arrow]  : Increase Reverse Speed / Slow Down
    [A] / [Left Arrow]  : Steer Left  (Differential)
    [D] / [Right Arrow] : Steer Right (Differential)
    [SPACE] / [X]       : Instant Stop (0% Thrust)

  Settings:
    [+] / [=]           : Increase Step Size
    [-] / [_]           : Decrease Step Size

  Exit:
    [Q] or [Ctrl+C]     : Stop Motors and Exit
======================================================
"""

class KeyboardReader:
    def __init__(self):
        self.is_tty = sys.stdin.isatty()
        if self.is_tty:
            self.settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())

    def get_key(self, timeout=0.05):
        if not self.is_tty:
            return None
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if not rlist:
            return None
        key = sys.stdin.read(1)
        # Handle escape sequence for arrow keys
        if key == '\x1b':
            extra = sys.stdin.read(2)
            if extra == '[A':
                return 'w' # Up arrow -> W
            elif extra == '[B':
                return 's' # Down arrow -> S
            elif extra == '[C':
                return 'd' # Right arrow -> D
            elif extra == '[D':
                return 'a' # Left arrow -> A
            return None
        return key

    def restore(self):
        if self.is_tty:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

class TeleopKeyboardNode(Node):
    def __init__(self):
        super().__init__('stinger_teleop_keyboard')

        # Publishers for both topic naming conventions
        self.pub_left = self.create_publisher(Float64, '/thrusters/left/thrust', 10)
        self.pub_right = self.create_publisher(Float64, '/thrusters/right/thrust', 10)
        self.pub_port_stinger = self.create_publisher(Float64, '/stinger/thruster_port/cmd_thrust', 10)
        self.pub_stbd_stinger = self.create_publisher(Float64, '/stinger/thruster_stbd/cmd_thrust', 10)

        # State
        self.linear_speed = 0.0      # Forward / Reverse [-100, 100]
        self.angular_speed = 0.0     # Turn bias [-50, 50]
        self.step_size = 5.0         # Increment per press (%)
        self.max_thrust = 80.0       # Max limit (%)

        self.get_logger().info("Teleop Keyboard Node initialized.")

    def update_and_publish(self):
        # Calculate differential thrust
        # Turning left: left motor slower/reverse, right motor faster
        # Turning right: right motor slower/reverse, left motor faster
        left_thrust = self.linear_speed - self.angular_speed
        right_thrust = self.linear_speed + self.angular_speed

        # Clamp within safety limits
        left_thrust = max(-self.max_thrust, min(self.max_thrust, left_thrust))
        right_thrust = max(-self.max_thrust, min(self.max_thrust, right_thrust))

        msg_l = Float64(data=float(left_thrust))
        msg_r = Float64(data=float(right_thrust))

        self.pub_left.publish(msg_l)
        self.pub_right.publish(msg_r)
        self.pub_port_stinger.publish(msg_l)
        self.pub_stbd_stinger.publish(msg_r)

        # Formatted status line
        status = f"\r[THROTTLE] Fwd/Rev: {self.linear_speed:+5.1f}% | Turn: {self.angular_speed:+5.1f}% |-> Left: {left_thrust:+5.1f}% | Right: {right_thrust:+5.1f}% | Step: {self.step_size:.1f}%    "
        sys.stdout.write(status)
        sys.stdout.flush()

    def stop_motors(self):
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        msg_zero = Float64(data=0.0)
        self.pub_left.publish(msg_zero)
        self.pub_right.publish(msg_zero)
        self.pub_port_stinger.publish(msg_zero)
        self.pub_stbd_stinger.publish(msg_zero)
        sys.stdout.write("\r[STATUS] MOTORS STOPPED (0% Thrust)                                                 \n")
        sys.stdout.flush()

def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboardNode()
    kbd = KeyboardReader()

    print(HELP_MSG)
    node.update_and_publish()

    try:
        while rclpy.ok():
            key = kbd.get_key(timeout=0.05)
            if key is not None:
                k = key.lower()
                if k == 'w':
                    node.linear_speed = min(node.max_thrust, node.linear_speed + node.step_size)
                    node.update_and_publish()
                elif k == 's':
                    node.linear_speed = max(-node.max_thrust, node.linear_speed - node.step_size)
                    node.update_and_publish()
                elif k == 'a':
                    node.angular_speed = max(-50.0, node.angular_speed - node.step_size)
                    node.update_and_publish()
                elif k == 'd':
                    node.angular_speed = min(50.0, node.angular_speed + node.step_size)
                    node.update_and_publish()
                elif k in (' ', 'x'):
                    node.stop_motors()
                elif k in ('+', '='):
                    node.step_size = min(20.0, node.step_size + 2.5)
                    node.update_and_publish()
                elif k in ('-', '_'):
                    node.step_size = max(1.0, node.step_size - 2.5)
                    node.update_and_publish()
                elif k == 'q' or key == '\x03': # Q or Ctrl+C
                    break

            rclpy.spin_once(node, timeout_sec=0.01)

    except (KeyboardInterrupt, Exception):
        pass
    finally:
        node.stop_motors()
        kbd.restore()
        node.destroy_node()
        rclpy.shutdown()
        print("\nTeleop stopped cleanly.")

if __name__ == '__main__':
    main()
