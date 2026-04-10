"""
This node subscribes to the motors topics, and controls the motors through ESC

Quote from a blue robotics discussion forum.
You will need to update your duty cycle settings accordingly to make sure 
your pulse widths are between 1100 and 1900 milliseconds.

TODO; adjust duty cycle as you go
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import pigpio
import time

# GPIO pin setup for Raspberry Pi 4
PORT_ESC_PIN = 13  # GPIO13 for port motor
STARBOARD_ESC_PIN = 12  # GPIO12 for starboard motor
PWM_FREQUENCY = 50  # Standard servo ESC frequency (50 Hz)

# --- UPDATED BIDIRECTIONAL LIMITS ---
MIN_PULSE_WIDTH = 1148      # Max Reverse (from BLHeliSuite)
MAX_PULSE_WIDTH = 1832      # Max Forward (from BLHeliSuite)
NEUTRAL_PULSE_WIDTH = 1490  # Exactly between 1148 and 1832
INITIAL_PULSE_WIDTH = 1490  # MUST be Neutral to arm in Bidirectional mode

class ESCControlNode(Node):
    def __init__(self):
        super().__init__('esc_control_node')
        
        # Initialize pigpio
        self.pi = pigpio.pi()
        if not self.pi.connected:
            self.get_logger().error("Failed to connect to pigpio daemon.")
            exit(1)
        
        # Set GPIO pins as output
        self.pi.set_mode(PORT_ESC_PIN, pigpio.OUTPUT)
        self.pi.set_mode(STARBOARD_ESC_PIN, pigpio.OUTPUT)
        
        # Initialize ESCs with the Neutral pulse to trigger arming
        self.pi.set_servo_pulsewidth(PORT_ESC_PIN, INITIAL_PULSE_WIDTH)
        self.pi.set_servo_pulsewidth(STARBOARD_ESC_PIN, INITIAL_PULSE_WIDTH)
        
        # MUST be 3 seconds to allow the arming melody to finish completely
        time.sleep(3.0)  

        # ROS2 subscriptions
        self.port_subscription = self.create_subscription(
            Float64, '/thrusters/left/thrust', self.port_callback, 10)
        self.stbd_subscription = self.create_subscription(
            Float64, '/thrusters/right/thrust', self.stbd_callback, 10)

        self.get_logger().info('ESC control node started. Listening for bidirectional commands.')

    def port_callback(self, msg):
        self.set_pwm_thrust(PORT_ESC_PIN, msg.data)
    
    def stbd_callback(self, msg):
        self.set_pwm_thrust(STARBOARD_ESC_PIN, msg.data)
    
    def set_pwm_thrust(self, pin, thrust_value):
        # Ensure thrust value is within bounds [-100, 100] for forward/reverse
        thrust_value = max(-100.0, min(100.0, thrust_value))
        
        # Convert thrust value to pulse width
        pulse_width = self.map_thrust_to_pulse(thrust_value)
        
        # Set PWM signal
        self.pi.set_servo_pulsewidth(pin, pulse_width)
        self.get_logger().info(f'Set PWM Pulse Width: {pulse_width:.0f}us for thrust: {thrust_value:.2f}')
    
    def map_thrust_to_pulse(self, thrust):
        # Math correctly scales around the 1490 center point
        bidirectional = (thrust / 100.0) * (MAX_PULSE_WIDTH - NEUTRAL_PULSE_WIDTH) + NEUTRAL_PULSE_WIDTH
        return bidirectional
    
    def destroy_node(self):
        # Stop ESC signals by sending the Neutral command, not 0
        self.pi.set_servo_pulsewidth(PORT_ESC_PIN, NEUTRAL_PULSE_WIDTH)
        self.pi.set_servo_pulsewidth(STARBOARD_ESC_PIN, NEUTRAL_PULSE_WIDTH)
        
        # Cleanup pigpio
        self.pi.stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    esc_control_node = ESCControlNode()

    try:
        rclpy.spin(esc_control_node)
    except KeyboardInterrupt:
        esc_control_node.get_logger().info('ESC control node shutting down.')
    finally:
        esc_control_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()