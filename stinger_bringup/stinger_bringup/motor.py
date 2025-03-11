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
import RPi.GPIO as GPIO
import time

# GPIO pin setup for Raspberry Pi 4
PORT_ESC_PIN = 13  # GPIO13 for port motor (PWM1)
STARBOARD_ESC_PIN = 12  # GPIO12 for starboard motor (PWM0)
PWM_FREQUENCY = 50  # servo ESC control (50 Hz)

class ESCControlNode(Node):
    def __init__(self):
        super().__init__('esc_control_node')
        
        # GPIO setup
        GPIO.setmode(GPIO.BCM) # for GPIO numbering, choose BCM
        GPIO.setup(PORT_ESC_PIN, GPIO.OUT)
        GPIO.setup(STARBOARD_ESC_PIN, GPIO.OUT)

        # Initialize PWM on both pins
        self.port_pwm = GPIO.PWM(PORT_ESC_PIN, PWM_FREQUENCY)
        self.stbd_pwm = GPIO.PWM(STARBOARD_ESC_PIN, PWM_FREQUENCY)

        # Start PWM with zero throttle
        self.port_pwm.start(0)
        self.stbd_pwm.start(0)

        # ROS2 subscriptions
        self.port_subscription = self.create_subscription(
            Float64, '/thrusters/left/thrust', self.port_callback, 10)
        self.stbd_subscription = self.create_subscription(
            Float64, '/thrusters/right/thrust', self.stbd_callback, 10)

        self.get_logger().info('ESC control node started. Listening for thrust commands.')

    def port_callback(self, msg):
        self.set_pwm_thrust(self.port_pwm, msg.data)

    def stbd_callback(self, msg):
        self.set_pwm_thrust(self.stbd_pwm, msg.data)

    def set_pwm_thrust(self, pwm_channel, thrust_value):
        # Ensure the thrust value is within bounds [0, 100]
        thrust_value = max(0.0, min(100.0, thrust_value))

        # Map thrust value (0-100) to appropriate duty cycle for ESC
        # Typically, 0% = 1ms pulse, 100% = 2ms pulse at 50Hz
        duty_cycle = self.map_thrust_to_duty_cycle(thrust_value)
        pwm_channel.ChangeDutyCycle(duty_cycle)
        self.get_logger().info(f'Set PWM Duty Cycle: {duty_cycle:.2f}% for thrust: {thrust_value:.2f}')

    def map_thrust_to_duty_cycle(self, thrust):
        # Map thrust [0, 100] to duty cycle [5, 10] (approx. 1ms-2ms pulses)
        # dc = pulse_width/period; T = 1/f 
        min_dc = 5
        max_dc = 10
        duty_cycle = min_dc + (thrust * (max_dc - min_dc) / 100)
        return duty_cycle

    def destroy_node(self):
        self.port_pwm.stop()
        self.stbd_pwm.stop()
        GPIO.cleanup()
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
        rclpy.shutdown()


if __name__ == '__main__':
    main()
