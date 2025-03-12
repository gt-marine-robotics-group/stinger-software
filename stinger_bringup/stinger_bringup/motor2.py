import pigpio
import time

SERVO_PIN = 13 # Change to your servo GPIO pin

pi = pigpio.pi()  # Connect to pigpio daemon
if not pi.connected:
    exit()

pi.set_servo_pulsewidth(SERVO_PIN, 1000)
time.sleep(3)

pi.set_servo_pulsewidth(SERVO_PIN, 1500)
time.sleep(3)

# # Move servo to left (-90°)
# pi.set_servo_pulsewidth(SERVO_PIN, 500)
# time.sleep(1)

# # Move servo to center (0°)
# pi.set_servo_pulsewidth(SERVO_PIN, 1500)
# time.sleep(1)

# # Move servo to right (+90°)
# pi.set_servo_pulsewidth(SERVO_PIN, 2500)
# time.sleep(1)

# Stop the servo
pi.set_servo_pulsewidth(SERVO_PIN, 0)  # Disables PWM signal
pi.stop()