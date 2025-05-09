'''
Adafruit TDK InvenSense ICM-20948 9-DoF IMU (4554)
This node is publish topic /imu/data

1. linear acceleration [m/s^2]
2. angular velocity [rad/s]
3. compass

Note the imu data is publishing in the 'imu_link'. Keep that in mind with writing urdf.

Resources: https://learn.adafruit.com/adafruit-tdk-invensense-icm-20948-9-dof-imu/python-circuitpython
'''

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import board
import adafruit_icm20x
import numpy as np
import math
# from ahrs.filters import Madgwick
# from transforms3d.euler import euler2quat
# from geometry_msgs.msg import Quaternion

class IMUPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        
        # Initialize IMU sensor over I2C
        i2c = board.I2C()
        self.imu_sensor = adafruit_icm20x.ICM20948(i2c)

        # madgwick filter for sensor fusion 
        self.dt = 0.1  # 10 Hz update rate
        self.alpha = 0.98  # Filter coefficient
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.q = np.array([1.0, 0.0, 0.0, 0.0])  # Initial quaternion


        self.ACCEL_NOISE = 0.02  # m/s² standard deviation
        self.GYRO_NOISE = 0.01  # rad/s standard deviation
        self.ORIENT_NOISE = 0.05  # quaternion std dev
        
        # publish IMU data
        self.imu_pub = self.create_publisher(Imu, '/stinger/imu/data', 10)
        self.timer = self.create_timer(0.1, self.publish_imu_data)  # Publish at 10 Hz

    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion."""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0] * 4
        q[0] = cr * cp * cy + sr * sp * sy  # w
        q[1] = sr * cp * cy - cr * sp * sy  # x
        q[2] = cr * sp * cy + sr * cp * sy  # y
        q[3] = cr * cp * sy - sr * sp * cy  # z

        return q
    
    def publish_imu_data(self):
        msg = Imu()
        msg.header.frame_id = "imu_link"
        msg.header.stamp = self.get_clock().now().to_msg()

        # Read IMU sensor data
        try:
            accel_x, accel_y, accel_z = self.imu_sensor.acceleration
            gyro_x, gyro_y, gyro_z = self.imu_sensor.gyro
            mag_x, mag_y, mag_z = self.imu_sensor.magnetic
            
            # Fill message fields
            msg.linear_acceleration.x = accel_x
            msg.linear_acceleration.y = accel_y
            msg.linear_acceleration.z = accel_z

            msg.angular_velocity.x = gyro_x
            msg.angular_velocity.y = gyro_y
            msg.angular_velocity.z = gyro_z

            roll_acc = math.atan2(accel_y, accel_z)
            pitch_acc = math.atan2(-accel_x, math.sqrt(accel_y**2 + accel_z**2))
            yaw_mag = math.atan2(mag_y, mag_x)

            self.roll = self.alpha * (self.roll + gyro_x * self.dt) + (1 - self.alpha) * roll_acc
            self.pitch = self.alpha * (self.pitch + gyro_y * self.dt) + (1 - self.alpha) * pitch_acc
            self.yaw = self.alpha * (self.yaw + gyro_z * self.dt) + (1 - self.alpha) * yaw_mag

            q = self.euler_to_quaternion(self.roll, self.pitch, self.yaw)
    
            msg.orientation.w = q[0]
            msg.orientation.x = q[1]
            msg.orientation.y = q[2]
            msg.orientation.z = q[3]

            msg.linear_acceleration_covariance = [self.ACCEL_NOISE**2 if i % 4 == 0 else 0.0 for i in range(9)]
            msg.angular_velocity_covariance = [self.GYRO_NOISE**2 if i % 4 == 0 else 0.0 for i in range(9)]
            msg.orientation_covariance = [self.ORIENT_NOISE**2 if i % 4 == 0 else 0.0 for i in range(9)]

            self.imu_pub.publish(msg)
            # self.get_logger().info(f"Accel: ({accel_x:.2f}, {accel_y:.2f}, {accel_z:.2f}) "
            #                         f"Gyro: ({gyro_x:.2f}, {gyro_y:.2f}, {gyro_z:.2f}) "
            #                         f"Mag: ({mag_x:.2f}, {mag_y:.2f}, {mag_z:.2f})")
        
        except Exception as e:
            self.get_logger().error(f"Error reading IMU data: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    imu_publisher = IMUPublisher()
    rclpy.spin(imu_publisher)
    imu_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
