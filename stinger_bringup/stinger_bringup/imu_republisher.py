import rclpy
from rclpy.node import Node
from rclpy.clock import Duration
from sensor_msgs.msg import Imu
import tf_transformations
import numpy as np
from tf2_ros import TransformListener, Buffer


class ImuRepublisher(Node):
    def __init__(self):
        super().__init__('imu_republisher')
        self.create_subscription(
            Imu,
            '/stinger/imu/data',
            self.imu_callback,
            10
        )
        self.imu_pub = self.create_publisher(
            Imu,
            '/stinger/imu/relative',
            10
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
    
    def rotate_vector(self, rot_matrix, v):
            vec = np.array([v.x, v.y, v.z])
            return rot_matrix @ vec

    def transfrom_imu(self, msg: Imu):
        transform = None
        # Extract transform from imu to base link from tf tree
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame='base_link',
                source_frame=msg.header.frame_id,
                time=rclpy.time.Time(),
                timeout=Duration(seconds=1.0)
            )
        except Exception as e:
            self.get_logger().warn(
                f"TF lookup 'base_link' <- '{msg.header.frame_id}' failed: "
                f"{type(e).__name__}: {e}",
                throttle_duration_sec=2.0
            )
            return None

        # Extract transform rotation
        q = transform.transform.rotation
        q_tf = [q.x, q.y, q.z, q.w]
        rot_matrix = tf_transformations.quaternion_matrix(q_tf)[:3, :3]

        # Transform orientation into base_link frame
        imu_q = msg.orientation
        imu_q_np = [imu_q.x, imu_q.y, imu_q.z, imu_q.w]
        transformed_orientation = tf_transformations.quaternion_multiply(
            tf_transformations.quaternion_multiply(q_tf, imu_q_np),
            tf_transformations.quaternion_conjugate(q_tf)
        )

        # Transform angular velocity and linear acceleration into base_link frame
        ang_vel = self.rotate_vector(rot_matrix, msg.angular_velocity)
        lin_acc = self.rotate_vector(rot_matrix, msg.linear_acceleration)

        transformed_msg = Imu()
        transformed_msg.header.stamp = msg.header.stamp
        # Notice that the new frame is now in our base_link frame as desired
        transformed_msg.header.frame_id = 'base_link'
        transformed_msg.orientation.x = transformed_orientation[0]
        transformed_msg.orientation.y = transformed_orientation[1]
        transformed_msg.orientation.z = transformed_orientation[2]
        transformed_msg.orientation.w = transformed_orientation[3]
        transformed_msg.angular_velocity.x = ang_vel[0]
        transformed_msg.angular_velocity.y = ang_vel[1]
        transformed_msg.angular_velocity.z = ang_vel[2]
        transformed_msg.linear_acceleration.x = lin_acc[0]
        transformed_msg.linear_acceleration.y = lin_acc[1]
        transformed_msg.linear_acceleration.z = lin_acc[2]
        transformed_msg.orientation_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        transformed_msg.angular_velocity_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        transformed_msg.linear_acceleration_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]

        return transformed_msg

    def imu_callback(self, msg: Imu):
        msg_base_link: Imu = self.transfrom_imu(msg)
        if msg_base_link is None:
            self.get_logger().warn("Failed to get transform, skipping")
            return
        q = msg_base_link.orientation
        quat = [q.x, q.y, q.z, q.w]
        r, p, y = tf_transformations.euler_from_quaternion(quat)
        quat_relative = tf_transformations.quaternion_from_euler(r, p, 0.0)
        rotation_matrix = tf_transformations.quaternion_matrix(quat_relative)[:3, :3]
        acceleration_world = self.rotate_vector(rotation_matrix, msg_base_link.linear_acceleration)
        angular_velocity = self.rotate_vector(rotation_matrix, msg_base_link.angular_velocity)
        msg_base_link.linear_acceleration.x = acceleration_world[0]
        msg_base_link.linear_acceleration.y = acceleration_world[1]
        msg_base_link.linear_acceleration.z = acceleration_world[2]

        msg_base_link.angular_velocity.x = angular_velocity[0]
        msg_base_link.angular_velocity.y = angular_velocity[1]
        msg_base_link.angular_velocity.z = angular_velocity[2]
        self.imu_pub.publish(msg_base_link)

def main(args=None):
    rclpy.init(args=args)
    node = ImuRepublisher()
    rclpy.spin(node)
