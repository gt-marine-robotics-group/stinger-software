import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class LidarFilterNode(Node):
    def __init__(self):
        super().__init__('lidar_filter_node')

        # Subscribe to original LiDAR scan
        self.scan_subscriber = self.create_subscription(
            LaserScan, '/stinger/laser/scan', self.lidar_callback, 10)

        # Publisher for filtered scan
        self.scan_publisher = self.create_publisher(
            LaserScan, '/stinger/laser/scan_filtered', 10)

    def lidar_callback(self, msg: LaserScan):
        """Filter out LiDAR points in the 120° to 240° range and republish."""
        angle_min = msg.angle_min  # Start angle of scan
        angle_max = msg.angle_max  # End angle of scan
        angle_increment = msg.angle_increment  # Angle between each measurement

        num_points = len(msg.ranges)
        angles = np.linspace(angle_min, angle_max, num_points)

        # Define the exclusion range in radians
        exclude_min = -np.pi + 2 * np.pi / 3  # 120°
        exclude_max = -np.pi + 4 * np.pi / 3  # 240°

        # Create filtered ranges, setting unwanted angles to NaN
        filtered_ranges = [
            r if not (exclude_min <= angle <= exclude_max) else float('nan')
            for r, angle in zip(msg.ranges, angles)
        ]

        # Create new LaserScan message
        filtered_scan = LaserScan()
        filtered_scan.header = msg.header
        filtered_scan.angle_min = msg.angle_min
        filtered_scan.angle_max = msg.angle_max
        filtered_scan.angle_increment = msg.angle_increment
        filtered_scan.time_increment = msg.time_increment
        filtered_scan.scan_time = msg.scan_time
        filtered_scan.range_min = msg.range_min
        filtered_scan.range_max = msg.range_max
        filtered_scan.ranges = filtered_ranges
        filtered_scan.intensities = msg.intensities  # Keep intensity values unchanged

        # Publish the filtered scan
        self.scan_publisher.publish(filtered_scan)

def main(args=None):
    rclpy.init(args=args)
    node = LidarFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
