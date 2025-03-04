'''
This node takes in camera image and identifies the pair of gates and the goal.
'''

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        # Subscribers
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        # Publishers
        self.waypoint_pub = self.create_publisher(Point, '/gate_waypoint', 10)
        self.goal_pub = self.create_publisher(Bool, '/goal_found', 10)

        self.bridge = CvBridge()
        self.lidar_data = None  # Store latest LiDAR data
        self.get_logger().info("Vision node initialized.")

    def image_callback(self, msg):
        """Process the camera feed to detect red, green, and yellow buoys."""
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define color ranges for red, green, and yellow in HSV
        red_lower = np.array([0, 120, 70])
        red_upper = np.array([10, 255, 255])
        green_lower = np.array([60, 80, 80])
        green_upper = np.array([80, 255, 255])
        yellow_lower = np.array([20, 100, 100])
        yellow_upper = np.array([30, 255, 255])

        self.get_logger().info("img callback working.")

        # Create masks
        red_mask = cv2.inRange(hsv, red_lower, red_upper)
        green_mask = cv2.inRange(hsv, green_lower, green_upper)
        yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)
        cv2.imshow("Green mas", green_mask)
        cv2.imshow("Red mask", red_mask)
        key = cv2.waitKey(1)
        # Find contours for each color
        red_buoy = self.find_circles(red_mask)
        green_buoy = self.find_circles(green_mask)
        self.get_logger().info(f"Red detections {red_buoy}")
        self.get_logger().info(f"Green detections {green_buoy}")
        if red_buoy:
            self.get_logger().info("Red buoy detected.")
        if green_buoy:
            self.get_logger().info("Green buoy detected.")
        if cv2.countNonZero(yellow_mask) > 50:  # If significant yellow pixels are detected
            self.get_logger().info("Yellow buoy detected.")

        # Check if the tug is centered between the buoys and publish waypoint
        if red_buoy and green_buoy and self.lidar_data:
            self.publish_waypoint(red_buoy[0][0], green_buoy[0][0])

        # Check if the goal (yellow buoy) is in front
        if cv2.countNonZero(yellow_mask) > 50:  # If significant yellow pixels are detected
            self.get_logger().info("Goal Found!")
            goal_msg = Bool()
            goal_msg.data = True
            self.goal_pub.publish(goal_msg)

    def lidar_callback(self, msg):
        """Store the latest LiDAR scan data."""
        self.lidar_data = msg.ranges

    def find_circles(self, mask):
        """Find circular contours in a binary mask."""
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detected = []

        for cnt in contours:
            (x, y), radius = cv2.minEnclosingCircle(cnt)
            if 40 < radius:  # Filter small or too large objects
                detected.append((int(x), int(y)))

        return detected

    def publish_waypoint(self, red_x, green_x):
        """Publish the midpoint between the red and green buoys as a waypoint if distances are similar."""
        mid_x = (red_x + green_x) / 2

        # Use LiDAR to measure the distance to each buoy
        red_distance = self.estimate_distance(red_x)
        green_distance = self.estimate_distance(green_x)

        if red_distance > 0 and green_distance > 0:
            error_margin = 0.2  # Allow some tolerance in distance measurement
            if abs(red_distance - green_distance) <= error_margin:
                current_y = (red_distance + green_distance) / 2  # Approximate y-position
                self.get_logger().info(f"Tug is centered. Publishing waypoint: ({mid_x}, {current_y})")

                waypoint_msg = Point()
                waypoint_msg.x = mid_x
                waypoint_msg.y = current_y
                waypoint_msg.z = 0.0  # No Z component in 2D

                self.waypoint_pub.publish(waypoint_msg)

    def estimate_distance(self, x_position):
        """Estimate the distance of the detected object using LiDAR data."""
        if self.lidar_data:
            angle_index = int((x_position / 640) * len(self.lidar_data))  # Map pixel position to LiDAR index
            return self.lidar_data[angle_index]
        return -1.0  # No valid LiDAR reading

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
