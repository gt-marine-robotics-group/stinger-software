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

        # Publishers
        self.waypoint_pub = self.create_publisher(Point, '/gate_waypoint', 10)
        self.goal_pub = self.create_publisher(Bool, '/goal_found', 10)

        self.bridge = CvBridge()
        self.waypoint_published = False # flag to stop multiple waypoints
        self.get_logger().info("Vision node initialized! Let there be light. You can see now.")

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

        # Create masks
        red_mask = cv2.inRange(hsv, red_lower, red_upper)
        green_mask = cv2.inRange(hsv, green_lower, green_upper)
        yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)

        # debugging only
        cv2.imshow("Green mask", green_mask)
        cv2.imshow("Red mask", red_mask)
        key = cv2.waitKey(1)

        # Find contours for each color
        red_buoy = self.find_circles(red_mask)
        green_buoy = self.find_circles(green_mask)
        yellow_buoy = self.find_circles(yellow_mask)

        self.get_logger().info(f"Red detections {red_buoy}")
        self.get_logger().info(f"Green detections {green_buoy}")
        self.get_logger().info(f"Yellow detections {yellow_buoy}")

        if red_buoy:
            self.get_logger().info("Red buoy detected.")
        if green_buoy:
            self.get_logger().info("Green buoy detected.")
        if yellow_buoy:
            self.get_logger().info("Yellow buoy detected.")
            goal_msg = Bool()
            goal_msg.data = True
            self.goal_pub.publish(goal_msg)

        # Check if the tug is centered between the buoys and publish waypoint
        if red_buoy and green_buoy and not self.waypoint_published:
            self.publish_waypoint(red_buoy[0], green_buoy[0]) # only taking in the x loc of the pixel

        # # Check if the goal (yellow buoy) is in front
        # if cv2.countNonZero(yellow_mask) > 50:  # If significant yellow pixels are detected
        #     self.get_logger().info("Goal Found!")
        #     goal_msg = Bool()
        #     goal_msg.data = True
        #     self.goal_pub.publish(goal_msg)

    def find_circles(self, mask):
        """Find circular contours in a binary mask."""
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detected = []

        for cnt in contours:
            (x, y), radius = cv2.minEnclosingCircle(cnt)
            if 40 < radius:  # Filter small objects
                detected.append((int(x), int(y), int(radius)))

        return detected

    def publish_waypoint(self, red_x, green_x):
        """Publish the midpoint between the red and green buoys as a waypoint if distances are similar."""
        red_x, red_y, red_radius = red_buoy
        green_x, green_y, green_radius = green_buoy

        # Ensure radii are similar (within tolerance)
        radius_tolerance = 10  # Pixels
        if abs(red_radius - green_radius) > radius_tolerance:
            return

        # Calculate midpoint in pixel space
        mid_x = (red_x + green_x) / 2  # This corresponds to ROS y-axis (left/right)
        waypoint_x = 2.0  # Move forward by 2 meters
        waypoint_y = mid_x  # Convert pixel x to ROS y

        self.get_logger().info(f"Publishing waypoint: x={waypoint_x}, y={waypoint_y}")

        waypoint_msg = Point()
        waypoint_msg.x = waypoint_x  # Forward movement
        waypoint_msg.y = waypoint_y  # Left/right movement
        waypoint_msg.z = 0.0  # No Z component in 2D

        self.waypoint_pub.publish(waypoint_msg)
        self.waypoint_published = True  # Ensure only one waypoint is published

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
