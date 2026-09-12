"""
https://jeffzzq.medium.com/ros2-image-pipeline-tutorial-3b18903e7329
This node publishes the camera feed topic /image_raw

Camera specs: 
width: 1280
height: 720
30fps

ABS
Length Approx.6cm 2.36in
Pixel 1million pixels
Photosensitive chip: OV9726(1/6.5“)
Field of view: 63°No Distortion
Output:USB2.0

TODO: remember to change the camera input channel
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

img_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('usbcamera_publisher')
        
        # Initialize OpenCV capture for the USB camera (0 is the default device ID)
        self.image_pub = self.create_publisher(
            Image,
            '/stinger/camera_0/image_raw',
            img_qos
        ) # queue size 10
        self.timer = self.create_timer(0.033, self.publish_image)  # Publish at 30 Hz
        self.cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2) # TODO: change this by locating the camera using cmd "ls /dev/video*"
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.cap.set(cv2.CAP_PROP_FPS, 60) # frames per second is 60
        self.br = CvBridge()

        if not self.cap.isOpened():
            self.get_logger().error("Could not open video device.")
            return

    def publish_image(self):
        ret, frame = self.cap.read() # capture a frame from the usb cam
        if ret: # is captured successfully
            # Convert OpenCV image to ROS Image message
            msg = self.br.cv2_to_imgmsg(frame, 'bgr8')
            
            # --- ADD THESE LINES ---
            # 1. Give the message a timestamp (required for Foxglove)
            msg.header.stamp = self.get_clock().now().to_msg()
            
            # 2. Give the message a frame_id (required for RViz/Foxglove)
            msg.header.frame_id = 'camera_link'
            # -----------------------

            # Publish the image frame
            self.image_pub.publish(msg)
            self.get_logger().info("Camera streaming active at 30 FPS", throttle_duration_sec=5.0)
        else:
            self.get_logger().error("Failed to capture image.")
            
    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    rclpy.spin(camera_publisher)
    camera_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
