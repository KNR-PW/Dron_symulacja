import rclpy  # Python Client Library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Image  # Image is the message type
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library
import numpy as np
from drone_detector.detection import Detection
from drone_interfaces.msg import DetectionMsg, DetectionsList
from drone_interfaces.srv import DetectTrees, TakePhoto, GetLocationRelative
from std_msgs.msg import Int32MultiArray
import os
import math

class PoolDetector(Node):
# Simulation version of server. It takes frames from topic not from hardware camera
    def __init__(self):
        super().__init__('detector_server')
        self.declare_parameter('camera_topic', 'camera')
        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.get_logger().info(f'camera_topic: {camera_topic}')
        self.camera_subscription = self.create_subscription(Image,
                                                                camera_topic,
                                                                self.camera_callback,
                                                                10)
        self.br = CvBridge()

    def camera_callback(self, img):
        # self.get_logger().info("Recieving frame")
        frame = self.br.imgmsg_to_cv2(img)
        self.frame = frame
        self.detect_pool()

    def detect_pool(self):
        if self.frame is not None:
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            area = cv2.contourArea(cnt)
                if area > 200:
                    x, y, w, h = cv2.boundingRect(cnt)
                    self.get_logger().info(f"Detected pool at x: {x}, y: {y}, w: {w}, h: {h}")

def main(args=None):
    rclpy.init(args=args)

    pool_detector = PoolDetector()

    rclpy.spin(pool_detector)

    pool_detector.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
