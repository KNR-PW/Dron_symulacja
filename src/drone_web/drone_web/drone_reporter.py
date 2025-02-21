import rclpy  # Python Client Library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Image  # Image is the message type
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library
import numpy as np
# from detection import Detection
from drone_interfaces.msg import DetectionMsg, DetectionsList


class WebReporter(Node):
    """
    Create an ImagePublisher class, which is a subclass of the Node class.
    """

    def __init__(self):
        super().__init__('web_reporter')
        self.report_subscription = self.create_subscription(
            Report,
            'report',
            self.report_callback,
            10)

        self.br = CvBridge()
        self.detections = []
        self.frame = 0
        # self.detection_msg = Detection()
        self.detections_list_msg = DetectionsList()
        self.get_logger().info('WebReporter node created')


    def send_to_server(self):
        pass

def main(args=None):
    rclpy.init(args=args)

    web_reporter = WebReporter()

    rclpy.spin(web_reporter)

    web_reporter.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
