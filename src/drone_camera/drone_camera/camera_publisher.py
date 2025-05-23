# Basic ROS 2 program to publish real-time streaming 
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com

# Import the necessary libraries
import rclpy  # Python Client Library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Image  # Image is the message type
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library
import os


class CameraPublisher(Node):
    """
  Create an ImagePublisher class, which is a subclass of the Node class.
  """

    def __init__(self):
        """
    Class constructor to set up the node
    """
        # Initiate the Node class's constructor and give it a name
        super().__init__('camera_publisher')
        # define parameter
        self.declare_parameter('camera', 'udp://172.17.0.1:5000?overrun_nonfatal=1&fifo_size=50000000')

        # Create the publisher. This publisher will publish an Image
        # to the video_frames topic. The queue size is 10 messages.
        self.publisher_ = self.create_publisher(Image, 'camera', 10)
        camera_index = self.get_parameter('camera').get_parameter_value().string_value
        if camera_index == '0':
            camera_index = 0
        # We will publish a message every 0.1 seconds
        timer_period = 0.1  # seconds

        # Create the timer
        self.timer = self.create_timer(timer_period, self.timer_callback)
        try:
            self.cap = cv2.VideoCapture(camera_index)
        except Exception as e:
            self.get_logger().info(f'Creating camera publisher failed. Camera error: {e}')
            return

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()
        self.get_logger().info('Image Publisher node created')


    def timer_callback(self):
        """
    Callback function.
    This function gets called every 0.1 seconds.
    """

        ret, frame = self.cap.read()
        if ret == True:
            frame = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_LINEAR)
            self.img = frame
        else:
            # if camera is not available, try to reconnect
            self.get_logger().info('Camera is not available. Trying to reconnect...')
            self.cap.release()
            self.cap = cv2.VideoCapture(0)
            return
        img = cv2.resize(self.img, (640, 480), interpolation=cv2.INTER_LINEAR)
        self.publisher_.publish(self.br.cv2_to_imgmsg(img))


def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    camera_publisher = CameraPublisher()

    # Spin the node so the callback function is called.
    rclpy.spin(camera_publisher)

    # Destroy the node explicitly
    camera_publisher.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == '__main__':
    main()
