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
import time

class ImagePublisher(Node):
    """
  Create an ImagePublisher class, which is a subclass of the Node class.
  """

    def __init__(self):
        """
    Class constructor to set up the node
    """
        # Initiate the Node class's constructor and give it a name
        super().__init__('image_publisher')

        # Create the publisher. This publisher will publish an Image
        # to the video_frames topic. The queue size is 10 messages.
        self.publisher_ = self.create_publisher(Image, 'camera', 10)

        # We will publish a message every 0.1 seconds
        timer_period = 0.1  # seconds

        # Create the timer
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # self.img = cv2.imread('/home/stas/Dron/drone_photos/drone_photosdrone_photo111.jpg', cv2.IMREAD_COLOR)
        video_file = "/home/stas/KNR/tests/vision/output_video_13.mp4"
        # video_file = "/home/stas/Videos/Screencasts/aruco_website.mp4"
        # video_file = "/home/stas/Dron/KNRDron/rosDron/install/drone_detector/lib/drone_detector/car_counting.mp4"
        self.cap = cv2.VideoCapture(video_file)

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()
        self.get_logger().info('Image Publisher node created')

    def timer_callback(self):
        """
    Callback function.
    This function gets called every 0.1 seconds.
    """
        # Capture frame-by-frame
        # This method returns True/False as well
        # as the video frame.
        ret, frame = self.cap.read()

        if ret == True:
            # Publish the image.
            # The 'cv2_to_imgmsg' method converts an OpenCV
            # image to a ROS 2 image message
            # frame = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_LINEAR)
            self.img = frame
            # self.publisher_.publish(self.br.cv2_to_imgmsg(frame))

            # Display the message on the console
            self.get_logger().info('Publishing video frame')
        else:
            self.cap.release()
            self.cap = cv2.VideoCapture(video_file)
            return
        img = self.img.copy()
        # img = cv2.resize(self.img, (640, 480), interpolation=cv2.INTER_LINEAR)

        # img = cv2.blur(img, (10, 10))  
        self.publisher_.publish(self.br.cv2_to_imgmsg(img))
        cv2.imshow('Video Frame', img)
        cv2.waitKey(2)


def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    image_publisher = ImagePublisher()

    # Spin the node so the callback function is called.
    rclpy.spin(image_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_publisher.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == '__main__':
    main()
