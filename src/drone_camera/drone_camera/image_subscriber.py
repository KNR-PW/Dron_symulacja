# Basic ROS 2 program to subscribe to real-time streaming 
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com

# Import the necessary libraries
import rclpy  # Python library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Image  # Image is the message type
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library


class ImageSubscriber(Node):
    """
    Create an ImageSubscriber class, which is a subclass of the Node class.
    """

    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name

        super().__init__('camera')

        # Create the subscriber. This subscriber will receive an Image
        # from the video_frames topic. The queue size is 10 messages.
        self.subscription = self.create_subscription(
            Image,
            "camera/image_raw",
            self.listener_callback,
            10)
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

        self.get_logger().info('image_subscriber node created')

    def listener_callback(self, data):
        self.get_logger().info('frame')
        """
        Callback function.
        """
        # Display the message on the console
        # self.get_logger().info('Receiving video frame')

        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(data)

        # convert frame in simulation
        # current_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB)
        # Display image
        # cv2.rectangle(current_frame, (int(current_frame.shape[0]/2), int(current_frame.shape[1]/2)), (int(current_frame.shape[0]/2)+20, int(current_frame.shape[1]/2)+20), (0,0,255), 3)
        cv2.imshow("camera", current_frame)

        cv2.waitKey(1)


def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    image_subscriber = ImageSubscriber()

    # Spin the node so the callback function is called.
    rclpy.spin(image_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_subscriber.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == '__main__':
    main()
