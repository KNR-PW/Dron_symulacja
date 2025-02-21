import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import socket
import struct
import numpy as np
import cv2
import time
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
class ArdupilotCameraHandler(Node):
    def __init__(self):
        super().__init__('ardupilot_camera_handler')
        self.camera_publisher = self.create_publisher(Image, 'camera', 10)
        self.bridge = CvBridge()
        self.port =   5599# Replace with the starting port number used in webots_vehicle.py
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.connect(('localhost', self.port))
        # self.server_socket.listen(1)

        self.get_logger().info(f"Listening for camera images on port {self.port}")
        my_callback_group = MutuallyExclusiveCallbackGroup()
        my_callback_group = MutuallyExclusiveCallbackGroup()

        self.timer = self.create_timer(3, self.publish_image, callback_group=my_callback_group)

    def publish_image(self):
        t1 = time.time()
        # self.server_socket.connect(('localhost', self.port))
        # try:
        # Receive the header
        header = self.server_socket.recv(4)
        if not header:
            return
        cam_width, cam_height = struct.unpack("=HH", header)

        # Receive the image data
        img_size = cam_width * cam_height
        img_data = b''
        self.get_logger().info(f"immg size: {img_size}")
        # while len(img_data) < img_size:
        #     packet = self.server_socket.recv(img_size - len(img_data))
        #     self.get_logger().info(f"immg data: {len(img_data)}")
        #     if not packet:
        #         self.get_logger().info("no packet")
        #         return
        #     img_data += packet
        img_data = self.server_socket.recv(img_size)
        self.get_logger().info(f"immg data: {len(img_data)}")
        # Convert the image data to a numpy array
        img_array = np.frombuffer(img_data, dtype=np.uint8).reshape((cam_height, cam_width))

        # Create a ROS2 Image message
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_frame'
        msg.height = cam_height
        msg.width = cam_width
        msg.encoding = 'mono8'
        msg.is_bigendian = False
        msg.step = cam_width
        msg.data = img_array.tobytes()

        # Publish the image message
        self.camera_publisher.publish(msg)

        # except Exception as e:
        #     pass
        #     self.get_logger().error(f"Error receiving image: {e}")
        #     return
        # self.server_socket.close()
        t2 = time.time()
        self.get_logger().info(f"Time taken: {t2-t1}")
def main(args=None):
    rclpy.init(args=args)
    node = ArdupilotCameraHandler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()