import rclpy  # Python Client Library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library
import numpy as np
# from detection import Detection
from drone_interfaces.msg import DetectionMsg, DetectionsList
from drone_interfaces.srv import DetectTrees, GetLocationRelative, GetAttitude
from std_msgs.msg import Int32MultiArray
import time


class Detection:
    def __init__(self, bounding_box=(0, 0, 0, 0), color="", gps_pos=(0, 0)):
        # Format x, y, w, h
        self.bounding_box = bounding_box
        self.color = color
        self.gps_pos = gps_pos

    def set_bounding_box(self, bb):
        self.bounding_box = bb

    def set_gps_pos(self, pos):
        self.gps_pos = pos

    def get_bounding_box(self):
        return self.bounding_box

    def get_gps_pos(self):
        return self.gps_pos

    def get_color(self):
        return self.color

    def add_image(self, image):
        pass


class DetectionClient(Node):

    def __init__(self):
        super().__init__('detection_client')
        self.det_cli = self.create_client(DetectTrees, 'detect_trees')
        while not self.det_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('detect_trees service not available, waiting again...')
        self.req = DetectTrees.Request()
        self.gps_cli = self.create_client(GetLocationRelative, 'get_location_relative')
        while not self.gps_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('GPS service not available, waiting again...')
        self.atti_cli = self.create_client(GetAttitude, 'get_attitude')
        while not self.atti_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('attitude service not available, waiting again...')
        self.get_logger().info('DetectionClient node created')

    def send_request(self, info=0, gps=[0.0,0.0,0.0], yaw=0.0):
        gps = [float(x) for x in gps]
        self.get_logger().info('Sending detection request')
        self.req.request_info = info
        self.req.gps = gps
        self.req.yaw = float(yaw)
        future = self.det_cli.call_async(self.req)
        print("called")
        rclpy.spin_until_future_complete(self, future, timeout_sec=10)
        self.get_logger().info('Client recieved detection response')
        # print(future.result().detections_list)
        return future.result()

    def get_gps(self):
        self.get_logger().info('Sending GPS request')
        request_gps = GetLocationRelative.Request()
        gps_future = self.gps_cli.call_async(request_gps)
        # gps_future.add_done_callback(self.gps_get_result)
        rclpy.spin_until_future_complete(self, gps_future, timeout_sec=5)
        if gps_future.result() is not None:
            self.north = gps_future.result().north
            self.east = gps_future.result().east
            self.down = gps_future.result().down
            self.drone_amplitude = -self.down
            self.get_logger().info('GPS Recieved')
        else:
            self.get_logger().info('GPS request failed')
            self.drone_amplitude = 0
        return [self.north, self.east, self.down]

    def get_yaw(self):
        self.get_logger().info('Sending yaw request')
        request_attitude = GetAttitude.Request()
        atti_future = self.atti_cli.call_async(request_attitude)
        rclpy.spin_until_future_complete(self, atti_future, timeout_sec=5)
        yaw = atti_future.result().yaw
        return yaw
def main(args=None):
    rclpy.init(args=args)

    detection_client = DetectionClient()

    # while True:
    #     detection_client.send_request()
    #     print("det requested")
    #     time.sleep(1)
    for i in range(1):
        gps = detection_client.get_gps()
        yaw = detection_client.get_yaw()
        a = detection_client.send_request(gps=gps, yaw=yaw)
        print(a)
    detection_client.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
