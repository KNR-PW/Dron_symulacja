import rclpy  # Python Client Library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library
import numpy as np  #np to znaczy duzo numpy

# from detection import Detection
from drone_interfaces.msg import DetectionMsg, DetectionsList
from drone_interfaces.srv import DetectTrees, GetLocationRelative, GetAttitude
from drone_interfaces.action import GotoRelative
from std_msgs.msg import Int32MultiArray
import time
from rclpy.action import ActionClient


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
        pass # Chat kazal napisac, ze sekcja rakietowa sie nie otwiera


class GotoDetectionGroup(Node):
    def __init__(self):
        super().__init__("goto_detection_group")
        self.det_cli = self.create_client(DetectTrees, "detect_trees")
        while not self.det_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "detect_trees service not available, waiting again..."
            )
        self.req = DetectTrees.Request()
        self.gps_cli = self.create_client(GetLocationRelative, "get_location_relative")
        while not self.gps_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("GPS service not available, waiting again...")
        self.atti_cli = self.create_client(GetAttitude, "get_attitude")
        while not self.atti_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("attitude service not available, waiting again...")
        self.goto_rel_action_client = ActionClient(self, GotoRelative, "goto_relative")
        self.get_logger().info("GotoDetectionGroup node created")

    def send_detection_request(self, info=0, gps=[0.0, 0.0, 0.0], yaw=0.0):
        gps = [float(x) for x in gps]
        self.get_logger().info("Sending detection request")
        self.req.request_info = info
        self.req.gps = gps
        self.req.yaw = float(yaw)
        future = self.det_cli.call_async(self.req)
        print("called")
        rclpy.spin_until_future_complete(self, future, timeout_sec=10)
        self.get_logger().info("Client recieved detection response")
        # print(future.result().detections_list)
        return future.result().detections_list.detections_list

    def goto_det_group(self, det_list):
        relative_move = [0, 0]
        for det in det_list:
            self.get_logger().info("Going to next det")
            gps_position = det.gps_position
            self.send_goto_relative(
                gps_position[0] - relative_move[0],
                gps_position[1] - relative_move[1],
                0.0,
            )
            relative_move[0] += gps_position[0]
            relative_move[1] += gps_position[1]
            time.sleep(15)

    def get_gps(self):
        self.get_logger().info("Sending GPS request")
        request_gps = GetLocationRelative.Request()
        gps_future = self.gps_cli.call_async(request_gps)
        # gps_future.add_done_callback(self.gps_get_result)
        rclpy.spin_until_future_complete(self, gps_future, timeout_sec=5)
        if gps_future.result() is not None:
            self.north = gps_future.result().north
            self.east = gps_future.result().east
            self.down = gps_future.result().down
            self.drone_amplitude = -self.down
            self.get_logger().info("GPS Recieved")
        else:
            self.get_logger().info("GPS request failed")
            se
            self.drone_amplitude = 0
        return [self.north, self.east, self.down]

    def get_yaw(self):
        self.get_logger().info("Sending yaw request")
        request_attitude = GetAttitude.Request()
        atti_future = self.atti_cli.call_async(request_attitude)
        rclpy.spin_until_future_complete(self, atti_future, timeout_sec=5)
        yaw = atti_future.result().yaw
        return yaw

    def send_goto_relative(self, north, east, down):
        self.get_logger().info("Sending goto relative action goal")
        goal_msg = GotoRelative.Goal()
        goal_msg.north = float(north)
        goal_msg.east = float(east)
        goal_msg.down = float(down)
        while not self.goto_rel_action_client.wait_for_server():
            self.get_logger().info("waiting for goto server...")

        self.goto_rel_action_client.send_goal_async(goal_msg)
        self.get_logger().info("Goto action sent")


def main(args=None):
    rclpy.init(args=args)

    goto_detection_group = GotoDetectionGroup()

    # while True:
    #     detection_client.send_request()
    #     print("det requested")
    #     time.sleep(1)
    for i in range(1):
        gps = goto_detection_group.get_gps()
        yaw = goto_detection_group.get_yaw()
        det_list = goto_detection_group.send_detection_request(gps=gps, yaw=yaw)
        goto_detection_group.goto_det_group(det_list)
    goto_detection_group.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
