import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from drone_interfaces.msg import ArucoMarkersList
from drone_interfaces.action import GotoRelative
from drone_interfaces.srv import GetLocationRelative, GetAttitude
import math
import numpy as np

class FlyToAruco(Node):

    def __init__(self):
        super().__init__('fly_to_aruco')

        self.aruco_markers = []
        self.img_size = (640, 480)

        self.subscription = self.create_subscription(
            ArucoMarkersList,
            'aruco_markers',
            self.aruco_callback,
            10)
        self.gps_cli = self.create_client(GetLocationRelative, 'get_location_relative')
        while not self.gps_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('GPS service not available, waiting again...')
        self.atti_cli = self.create_client(GetAttitude, 'get_attitude')
        while not self.atti_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('attitude service not available, waiting again...')
        self.goto_rel_action_client = ActionClient(self, GotoRelative, 'goto_relative')
        self.get_logger().info('FlyToAruco node created')

    def aruco_callback(self, msg):
        self.aruco_markers = msg.aruco_markers

        # self.get_logger().info(f'Nuber of aruco markers detected: {len(self.aruco_markers)}')

    def get_gps(self):
        self.get_logger().info('Sending GPS request')
        request_gps = GetLocationRelative.Request()
        gps_future = self.gps_cli.call_async(request_gps)
        rclpy.spin_until_future_complete(self, gps_future, timeout_sec=5)
        if gps_future.result() is not None:
            self.north = gps_future.result().north
            self.east = gps_future.result().east
            self.down = gps_future.result().down
            self.get_logger().info('GPS received')
        else:
            self.get_logger().info('GPS request failed')
            self.north = self.east = self.down = 0.0
        return [self.north, self.east, self.down]

    def get_yaw(self):
        self.get_logger().info('Sending yaw request')
        request_attitude = GetAttitude.Request()
        atti_future = self.atti_cli.call_async(request_attitude)
        rclpy.spin_until_future_complete(self, atti_future, timeout_sec=5)
        if atti_future.result() is not None:
            yaw = atti_future.result().yaw
            self.get_logger().info('Yaw received')
        else:
            self.get_logger().info('Yaw request failed')
            yaw = 0.0
        return yaw

    def marker2pos(self, corners, gps, yaw):

        HFOV=math.radians(62.2)
        VFOV=math.radians(48.8)
        
        corners = np.array(corners)
        corners = corners.reshape((4,2))
        marker_middle = np.mean(corners, axis=0)

        altitude = -self.down
        cam_range=(math.tan(HFOV/2)*altitude*2,math.tan(VFOV/2)*altitude*2)
        target_pos_rel=np.multiply(np.divide(marker_middle, self.img_size)-np.array([0.5, 0.5]), cam_range)

        # Negative Yaw!
        pt = np.matmul(self.Rot(-yaw), target_pos_rel)
        pos = [-pt[0,1], pt[0,0]]
        return pos

    def Rot(self, yaw):
        res = np.matrix([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
        return res

    def fly_to_marker(self):
        self.get_logger().info("Trying to fly to marker...")
        while len(self.aruco_markers) == 0:
            self.get_logger().info("No markers detected trying again...")
            rclpy.spin_once(self)
        
        gps = self.get_gps()
        yaw = self.get_yaw()

        # Choose the first marker
        marker = self.aruco_markers[0]
        corners = marker.corners
        pos = self.marker2pos(corners, gps, yaw)
        self.send_goto_relative(pos[0], pos[1], 0.0)

    def send_goto_relative(self, north, east, down):
        self.get_logger().info("Sending goto relative action goal")
        goal_msg = GotoRelative.Goal()
        goal_msg.north = float(north)
        goal_msg.east = float(east)
        goal_msg.down = float(down)
        while not self.goto_rel_action_client.wait_for_server():
            self.get_logger().info('waiting for goto relative server...')

        self.goto_rel_action_client.send_goal_async(goal_msg)
        self.get_logger().info("Goto action goal sent")

def main(args=None):
    rclpy.init(args=args)
    fly_to_aruco = FlyToAruco()
    fly_to_aruco.fly_to_marker()
    fly_to_aruco.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()