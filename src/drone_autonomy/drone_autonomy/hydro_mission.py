import threading
import time
from datetime import datetime
import os

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from drone_interfaces.msg import ArucoMarkers, Telemetry
from drone_interfaces.srv import GetLocationRelative, GetAttitude, SetMode, SetSpeed, TurnOnVideo, TurnOffVideo
from drone_interfaces.action import Arm, Takeoff, GotoRelative, GotoGlobal, SetYawAction

from drone_comunication.drone_controller import DroneController

import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geopy.distance import geodesic

import json

import sys
from pathlib import Path
import cv2
import numpy as np

def scale_intrinsics(K, original_size, new_size):
    orig_w, orig_h = original_size
    new_w, new_h = new_size

    scale_x = new_w / orig_w
    scale_y = new_h / orig_h

    K_scaled = K.copy()
    K_scaled[0, 0] *= scale_x  # fx
    K_scaled[0, 2] *= scale_x  # cx
    K_scaled[1, 1] *= scale_y  # fy
    K_scaled[1, 2] *= scale_y  # cy

    return K_scaled



def image_to_ground(point, K, height):
    u, v = point
    fx = K[0, 0]
    fy = K[1, 1]
    cx = K[0, 2]
    cy = K[1, 2]

    # Direction vector in camera coordinates
    x = (u - cx) / fx
    y = (v - cy) / fy
    z = 1.0

    # Scale by known height to intersect Z=0 plane
    scale = height / z

    # Real-world coordinates relative to camera frame
    X = x * scale
    Y = y * scale

    return X, Y


class MissionRunner(DroneController):
    def __init__(self, waypoints):
        super().__init__()

        self.mission_id = None
        self.waypoints = waypoints

        self._latest_image = None
        self._cv_bridge = CvBridge()

        # Subscribe to camera and ArUco markers
        self.create_subscription(Image, 'camera', self._camera_callback, 10)

        self._mission_started = False

        self.pool_found = False

        self.camera_instricts = np.array([[3.97045297e+03, 0.00000000e+00, 2.04507985e+03], [0.00000000e+00, 3.97159037e+03, 1.55103947e+03], [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
        self.calibration_img_shape = (4056, 3040)
        threading.Thread(target=self._delayed_start, daemon=True).start()

    def _delayed_start(self):
        time.sleep(2.0)
        self._run_mission()

    def detect_pool(self):
        RESIZE_WIDTH = 640
        LOWER_BLUE = np.array([90,  50,  50], dtype=np.uint8)
        UPPER_BLUE = np.array([130, 255, 255], dtype=np.uint8)
        MIN_RADIUS = 30
        ROUNDEDNESS = 0.60

        KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

        frame = self.get_camera_image()

        def _equalise_value_channel(hsv: np.ndarray) -> np.ndarray:
            h, s, v = cv2.split(hsv)
            v_eq = cv2.equalizeHist(v)
            return cv2.merge([h, s, v_eq])


        def _preprocess(frame: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
            h, w = frame.shape[:2]
            scale = RESIZE_WIDTH / float(w)
            frame_small = cv2.resize(frame, (RESIZE_WIDTH, int(h * scale)), interpolation=cv2.INTER_AREA)

            hsv = cv2.cvtColor(frame_small, cv2.COLOR_BGR2HSV)
            hsv = _equalise_value_channel(hsv)

            mask = cv2.inRange(hsv, LOWER_BLUE, UPPER_BLUE)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL, iterations=2)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, KERNEL, iterations=2)
            return frame_small, mask


        frame_small, mask = _preprocess(frame)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return frame_small, None

        # Largest contour is the most likely pool candidate.
        cnt = max(cnts, key=cv2.contourArea)
        area = cv2.contourArea(cnt)
        (x, y), radius = cv2.minEnclosingCircle(cnt)

        # Quick rejection of small or misshapen blobs.
        if radius < MIN_RADIUS:
            return frame_small, None
        if area < ROUNDEDNESS * np.pi * radius * radius:
            return frame_small, None

        output = frame_small.copy()
        centre = (int(x), int(y))
        cv2.circle(output, centre, int(radius), (0, 255, 0), 2, lineType=cv2.LINE_AA)
        cv2.circle(output, centre, 2, (0, 0, 255), -1, lineType=cv2.LINE_AA)  # centre dot
        return output, (centre[0], centre[1], int(radius))


    def _run_mission(self):

        if self._mission_started:
            return
        self._mission_started = True


        if not self.arm():
            self.get_logger().error("Arm failed; aborting mission.")
            return
        if not self.takeoff(10.0):
            self.get_logger().error("Takeoff failed; aborting mission.")
            return

        time.sleep(2.0)  

        self.send_goto_relative(0.0, 0.0, 0.0)
        self.set_speed(0.8)

        for idx, (north, east, down) in enumerate(self.waypoints, start=1):
            self.get_logger().info(f"Heading to waypoint {idx}: N={north}, E={east}, D={down}")

            if not self.send_goto_global(north, east, down):
                self.get_logger().error(f"Goto waypoint {idx} failed; skipping.")
                continue

            time.sleep(3.0)
            
            det_img, pool_data = self.detect_pool()
            if pool_data is not None:
                self.pool_found = True
                self.get_logger().info(f"Pool detected at waypoint {idx} with data: {pool_data}")
                break

            # Small pause before next waypoint
            time.sleep(1.0)

        if not self.pool_found:
            self.get_logger().info("No pool detected in the waypoints. Returning to home.")
            self.rtl()   
            return
        else:
            pool_position = pool_data[:2]
            pool_radius = pool_data[2]

            scaled_instricts = scale_intrinsics(self.camera_instricts, self.calibration_img_shape, self._latest_image.shape[:2])
            alt = self.alt

            X_rel, Y_rel = image_to_ground(pool_position, scaled_instricts, alt)
            self.get_logger().info(f"Pool position in ground coordinates: X={X_rel}, Y={Y_rel}, Alt={alt}")
            # time.sleep(10)
            self.send_goto_relative(X_rel, Y_rel, 0.0)

            
        self.rtl()
        return

    def _camera_callback(self, msg):
        try:
            encoding = 'passthrough' if msg.encoding == '8UC3' else 'bgr8'
            self._latest_image = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding=encoding)
        except Exception as e:
            self.get_logger().error(f"Failed to convert camera image: {e}")
            self._latest_image = None

    def get_camera_image(self):
        return self._latest_image


def main(args=None):
    rclpy.init(args=args)
    alt = 10.0
    # waypoints = [(2.0, 0.0, 0.0), (0.0, 3.0, 0.0), (-2.0, 0.0, 0.0), (0.0, -3.0, 0.0)]
    waypoints = [
        (-35.363319396972656, 149.16531372070312, 10),
        (-35.36327258544922, 149.16510009765625, 10)
        # (50.2715662, 18.6443051, alt),
        # (50.2714623, 18.6442565, alt),
        # (50.2717372, 18.6440945, alt),
        # (50.2719005, 18.6444969, alt)
        
        ]
    node = MissionRunner(waypoints)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
