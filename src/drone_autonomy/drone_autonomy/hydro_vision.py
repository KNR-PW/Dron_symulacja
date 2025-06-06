import threading
import time
from datetime import datetime
import os

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from drone_interfaces.msg import ArucoMarkers, Telemetry
from drone_interfaces.srv import (
    GetLocationRelative,
    GetAttitude,
    SetMode,
    SetSpeed,
    TurnOnVideo,
    TurnOffVideo,
    PostLog
)
from drone_interfaces.action import Arm, Takeoff, GotoRelative, GotoGlobal, SetYawAction

from drone_comunication.drone_controller import DroneController

import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

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

class WebLogger(Node):
    def __init__(self):
        super().__init__('web_logger')
        self.client = self.create_client(PostLog, 'post_log_to_web')

    def log(self, message="Zbiornik został wykryty", level="info"):
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service post_log_to_server...')

        request = PostLog.Request()
        request.message = message
        request.level = level

        future = self.client.call_async(request)
        # rclpy.spin_until_future_complete(self, future)

        # if future.result() is not None:
        #     if future.result().result:
        #         self.get_logger().info('Successfully posted web log')
        #     else:
        #         self.get_logger().warn(f"Failed to post log")
        # else:
        #     self.get_logger().error('Service call failed')

class MissionRunner(DroneController):
    def __init__(self, takeoff_alt):
        super().__init__()

        self.takeoff_alt = takeoff_alt

        self.mission_id = None

        self._latest_image = None
        self._cv_bridge = CvBridge()

        # Subscribe to camera and ArUco markers
        self.create_subscription(Image, "camera/image_raw", self._camera_callback, 10)

        self._mission_started = False

        self.pool_found = False
        self.debug_mode = True

        self.camera_instricts = np.array(
            [
                [3.97045297e03, 0.00000000e00, 2.04507985e03],
                [0.00000000e00, 3.97159037e03, 1.55103947e03],
                [0.00000000e00, 0.00000000e00, 1.00000000e00],
            ]
        )
        self.calibration_img_shape = (4056, 3040)

        self.web_logger = WebLogger()

        threading.Thread(target=self._delayed_start, daemon=True).start()

    def _delayed_start(self):
        time.sleep(1.0)
        self._run_mission()

    def detect_pool(self):
        RESIZE_WIDTH = 640
        LOWER_BLUE = np.array([90, 50, 50], dtype=np.uint8)
        UPPER_BLUE = np.array([130, 255, 255], dtype=np.uint8)
        MIN_RADIUS = 10
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
            frame_small = cv2.resize(
                frame, (RESIZE_WIDTH, int(h * scale)), interpolation=cv2.INTER_AREA
            )

            hsv = cv2.cvtColor(frame_small, cv2.COLOR_BGR2HSV)
            hsv = _equalise_value_channel(hsv)

            mask = cv2.inRange(hsv, LOWER_BLUE, UPPER_BLUE)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL, iterations=2)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, KERNEL, iterations=2)
            return frame_small, mask

        frame_small, mask = _preprocess(frame)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            if self.debug_mode:
                print("No contours found.")
            return None

        # Largest contour is the most likely pool candidate.
        cnt = max(cnts, key=cv2.contourArea)
        area = cv2.contourArea(cnt)
        (x, y), radius = cv2.minEnclosingCircle(cnt)

        # Quick rejection of small or misshapen blobs.
        if radius < MIN_RADIUS:
            if self.debug_mode:
                print(f"Rejected small radius: {radius} < {MIN_RADIUS}")
            return None
        if area < ROUNDEDNESS * np.pi * radius * radius:
            if self.debug_mode:
                print(
                    f"Rejected area: {area} < {ROUNDEDNESS * np.pi * radius * radius}"
                )
            return None

        centre = (int(x), int(y))
        return (centre[0], centre[1], int(radius))

    def _run_mission(self):

        if self._mission_started:
            return
        self._mission_started = True


        while True:
            self.pool_found = False
            if self._latest_image is not None:
                pool_data = self.detect_pool()
            else:
                pool_data = None

            if pool_data is not None:
                self.pool_found = True
                self.get_logger().info(f"Pool detected with data: {pool_data}")
                threading.Thread(target=self.web_logger.log, daemon=True).start()

                pool_position = pool_data[:2]
                pool_radius = pool_data[2]

                scaled_instricts = scale_intrinsics(
                    self.camera_instricts,
                    self.calibration_img_shape,
                    self._latest_image.shape[:2],
                )
                alt = self.alt

                X_rel, Y_rel = image_to_ground(pool_position, scaled_instricts, alt)
                self.get_logger().info(
                    f"Pool position in ground coordinates: X={X_rel}, Y={Y_rel}, Alt={alt}"
                )
                # time.sleep(10)
            start_time = time.time()
            while time.time() - start_time < 1.0:
                rclpy.spin_once(self, timeout_sec=0.1)

        return

    def _camera_callback(self, msg):
        try:
            encoding = "passthrough" if msg.encoding == "8UC3" else "bgr8"
            self._latest_image = self._cv_bridge.imgmsg_to_cv2(
                msg, desired_encoding=encoding
            )
        except Exception as e:
            self.get_logger().error(f"Failed to convert camera image: {e}")
            self._latest_image = None

    def get_camera_image(self):
        return self._latest_image


def main(args=None):
    rclpy.init(args=args)

    alt = 10.0

    node = MissionRunner(alt)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
