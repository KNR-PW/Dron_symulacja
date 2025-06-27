#!/usr/bin/env python3
"""
mission_runner.py

This script defines a ROS2 node that:
  1. Creates a new mission entry in the SQLite database.
  2. Sequentially navigates the drone through a list of waypoints.
  3. Subscribes to ArUco marker detections; when a marker is detected at a waypoint,
     records it in the database (using DroneDB.add_arucos).
  4. At the end of the sequence, lands the drone.

Threading/parallelism:
  - Uses a MultiThreadedExecutor to allow subscription callbacks (marker detections)
    to run concurrently with the mission logic.
  - The main mission logic runs in a dedicated Python thread (so it doesn’t block
    the ROS executor thread).
"""

import threading
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from drone_interfaces.msg import ArucoMarkers
from drone_interfaces.msg import Telemetry
from drone_interfaces.srv import GetLocationRelative, GetAttitude, SetMode, SetSpeed, TurnOnVideo, TurnOffVideo
from drone_interfaces.action import Arm, Takeoff, GotoRelative, GotoGlobal, SetYawAction

from droniada_inspekcja.inspekcja_db import DroneDB  # Adjust if module path differs
from drone_comunication.drone_controller import DroneController  # Adjust import to your package structure

import cv2
import os

class MissionRunner(DroneController):
    def __init__(self, waypoints):
        """
        :param waypoints: List of (north, east, down) tuples for relative navigation.
        """
        super().__init__()

        # --- Database setup ---
        self.db = DroneDB("drone_data.db")
        self.mission_id = None

        # --- Waypoints to visit ---
        self.waypoints = waypoints

        # --- Marker detection state ---
        self._marker_event = threading.Event()
        self._marker_pose = None
        self._marker_id = None

        # --- Camera image state ---
        self._latest_image = None
        from sensor_msgs.msg import Image
        from cv_bridge import CvBridge
        self._cv_bridge = CvBridge()
        self.create_subscription(
            Image,
            'camera',
            self._camera_callback,
            10
        )

        # Subscribe to ArUco marker topic
        self.create_subscription(
            ArucoMarkers,
            'aruco_markers',
            self._marker_callback,
            10
        )

        # Delay to let everything initialize before starting mission
        self._mission_started = False

        # Start mission logic in its own thread after small delay
        threading.Thread(target=self._delayed_start, daemon=True).start()

    def _delayed_start(self):
        # Wait a bit for vehicle to be fully initialized
        time.sleep(2.0)
        self._run_mission()

    def _marker_callback(self, msg: ArucoMarkers):
        """
        Triggered whenever ArUco markers are published.
        If at least one marker is detected and we haven't recorded it yet for the current waypoint,
        capture the first marker’s ID and pose.
        """
        if msg.marker_ids and not self._marker_event.is_set():
            self._marker_id = msg.marker_ids[0]
            self._marker_pose = msg.poses[0]
            self.get_logger().info(
                f"Detected ArUco marker {self._marker_id} at x={self._marker_pose.position.x:.2f}, "
                f"y={self._marker_pose.position.y:.2f}, z={self._marker_pose.position.z:.2f}"
            )
            # --- Save camera image ---
            try:
                img = self.get_camera_image()  # Implement this method to get latest camera image (cv2 image)
                if img is not None:
                    img_path = os.path.abspath(f"./aruco_{self._marker_id}_{int(time.time())}.jpg")
                    # img_path = f"./aruco_{self._marker_id}_{int(time.time())}.jpg"
                    # --- Crop image around ArUco marker ---
                    # Get marker center in image coordinates
                    # Ensure marker_pose.position.x/y are pixel coordinates; if not, conversion is needed
                    x = int(round(self._marker_pose.position.x))
                    y = int(round(self._marker_pose.position.y))
                    # Debug: Save full image with marker center drawn for verification
                    debug_img = img.copy()
                    cv2.circle(debug_img, (x, y), 20, (0, 0, 255), 3)
                    debug_img_path = os.path.abspath(f"./aruco_{self._marker_id}_{int(time.time())}_debug.jpg")
                    cv2.imwrite(debug_img_path, debug_img)
                    self.get_logger().info(f"Saved debug image with marker center to {debug_img_path}")

                    # Define padding (in pixels) around marker
                    padding = 120  # Increased padding for better visibility
                    # Estimate marker size (if available), else use default
                    marker_size = 180  # Increased marker size for larger crop
                    # Compute crop boundaries and ensure they are within image bounds
                    # Reverse vertical axis (y) if needed: OpenCV images have (0,0) at top-left,
                    # but if marker y is from bottom, flip it.
                    x1 = max(x - marker_size // 2 - padding, 0)
                    # Revert vertical axis: OpenCV (0,0) is top-left, but if marker y is from bottom, flip it
                    y_img = img.shape[0] - int(round(self._marker_pose.position.y))
                    y1 = max(y_img - marker_size // 2 - padding, 0)
                    x2 = min(x + marker_size // 2 + padding, img.shape[1])
                    y2 = min(y + marker_size // 2 + padding, img.shape[0])
                    # Ensure x1 < x2 and y1 < y2
                    if x2 > x1 and y2 > y1:
                        cropped_img = img[y1:y2, x1:x2]
                    else:
                        cropped_img = img
                        self.get_logger().warn("Invalid crop region, saving full image.")
                    cv2.imwrite(img_path, cropped_img)
                    # cv2.imwrite(img_path, img)
                    self._marker_image_path = img_path
                    self.get_logger().info(f"Saved marker image to {img_path}")
                else:
                    self._marker_image_path = ""
                    self.get_logger().warn("No camera image available to save.")
            except Exception as e:
                self._marker_image_path = ""
                self.get_logger().error(f"Failed to save marker image: {e}")
            self._marker_event.set()

    def _create_mission_row(self):
        """
        Use the DroneDB wrapper to insert a new mission with placeholder data.
        """
        mission_time_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.mission_id = self.db.add_mission(
            team="Team A",
            email="pilot@example.com",
            pilot="Test Pilot",
            phone="000-000-0000",
            mission_time=mission_time_str,
            mission_no=f"M{int(time.time())}",
            duration="0m",
            battery_before="100%",
            battery_after="100%",
            kp_index=0,
            infra_map="/static/img/mapa.jpg",
        )
        self.get_logger().info(f"Created mission ID {self.mission_id}")

    def _run_mission(self):
        """
        Main mission logic:
          1. Create mission row in DB.
          2. Arm, take off.
          3. For each waypoint:
             a. Fly to waypoint.
             b. Wait for arrival + marker detection (if any).
             c. If marker detected, save to DB via add_arucos.
          4. Land.
          5. Update mission duration/battery (optional).
        """
        if self._mission_started:
            return
        self._mission_started = True

        # 1) Create mission in DB
        self._create_mission_row()

        # 2) Arm and Takeoff to 5 meters
        if not self.arm():
            self.get_logger().error("Arm failed; aborting mission.")
            return
        if not self.takeoff(5.0):
            self.get_logger().error("Takeoff failed; aborting mission.")
            return

        # 3) Visit each waypoint sequentially
        for idx, (north, east, down) in enumerate(self.waypoints, start=1):
            self.get_logger().info(f"Heading to waypoint {idx}: N={north}, E={east}, D={down}")
            # Clear previous marker event
            self._marker_event.clear()
            self._marker_pose = None
            self._marker_id = None

            # Send goto_relative
            if not self.send_goto_relative(north, east, down):
                self.get_logger().error(f"Goto waypoint {idx} failed; skipping.")
                continue

            # Wait for drone to reach within 0.5m of target
            arrived = False
            while rclpy.ok():
                # Get current local position
                gps = self.get_gps()
                if gps is None:
                    self.get_logger().warn("GPS unavailable; cannot check arrival.")
                    break
                cur_north, cur_east, cur_down = gps
                d_n = abs(cur_north - north)
                d_e = abs(cur_east - east)
                d_d = abs(cur_down - down)
                if d_n <= 0.5 and d_e <= 0.5 and d_d <= 0.5:
                    arrived = True
                    self.get_logger().info(f"Arrived at waypoint {idx}.")
                    break
                # Also check for marker detection while en route
                if self._marker_event.is_set():
                    break
                time.sleep(0.2)

            # 4) If marker detected at this waypoint, save to DB
            if self._marker_event.is_set() and self._marker_id is not None and self._marker_pose is not None:
                aruco_record = {
                    "content": str(self._marker_id),
                    "location": f"x={self._marker_pose.position.x:.2f},"
                                f"y={self._marker_pose.position.y:.2f},"
                                f"z={self._marker_pose.position.z:.2f}",
                    "location_changed": "Nie",
                    "content_changed": "Nie",
                    "image": getattr(self, '_marker_image_path', ""),
                    "jury": "None"
                }
                try:
                    self.db.add_arucos(self.mission_id, [aruco_record])
                    self.get_logger().info(f"Saved ArUco {self._marker_id} to DB for waypoint {idx}")
                except Exception as e:
                    self.get_logger().error(f"Failed to save ArUco to DB: {e}")

            # Small pause before next waypoint
            time.sleep(1.0)

        # 5) Land at final position
        self.get_logger().info("Mission complete. Landing...")
        if not self.land():
            self.get_logger().error("Landing failed.")
            return
        self.get_logger().info("Landed successfully.")

        # 6) (Optional) Update mission duration and battery
        # We won’t update mission row here because DroneDB lacks update_mission.
        # In practice, you could extend DroneDB with an update_mission(...) method.

        self.get_logger().info("MissionRunner: Shutting down node.")
        # Signal ROS shutdown
        rclpy.shutdown()

    def _camera_callback(self, msg):
        try:
            # If encoding is already 8UC3, use 'passthrough', else use 'bgr8'
            encoding = 'passthrough' if msg.encoding == '8UC3' else 'bgr8'
            self._latest_image = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding=encoding)
        except Exception as e:
            self.get_logger().error(f"Failed to convert camera image: {e}")

    def get_camera_image(self):
        """
        Returns the latest camera image as a numpy array (cv2 image), or None if not available.
        """
        return self._latest_image


def main(args=None):
    rclpy.init(args=args)
    # Example list of relative waypoints: [(north, east, down), …]
    waypoints = [
        (2.0, 0.0, 0.0),
        (0.0, 2.0, 0.0),
        (-2.0, 0.0, 0.0),
        (0.0, -2.0, 0.0)
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
