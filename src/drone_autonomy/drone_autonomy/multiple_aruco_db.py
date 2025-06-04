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

from droniada_inspekcja.inspekcja_db import DroneDB
from drone_comunication.drone_controller import DroneController

import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geopy.distance import geodesic

class MissionRunner(DroneController):
    def __init__(self, waypoints):
        super().__init__()

        self.db = DroneDB("drone_data.db")
        self.mission_id = None
        self.waypoints = waypoints

        self._marker_event = threading.Event()
        self._marker_ids = []
        self._marker_poses = []
        self._marker_image_paths = []
        self._marker_locations = []

        self._latest_image = None
        self._cv_bridge = CvBridge()

        self.create_subscription(Image, 'camera', self._camera_callback, 10)
        self.create_subscription(ArucoMarkers, 'aruco_markers', self._marker_callback, 10)

        self._mission_started = False
        threading.Thread(target=self._delayed_start, daemon=True).start()

    def _calculate_gps_offset(self, base_lat, base_lon, offset_north, offset_east):
        origin = (base_lat, base_lon)
        new_lat = geodesic(meters=offset_north).destination(origin, 0).latitude
        new_lon = geodesic(meters=offset_east).destination(origin, 90).longitude
        return new_lat, new_lon

    def _delayed_start(self):
        time.sleep(2.0)
        self._run_mission()

    def _marker_callback(self, msg: ArucoMarkers):
        if not msg.marker_ids or self._marker_event.is_set():
            return

        self._marker_ids.clear()
        self._marker_poses.clear()
        self._marker_image_paths.clear()
        self._marker_locations.clear()

        img = self.get_camera_image()
        gps_global = (self.lat, self.lon) if hasattr(self, 'lat') else (None, None)

        h, w = img.shape[:2] if img is not None else (0, 0)
        half_box = 50

        existing_markers = {int(a['content']) for a in self.db.get_arucos(self.mission_id)}

        for idx, aruco_id in enumerate(msg.marker_ids):
            if aruco_id in existing_markers:
                continue

            pose = msg.poses[idx]
            px = int(round(pose.position.x))
            py = int(round(pose.position.y))

            if img is not None:
                # Crop around detected marker pixel coordinates (px, py)
                x1 = max(px - half_box, 0)
                y1 = max(py - half_box, 0)
                x2 = min(px + half_box, w - 1)
                y2 = min(py + half_box, h - 1)
                cropped = img[y1:y2, x1:x2]
                crop_path = os.path.abspath(f"aruco_{aruco_id}_{int(time.time())}.jpg")
                cv2.imwrite(crop_path, cropped)
            else:
                crop_path = ""

            if gps_global != (None, None):
                try:
                    marker_lat, marker_lon = self._calculate_gps_offset(
                        gps_global[0], gps_global[1],
                        offset_north=pose.position.x,
                        offset_east=pose.position.y
                    )
                    marker_loc = f"lat={marker_lat:.7f},lon={marker_lon:.7f}"
                except Exception as e:
                    self.get_logger().warn(f"Failed to compute global marker GPS: {e}")
                    marker_loc = f"x={pose.position.x:.2f},y={pose.position.y:.2f},z={pose.position.z:.2f}"
            else:
                marker_loc = f"x={pose.position.x:.2f},y={pose.position.y:.2f},z={pose.position.z:.2f}"

            self._marker_ids.append(aruco_id)
            self._marker_poses.append(pose)
            self._marker_image_paths.append(crop_path)
            self._marker_locations.append(marker_loc)

        self._marker_event.set()

    def _create_mission_row(self):
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
        if self._mission_started:
            return
        self._mission_started = True

        self._create_mission_row()

        if not self.arm():
            self.get_logger().error("Arm failed; aborting mission.")
            return
        if not self.takeoff(5.0):
            self.get_logger().error("Takeoff failed; aborting mission.")
            return

        for idx, (north, east, down) in enumerate(self.waypoints, start=1):
            self.get_logger().info(f"Heading to waypoint {idx}: N={north}, E={east}, D={down}")
            self._marker_event.clear()
            self._marker_ids.clear()
            self._marker_poses.clear()
            self._marker_image_paths.clear()
            self._marker_locations.clear()

            if not self.send_goto_relative(north, east, down):
                self.get_logger().error(f"Goto waypoint {idx} failed; skipping.")
                continue

            while rclpy.ok():
                gps = self.get_gps()
                if gps is None:
                    self.get_logger().warn("GPS unavailable; cannot check arrival.")
                    break
                cur_north, cur_east, cur_down = gps
                if abs(cur_north - north) <= 0.5 and abs(cur_east - east) <= 0.5 and abs(cur_down - down) <= 0.5:
                    self.get_logger().info(f"Arrived at waypoint {idx}.")
                    break
                if self._marker_event.is_set():
                    break
                time.sleep(0.2)

            if self._marker_event.is_set() and self._marker_ids:
                to_store = []
                for (aruco_id, pose, img_path, gps_loc) in zip(
                    self._marker_ids, self._marker_poses, self._marker_image_paths, self._marker_locations
                ):
                    record = {
                        "content": str(aruco_id),
                        "location": gps_loc,
                        "location_changed": "-",
                        "content_changed": "-",
                        "image": img_path,
                        "jury": "-"
                    }
                    to_store.append(record)

                try:
                    self.db.add_arucos(self.mission_id, to_store)
                    self.get_logger().info(f"Saved {len(to_store)} ArUco marker(s) to DB for waypoint {idx}")
                except Exception as e:
                    self.get_logger().error(f"Failed to save ArUco(s) to DB: {e}")

            time.sleep(1.0)

        self.get_logger().info("Mission complete. Landing...")
        if not self.land():
            self.get_logger().error("Landing failed.")
            return
        self.get_logger().info("Landed successfully.")
        self.get_logger().info("MissionRunner: Shutting down node.")
        rclpy.shutdown()

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
    waypoints = [(2.0, 0.0, 0.0), (0.0, 3.0, 0.0), (-2.0, 0.0, 0.0), (0.0, -3.0, 0.0)]
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
