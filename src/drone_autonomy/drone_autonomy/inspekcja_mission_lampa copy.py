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

from droniada_inspekcja.vision_agent import VisionAgent

import json

class MissionRunner(DroneController):
    def __init__(self, waypoints, orto_waypoint):
        super().__init__()

        # --- Existing attributes ---
        self.db = DroneDB("drone_data.db")
        self.mission_id = None
        self.waypoints = waypoints
        self.orto_waypoint = orto_waypoint

        self._marker_event = threading.Event()
        self._marker_ids = []
        self._marker_poses = []
        self._marker_image_paths = []
        self._marker_locations = []

        self._latest_image = None
        self._cv_bridge = CvBridge()

        # Subscribe to camera and ArUco markers
        self.create_subscription(Image, 'camera/image_raw', self._camera_callback, 10)
        self.create_subscription(ArucoMarkers, 'aruco_markers', self._marker_callback, 10)

        self._mission_started = False

        # --- NEW: instantiate VisionAgent here ---
        self.vision_agent = VisionAgent()

        # Delay start by 2 seconds, then run mission
        threading.Thread(target=self._delayed_start, daemon=True).start()


    def parse_json_objects(self, raw: str):
        """
        Given a string that may contain one or more JSON objects back‐to‐back (possibly separated by commas or whitespace),
        find each balanced { … } block, parse it with json.loads, and return a list of dicts. 
        If only one object is found, a list of length 1 is returned.
        
        Example:
        s = '{"contains_orange_light": true, "light_status": "unknown"}, {"contains_orange_light": true, "light_status": "on"}'
        parse_json_objects(s)
        # ⇒ [
        #      {"contains_orange_light": True, "light_status": "unknown"},
        #      {"contains_orange_light": True, "light_status": "on"}
        #    ]
        """
        objects = []
        brace_level = 0
        start_idx = None

        for i, ch in enumerate(raw):
            if ch == "{":
                if brace_level == 0:
                    # mark the start of a new JSON object
                    start_idx = i
                brace_level += 1

            elif ch == "}":
                brace_level -= 1
                if brace_level == 0 and start_idx is not None:
                    # we have a complete {...} block from start_idx to i
                    json_str = raw[start_idx : i + 1]
                    try:
                        parsed = json.loads(json_str)
                        objects.append(parsed)
                    except json.JSONDecodeError:
                        # If parsing fails, you can either skip or raise an error.
                        # Here we choose to skip invalid JSON fragments.
                        pass
                    start_idx = None

        return objects

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
        gps_global = (self.global_lat, self.global_lon) if hasattr(self, 'global_lat') else (None, None)

        h, w = img.shape[:2] if img is not None else (0, 0)
        half_box = 50

        existing_markers = {int(a['content']) for a in self.db.get_arucos(self.mission_id)}

        for idx, aruco_id in enumerate(msg.marker_ids):
            if aruco_id in existing_markers:
                continue

            pose = msg.poses[idx]

            # Use corners to crop around the marker
            marker_corners = None
            if img is not None and hasattr(msg, 'corners') and len(msg.corners) >= (idx+1)*8:
                # Each marker has 4 corners, each with (x, y): 4*2=8 values per marker
                start = idx * 8
                marker_corners = [
                    [msg.corners[start + 0], msg.corners[start + 1]],
                    [msg.corners[start + 2], msg.corners[start + 3]],
                    [msg.corners[start + 4], msg.corners[start + 5]],
                    [msg.corners[start + 6], msg.corners[start + 7]],
                ]
                xs = [int(round(pt[0])) for pt in marker_corners]
                ys = [int(round(pt[1])) for pt in marker_corners]
                x1, x2 = max(min(xs), 0), min(max(xs), w - 1)
                y1, y2 = max(min(ys), 0), min(max(ys), h - 1)
                # Add some padding
                pad = 25
                x1 = max(x1 - pad, 0)
                y1 = max(y1 - pad, 0)
                x2 = min(x2 + pad, w - 1)
                y2 = min(y2 + pad, h - 1)
                cropped = img[y1:y2, x1:x2]
                crop_path = os.path.abspath(f"aruco_{aruco_id}_{int(time.time())}.jpg")
                self.get_logger().info(f"Saving cropped image to {crop_path}")
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
            team="KNR PW",
            email="franek.jozwiak.stud@pw.edu.pl",
            pilot="Franek Jóźwiak",
            phone="724466512",
            mission_time=mission_time_str,
            mission_no=f"M{int(time.time())}",
            duration="0m",
            battery_before="98%",
            battery_after="-%",
            kp_index=0,
            infra_map="/data/img/mapa.jpg",
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

            if not self.send_goto_global(north, east, down):
                self.get_logger().error(f"Goto waypoint {idx} failed; skipping.")
                continue

            # # Wait until we arrive or until a marker is detected
            # while rclpy.ok():
            #     gps = self.get_gps()
            #     if gps is None:
            #         self.get_logger().warn("GPS unavailable; cannot check arrival.")
            #         break
            #     cur_north, cur_east, cur_down = gps
            #     if (abs(cur_north - north) <= 0.5 and
            #         abs(cur_east - east) <= 0.5 and
            #         abs(cur_down - down) <= 0.5):
            #         self.get_logger().info(f"Arrived at waypoint {idx}.")
            #         break
            #     if self._marker_event.is_set():
            #         break
            #     time.sleep(0.2)

            # If we found ArUco markers, save them
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
                        "image": img_path.split('/')[-1] if img_path else "",
                        "jury": "-"
                    }
                    to_store.append(record)

                try:
                    self.db.add_arucos(self.mission_id, to_store)
                    self.get_logger().info(f"Saved {len(to_store)} ArUco marker(s) to DB for waypoint {idx}")
                except Exception as e:
                    self.get_logger().error(f"Failed to save ArUco(s) to DB: {e}")

            # --- NEW: Capture a snapshot of the camera and run lamp detection in a separate thread ---
            current_img = self.get_camera_image()
            if current_img is not None:
                lamp_image_path = os.path.abspath(f"lamp_wp{idx}_{int(time.time())}.jpg")
                cv2.imwrite(lamp_image_path, current_img)
                self.get_logger().info(f"Saved waypoint {idx} image for lamp detection: {lamp_image_path}")

                # Launch a daemon thread so this is non-blocking
                threading.Thread(
                    target=self._run_lamp_detection,
                    args=(lamp_image_path, idx),
                    daemon=True
                ).start()
            else:
                self.get_logger().warn(f"No camera image to save for lamp detection at waypoint {idx}.")

            # Small pause before next waypoint
            time.sleep(1.0)



        # Fly to the orthophoto waypoint, then RTL
        self.send_goto_global(self.orto_waypoint[0], self.orto_waypoint[1], self.orto_waypoint[2])

        final_img = self.get_camera_image()
        if final_img is not None:
            ortho_path = os.path.abspath("mapa.jpg")
            cv2.imwrite(ortho_path, final_img)
            record = {
                "content": str(-1),
                "location": "",
                "location_changed": "-",
                "content_changed": "-",
                "image": ortho_path.split('/')[-1],
                "jury": "-"
            }
            try:
                self.db.add_arucos(self.mission_id, [record])
                self.get_logger().info("Saved final orthophoto to DB.")
            except Exception as e:
                self.get_logger().error(f"Failed to save final photomap to DB: {e}")

        self.rtl()

    def _run_lamp_detection(self, image_path: str, waypoint_idx: int):
        self.get_logger().info(f"Running lamp detection for waypoint {waypoint_idx} using image: {image_path}...")
        """
        Runs VisionAgent.describe_image(...) on the given image_path.
        This is launched in a separate thread so as not to block the main mission logic.
        """
        try:
            self.get_logger().info(f"(Thread) Starting lamp detection for waypoint {waypoint_idx} using {image_path}")
            detection_results = self.vision_agent.describe_image(image_path)
            detection_results = self.parse_json_objects(detection_results)
            self.get_logger().info(f"results : {detection_results}")
            if not detection_results:
                self.get_logger().warn(f"(Thread) No JSON output from VisionAgent for {image_path}")
                return

            for idx, result in enumerate(detection_results, start=1):
                contains = result.get("contains_orange_light", False)
                status = result.get("light_status", "unknown")
                self.get_logger().info(
                    f"(Thread) Waypoint {waypoint_idx} → JSON #{idx}: "
                    f"contains_orange_light={contains}, light_status={status}"
                )
                # Add to incidents if lamp detected
                status = "on"
                if status:
                    
                    event_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                    self.get_logger().info(f"event time: {event_time}")
                    incident_record = {
                        "event": f"Lamp detected (status: {status})",
                        "event_time": event_time,
                        "location": f"-",
                        "image": image_path.split('/')[-1] if image_path else "",
                        "notified": "Tak",  # or "Tak" if you want to mark as notified
                        "jury": "-"
                    }
                    try:
                        self.db.add_incidents(self.mission_id, [incident_record])
                        self.get_logger().info(f"Added lamp incident to DB for waypoint {waypoint_idx}.")
                    except Exception as e:
                        self.get_logger().error(f"Failed to add lamp incident to DB: {e}")
        except Exception as e:
            self.get_logger().error(f"(Thread) Lamp detection failed for {image_path}: {e}")

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
    waypoints = [(50.2715662, 18.6443051, alt),
        (50.2714623, 18.6442565, alt),
        # (50.2717372, 18.6440945, alt),
        # (50.2719005, 18.6444969, alt)
        ]
    orto_waypoint = [50.2719005, 18.6444969, alt+3]
    node = MissionRunner(waypoints, orto_waypoint)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
