#!/usr/bin/env python3
"""
person_json_to_db_node.py

- Publishes any new camera JPEG (from a specified folder) as a ROS Image topic.
- Whenever a new JSON file appears (named "<n>.json") in a specified folder,
  reads the "person" object(s) out of it, calls the GPS service once, and writes
  each detected person to the `employees` table of the latest mission.

Usage (as a ROS 2 Python node):
    ros2 run <your_package> person_json_to_db_node
    # or, if you run directly:
    python3 person_json_to_db_node.py
"""

import os
import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from droniada_inspekcja.inspekcja_db import DroneDB
from drone_interfaces.srv import GetLocationRelative
import time
# -----------------------------------------------------------------------------
# Helper: find the maximum integer‐named file in a directory
# -----------------------------------------------------------------------------
def find_latest_index_in_dir(path: str, extension: str) -> int:
    """
    Look for files in 'path' named '<number>.<extension>' (e.g. '12.json').
    Return the maximum integer <number> found, or -1 if none exist.
    """
    max_idx = -1
    try:
        for fname in os.listdir(path):
            # print(f"fname: {fname}")
            if not fname.endswith(f".{extension}"):
                continue
            stem = fname[:-(len(extension) + 1)]  # strip ".json" or ".jpg"
            if extension == "json":
                stem = stem.split("json")[-1]
            elif extension == "jpg":
                stem = stem.split("photo")[-1]
            # print(f"stem: {stem}")
            if not stem.isdigit():
                continue
            idx = int(stem)
            if idx > max_idx:
                max_idx = idx
    except (FileNotFoundError, PermissionError):
        return -1

    return max_idx


# -----------------------------------------------------------------------------
# Our ROS 2 Node
# -----------------------------------------------------------------------------
class PersonJsonToDbNode(Node):
    def __init__(self):
        super().__init__('person_json_to_db_node')

        # === PARAMETERS (hard‐coded for now) ===
        self.declare_parameter("camera_topic", "camera")
        # self.declare_parameter("photo_dir", "/home/knr/Dron_symulacja/hailo/photo")
        # self.declare_parameter("json_dir", "/home/knr/Dron_symulacja/hailo/json")

        self.declare_parameter("photo_dir", "/home/stas/KNR/tests/photo")
        self.declare_parameter("json_dir", "/home/stas/KNR/tests/json")

        self.declare_parameter("db_path", "drone_data.db")

        self.photo_dir = self.get_parameter("photo_dir").get_parameter_value().string_value
        self.json_dir = self.get_parameter("json_dir").get_parameter_value().string_value
        self.db_path = self.get_parameter("db_path").get_parameter_value().string_value
        self.camera_topic = self.get_parameter("camera_topic").get_parameter_value().string_value
        print(self.db_path)
        # self.photo_dir = "/root/ros_ws/hailo/photo"
        # self.json_dir = "/root/ros_ws/hailo/json"
        # self.db_path = "drone_data.db"

        # 1) Publisher for camera images
        self.publisher_ = self.create_publisher(Image, self.camera_topic, 10)
        self.br = CvBridge()

        # 2) GPS client setup
        self.gps_client = self.create_client(GetLocationRelative, 'get_location_relative')

        self.get_logger().info(f"waiting for gps service")
        self.gps_client.wait_for_service()

        # if not self.gps_client.wait_for_service(timeout_sec=2.0):
        #     self.get_logger().error("GPS service 'get_location_relative' not available. Shutting down.")
        #     rclpy.shutdown()
        #     return

        # 3) Initialize the DB (creates schema if needed)
        try:
            self.db = DroneDB(self.db_path, ensure_schema=False)
        except Exception as e:
            self.get_logger().error(f"Failed to initialize DroneDB: {e}")
            rclpy.shutdown()
            return

        # 4) Always grab the latest mission ID rather than creating a new one:
        with self.db._get_connection() as conn:
            row = conn.execute("SELECT MAX(id) AS maxid FROM missions").fetchone()
            latest_id = row["maxid"] if row and row["maxid"] is not None else None

        if latest_id is None:
            self.get_logger().error("No missions exist in the database. Cannot proceed.")
            rclpy.shutdown()
            return

        self.mission_id = latest_id
        self.get_logger().info(f"Using latest mission_id={self.mission_id}")

        # 5) Track which indices we’ve already processed
        self.last_photo_idx = -1
        self.last_json_idx = -1

        # 6) Create a timer to check for new files every 0.1 seconds
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info("PersonJsonToDbNode started (publishing camera + inserting employees)")

    def timer_callback(self):
        """
        Called every 0.1 seconds. Checks for:
          1) A new camera JPEG (publish it).
          2) A new JSON file (<n>.json), read it, extract 'person' data,
             get GPS once, and insert into DroneDB.employees under self.mission_id.
        """
        with self.db._get_connection() as conn:
            row = conn.execute("SELECT MAX(id) AS maxid FROM missions").fetchone()
            latest_id = row["maxid"] if row and row["maxid"] is not None else None

        if latest_id is None:
            self.get_logger().error("No missions exist in the database. Cannot proceed.")
            return

        self.mission_id = latest_id
        # ——————————————————————
        # 1) Publish any NEW JPEG
        # ——————————————————————
        latest_photo_idx = find_latest_index_in_dir(self.photo_dir, "jpg")
        if latest_photo_idx > self.last_photo_idx:
            jpg_path = os.path.join(self.photo_dir, f"photo{latest_photo_idx}.jpg")
            if os.path.isfile(jpg_path):
                frame = cv2.imread(jpg_path)
                if frame is not None:
                    ros_img = self.br.cv2_to_imgmsg(frame, encoding="bgr8")
                    self.publisher_.publish(ros_img)
                    self.last_photo_idx = latest_photo_idx
                    self.get_logger().info(f"Published new image #{latest_photo_idx}.jpg")
                else:
                    self.get_logger().warn(f"Failed to read image at {jpg_path}")
            else:
                self.get_logger().warn(f"Expected JPEG not found: {jpg_path}")

        # ——————————————————————
        # 2) Process any NEW JSON
        # ——————————————————————
        latest_json_idx = find_latest_index_in_dir(self.json_dir, "json")
        self.get_logger().info(f"latest_json_idx: {latest_json_idx}")
        if latest_json_idx > self.last_json_idx:
            json_path = os.path.join(self.json_dir, f"json{latest_json_idx}.json")
            self.get_logger().info(f"json_path: {json_path}")
            if os.path.isfile(json_path):
                self.get_logger().info(f"siur")
                try:
                    with open(json_path, "r", encoding="utf-8") as f:
                        data = json.load(f)
                except Exception as e:
                    self.get_logger().error(f"Failed to load JSON '{json_path}': {e}")
                    # Even if parsing fails, mark it as handled to avoid infinite loop
                    self.last_json_idx = latest_json_idx
                    return

                # Validate JSON structure
                if not isinstance(data, dict) or "objects" not in data:
                    self.get_logger().error(f"Unexpected JSON structure in {json_path}; skipping.")
                    self.last_json_idx = latest_json_idx
                    return

                # Call GPS service ONCE
                # gps_tuple = self.get_gps()
                gps_tuple = (0, 0, 0)
                if gps_tuple is None:
                    # GPS failed; still mark this JSON as done to avoid retry
                    self.last_json_idx = latest_json_idx
                    return
                (north, east, down) = gps_tuple
                gps_str = f"{down} {east} {north}"

                # Extract all “person” objects
                person_objs = [obj for obj in data["objects"] if obj.get("class") == "person"]
                if not person_objs:
                    self.get_logger().info(f"No 'person' entries in {json_path}")
                    self.last_json_idx = latest_json_idx
                    return

                # Build a list of rows to insert into employees
                to_insert = []
                for p in person_objs:
                    # Each p should have keys:
                    #   "class" = "person", "id", "x", "y", "width", "height", "hat", "vest"
                    px = int(p.get("x", 0))
                    py = int(p.get("y", 0))
                    pw = int(p.get("width", 0))
                    ph = int(p.get("height", 0))
                    hat_label = p.get("hat", "none")
                    vest_label = p.get("vest", "none")

                    # present = "Jest"
                    present = "Jest"

                    # bhp = "Tak" if they have hat=="hat" or vest=="vest", else "Nie"
                    has_hat = (hat_label == "hat")
                    has_vest = (vest_label == "vest")
                    bhp = "Tak" if (has_hat or has_vest) else "Nie"

                    # location = the GPS string
                    location = gps_str

                    # We do not track movement here, so:
                    location_changed = "Nie"

                    # The image path from the same index (if it exists)
                    corresponding_jpg = os.path.join(self.photo_dir, f"{latest_json_idx}.jpg")
                    image_field = corresponding_jpg if os.path.isfile(corresponding_jpg) else ""

                    # jury default
                    jury = "None"

                    row = {
                        "present": present,
                        "bhp": bhp,
                        "location": location,
                        "location_changed": location_changed,
                        "image": image_field,
                        "jury": jury
                    }
                    to_insert.append(row)

                # Insert into employees table (under self.mission_id)
                self.get_logger().info(f"{to_insert}")
                try:
                    self.db.add_employees(self.mission_id, to_insert)
                    self.get_logger().info(f"Inserted {len(to_insert)} employee(s) from {json_path}")
                except Exception as e:
                    self.get_logger().error(f"Failed to insert employees from {json_path}: {e}")

                # Mark this JSON as handled
                self.last_json_idx = latest_json_idx

            else:
                self.get_logger().warn(f"Expected JSON not found: {json_path}")

    def get_gps(self):
        """
        Calls the GetLocationRelative service once, returns (north, east, down) or None on failure.
        """
        req = GetLocationRelative.Request()
        fut = self.gps_client.call_async(req)
        # Wait up to 2 seconds for a response
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
        if fut.result() is None:
            self.get_logger().error("GPS service call returned no result")
            return None
        return (fut.result().north, fut.result().east, fut.result().down)


def main():
    rclpy.init()
    node = PersonJsonToDbNode()
    # If initialization failed (e.g. no GPS or no mission), node will call rclpy.shutdown()
    if rclpy.ok():
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
