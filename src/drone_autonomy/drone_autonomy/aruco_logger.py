#!/usr/bin/env python3
import os
import threading
import time
from datetime import datetime

import rclpy
from rclpy.node import Node

from drone_interfaces.msg import ArucoMarkers
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from droniada_inspekcja.inspekcja_db import DroneDB


class ArucoLogger(Node):
    """
    A ROS 2 node that:
      1) Creates a “mission” row in the SQLite DB at startup.
      2) Subscribes to ArUco detections (aruco_markers topic).
      3) Whenever a new marker is detected (i.e. first time we see that ID), saves:
         - marker ID
         - marker pose (as a text field)
         - (optionally) a cropped image around the marker
         - timestamp
         into the DB under that mission ID.
    """

    def __init__(self):
        super().__init__('aruco_logger')

        # 1) Open (or create) the SQLite DB via your DroneDB wrapper
        db_path = os.path.join(os.getcwd(), 'drone_data.db')
        self.db = DroneDB(db_path)

        # 1a) Insert a new “mission” row so that arucos can reference it.
        #     Fill in placeholders as needed; you can adjust these fields later.
        mission_time_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        try:
            self.mission_id = self.db.add_mission(
                team="NoControlLogger",
                email="logger@example.com",
                pilot="LoggerNode",
                phone="000-000-0000",
                mission_time=mission_time_str,
                mission_no=f"LOG{int(time.time())}",
                duration="0m",
                battery_before="N/A",
                battery_after="N/A",
                kp_index=0,
                infra_map="/static/img/mapa.jpg",
            )
            self.get_logger().info(f"[DB] Created mission ID {self.mission_id} for ArUco logging")
        except Exception as e:
            self.get_logger().error(f"[DB] Failed to create mission row: {e}")
            raise

        # 2) State for handling ArUco callback
        self._processed_marker_ids = set()  # so that we only insert each marker once
        self._marker_lock = threading.Lock()

        # 3) Subscribe to camera (so we can grab & crop images around detected markers)
        self._latest_image = None
        self._cv_bridge = CvBridge()
        self.create_subscription(
            Image,
            'camera',
            self._camera_callback,
            10
        )

        # 4) Subscribe to ArUco detections
        self.create_subscription(
            ArucoMarkers,
            'aruco_markers',
            self._marker_callback,
            10
        )

        self.get_logger().info("ArucoLogger node ready: waiting for ArUco detections...")

    def _camera_callback(self, msg: Image):
        """
        Simply convert the incoming Image msg into a CV2 array and store it.
        Overwrites previous frame; when a marker arrives, we’ll crop from the latest.
        """
        try:
            # Always request 'bgr8' so we get a standard OpenCV format
            cv_img = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self._latest_image = cv_img
        except Exception as e:
            self.get_logger().error(f"Failed to convert camera image: {e}")

    def _marker_callback(self, msg: ArucoMarkers):
        """
        Called whenever ArUco markers are published. For each marker in msg.marker_ids,
        if we haven’t already logged that ID (in this Node’s lifetime), save it to the DB.
        """
        # If no markers detected, do nothing
        if not msg.marker_ids:
            return

        # We’ll loop through all detected IDs in this message; you can filter to just msg.marker_ids[0] if you want
        for idx, marker_id in enumerate(msg.marker_ids):
            with self._marker_lock:
                if marker_id in self._processed_marker_ids:
                    # Already logged this ID once—skip it
                    continue
                self._processed_marker_ids.add(marker_id)

            # Extract the corresponding pose
            try:
                marker_pose = msg.poses[idx]
            except IndexError:
                self.get_logger().warn(
                    f"Marker ID {marker_id} detected, but no corresponding pose in msg.poses"
                )
                marker_pose = None

            # 1) Build a text‐field "location" from the pose (if available)
            if marker_pose is not None:
                px = marker_pose.position.x
                py = marker_pose.position.y
                pz = marker_pose.position.z
                location_str = f"x={px:.2f},y={py:.2f},z={pz:.2f}"
            else:
                location_str = ""

            # 2) Attempt to crop & save the latest camera image around the marker
            #    (Assumes marker_pose.x/y are already pixel coords with origin at top-left;
            #     if your ArUco node publishes meter-based poses, skip cropping entirely.)
            image_path = ""
            if self._latest_image is not None and marker_pose is not None:
                try:
                    # Convert pose.x/y → integer pixel indices
                    # If your ArUco node uses bottom-left origin, uncomment the flip logic below:
                    x_px = int(round(marker_pose.position.x))
                    y_px = int(round(marker_pose.position.y))
                    # If origin is bottom-left, do:
                    # y_px = self._latest_image.shape[0] - y_px

                    # Define a crop‐box around (x_px, y_px)
                    box_size = 180        # size of square region to capture (in pixels)
                    padding = 60          # extra padding around that box
                    half = box_size // 2

                    x1 = max(x_px - half - padding, 0)
                    y1 = max(y_px - half - padding, 0)
                    x2 = min(x_px + half + padding, self._latest_image.shape[1])
                    y2 = min(y_px + half + padding, self._latest_image.shape[0])

                    if x2 > x1 and y2 > y1:
                        crop = self._latest_image[y1:y2, x1:x2]
                    else:
                        # If the crop region is invalid (marker near edge), just save the full image
                        crop = self._latest_image.copy()
                        self.get_logger().warn(
                            f"Marker {marker_id} near edge: invalid crop box ({x1},{y1},{x2},{y2}), using full image"
                        )

                    # Save to disk
                    timestamp = int(time.time())
                    filename = f"aruco_{marker_id}_{timestamp}.png"
                    path = os.path.join(os.getcwd(), filename)
                    cv2.imwrite(path, crop)
                    image_path = path
                    self.get_logger().info(f"Saved ArUco {marker_id} image to {path}")
                except Exception as e:
                    self.get_logger().error(f"Failed to crop/save image for marker {marker_id}: {e}")
                    image_path = ""

            # 3) Build the DB record
            aruco_record = {
                "content": str(marker_id),
                "location": location_str,
                "location_changed": "Nie",
                "content_changed": "Nie",
                "image": image_path,
                "jury": "None"
            }

            # 4) Insert into the DB
            try:
                self.db.add_arucos(self.mission_id, [aruco_record])
                self.get_logger().info(f"[DB] Logged ArUco {marker_id} (mission {self.mission_id})")
            except Exception as e:
                self.get_logger().error(f"[DB] Failed to insert ArUco {marker_id}: {e}")

        # end for each marker


def main(args=None):
    rclpy.init(args=args)
    node = ArucoLogger()

    # Use a single-threaded executor: all we need is to process subscriptions
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down ArucoLogger (keyboard interrupt)")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
