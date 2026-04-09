#!/usr/bin/env python3
"""Klasa bazowa dla detektorow YOLO (wspolna logika SIM i JETSON)."""

import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO

from drone_interfaces.msg import TentDetection


class YoloDetectorBase(Node):
    """Wspolna logika: subskrypcja kamery, publikacja detekcji i podglad debug."""

    def __init__(self, node_name: str, default_model: str, track_kwargs: dict = None):
        super().__init__(node_name)

        # --- parametry ROS ---
        self.declare_parameter("camera_topic", "/rgb_camera/image")
        self.declare_parameter("model_path", default_model)
        self.declare_parameter("conf", 0.5)
        self.declare_parameter("input_size", 1024)

        model_path = self.get_parameter("model_path").value
        self.conf = self.get_parameter("conf").value
        self.input_size = self.get_parameter("input_size").value
        cam_topic = self.get_parameter("camera_topic").value

        # --- model YOLO ---
        self.get_logger().info(f"Loading model: {model_path}")
        self.model = YOLO(model_path, task="detect")
        self.br = CvBridge()

        # dodatkowe flagi przekazywane do model.track() (np. half=True)
        self._track_kwargs = track_kwargs or {}

        # --- pub / sub ---
        self.pub = self.create_publisher(TentDetection, "/tent_detections", 10)
        self.img_pub = self.create_publisher(Image, "/tent_detections/image", 1)
        self.create_subscription(Image, cam_topic, self._on_image, 10)

        # --- FPS counter ---
        self._frames = 0
        self._detected_frames = 0
        self._t0 = time.monotonic()

        self.get_logger().info(f"{node_name} ready  |  input_size={self.input_size}")

    # ------------------------------------------------------------------
    def _on_image(self, msg: Image):
        frame = self.br.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        results = self.model.track(
            frame,
            conf=self.conf,
            persist=True,
            verbose=False,
            imgsz=self.input_size,
            **self._track_kwargs,
        )

        # --- publikuj detekcje ---
        det = TentDetection()
        det.detected = False
        det.bounding_box = [0.0, 0.0, 0.0, 0.0]
        det.confidence = 0.0

        if results and len(results[0].boxes) > 0:
            box = results[0].boxes[0]
            x1, y1, x2, y2 = map(float, box.xyxy[0].tolist())
            det.detected = True
            det.bounding_box = [x1, y1, x2 - x1, y2 - y1]
            det.confidence = float(box.conf[0])

        self.pub.publish(det)

        # --- podglad debug (tylko gdy ktos subskrybuje) ---
        if self.img_pub.get_subscription_count() > 0:
            if det.detected:
                import cv2
                x, y, w, h = float(det.bounding_box[0]), float(det.bounding_box[1]), float(det.bounding_box[2]), float(det.bounding_box[3])
                cv2.rectangle(frame, (int(x), int(y)), (int(x+w), int(y+h)), (0, 255, 0), 2)
                cv2.putText(frame, f"{det.confidence:.2f}", (int(x), int(y)-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            self.img_pub.publish(self.br.cv2_to_imgmsg(frame, encoding="bgr8"))

        # --- FPS ---
        self._frames += 1
        if det.detected:
            self._detected_frames += 1

        dt = time.monotonic() - self._t0
        if dt >= 2.0:
            fps = self._frames / dt
            self.get_logger().info(f"FPS: {fps:.1f}  ({1000/fps:.0f} ms/frame) | Detekcje namiotu: {self._detected_frames}/{self._frames}")
            self._frames = 0
            self._detected_frames = 0
            self._t0 = time.monotonic()
