#!/usr/bin/env python3
"""Klasa bazowa dla detektorow YOLO (wspolna logika SIM i JETSON)."""

import time
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO

from drone_interfaces.msg import TentDetection

# ── 1. STALE ────────────────────────────────────────────────────────
DEBUG_MAX_WIDTH = 960   # szerokosc obrazu debug (px); mniejszy = tanszy encode/transfer
DEBUG_EVERY_N = 2       # publikuj co N-ta klatke debug (1 = kazda)


class YoloDetectorBase(Node):
    """Wspolna logika: subskrypcja kamery, publikacja detekcji i podglad debug."""

    def __init__(self, node_name: str, default_model: str, track_kwargs: dict = None):
        super().__init__(node_name)

        # ── 2. PARAMETRY ROS ────────────────────────────────────────
        self.declare_parameter("camera_topic", "/rgb_camera/image")
        self.declare_parameter("model_path", default_model)
        self.declare_parameter("conf", 0.5)
        self.declare_parameter("input_size", 1024)

        model_path = self.get_parameter("model_path").value
        self.conf = self.get_parameter("conf").value
        self.input_size = self.get_parameter("input_size").value
        cam_topic = self.get_parameter("camera_topic").value

        # ── 3. MODEL YOLO ───────────────────────────────────────────
        self.get_logger().info(f"Loading model: {model_path}")
        self.model = YOLO(model_path, task="detect")
        self.br = CvBridge()
        self._track_kwargs = track_kwargs or {}

        # ── 4. PUB / SUB ────────────────────────────────────────────
        self.pub = self.create_publisher(TentDetection, "/tent_detections", 10)
        self.img_pub = self.create_publisher(Image, "/tent_detections/image", 1)
        cam_qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, cam_topic, self._on_image, cam_qos)

        # ── 5. LICZNIKI ─────────────────────────────────────────────
        self._frames = 0
        self._detected_frames = 0
        self._t0 = time.monotonic()

        self.get_logger().info(f"{node_name} ready  |  input_size={self.input_size}")

    # ────────────────────────────────────────────────────────────────
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

        # ── 6. PUBLIKACJA DETEKCJI ──────────────────────────────────
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

        # ── 7. PODGLAD DEBUG (pomniejszony, co N-ta klatka) ─────────
        if self.img_pub.get_subscription_count() > 0 and self._frames % DEBUG_EVERY_N == 0:
            dbg = frame
            if det.detected:
                x, y, w, h = det.bounding_box
                cv2.rectangle(dbg, (int(x), int(y)), (int(x + w), int(y + h)), (0, 255, 0), 2)
                cv2.putText(dbg, f"{det.confidence:.2f}", (int(x), int(y) - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            if dbg.shape[1] > DEBUG_MAX_WIDTH:
                scale = DEBUG_MAX_WIDTH / dbg.shape[1]
                dbg = cv2.resize(dbg, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
            self.img_pub.publish(self.br.cv2_to_imgmsg(dbg, encoding="bgr8"))

        # ── 8. FPS ──────────────────────────────────────────────────
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