import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from drone_comunication import DroneController
import threading
import time
import os


SAVE_DIR = "/root/ros_ws/detections"
os.makedirs(SAVE_DIR, exist_ok=True)


class PoolDetector(Node):
    def __init__(self, mission: DroneController):
        super().__init__('pool_detector_node')
        self.declare_parameter('camera_topic', 'camera')
        self.declare_parameter('min_pool_area_px', 500)
        self.declare_parameter('save_dir', SAVE_DIR)

        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.min_area = self.get_parameter('min_pool_area_px').get_parameter_value().integer_value
        self.save_dir = self.get_parameter('save_dir').get_parameter_value().string_value
        os.makedirs(self.save_dir, exist_ok=True)

        self.subscription = self.create_subscription(
            Image,
            camera_topic,
            self.camera_callback,
            10)

        self.br = CvBridge()
        self.frame = None
        self.frame_lock = threading.Lock()
        self._detecting = threading.Event()

        self.pool_seen = False
        self.mission = mission
        self.save_counter = 0

    def camera_callback(self, msg):
        frame = self.br.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        with self.frame_lock:
            self.frame = frame

        if self._detecting.is_set():
            boxes = self.detect_pool()
            if boxes:
                self.pool_seen = True

    def detect_pool(self) -> list:
        with self.frame_lock:
            if self.frame is None:
                return []
            frame = self.frame.copy()

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([90, 50, 50])
        upper_blue = np.array([130, 255, 255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        boxes = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > self.min_area:
                x, y, w, h = cv2.boundingRect(cnt)
                self.get_logger().info(f"Pool detected! | x: {x}, y: {y}, w: {w}, h: {h}")
                boxes.append((x, y, w, h))

        return boxes

    def scan(self, duration: float = 3.0) -> bool:
        self.get_logger().info(f"Scanning for {duration}s...")
        self.pool_seen = False
        self._detecting.set()
        time.sleep(duration)
        self._detecting.clear()
        self.get_logger().info(f"Scan complete. Pool seen: {self.pool_seen}")
        return self.pool_seen

    def get_live_offset(self):
        """Zwraca (offset_x_px, offset_y_px, frame, boxes) z aktualnej klatki.
        """
        with self.frame_lock:
            if self.frame is None:
                return None
            frame = self.frame.copy()

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv,
                           np.array([90, 50, 50]),
                           np.array([130, 255, 255]))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        boxes = [cv2.boundingRect(c) for c in contours
                 if cv2.contourArea(c) > self.min_area]
        if not boxes:
            return None

        bx, by, bw, bh = max(boxes, key=lambda b: b[2] * b[3])
        fh, fw = frame.shape[:2]
        offset_x = (bx + bw // 2) - fw // 2
        offset_y = (by + bh // 2) - fh // 2
        return offset_x, offset_y, frame, boxes

    def save_annotated(self, frame, boxes, tag: str = ""):
        """Zapisuje klatke z narysowanymi bboxami"""
        annotated = frame.copy()
        for (x, y, w, h) in boxes:
            cv2.rectangle(annotated, (x, y), (x + w, y + h), (0, 255, 0), 2)
        suffix = f"_{tag}" if tag else ""
        fname = os.path.join(
            self.save_dir,
            f"detection_{self.save_counter:04d}{suffix}.jpg"
        )
        cv2.imwrite(fname, annotated)
        self.save_counter += 1
        self.get_logger().info(f"Saved: {fname}")
        return fname


def center_over_pool(mission: DroneController,
                     detector: PoolDetector,
                     tag: str = "") -> bool:
    """Centruje drona nad basenem.
    Po wycentrowaniu robi zdjecie."""

    # parametry velocity servoing
    Kp               = 0.004
    MAX_CORR         = 0.5
    CENTER_THRESH    = 50
    STABLE_NEEDED    = 10
    EMA              = 0.3
    APPROACH_TIMEOUT = 30.0

    mission.toggle_control()
    mission.send_vectors(0.0, 0.0, 0.0)
    time.sleep(2.0)

    sm_vx = sm_vy = 0.0
    stable_count = 0
    deadline = time.time() + APPROACH_TIMEOUT
    centered = False

    while time.time() < deadline:
        det = detector.get_live_offset()

        if det is None:
            mission.send_vectors(0.0, 0.0, 0.0)
            sm_vx = sm_vy = 0.0
            stable_count = 0
            time.sleep(0.05)
            continue

        offset_x, offset_y, frame, boxes = det
        mission.get_logger().info(
            f"[{tag}] Approach offset: dx={offset_x:+d} dy={offset_y:+d} | stable={stable_count}",
            throttle_duration_sec=0.5
        )

        if abs(offset_x) < CENTER_THRESH and abs(offset_y) < CENTER_THRESH:
            stable_count += 1
            sm_vx = (1 - EMA) * sm_vx
            sm_vy = (1 - EMA) * sm_vy
            mission.send_vectors(sm_vx, sm_vy, 0.0)

            if stable_count >= STABLE_NEEDED:
                mission.send_vectors(0.0, 0.0, 0.0)
                time.sleep(1.5)
                mission.get_logger().info(f"[{tag}] Hovering - saving photo.")
                detector.save_annotated(frame, boxes, tag=tag)
                centered = True
                break
        else:
            stable_count = 0
            vx_raw = max(-MAX_CORR, min(MAX_CORR, -Kp * offset_y))
            vy_raw = max(-MAX_CORR, min(MAX_CORR,  Kp * offset_x))
            sm_vx = EMA * vx_raw + (1 - EMA) * sm_vx
            sm_vy = EMA * vy_raw + (1 - EMA) * sm_vy
            mission.get_logger().info(
                f"[{tag}] img_offset=({offset_x:+d},{offset_y:+d}) | "
                f"send vx={sm_vx:+.3f} vy={sm_vy:+.3f}",
                throttle_duration_sec=0.5
            )
            mission.send_vectors(sm_vx, sm_vy, 0.0)

        time.sleep(0.05)

    if not centered:
        mission.get_logger().warn(f"[{tag}] Approach timeout.")

    mission.send_vectors(0.0, 0.0, 0.0)
    time.sleep(0.5)
    mission.toggle_control()
    return centered


def main(args=None):
    rclpy.init(args=args)

    mission = DroneController()
    detector = PoolDetector(mission)

    detector_executor = MultiThreadedExecutor()
    detector_executor.add_node(mission)
    detector_executor.add_node(detector)

    def mission_thread():
        mission.arm()
        mission.takeoff(10.0)

        waypoints = [
            ( -6.5, -6.5, 0.0),
            ( 13.0,  -4.0, 0.0),
            ( -10.0,-10.0, 0.0),
            (  14.0, 1.0, 0.0), 
        ]

        total = len(waypoints)

        for idx, wp in enumerate(waypoints, start=1):
            is_last = (idx == total)
            mission.get_logger().info(f" Pool {idx}/{total} -> waypoint {wp}")

            mission.send_goto_relative(*wp)
            time.sleep(5.0)

            found = detector.scan(3.0)
            if not found:
                mission.get_logger().warn(f"Pool {idx}: not detected.")
                continue

            mission.get_logger().info(f"Pool {idx}: detected.")
            center_over_pool(mission, detector, tag=f"pool{idx}")
            if is_last:
                mission.get_logger().info("Last pool.")
                mission.send_goto_relative(0.0, 0.0, 7.0)
                time.sleep(5.0)

                mission.get_logger().info("Hover na 3m...")
                time.sleep(3.0)
                det = detector.get_live_offset()
                if det is not None:
                    _, _, frame, boxes = det
                    detector.save_annotated(frame, boxes, tag=f"pool{idx}_close")

                mission.get_logger().info(" alt 10m.")
                mission.send_goto_relative(0.0, 0.0, -7.0)
                time.sleep(5.0)

        mission.get_logger().info("RTL.")
        mission.rtl()

    t = threading.Thread(target=mission_thread, daemon=True)
    t.start()

    try:
        detector_executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        t.join(timeout=2.0)
        mission.destroy_node()
        detector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()