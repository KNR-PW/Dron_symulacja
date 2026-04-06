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


def scale_intrinsics(K, original_size, new_size):
    """Scale camera intrinsic matrix K from original_size to new_size.
    """
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

        self.photo_taken = False
        self.mission = mission
        self.save_counter = 0

        # Camera calibration for webots
        self.calibration_img_shape = (640, 480)  
        self.camera_intrinsics = np.array(
            [
                [772.74,   0.0,  320.0], #basicowo w webotsie jest fx=fy=320/tan(FOV/2)
                [  0.0,  772.74, 240.0],
                [  0.0,    0.0,    1.0],
            ]
        )
        self.pending_goto = None

    def camera_callback(self, msg):
        frame = self.br.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        with self.frame_lock:
            self.frame = frame

        if self._detecting.is_set():
            boxes = self.detect_pool()
            if boxes:
                self.handle_detections(boxes)

    def detect_pool(self) -> list:
        with self.frame_lock:
            if self.frame is None or self.photo_taken:
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
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                boxes.append((x, y, w, h))

        # save detections frame 
        if boxes:
            with self.frame_lock:
                self.frame = frame

        return boxes

    def handle_detections(self, boxes: list):
        biggest = max(boxes, key=lambda b: b[2] * b[3])
        bx, by, bw, bh = biggest

        with self.frame_lock:
            frame_h, frame_w = self.frame.shape[:2]
            frame_copy = self.frame.copy()

        frame_cx = frame_w // 2
        frame_cy = frame_h // 2
        offset_x = (bx + bw // 2) - frame_cx
        offset_y = (by + bh // 2) - frame_cy
        self.get_logger().info(f"Offset from center: dx={offset_x}, dy={offset_y}")

        if not self.photo_taken:
            filename = os.path.join(self.save_dir, f"detection_{self.save_counter:04d}.jpg")
            cv2.imwrite(filename, frame_copy)
            self.get_logger().info(f"Saved: {filename}")
            self.save_counter += 1
            self.photo_taken = True
            self._detecting.clear()

        if abs(offset_x) > 20 or abs(offset_y) > 20:
            scaled_intrinsics = scale_intrinsics(
                self.camera_intrinsics,
                self.calibration_img_shape,          # (640, 480) — (w, h)
                (frame_w, frame_h),                  # current frame (w, h)
            )
            alt = self.mission.alt
            X_rel, Y_rel = image_to_ground(
                (frame_cx + offset_x, frame_cy + offset_y),
                scaled_intrinsics,
                alt,
            )
            self.pending_goto = (-Y_rel, X_rel, 0.0)

    def scan(self, duration: float = 3.0) -> bool:
        """Scan for a pool for `duration` seconds.
        """
        self.get_logger().info(f"Scanning for {duration}s...")
        self._detecting.set()
        time.sleep(duration)
        self._detecting.clear()
        self.get_logger().info("Scan complete.")
        return self.photo_taken


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
            (-2.0, -8.0, 0.0),
            ( 0.0,  8.0, 0.0),
            ( 2.0, -8.0, 0.0),
            ( 0.0,  8.0, 0.0),
        ]

        for wp in waypoints:
            mission.send_goto_relative(*wp)
            time.sleep(5.0)

            found = detector.scan(3.0)
            if found:
                mission.get_logger().info("Pool confirmed.")
                if detector.pending_goto is not None:
                    mission.get_logger().info("Repositioning over pool center...")
                    mission.send_goto_relative(*detector.pending_goto)
                    detector.pending_goto = None
                mission.get_logger().info("Descending 5 m over pool...")
                mission.send_goto_relative(0.0, 0.0, 5.0)  
                time.sleep(5.0)
                break

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
