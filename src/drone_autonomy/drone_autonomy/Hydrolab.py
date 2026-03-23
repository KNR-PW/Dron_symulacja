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


SAVE_DIR = "/root/ros_ws/detections" #zapis wewnątrz dockera 
os.makedirs(SAVE_DIR, exist_ok=True)



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

class PoolDetector(Node):
    def __init__(self, mission: DroneController):
        super().__init__('pool_detector_node')
        self.declare_parameter('camera_topic', 'camera')
        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value

        self.subscription = self.create_subscription(
            Image,
            camera_topic,
            self.camera_callback,
            10)

        self.br = CvBridge()
        self.frame = None
        self.detecting = False
        self.photo_taken = False
        self.mission = mission
        self.save_counter = 0
        self.calibration_img_shape = (640, 480) #trzeba zmienić na faktyczny rozmiar bo sie wywala
        self.camera_intrinsics = np.array(  
            [
                [640.0,   0.0, 320.0],          
                [  0.0, 640.0, 240.0],
                [  0.0,   0.0,   1.0],
            ]
        )
        self.pending_goto = None

    def camera_callback(self, msg):
        self.frame = self.br.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        if self.detecting:
            boxes = self.detect_pool()
            if boxes:
                self.handle_detections(boxes)

    def detect_pool(self) -> list[tuple[int, int, int, int]]:
        if self.frame is None or self.photo_taken:
            return []

        hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([90, 50, 50])  # trzeba będzie wymyśleć co innego pewnie, bo w rzeczywistości to nie bedzie niebieskie
        upper_blue = np.array([130, 255, 255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        boxes = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 500:
                x, y, w, h = cv2.boundingRect(cnt)
                self.get_logger().info(f"Pool detected! | x: {x}, y: {y}, w: {w}, h: {h}")
                cv2.rectangle(self.frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                boxes.append((x, y, w, h))

        return boxes

    def handle_detections(self, boxes: list[tuple[int, int, int, int]]):
        biggest = max(boxes, key=lambda b: b[2] * b[3])
        bx, by, bw, bh = biggest

        frame_cx = self.frame.shape[1] // 2
        frame_cy = self.frame.shape[0] // 2
        offset_x = (bx + bw // 2) - frame_cx
        offset_y = (by + bh // 2) - frame_cy
        self.get_logger().info(f"Offset from center: dx={offset_x}, dy={offset_y}")

        if not self.photo_taken:
            filename = os.path.join(SAVE_DIR, f"detection_{self.save_counter:04d}.jpg")
            cv2.imwrite(filename, self.frame)
            self.get_logger().info(f"Saved: {filename}")
            self.save_counter += 1
            self.photo_taken = True
            self.detecting = False

        if abs(offset_x) > 20 or abs(offset_y) > 20:
            scaled_intrinsics = scale_intrinsics(
                self.camera_intrinsics,
                self.calibration_img_shape,
                self.frame.shape[:2],
            )
            alt = self.mission.alt
            X_rel, Y_rel = image_to_ground(
                (frame_cx + offset_x, frame_cy + offset_y),
                scaled_intrinsics,
                alt,
            )
            self.pending_goto = (-Y_rel, X_rel, 0.0) 

    def scan(self, duration: float = 3.0):  #zmienić żeby robił zdjęcie po wycentrowaniu
        if self.photo_taken:                            
            self.get_logger().info("Photo already taken, skipping scan.")
            return

        self.get_logger().info(f"Scanning for {duration}s...")
        self.detecting = True
        time.sleep(duration)
        self.detecting = False
        self.get_logger().info("Scan complete.")


def main(args=None):
    rclpy.init(args=args)

    mission = DroneController()
    detector = PoolDetector(mission)

    detector_executor = MultiThreadedExecutor()
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
            deadline = time.time() + 5.0
            while time.time() < deadline:
                rclpy.spin_once(mission, timeout_sec=0.1)
            detector.scan(3.0)
            if detector.pending_goto is not None:
                mission.send_goto_relative(*detector.pending_goto)
                detector.pending_goto = None

        mission.land()

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