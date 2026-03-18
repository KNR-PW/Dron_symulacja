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
    def __init__(self):
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
        self.save_counter = 0

    def camera_callback(self, msg):
        self.frame = self.br.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        if self.detecting:
            self.detect_pool()

    def detect_pool(self):
        if self.frame is None:
            return

        hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV) 
        lower_blue = np.array([90, 50, 50]) #trzeba będzie wymyśleć co innego pewnie, bo w rzeczywistości to nie bedzie niebieskie
        upper_blue = np.array([130, 255, 255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        found = False
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 500:
                found = True
                x, y, w, h = cv2.boundingRect(cnt)
                self.get_logger().info(f"Pool detected! | x: {x}, y: {y}, w: {w}, h: {h}")
                cv2.rectangle(self.frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        if found:
            filename = os.path.join(SAVE_DIR, f"detection_{self.save_counter:04d}.jpg")
            cv2.imwrite(filename, self.frame)
            self.get_logger().info(f"Saved: {filename}")
            self.save_counter += 1

    def scan(self, duration: float = 3.0):
        self.get_logger().info(f"Scanning for {duration}s...")
        self.detecting = True
        time.sleep(duration)
        self.detecting = False
        self.get_logger().info("Scan complete.")


def main(args=None):
    rclpy.init(args=args)

    mission = DroneController()
    detector = PoolDetector()

    executor = MultiThreadedExecutor()
    executor.add_node(mission)
    executor.add_node(detector)

    def mission_thread():
        mission.arm()
        mission.takeoff(10.0)

        waypoints = [
            ( -2.0, -8.0, 0.0),
            ( 0.0, 8.0, 0.0),
            ( 2.0, -8.0, 0.0),
            ( 0.0, 8.0, 0.0),
        ]

        for wp in waypoints:
            mission.send_goto_relative(*wp)
            time.sleep(5)
            detector.scan(3.0)

        mission.land()

    t = threading.Thread(target=mission_thread, daemon=True)
    t.start()

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        t.join(timeout=2.0)
        mission.destroy_node()
        detector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()