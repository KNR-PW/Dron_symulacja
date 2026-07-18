import os
import threading
import time

import cv2
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO

from drone_autonomy.drone_comunication import DroneController
from drone_detector.detection import Detection

ALT = 20.0

CENTER_NORTH = 10.0
CENTER_EAST = -10.0
CENTER_DOWN = -16.0

SQUARE_SIDE = 5
LOST_SECONDS = 2.0


MODEL_PATH = os.path.expanduser(
    '~/ros_ws/src/drone_detector/drone_detector/yolo_models/drom.pt')
VIDEO_OUT = os.path.expanduser(
    '~/ros_ws/src/drone_detector/video_files/swarm_detekcja.mp4')


class DroneYoloDetector(Node):

    def __init__(self):
        super().__init__('drone_yolo_detector')
        self.declare_parameter('camera_topic', 'camera')
        self.declare_parameter('yolo_model', MODEL_PATH)
        self.declare_parameter('conf_threshold', 0.25)
        self.declare_parameter('video_out', VIDEO_OUT)
        self.declare_parameter('video_fps', 20.0)

        model_path = self.get_parameter('yolo_model').get_parameter_value().string_value
        self.get_logger().info(f'Ładowanie modelu YOLO: {model_path}')
        self.model = YOLO(model_path, task='detect')
        self.conf_threshold = float(self.get_parameter('conf_threshold').value)
        self.video_out = self.get_parameter('video_out').get_parameter_value().string_value
        self.video_fps = float(self.get_parameter('video_fps').value)

        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.get_logger().info(f'camera_topic: {camera_topic}')
        self.subscription = self.create_subscription(
            Image,
            camera_topic,
            self.listener_callback,
            10)
        self.br = CvBridge()

        self.active = False
        self.detections = []      
        self.writer = None
        self.frames = 0         
        self.frames_with_det = 0  #Klatki z detekcją
        self.confs = []           
        self.last_seen = 0.0
        self._lock = threading.Lock()
        self.create_timer(0.2, self._check_lost)

    def start(self):
        self.detections.clear()
        self.frames = 0
        self.frames_with_det = 0
        self.confs.clear()
        self.last_seen = time.monotonic()
        self.active = True
        self.get_logger().info('Detekcja WŁĄCZONA')

    def stop(self):
        self.active = False
        with self._lock:
            if self.writer is not None:
                self.writer.release()
                self.writer = None
        self.get_logger().info('Detekcja WYŁĄCZONA')

        pct = 100 * self.frames_with_det / self.frames if self.frames else 0
        avg_conf = sum(self.confs) / len(self.confs) if self.confs else 0
        self.get_logger().info(f'Klatek łącznie:   {self.frames}')
        self.get_logger().info(f'Klatek z dronem:  {self.frames_with_det} ({pct:.1f}%)')
        self.get_logger().info(f'Detekcji łącznie: {len(self.confs)}')
        self.get_logger().info(f'Średnia pewność:  {avg_conf:.3f}')
        self.get_logger().info(f'Zapisano film:    {self.video_out}')

    def listener_callback(self, data):
        if not self.active:
            return

        image = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')
        result = self.model.predict(image, conf=self.conf_threshold, verbose=False)[0]
        annotated = result.plot()

        with self._lock:
            if not self.active:
                return
            if self.writer is None:
                os.makedirs(os.path.dirname(self.video_out), exist_ok=True)
                h, w = annotated.shape[:2]
                self.writer = cv2.VideoWriter(
                    self.video_out, cv2.VideoWriter_fourcc(*'mp4v'),
                    self.video_fps, (w, h))
            self.frames += 1
            if len(result.boxes) > 0:
                self.frames_with_det += 1
                self.confs.extend(float(c) for c in result.boxes.conf)
                self.last_seen = time.monotonic()
            self.writer.write(annotated)

        now = time.monotonic()
        for box in result.boxes:
            cls_id = int(box.cls)
            conf = float(box.conf)
            class_name = self.model.names[cls_id]
            x1, y1, x2, y2 = map(float, box.xyxy[0])
            middle_x = (x1 + x2) / 2
            middle_y = (y1 + y2) / 2

            detection = Detection(
                bounding_box=(int(x1), int(y1), int(x2 - x1), int(y2 - y1)),
                color=class_name)
            self.detections.append((now, detection))
            self.get_logger().info(
                f'Wykryto: {class_name} conf={conf:.2f} środek=({middle_x:.0f}, {middle_y:.0f})')

    def seconds_since_seen(self):
        return time.monotonic() - self.last_seen

    def _check_lost(self):
        if self.active and self.seconds_since_seen() >= LOST_SECONDS:
            self.get_logger().info(
                f'Dron zgubiony {self.seconds_since_seen():.1f}')
            self.stop()


def kwadrat():
    corners = [(1, 1), (1, -1), (-1, -1), (-1, 1), (1, 1)]
    return [(CENTER_NORTH + SQUARE_SIDE / 2 * dn,
             CENTER_EAST + SQUARE_SIDE / 2 * de,
             CENTER_DOWN) for dn, de in corners]


def main(args=None):
    rclpy.init(args=args)

    detector = DroneYoloDetector()
    executor = SingleThreadedExecutor()
    executor.add_node(detector)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    mission = DroneController(namespace='drone1')

    mission.arm()
    mission.takeoff(ALT)
    time.sleep(5)

    waypoints = kwadrat()
    current = (0.0, 0.0, 0.0)
    for i, (n, e, d) in enumerate(waypoints):
        delta = (n - current[0], e - current[1], d - current[2])
        if any(delta):
            mission.send_goto_relative(*delta)
        current = (n, e, d)
        time.sleep(5)
        if i == 0:
            detector.start()

    mission.get_logger().info('RTL')
    mission.rtl()

    # Detekcja wyłącza się sama po LOST_SECONDS bez wykrycia; jeśli na koniec
    # misji wciąż działa, zamykamy ją, żeby zapisać film i statystyki.
    if detector.active:
        detector.stop()
    mission.get_logger().info(
        f'Wykrycia łącznie: {len(detector.detections)}')

    mission.destroy_node()
    executor.shutdown()
    detector.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
