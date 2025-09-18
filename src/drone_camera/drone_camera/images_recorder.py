#!/usr/bin/env python3
import os
import re
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from pathlib import Path

from drone_interfaces.srv import MakePhoto   # << serwis

class ImagesRecorder(Node):
    def __init__(self):
        super().__init__('images_recorder')

        self.declare_parameter('camera_topic', 'camera')
        self.declare_parameter('fps', 1.0, descriptor=ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description="Frames per second for the video recording.",
        ))
        ### Chat powiedzial, zeby dodac, zeby moc opcjonalnie wylaczyc stary timer
        self.declare_parameter('enable_timer', False)

        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.get_logger().info(f'camera_topic: {camera_topic}')

        self.subscription = self.create_subscription(
            Image, camera_topic, self.listener_callback, 10
        )

        # <<< TU ZMIANA: bazowy katalog na sztywno >>>
        base_dir = Path.home() / "ros_ws" / "Mission_Photos"
        base_dir.mkdir(parents=True, exist_ok=True)

        # >>> NOWE: auto-inkrementacja Mission#idx przy każdym uruchomieniu
        self.save_directory = self._create_next_mission_dir(base_dir)
        self.get_logger().info(f'Saving images to {self.save_directory}')

        ### Powiedzial, zeby dodac self.current_frame = None, przetestuje czy dziala bez itd.
        self.br = CvBridge()
        self.current_frame = None

        # Timer staskowy opcjonalny
        if self.get_parameter('enable_timer').get_parameter_value().bool_value:
            fps = self.get_parameter('fps').get_parameter_value().double_value
            timer_period = 1.0 / max(fps, 1e-6)
            self.timer = self.create_timer(timer_period, self.save_frame)
            self.get_logger().info('video_recorder timer enabled')

        # Service
        self.srv = self.create_service(MakePhoto, 'make_photo', self.make_photo)

        self.get_logger().info('ImagesRecorder node with service ready')

    # --- helper: wyznacz + utwórz kolejny katalog Mission#idx ---
    def _create_next_mission_dir(self, base_dir: Path) -> str:
        """
        Szuka podkatalogów Mission<liczba> i Mission#<liczba>,
        wybiera najwyższy indeks i tworzy kolejny w formacie 'Mission#<idx>'.
        Zwraca absolutną ścieżkę jako string.
        """
        import re
        pattern = re.compile(r"^Mission#?(\d+)$")  # dopasuj Mission1 i Mission#1
        max_idx = 0

        for entry in base_dir.iterdir():
            if entry.is_dir():
                m = pattern.match(entry.name)
                if m:
                    try:
                        idx = int(m.group(1))
                        if idx > max_idx:
                            max_idx = idx
                    except ValueError:
                        pass

        next_idx = max_idx + 1
        name = f"Mission#{next_idx}"             # zawsze z '#'
        mission_dir = base_dir / name
        mission_dir.mkdir(parents=True, exist_ok=True)
        self.get_logger().info(f'Created mission directory: {mission_dir}')
        return str(mission_dir)

    # Callback
    def listener_callback(self, data: Image):
        # Twierdzi, ze trzeba wymusic ale imo to bujda sprawdze i dodam jakby nie dzialalo
        # self.current_frame = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')
        self.current_frame = self.br.imgmsg_to_cv2(data)

    # Save frame
    def save_frame(self, filename=None):
        if self.current_frame is not None:
            if filename is None:
                filename = f'frame_{self.get_clock().now().nanoseconds}.jpg'
            filepath = os.path.join(self.save_directory, filename)
            ok = cv2.imwrite(filepath, self.current_frame)
            if ok:
                self.get_logger().info(f"Saved frame: {filepath}")
                return filepath
            else:
                self.get_logger().error(f"cv2.imwrite failed for {filepath}")
                return None
        else:
            self.get_logger().warn('No frame to save yet')
            return None

    # Obsluga service'u
    def make_photo(self, req, res):           #Req = request, req.prefix = 'Zdj#', req.ext = 'jpg'
        fname = f'{req.prefix}.jpg'
        fpath = self.save_frame(filename=fname)

        if fpath is None:
            res.success = 'error: no frame to save'
        else:
            res.success = f'saved: {fpath}'
        return res


def main(args=None):
    rclpy.init(args=args)
    node = ImagesRecorder()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
