#!/usr/bin/env python3
import os
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from pathlib import Path

from drone_interfaces.srv import MakePhoto

class ImagesRecorder(Node):
    def __init__(self):
        self.current_frame = None
        super().__init__('images_recorder')

        self.declare_parameter('camera_topic', 'camera')
        self.declare_parameter('fps', 1.0, descriptor=ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description="Frames per second for the video recording.",
        ))
        self.declare_parameter('enable_timer', False)

        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.get_logger().info(f'camera_topic: {camera_topic}')
        self.subscription = self.create_subscription(Image, camera_topic, self.listener_callback, 10)

        # --- Struktura: saved_images/<nr_misji>/{mission_images,log_images} ---
        base_root = Path.cwd() / "saved_images"
        base_root.mkdir(parents=True, exist_ok=True)

        # >>> ZAMIANA: używamy OSTATNIEJ istniejącej misji zamiast tworzyć nową
        mission_dir, mission_images_dir = self._use_latest_mission_dirs(base_root)
        self.mission_dir = mission_dir
        self.save_directory = mission_images_dir  # ten node zapisuje do mission_images
        self.get_logger().info(f'Using mission directory: {self.mission_dir}')
        self.get_logger().info(f'Saving images to {self.save_directory}')

        self.br = CvBridge()

        if self.get_parameter('enable_timer').get_parameter_value().bool_value:
            fps = max(self.get_parameter('fps').get_parameter_value().double_value, 1e-6)
            timer_period = 1.0 / fps
            self.timer = self.create_timer(timer_period, self.save_frame)
            self.get_logger().info('images_recorder timer enabled')

        # Service
        self.srv = self.create_service(MakePhoto, '/make_photo', self.make_photo)
        self.get_logger().info('ImagesRecorder node with service ready')
        self.heartbeat = self.create_timer(2.0, lambda: self.get_logger().debug("alive"))

    # --- helper: użyj ostatniej misji; jeśli brak, utwórz '1' i podkatalogi ---
    def _use_latest_mission_dirs(self, root: Path):
        max_idx = 0
        for entry in root.iterdir():
            if entry.is_dir() and entry.name.isdigit():
                try:
                    idx = int(entry.name)
                    if idx > max_idx:
                        max_idx = idx
                except ValueError:
                    pass

        if max_idx == 0:
            max_idx = 1  # pierwsza misja, jeśli żadnej nie ma

        mission_dir = root / str(max_idx)
        mission_images_dir = mission_dir / "mission_images"
        log_images_dir = mission_dir / "log_images"

        mission_images_dir.mkdir(parents=True, exist_ok=True)
        log_images_dir.mkdir(parents=True, exist_ok=True)

        return str(mission_dir), str(mission_images_dir)

    def listener_callback(self, data: Image):
        self.current_frame = self.br.imgmsg_to_cv2(data)

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

    def make_photo(self, req, res):
        ext = getattr(req, "ext", "jpg") or "jpg"
        fname = f'{req.prefix}.{ext}'
        fpath = self.save_frame(filename=fname)

        res.success = 'error: no frame to save' if fpath is None else f'saved: {fpath}'
        return res

def main(args=None):
    rclpy.init(args=args)
    node = ImagesRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down (Ctrl+C)")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
