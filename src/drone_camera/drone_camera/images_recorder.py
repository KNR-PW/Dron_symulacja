#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from pathlib import Path
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

# made by Stanisław Kołodziejski 

class ImagesRecorder(Node):

    def __init__(self):
        super().__init__('images_recorder')

        self.declare_parameter('camera_topic', 'camera')
        self.declare_parameter('save_directory_base', 'saved_images')
        self.declare_parameter('fps', 1.0, descriptor=ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description="Frames per second for the image recording.",
        ))

        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.get_logger().info(f'camera_topic: {camera_topic}')
        self.subscription = self.create_subscription(
            Image, camera_topic, self.listener_callback, 10
        )

        # --- Struktura: saved_images/<nr_misji>/{mission_images,log_images} ---
        base_root = Path(self.get_parameter('save_directory_base').get_parameter_value().string_value)
        base_root.mkdir(parents=True, exist_ok=True)

        # znajdź najwyższy numer misji (foldery będące liczbą)
        max_idx = 0
        for entry in base_root.iterdir():
            if entry.is_dir() and entry.name.isdigit():
                try:
                    idx = int(entry.name)
                    if idx > max_idx:
                        max_idx = idx
                except ValueError:
                    pass
        next_idx = max_idx + 1

        mission_dir = base_root / str(next_idx)
        mission_images_dir = mission_dir / "mission_images"
        log_images_dir = mission_dir / "log_images"

        mission_images_dir.mkdir(parents=True, exist_ok=True)
        log_images_dir.mkdir(parents=True, exist_ok=True)

        # Ten node zapisuje do log_images
        self.save_directory = str(log_images_dir)
        self.get_logger().info(f'Created mission directory: {mission_dir}')
        self.get_logger().info(f'Recording images to directory: {self.save_directory}')

        # Timer
        fps = max(self.get_parameter('fps').get_parameter_value().double_value, 1e-6)
        timer_period = 1.0 / fps
        self.timer = self.create_timer(timer_period, self.save_frame)

        self.br = CvBridge()
        self.current_frame = None

        self.get_logger().info('images_recorder node created')

    def listener_callback(self, data):
        self.current_frame = self.br.imgmsg_to_cv2(data)

    def save_frame(self):
        if self.current_frame is not None:
            filename = f'frame_{self.get_clock().now().nanoseconds}.jpg'
            filepath = os.path.join(self.save_directory, filename)
            ok = cv2.imwrite(filepath, self.current_frame)
            if not ok:
                self.get_logger().error(f'cv2.imwrite failed for {filepath}')
        else:
            self.get_logger().warn('No frame to save yet')

def main(args=None):
    rclpy.init(args=args)
    images_recorder = ImagesRecorder()
    try:
        rclpy.spin(images_recorder)
    except KeyboardInterrupt:
        images_recorder.get_logger().info('Shutting down (Ctrl+C)')
    finally:
        images_recorder.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()