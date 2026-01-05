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

from drone_interfaces.srv import MakePhoto

class ImagesRecorder(Node):
    def __init__(self):
        super().__init__('images_recorder')

        self.declare_parameter('camera_topic', 'camera')
        self.declare_parameter('fps', 1.0, descriptor=ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description="Frames per second for the video recording.",
        ))
        self.declare_parameter('enable_timer', False)

        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.get_logger().info(f'camera_topic: {camera_topic}')

        self.subscription = self.create_subscription(
            Image, camera_topic, self.listener_callback, 10
        )

        base_dir = Path.home() / "ros_ws" / "Mission_Photos"
        base_dir.mkdir(parents=True, exist_ok=True)

        self.save_directory = self._create_next_mission_dir(base_dir)
        self.get_logger().info(f'Saving images to {self.save_directory}')

        self.br = CvBridge()
        self.current_frame = None

        if self.get_parameter('enable_timer').get_parameter_value().bool_value:
            fps = self.get_parameter('fps').get_parameter_value().double_value
            timer_period = 1.0 / max(fps, 1e-6)
            self.timer = self.create_timer(timer_period, self.save_frame)
            self.get_logger().info('Timer enabled')

        # Serwis do robienia zdjęć
        self.srv = self.create_service(MakePhoto, 'make_photo', self.make_photo_callback)
        self.get_logger().info('ImagesRecorder ready - service: /make_photo')

    def _create_next_mission_dir(self, base_dir: Path) -> str:
        pattern = re.compile(r"^Mission#?(\d+)$")
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
        mission_dir = base_dir / f"Mission#{next_idx}"
        mission_dir.mkdir(parents=True, exist_ok=True)
        return str(mission_dir)

    def listener_callback(self, data: Image):
        self.current_frame = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')

    def save_frame(self, filename=None):
        if self.current_frame is None:
            self.get_logger().warn('No frame available')
            return None
            
        if filename is None:
            filename = f'frame_{self.get_clock().now().nanoseconds}.jpg'
        
        filepath = os.path.join(self.save_directory, filename)
        
        if cv2.imwrite(filepath, self.current_frame):
            self.get_logger().info(f"Saved: {filepath}")
            return filepath
        else:
            self.get_logger().error(f"Failed to save: {filepath}")
            return None

    # >>> POPRAWIONE - metoda w klasie <
    def make_photo_callback(self, req, res):
        fname = f'{req.prefix}.jpg'
        fpath = self.save_frame(filename=fname)
        
        if fpath is None:
            res.success = 'error: no frame available'
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


if __name__ == '__main__':
    main()
