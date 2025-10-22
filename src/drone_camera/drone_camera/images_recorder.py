import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
# made by Stanisław Kołodziejski

class ImagesRecorder(Node):

    def __init__(self):
        super().__init__('images_recorder')

        self.declare_parameter('camera_topic', 'camera')
        self.declare_parameter('save_directory_base', 'saved_images')
        self.declare_parameter('fps', 1.0, descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Frames per second for the video recording.",
            ))

        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.get_logger().info(f'camera_topic: {camera_topic}')
        self.subscription = self.create_subscription(
            Image,
            camera_topic,
            self.listener_callback,
            10)

        save_directory_base = self.get_parameter('save_directory_base').get_parameter_value().string_value
        if not os.path.exists(save_directory_base):
            os.makedirs(save_directory_base, exist_ok=True)
        existing_dirs = [d for d in os.listdir(save_directory_base) if os.path.isdir(os.path.join(save_directory_base, d)) and d.isdigit()]
        max_dir_number = max(map(int, existing_dirs), default=0)
        new_dir_name = str(max_dir_number + 1)
        self.save_directory = os.path.join(save_directory_base, new_dir_name)
        os.makedirs(self.save_directory, exist_ok=True)
        self.get_logger().info(f'Saving images to {self.save_directory}')

        # Timer
        fps = self.get_parameter('fps').get_parameter_value().double_value
        timer_period = 1.0 / fps  # seconds
        self.timer = self.create_timer(timer_period, self.save_frame)

        self.br = CvBridge()

        self.get_logger().info('video_recorder node created')
        self.get_logger().info(f'Recording images to directory: {self.save_directory}')

    def listener_callback(self, data):
        self.current_frame = self.br.imgmsg_to_cv2(data)

    def save_frame(self):
        if hasattr(self, 'current_frame'):
            # Save the current frame to a file
            filename = f'frame_{self.get_clock().now().nanoseconds}.jpg'
            filepath = os.path.join(self.save_directory, filename)
            cv2.imwrite(filepath, self.current_frame)
            # self.get_logger().info(f'Saved frame: {filepath}')
        else:
            self.get_logger().warn('No frame to save yet')
            
def main(args=None):
    rclpy.init(args=args)

    images_recorder = ImagesRecorder()

    rclpy.spin(images_recorder)

    images_recorder.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
