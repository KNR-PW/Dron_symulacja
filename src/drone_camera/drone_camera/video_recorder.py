import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from drone_interfaces.srv import TurnOnVideo, TurnOffVideo

#the main core of this node was copy from Images_recorder made by Stanislaw Kolodziejski

class VideoRecorder(Node):

    def __init__(self):
        super().__init__('images_recorder')
        # prepering dir to save video
        self.declare_parameter('camera_topic', 'camera')
        self.declare_parameter('save_directory_base', 'saved_video')


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
        self.get_logger().info(f'Saving video to {self.save_directory}')

        

        self.br = CvBridge()
        #makes servise to start and end video
        self.start_video_recorder = self.create_service(TurnOnVideo, 'turn_on_video', self.start_video_callback)
        self.stop_video_recorder = self.create_service(TurnOffVideo, 'turn_off_video', self.stop_video_callback)

        #declare variables to make video

        self._start_video_flag = False
        self.video_name = 0

        self.get_logger().info('video_recorder node created')

    def listener_callback(self, data):
        self.current_frame = self.br.imgmsg_to_cv2(data)

        if self._start_video_flag:
            self.result.write(self.current_frame)

    def start_video_callback(self, request, response):
        self.video_name += 1
        width = self.current_frame.get(3)
        height = self.current_frame.get(4)
        size = (width, height)
        self.result = cv2.VideoWriter("video"+str(self.video_name)+".mp4", cv2.VideoWriter_fourcc(*'MJPG'), 10, size) 
        self._start_video_flag = True
        response.error = False
        return response
    
    def stop_video_callback(self, request, response):
        self._start_video_flag = False
        self.result.release()
        response.error = False
        return response


    def save_frame(self):
        if hasattr(self, 'current_frame'):
            # Save the current frame to a file
            filename = f'frame_{self.get_clock().now().nanoseconds}.jpg'
            filepath = os.path.join(self.save_directory, filename)
            cv2.imwrite(filepath, self.current_frame)
            self.get_logger().info(f'Saved frame: {filepath}')
        else:
            self.get_logger().warn('No frame to save yet')
def main(args=None):
    rclpy.init(args=args)

    images_recorder = VideoRecorder()

    rclpy.spin(images_recorder)

    images_recorder.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
