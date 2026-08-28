import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import ExternalShutdownException

from drone_interfaces.srv import TurnOnVideo, TurnOffVideo

#the main core of this node was copy from Images_recorder made by Stanislaw Kolodziejski

class VideoRecorder(Node):

    def __init__(self):
        super().__init__('images_recorder')
        # prepering dir to save video
        self.declare_parameter('camera_topic', 'camera')
        self.declare_parameter('save_directory_base', 'saved_video')
        # fps zapisu do pliku - ustaw na REALNA czestotliwosc nagrywanego topicu
        # (sprawdz: ros2 topic hz <topic>), inaczej wideo odtwarza sie za szybko/wolno
        self.declare_parameter('fps', 15.0)
        # Kamery publikujace BEST_EFFORT (np. OAK-D) nie dogadaja sie z domyslnym
        # subskrybentem RELIABLE — wtedy nie przychodzi ani jedna klatka.
        self.declare_parameter('best_effort', False)
        # Kontener i kodek. mp4v/mp4 daje pliki kilkukrotnie mniejsze niz MJPG/avi.
        # Gdyby OpenCV na tej maszynie nie potrafilo zapisac mp4, node sam
        # przelaczy sie na MJPG/avi (patrz start_video_callback).
        self.declare_parameter('video_ext', 'mp4')
        self.declare_parameter('fourcc', 'mp4v')
        # true = nagrywaj od razu po pierwszej klatce, bez wolania serwisu.
        # Zatrzymanie node'a (Ctrl+C) domyka wtedy plik.
        self.declare_parameter('autostart', False)

        self.fps = float(self.get_parameter('fps').value)
        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.get_logger().info(f'camera_topic: {camera_topic}')
        if self.get_parameter('best_effort').get_parameter_value().bool_value:
            qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.BEST_EFFORT)
            self.get_logger().info('QoS: BEST_EFFORT')
        else:
            qos = 10
        self.subscription = self.create_subscription(
            Image,
            camera_topic,
            self.listener_callback,
            qos)

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
        # declare namespace
        NAMESPACE = 'knr_video/'
        #makes servise to start and end video
        self.start_video_recorder = self.create_service(TurnOnVideo, NAMESPACE+'turn_on_video', self.start_video_callback)
        self.stop_video_recorder = self.create_service(TurnOffVideo, NAMESPACE+'turn_off_video', self.stop_video_callback)

        #declare variables to make video

        self._start_video_flag = False
        self.video_name = 0
        self._autostart = self.get_parameter('autostart').get_parameter_value().bool_value
        if self._autostart:
            self.get_logger().info('autostart: nagrywanie ruszy po pierwszej klatce')

        self.get_logger().info('video_recorder node created')

    def listener_callback(self, data):
        self.current_frame = self.br.imgmsg_to_cv2(data)

        # Rozmiar klatki znamy dopiero tutaj, wiec autostart odpala sie
        # przy pierwszym obrazie, a nie w konstruktorze.
        if self._autostart:
            self._autostart = False
            self._open_video()

        if self._start_video_flag:
            self.result.write(self.current_frame)

    def _open_video(self) -> bool:
        """Otwiera nowy plik wideo. Zwraca True, gdy sie udalo."""
        self.close_video()   # domknij poprzedni, gdyby ktos wolal start dwa razy
        self.video_name += 1
        height ,width , c = self.current_frame.shape
        size = (int(width), int(height))

        ext = self.get_parameter('video_ext').get_parameter_value().string_value
        fourcc = self.get_parameter('fourcc').get_parameter_value().string_value
        path = os.path.join(self.save_directory, f"video{self.video_name}.{ext}")
        self.result = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*fourcc), self.fps, size)

        # OpenCV nie rzuca wyjatku, gdy brakuje kodeka — po prostu nie otwiera
        # pliku i kazda klatka leci w prozne. Stad jawny test + zapasowy MJPG/avi.
        if not self.result.isOpened():
            self.get_logger().warn(
                f'Nie udalo sie otworzyc {path} (kodek {fourcc}). '
                f'Przelaczam na MJPG/avi.')
            path = os.path.join(self.save_directory, f"video{self.video_name}.avi")
            self.result = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*'MJPG'), self.fps, size)
            if not self.result.isOpened():
                self.get_logger().error(f'Nie moge zapisywac wideo do {path}')
                return False

        self._start_video_flag = True
        self.get_logger().info(f'Nagrywam do {path} @ {self.fps} fps')
        return True

    def close_video(self):
        """Domyka plik. Bez tego nagranie zostaje uszkodzone."""
        if self._start_video_flag:
            self._start_video_flag = False
            self.result.release()
            self.get_logger().info('Nagranie zapisane')

    def start_video_callback(self, request, response):
        if not hasattr(self, 'current_frame'):
            self.get_logger().error(
                'Brak klatek z kamery — nie zaczynam nagrywania. '
                'Sprawdz camera_topic i czy kamera publikuje.')
            response.error = True
            return response
        response.error = not self._open_video()
        return response

    def stop_video_callback(self, request, response):
        self.close_video()
        response.error = False

        return response

def main(args=None):
    rclpy.init(args=args)

    images_recorder = VideoRecorder()

    try:
        rclpy.spin(images_recorder)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Ctrl+C tez musi domknac plik — inaczej mp4 zostaje bez naglowka
        # i nie da sie go odtworzyc.
        images_recorder.close_video()
        images_recorder.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
