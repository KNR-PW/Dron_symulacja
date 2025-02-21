from flask import Flask, render_template, request
import datetime
import cv2
import os
import rclpy  # Python Client Library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Image  # Image is the message type
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import os
from threading import Event
from drone_interfaces.msg import DetectionMsg, DetectionsList

dir_path = os.path.dirname(__file__)


class CameraNode(Node):
    """
    Create an ImagePublisher class, which is a subclass of the Node class.
    """

    def __init__(self):
        super().__init__('web_camera')
        self.image_subscription = self.create_subscription(
            Image,
            'camera',
            self.image_callback,
            10)
        self.detection_subscription = self.create_subscription(
            DetectionsList,
            'detections',
            self.detection_callback,
            10)
        self.br = CvBridge()
        self.detections = []
        self.frame = 0
        self.detections_list = []
        self.get_logger().info('Web Camera node created')

    def image_callback(self, frame):
        print("clncdj")
        # Convert ROS Image message to OpenCV image
        frame = self.br.imgmsg_to_cv2(frame)
        # frame = cv2.cvtColor(frame, cv2.COLOR_RGB2GBR)
        frame = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_AREA)

        if os.path.exists(dir_path + "/static/ros_frame.jpg"):
            os.remove(dir_path + "/static/ros_frame.jpg")
        cv2.imwrite(dir_path + "/static/ros_frame.jpg", frame)
        self.frame = frame

    def detection_callback(self, detections):
        self.detections_list = detections.detections_list
        for det in detections.detections_list:
            x, y, w, h = det.bounding_box
            cv2.rectangle(self.frame, (x, y),
                          (x + h, y + w),
                          (0, 255, 0), 5)
        if os.path.exists(dir_path + "/static/ros_det_frame.jpg"):
            os.remove(dir_path + "/static/ros_det_frame.jpg")
        if len(detections.detections_list):
            cv2.imwrite(dir_path + "/static/ros_det_frame.jpg", self.frame)

    def get_detections_strings(self):
        # Function returns list of string descriptions of detections
        det_num = 1
        detections_strings = []
        for det in self.detections_list:
            det_str = "Det " + str(det_num) + ":         "
            temp = "Color: " + str(det.color_name) + ",  " + "Bounding Box: " + str(
                det.bounding_box) + " , " + "GPS Position: " + str(det.gps_position)
            det_str += temp
            detections_strings.append(det_str)
            det_num += 1
        return detections_strings

    def get_frame(self):
        return self.frame


rclpy.init(args=None)
event = Event()
ros_node = CameraNode()

app = Flask(__name__)


@app.route('/')
@app.route('/index.html')
def index():
    now = datetime.datetime.now()
    timeString = now.strftime("%Y-%m-%d %H:%M:%S")
    return render_template('index.html')


@app.route('/detector.html', methods=['GET', 'POST'])
def detector():
    img_path = ""
    detections_strings = ["No detections"]
    if request.method == 'POST':
        rclpy.spin_once(ros_node, timeout_sec=1)

        button_name = request.form.get("button_name")
        if button_name == 'img_button':
            img_path = "./static/ros_frame.jpg"

        if button_name == 'img_det_button':
            img_path = "./static/ros_det_frame.jpg"

        if button_name == 'load_detections':
            detections_strings = ros_node.get_detections_strings()

    return render_template('detector.html', img_jpg=img_path, detections_strings=detections_strings)


def main():
    print(os.path.dirname(__file__))
    app.run(debug=True, host='0.0.0.0')


if __name__ == '__main__':
    main()
