import rclpy  # Python Client Library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Image  # Image is the message type
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library
import numpy as np
from drone_interfaces.msg import Telemetry
from drone_interfaces.srv import PostLog
import requests
import time
from datetime import datetime
import io

class RosMissionWebsite(Node):

    def __init__(self):
        super().__init__('ros_mission_website')

        self.declare_parameter('base_url', 'http://localhost:5000')
        self.web_app_base_url = self.get_parameter('base_url').get_parameter_value().string_value

        self.report_subscription = self.create_subscription(
            Telemetry, 
            'telemetry',
            self.telemetry_callback,
            10)
        
        self.image_subscription = self.create_subscription(
            Image,
            'camera',
            self.image_callback,
            10)

        self.post_log_service = self.create_service(PostLog, 'post_log_to_web', self.handle_post_log_request)

        self.br = CvBridge()

        self.current_image = None
        self.current_telemetry = {
        "altitude": 0,
        "speed": 0,
        "battery": 0,
        "gps": "0, 0",
        "signal_strength": 0,
        "flight_mode": "NONE",
        "temperature": 0
    }

        # self.web_app_base_url = "http://localhost:5000"
        # self.web_app_base_url = "https://osadniik.pythonanywhere.com/"

        # Post data to website with specified time interval
        post_interval = 0.5
        self.timer = self.create_timer(post_interval, self.post_all_to_server)
        self.get_logger().info('RosMissionWebsite client node created')
        self.test_server_connection()


    def test_server_connection(self):
        try:
            response = requests.get(self.web_app_base_url)
        except Exception as e:
            self.get_logger().warn(f"❌ Server is not available: {e}")
            return False
        if response.status_code == 200:
            self.get_logger().info("✅ Server is available.")
        return True

    def telemetry_msg_to_json(self, telemetry_msg):
        json_msg = {
            "altitude": telemetry_msg.alt,
            "speed": -1,
            "battery": telemetry_msg.battery_percentage,
            "gps": f"{telemetry_msg.lat},{telemetry_msg.lon}",
            "signal_strength": -1,
            "flight_mode": telemetry_msg.flight_mode,
            "temperature": -1
        }

        return json_msg

    def telemetry_callback(self, msg):
        self.get_logger().warn("UPDATE TELE")
        self.current_telemetry = self.telemetry_msg_to_json(msg)

    def image_callback(self, msg):
        self.current_image = self.br.imgmsg_to_cv2(msg)

    def post_image_to_server(self):
        _, buffer = cv2.imencode('.jpg', self.current_image)  # Encode as JPEG (or PNG)
        file_bytes = io.BytesIO(buffer.tobytes())  # Wrap in a file-like object

        files = {
            'image': ('image.jpg', file_bytes, 'image/jpeg')  # filename, fileobj, MIME type
        }

        # Send the POST request
        try:
            response = requests.post(f"{self.web_app_base_url}/api/image", files=files)
            self.get_logger().info(f"Status image post: {response.status_code}")
        except Exception as e:
            self.get_logger().warn(f"❌ Failed to post image. Server is not available: {e}")

    def post_telemetry_to_server(self):
        try:
            response = requests.post(f"{self.web_app_base_url}/api/status", json=self.current_telemetry)
            self.get_logger().info(f"Status telemetry post: {response.status_code}")
        except Exception as e:
            self.get_logger().warn(f"❌ Failed to post telemetry. Server is not available: {e}")

    def post_all_to_server(self):
        self.post_telemetry_to_server()
        if self.current_image is not None:
            self.post_image_to_server() 
        self.get_logger().info('Posted all data to server')

    def handle_post_log_request(self, request, response):
        payload = {
            "message": request.message,
            "level": request.level if request.level else "info"
        }

        try:
            res = requests.post(f"{self.web_app_base_url}/api/log", json=payload)
            if res.status_code == 200:
                response.result = 1
                self.get_logger().info(f"Successfully posted log: {payload}")
            else:
                response.result = 0
                self.get_logger().warn(f"Failed to post log, status code: {res.status_code}")
        except Exception as e:
            response.result = 0
            self.get_logger().warn(f"❌ Failed to post log to server: {e}")

        return response


def main(args=None):
    rclpy.init(args=args)

    mission_website_client = RosMissionWebsite()

    rclpy.spin(mission_website_client)

    mission_website_client.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
