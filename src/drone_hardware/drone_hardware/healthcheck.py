import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.client import Client
from sensor_msgs.msg import Image
from drone_interfaces.msg import Telemetry
from drone_interfaces.srv import GetAttitude, PostLog
from std_msgs.msg import String
import requests
import time
from colorama import Fore, Style
from tabulate import tabulate


class DroneHealthCheck(Node):
    def __init__(self):
        super().__init__('drone_healthcheck')

        self.declare_parameter('camera_topic', 'camera')
        self.camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value

        self.declare_parameter('required_nodes', ['aruco_node', 'ros_mission_website'])
        self.required_nodes = self.get_parameter('required_nodes').get_parameter_value().string_array_value

        self.report = {"successes": [], "errors": [], "warnings": []}
        self.camera_frame_received = False
        self.camera_frame_empty = False
        self.telemetry_received = False

        # Subscribe to camera topic
        self.camera_subscription = self.create_subscription(
            Image,
            'camera',
            self.camera_callback,
            10
        )

        # Subscribe to telemetry topic
        self.telemetry_subscription = self.create_subscription(
            Telemetry,
            'telemetry',
            self.telemetry_callback,
            10
        )

        # self.required_nodes = ['aruco_node', 'ros_mission_website']
    def camera_callback(self, msg):
        self.camera_frame_received = True
        if not msg.data:
            self.camera_frame_empty = True

    def telemetry_callback(self, msg):
        self.telemetry_received = True

    def check_drone_handler_services(self):
        self.get_logger().info("Checking drone_handler services...")
        service_name = 'get_attitude'
        client = self.create_client(GetAttitude, service_name)
        if not client.wait_for_service(timeout_sec=5.0):
            self.report["errors"].append(f"Service '{service_name}' is not available. (drone_handler not healthy)")
        else:
            self.report["successes"].append(f"Service '{service_name}' is available.")
            request = GetAttitude.Request()
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            if future.result() is None:
                self.report["errors"].append(f"Service '{service_name}' did not respond. (drone_handler not healthy)")
            else:
                self.report["successes"].append(f"Service '{service_name}' responded successfully. (drone_handler healthy)")

    def check_camera_frames(self):
        self.get_logger().info("Checking camera frames...")
        start_time = time.time()
        timeout = 3

        while time.time() - start_time < timeout:
            rclpy.spin_once(self, timeout_sec=0.5)
            if self.camera_frame_received:
                break
        if not self.camera_frame_received:
            self.report["errors"].append(f"No frames received on '{self.camera_topic}' topic.")
        elif self.camera_frame_empty:
            self.report["warnings"].append(f"Frames received on '{self.camera_topic}' topic are empty.")
        else:
            self.report["successes"].append("Camera frames are OK.")

    def check_telemetry(self):
        self.get_logger().info("Checking telemetry topic...")
        start_time = time.time()
        timeout = 5

        while time.time() - start_time < timeout:
            rclpy.spin_once(self, timeout_sec=0.5)
            if self.telemetry_received:
                break
    
        if not self.telemetry_received:
            self.report["errors"].append("No telemetry data received on 'telemetry' topic.")
        else:
            self.report["successes"].append("Telemetry data is being received successfully.")

    def check_running_nodes(self):
        self.get_logger().info("Checking running nodes...")
        node_names = self.get_node_names()
        for node in self.required_nodes:
            if node not in node_names:
                self.report["warnings"].append(f"Node '{node}' is not running.")
            else:
                self.report["successes"].append(f"Node '{node}' is running.")

    def check_web_node(self):
        self.get_logger().info("Checking web node connection...")
        # Check the post_log_to_web service
        self.get_logger().info("Checking 'post_log_to_web' service...")
        service_name = 'post_log_to_web'
        client = self.create_client(PostLog, service_name)
        if not client.wait_for_service(timeout_sec=5.0):
            self.report["warnings"].append(f"Service '{service_name}' is not available.")
        else:
            self.report["successes"].append(f"Service '{service_name}' is available.")
            request = PostLog.Request()
            request.message = "Healthcheck test log"
            request.level = "info"
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future , timeout_sec=5.0)
            if future.result() is None or future.result().result != 1:
                self.report["warnings"].append(f"Service '{service_name}' did not respond or failed.")
            else:
                self.report["successes"].append(f"Service '{service_name}' responded successfully.")

    def generate_report(self):
        self.get_logger().info("Generating health check report...")
        if not self.report["errors"] and not self.report["warnings"]:
            self.get_logger().info("All systems are operational.")
        else:
            self.get_logger().info("Health check completed with issues:")
            for error in self.report["errors"]:
                self.get_logger().error(f"ERROR: {error}")
            for warning in self.report["warnings"]:
                self.get_logger().warn(f"WARNING: {warning}")


    def generate_table_report(self):
        headers = ["Status", "Message"]
        rows = []

        for success in self.report["successes"]:
            rows.append((f"{Fore.GREEN}SUCCESS{Style.RESET_ALL}", success))
        for warning in self.report["warnings"]:
            rows.append((f"{Fore.YELLOW}WARNING{Style.RESET_ALL}", warning))
        for error in self.report["errors"]:
            rows.append((f"{Fore.RED}ERROR{Style.RESET_ALL}", error))

        self.get_logger().info(tabulate(rows, headers=headers, tablefmt="pretty"))
def main(args=None):
    rclpy.init(args=args)
    healthcheck_node = DroneHealthCheck()

    try:
        healthcheck_node.check_drone_handler_services()
        healthcheck_node.check_camera_frames()
        healthcheck_node.check_telemetry()
        healthcheck_node.check_running_nodes()
        healthcheck_node.check_web_node()
        healthcheck_node.generate_table_report()
    finally:
        healthcheck_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()