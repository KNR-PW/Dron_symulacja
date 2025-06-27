import rclpy
from rclpy.node import Node
from drone_interfaces.srv import CreateReport, UpdateReport
import requests
import json

class RosReportWebsite(Node):
    def __init__(self):
        super().__init__('ros_mission_report')

        self.declare_parameter('base_url', 'http://localhost:5000')
        self.web_app_base_url = self.get_parameter('base_url').get_parameter_value().string_value

        self.create_service(CreateReport, 'create_report', self.handle_create_report)
        self.create_service(UpdateReport, 'update_report', self.handle_update_report)

        self.get_logger().info('RosReportWebsite client node created')
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

    def handle_create_report(self, request, response):
        try:
            report_data = json.loads(request.json_report)
            res = requests.post(f"{self.web_app_base_url}/api/report/create", json=report_data)
            response.success = res.status_code == 200
            response.message = res.json().get("message", "OK") if response.success else res.text
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def handle_update_report(self, request, response):
        try:
            update_data = json.loads(request.json_update)
            res = requests.post(f"{self.web_app_base_url}/api/report/update", json=update_data)
            response.success = res.status_code == 200
            response.message = res.json().get("message", "Updated") if response.success else res.text
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def post_images_to_server(self, images_list):
        for image in images_list:
            _, buffer = cv2.imencode('.jpg', self.image)  # Encode as JPEG (or PNG)
            file_bytes = io.BytesIO(buffer.tobytes())  # Wrap in a file-like object

            files = {
                'image': ('image.jpg', file_bytes, 'image/jpeg')  # filename, fileobj, MIME type
            }

            # Send the POST request
            try:
                response = requests.post(f"{self.web_app_base_url}/api/report/image", files=files)
                self.get_logger().debug(f"Status image post: {response.status_code}")
            except Exception as e:
                self.get_logger().warn(f"❌ Failed to post image. Server is not available: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = RosReportWebsite()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
