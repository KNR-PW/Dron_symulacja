import rclpy
from rclpy.node import Node
from drone_interfaces.srv import CreateReport, UpdateReport
import requests
import json
import cv2
import io
import os

class RosReportWebsite(Node):
    def __init__(self):
        super().__init__('ros_report_website')

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
        images_list = getattr(request, 'images_list', None)
        if images_list:
            self.get_logger().info(f"Posting {len(images_list)} images to server...")
            self.post_images_to_server(images_list)
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
        self.get_logger().info(f"Posting {images_list} images to server...")
        # images_list is a list of image filepaths
        images_list_read = [cv2.imread(img_path) for img_path in images_list]
        for idx, image in enumerate(images_list_read):
            try:
                filename = os.path.basename(images_list[idx])
                # Encode the image as JPEG
                _, buffer = cv2.imencode('.jpg', image)
                file_bytes = io.BytesIO(buffer.tobytes())

                files = {
                    'image': (filename, file_bytes, 'image/jpeg')
                }

                response = requests.post(f"{self.web_app_base_url}/api/report/image", files=files)
                if response.status_code == 200:
                    self.get_logger().debug(f"✅ Successfully posted image {idx+1}/{len(images_list)}")
                else:
                    self.get_logger().warn(f"⚠️ Failed to post image {idx+1}: {response.status_code} {response.text}")
            except Exception as e:
                self.get_logger().warn(f"❌ Exception posting image {idx+1}: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = RosReportWebsite()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
