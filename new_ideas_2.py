import rclpy
from rclpy.node import Node
from drone_database_wrapper.drone_db import DroneDB
from drone_interfaces.srv import CreateReport
import json

class MissionReportUpdater(Node):
    def __init__(self):
        super().__init__('mission_report_updater')
        
        # Initialize database wrapper
        self.db = DroneDB('/path/to/drone_data.db')
        
        # Initialize client for RosReportWebsite service
        self.create_report_client = self.create_client(CreateReport, 'create_report')
        
        # Wait for the service to be available
        while not self.create_report_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        
        # Create a timer for periodic updates (e.g., every 60 seconds)
        self.timer = self.create_timer(60.0, self.update_website)
    
    def update_website(self):
        try:
            # Fetch all flight logs and detections from the database
            flight_logs = self.db.get_all_flight_logs()
            detections = self.db.get_all_detections()
            
            # Prepare report data
            report_data = {
                'flight_logs': flight_logs,
                'detections': detections
            }
            
            # Convert report data to JSON
            json_report = json.dumps(report_data)
            
            # Call the CreateReport service to update the website
            request = CreateReport.Request()
            request.json_report = json_report
            future = self.create_report_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            
            if future.result().success:
                self.get_logger().info('Website updated successfully.')
            else:
                self.get_logger().warn(f'Failed to update website: {future.result().message}')
        except Exception as e:
            self.get_logger().error(f'Error updating website: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = MissionReportUpdater()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
