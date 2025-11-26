import rclpy
from rclpy.node import Node
from drone_interfaces.srv import VtolServoCalib

class TestClient(Node):
    def __init__(self):
        super().__init__('test_calib_tilts_client')
        self.cli = self.create_client(VtolServoCalib, 'knr_hardware/calib_servo')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for calib_servo service...')
        req = VtolServoCalib.Request()
        req.angle_3 = 0.0   # pion
        req.angle_4 = 90.0  # do przodu
        self.future = self.cli.call_async(req)

def main():
    rclpy.init()
    node = TestClient()
    rclpy.spin_until_future_complete(node, node.future)
    print(node.future.result())
    node.destroy_node()
    rclpy.shutdown()