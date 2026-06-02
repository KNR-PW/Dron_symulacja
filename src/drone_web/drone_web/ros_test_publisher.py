import random
import rclpy
from rclpy.node import Node
from drone_interfaces.msg import Telemetry


class TestTelemetryPublisher(Node):
    """Publishes fake drone_interfaces/Telemetry for dashboard testing."""

    def __init__(self):
        super().__init__('test_telemetry_publisher')
        self.pub = self.create_publisher(Telemetry, 'knr_hardware/telemetry', 10)
        self.timer = self.create_timer(1.0, self.publish)
        self.get_logger().info("Publishing fake telemetry on knr_hardware/telemetry")

    def publish(self):
        msg = Telemetry()
        msg.alt = random.uniform(0.0, 120.0)
        msg.speed = random.uniform(0.0, 20.0)
        msg.battery_percentage = random.randint(0, 100)
        msg.battery_voltage = random.uniform(10.0, 16.8)
        msg.battery_current = random.uniform(0.0, 30.0)
        msg.lat = random.uniform(-0.001, 0.001)
        msg.lon = random.uniform(-0.001, 0.001)
        msg.global_lat = 52.0 + random.uniform(-0.01, 0.01)
        msg.global_lon = 20.0 + random.uniform(-0.01, 0.01)
        msg.global_alt = random.uniform(0.0, 120.0)
        msg.flight_mode = random.choice(["GUIDED", "AUTO", "STABILIZE", "RTL", "LAND"])
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TestTelemetryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
