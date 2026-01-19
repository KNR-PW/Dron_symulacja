import rclpy
from rclpy.node import Node
from px4_msgs.msg import PreflightCalibrationControl


class PreflightCalibrationPublisher(Node):
    def __init__(self):
        super().__init__('preflight_calibration_publisher')

        self.pub = self.create_publisher(
            PreflightCalibrationControl,
            '/fmu/in/preflight_calibration_control',
            10
        )

        # publikujemy raz po starcie
        self.timer = self.create_timer(0.5, self.publish_once)
        self.published = False

    def publish_once(self):
        if self.published:
            return

        self.get_logger().info('Publishing preflight calibration...')
        
        msg = PreflightCalibrationControl()
        msg.action = 1  # START
        msg.timestamp = self.get_clock().now().nanoseconds // 1000  # us

        self.get_logger().info(f'Message action: {msg.action}, timestamp: {msg.timestamp}')
        
        self.pub.publish(msg)
        self.get_logger().info('Preflight calibration START sent')

        self.published = True


def main():
    try:
        rclpy.init()
        print("ROS2 initialized")
        node = PreflightCalibrationPublisher()
        print("Node created")
        node.get_logger().info('Node started successfully')
        rclpy.spin(node)
        node.destroy_node()
    except Exception as e:
        print(f"ERROR in main: {e}")
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()