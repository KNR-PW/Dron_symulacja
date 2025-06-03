import rclpy
from rclpy.node import Node
import serial

class UartBridge(Node):
    def __init__(self):
        super().__init__('uart_bridge')

        # Declare hardware port - USB0 or ACM0
        self.declare_parameter('uart_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 9600)

        uart_port = self.get_parameter('uart_port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value

        try:
            self.serial = serial.Serial(uart_port, baud_rate, timeout=1)
            self.get_logger().info(f'Connected with {uart_port} with {baud_rate} baud')
        except serial.SerialException as e:
            self.get_logger().error(f'Error when opening port: {e}')
            return

        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        try:
            message = 'Hello ESP32!\n'
            self.serial.write(bytearray(message,'ascii'))
            self.get_logger().info(f'Message sent: {message.strip()}')

            if self.serial.in_waiting:
                response = self.serial.write(bytearray(message,'ascii'))
                self.get_logger().info(f'Message recieved: {response}')

        except Exception as e:
            self.get_logger().error(f'UART error: {e}')


def main(args=None):
    rclpy.init(args=args)

    node = UartBridge()
    rclpy.spin(node)
    node.destroy_node()
    
    rclpy.shutdown()