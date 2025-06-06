import rclpy

from rclpy.node import Node
from drone_interfaces.srv import Dropper

import serial
import time

class HostHardware(Node):
    def __init__(self):
        super().__init__("HardwareNode")

        self.declare_parameter('uart_port', '/dev/ttyAMA0')
        self.uart_port = self.get_parameter('uart_port').get_parameter_value().string_value

        self.get_logger().info(f'Connecting on port: {self.uart_port}')

        uart_connected = False
        while not uart_connected:
            try:
                self.uart = serial.Serial(self.uart_port, 9600)
                self.get_logger().info(f'serial connected:)')
                uart_connected = True
            except Exception as e:
                self.get_logger().error(f'Error while opening serial: {e}')
                self.get_logger().error("reconecting...")
                time.sleep(3.0)

        self.create_service(Dropper,"dropper",self.dropper_callback)

        self.get_logger().info(f'sucsefull made a node to control host perif')


    def dropper_callback(self,request,response):
        message = str(request.beacon_msg)+"\n"
        response = Dropper.Response()
        try:
            self.uart.write(bytearray(message,'ascii'))
        except Exception as e:
            self.get_logger().error(f'Error while sending message to esp: {e}')
            response.message = str(e)
            response.error = True
            return response
        
        response.error = False
        self.get_logger().info(f'sent a message to esp: {message}')

        return response

def main(args=None):
    rclpy.init(args=args)
    node = HostHardware()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()