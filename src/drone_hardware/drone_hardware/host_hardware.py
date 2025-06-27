import rclpy

from rclpy.node import Node
from drone_interfaces.srv import Dropper

import serial

class HostHardware(Node):
    def __init__(self):
        super().__init__("HardwareNode")

        self.uart = serial.Serial("/dev/ttyAMA0", 9600)

        self.create_service(Dropper,"dropper",self.dropper_callback)

        self.get_logger().info(f'sucsefull made a node to control host perif')


    def dropper_callback(self,request,response):
        message = "d"+str(request.beacon_number)+"\n"
        self.uart.write(bytearray(message,'ascii'))
        response = Dropper.Response()
        response.error = False
        self.get_logger().info(f'send a message to esp')

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