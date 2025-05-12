import rclpy
from rclpy.node import Node
from std_msg.msg import String

import board
import neo_pixel

class ledStrip(Node):
    def __init__(self):
        super().__init__('led_strip_node')

        # Define control parameters of LED Strip
        self.declare_parameter('led_number',5)
        self.declare_parameter('led_pin','D18')
        self.declare_parameter('brightness', 1.0)     # 0.0–1.0

        num = self.get_parameter('led_number').value
        pin_name = self.get_parameter('led_pin').value
        brightness = self.get_parameter('brightness').value

        pin = getattr(board,pin_name)
        self.strip = neopixel.NeoPixel(
            pin, 
            num, 
            brightness=brightness,
            autowrite=False
            pixel_order=neopixel.GRB
        )

        # Create subscriber to recieve mission commands
        self.create_subscribtion(
            String,
            'led_command',
            self.command_callback,
            10
        )

        self.get_logger().info(f'[{self.get_name()}] Ready to flash {count} LEDs on pin  {pin_name}')

    def command_callback(self,msg:String):
        cmd = msg.data.strip().lower()
        color = None

        if cmd == 'r':
            color = (255, 0,   0)
        elif cmd == 'g':
            color = (0,   255, 0)
        elif cmd == 'b':
            color = (0,   0, 255)
        elif cmd == 'w':
            color = (255, 255, 255)
        elif cmd == 'o':
            color = (0,   0,   0)
        elif cmd == 'q':
            self.get_logger().info(' "q" command – node shutting down')
            rclpy.shutdown()
            return
        else:
            self.get_logger().warn(f'Unknown command: "{cmd}"')
            return     

        # Set LED strip color
        for i in range(len(self.strip)):
            self.strip[i] = color
        self.strip.show()
        self.get_logger().info(f'Ustawiono kolor {color}')

    def destroy_node(self):
        # na zamknięcie – wyłączamy diody
        for i in range(len(self.strip)):
            self.strip[i] = (0, 0, 0)
        self.strip.show()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LedStripNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()