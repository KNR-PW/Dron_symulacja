import rclpy
from dronekit import connect, VehicleMode
from pymavlink import mavutil

import time

# LED control imports
# import neopixel_spi as neopixel
# import board

from rclpy.node import Node
from rclpy.action import ActionServer

from std_msg.msg import ColorRGBA
from rpi_ws281x import Adafruit_NeoPixel, Color

############ TO DO LIST ##############
# 1. Handle LED strip via spi drivers
# 2. Handle servo movement with FC interface
# 3. Handle servo with some RPi external libraries
# 4. 
######################################

class DroneUtils(Node):
    def __init__(self):
        super().__init__('drone_utils')
        self.vehicle = None

        ## DECLARE HARDWARE PARAMETERS

        # Define RPi USB port where flight controler is plugged 
        self.declare_parameter('fc_ip', '/dev/ttyACM0') 
        # Initialize SPI bus
        strip = Adafruit_NeoPixel(STRIP_LED_NUMBER,LED_PIN)
        strip.begin()
        strip.show()

        # Define neo_pixel object

        # Define number of LEDs in one strip to control
        self.STRIP_LED_NUMBER = 5
        self.LED_PIN = 18


        ## DECLARE VARIABLES
        self.red_color = [255,0,0]
        self.green_color = [0,255,0]
        self.blue_color = [0,0,255]

    def __del__(self):
        if self.vehicle:
            self.vehicle.mode=VehicleMode("RTL")

    def shoot_callback(self, goal_handle):
        self.get_logger().info(f"Incoming shoot goal for color: {goal_handle.request.color}")
        stop = 1000
        shoot = 1200
        load =1500

        left = 2000
        mid = 1400
        right = 800

        self.set_servo(10,stop)
        self.set_servo(11,stop)
        time.sleep(1)
        self.set_servo(9,left if goal_handle.request.color == 'yellow' else right)
        self.set_servo(10,load)
        self.set_servo(11,load)
        time.sleep(2)
        self.set_servo(10,shoot)
        self.set_servo(11,shoot)
        self.set_servo(9,mid - 300 if goal_handle.request.color == 'yellow' else mid+300)
        time.sleep(1)
        self.set_servo(10,stop)
        self.set_servo(11,stop)
        self.set_servo(9,mid)

        self.get_logger().info("Shoot action completed:" + goal_handle.request.color)
        goal_handle.succeed()
        result = Shoot.Result()
        return result

    def color_callback(msg):
        for i in range(STRIP_LED_NUMBER):
            strip.setPixelColor(i,Color(int(msg.r*255), int(msg.g*255), int(msg.b*255)))
            strip.show()
    
    # def handle_led_strip(self):
        # # Flash LED strip with red, then green, then blue color
        # # Probably there should we use some kind if service to publish colors? maybe we should run drone_utils in separate thread/ service dedicated only for hardware?
        # while True:
        #     for i in range(self.STRIP_LED_NUMBER):
        #         pixels[i] = self.red_color
        #         pixels.show()
        #         time.sleep(1)
        #     for i in range(self.STRIP_LED_NUMBER):
        #         pixels[i] = self.green_color
        #         pixels.show()
        #         time.sleep(1)
        #     for i in range(self.STRIP_LED_NUMBER):
        #         pixels[i] = self.blue_color
        #         pixels.show()
        #         time.sleep(1)


def main():
    rclpy.init()
    
    drone = DroneUtils()

    rclpy.spin(drone)

    subscribtion = node.create_subscribtion(ColorRGBA,'color',color_callback,10)
    subscribtion

    rclpy.spin(drone)

    drone.destroy_node()
    rclpy.shutdown()


if __name__ == 'main':
    main()
