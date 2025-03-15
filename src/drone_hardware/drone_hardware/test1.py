from std_msg.msg import ColorRGBA
from rpi_ws281x import Adafruit_NeoPixel, Color
import time

    strip = Adafruit_NeoPixel(STRIP_LED_NUMBER,LED_PIN)
    strip.begin()
    strip.show()

    # Define neo_pixel object

    # Define number of LEDs in one strip to control
    self.STRIP_LED_NUMBER = 5
    self.LED_PIN = 18


    def color_callback(msg):
    for i in range(STRIP_LED_NUMBER):
        strip.setPixelColor(i,Color(int(msg.r*255), int(msg.g*255), int(msg.b*255)))
        strip.show()

    def main():

        color_callback()
        


if __name__ == 'main':
    main()