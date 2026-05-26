import rclpy
from rclpy.node import Node

from drone_interfaces.srv import SpinDCMotor
import lgpio
import time

class Hydrolab2026MotorControl(Node):
    def __init__(self):
        super.__init__("DCcontroler")
        NAMESPACE_HARDWARE = 'knr_hardware/'

        # pins to control the dc motor controler
        self.clock_wise = 23
        self.counter_clock_wise = 24

        # define the lgpio class to control the pins
        self.gpio = lgpio.gpiochip_open(4)

        #setup the pins
        lgpio.gpio_claim_output(self.gpio, self.clock_wise)
        lgpio.gpio_claim_output(self.gpio, self.counter_clock_wise)
        lgpio.gpio_write(self.gpio, self.clock_wise, 0)
        lgpio.gpio_write(self.gpio, self.counter_clock_wise, 0)

        # setup the ROS service
        self.create_service(
            SpinDCMotor,
            NAMESPACE_HARDWARE+'dc_motor_control',
            self.spin_dcmotor_callback
        )

        # to prevent short circut i define here a mutex
        self.is_running = False


    def spin_dcmotor_callback(self, request, response):
        response = SpinDCMotor.Response()
        if self.is_running:
            response.result = False
            response.message = "Motor is now running"
            return response

        self.is_running = True
        # true means clock wise direction
        direction = request.spin_motor
        if direction:
            lgpio.gpio_write(self.gpio, self.clock_wise, 1)
        else:
            lgpio.gpio_write(self.gpio, self.counter_clock_wise, 1)

        # wait for request time
        time.sleep(request.time)

        self.stop_dcmotor()

        self.is_running = False

        response.result = True
        response.message = "success"

        return response

    def stop_dcmotor(self):
        lgpio.gpio_write(self.gpio, self.clock_wise, 0)
        lgpio.gpio_write(self.gpio, self.counter_clock_wise, 0)
        self.is_running = False

    def destroy_node(self):
        self.stop_dcmotor()
        lgpio.gpiochip_close(self.gpio)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Hydrolab2026MotorControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
