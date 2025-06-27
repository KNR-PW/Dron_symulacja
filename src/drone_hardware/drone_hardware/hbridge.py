import rclpy
from rclpy.node import Node

from drone_interfaces.srv import SpinDCMotor
import lgpio
import time

class MotorHBridgeNode(Node):
    def __init__(self):
        super().__init__('hbridge')

        # piny H-bridge
        self.in1_pin = 17
        self.in2_pin = 27
        self.pwm_pin = 22
        self.pwm_freq = 10000

        self.gpioh = lgpio.gpiochip_open(4)  
        for pin in (self.in1_pin, self.in2_pin, self.pwm_pin):
            lgpio.gpio_claim_output(self.gpioh, pin)

        # ustawiamy PWM (freq, duty=0)
        lgpio.tx_pwm(self.gpioh, self.pwm_pin, self.pwm_freq, 0)

        self.create_service(
            SpinDCMotor,
            'spin_dc_motor',
            self.spin_dcmotor_callback
        )
        self.get_logger().info('Motor H-Bridge node initialized')

    def spin_dcmotor_callback(self, request, response):
        self.get_logger().info('Spinning DC motor forward at 50% PWM')
        self.set_direction(True)
        self.set_speed(50.0)

        time.sleep(5.0)  # kręć przez 2 sekundy

        self.stop()
        self.get_logger().info('Motor stopped')

        response.result = True
        response.message = 'Spun at 50% duty cycle for 2 seconds.'
        return response

    def set_direction(self, forward: bool):
        lgpio.gpio_write(self.gpioh, self.in1_pin, 1 if forward else 0)
        lgpio.gpio_write(self.gpioh, self.in2_pin, 0 if forward else 1)

    def set_speed(self, duty: float):
        # duty: 0–100
        lgpio.tx_pwm(self.gpioh, self.pwm_pin, self.pwm_freq, duty)

    def stop(self):
        lgpio.gpio_write(self.gpioh, self.in1_pin, 0)
        lgpio.gpio_write(self.gpioh, self.in2_pin, 0)
        lgpio.tx_pwm(self.gpioh, self.pwm_pin, self.pwm_freq, 0)

    def destroy_node(self):
        # cleanup przed zamknięciem
        self.stop()
        lgpio.gpiochip_close(self.gpioh)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MotorHBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
