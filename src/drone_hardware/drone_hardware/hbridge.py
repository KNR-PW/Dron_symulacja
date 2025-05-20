import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32

from drone_interfaces.srv import SpinDCMotor
import RPi.GPIO as GPIO

class HBridge:
    def __init__(self, in1, in2, pwm_pin, pwm_freq=20000):
        self.in1 = in1
        self.in2 = in2
        self.pwm_pin = pwm_pin

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(in1, GPIO.OUT)
        GPIO.setup(in2, GPIO.OUT)
        GPIO.setup(pwm_pin, GPIO.OUT)

        self.pwm = GPIO.PWM(pwm_pin, pwm_freq)
        self.pwm.start(0)

    def set_direction(self, forward: bool):
        GPIO.output(self.in1, GPIO.HIGH if forward else GPIO.LOW)
        GPIO.output(self.in2, GPIO.LOW if forward else GPIO.HIGH)

    def set_speed(self, duty: float):
        self.pwm.ChangeDutyCycle(duty)

    def stop(self):
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.LOW)
        self.pwm.ChangeDutyCycle(0)

    def cleanup(self):
        self.pwm.stop()
        GPIO.cleanup()

class MotorHBridgeNode(Node):
    def __init__(self):
        super().__init__('motor_hbridge_node')
        # Declare parameters
        self.declare_parameter('in1_pin', 17)
        self.declare_parameter('in2_pin', 27)
        self.declare_parameter('pwm_pin', 22)
        self.declare_parameter('pwm_freq', 20000)

        in1 = self.get_parameter('in1_pin').value
        in2 = self.get_parameter('in2_pin').value
        pwm = self.get_parameter('pwm_pin').value
        freq = self.get_parameter('pwm_freq').value

        # Initialize hardware
        self.hbridge = HBridge(in1, in2, pwm, freq)
        self.current_speed = 0.0
        self.current_direction = 'stopped'

        # Publisher for status
        self.speed_pub = self.create_publisher(Float32, 'hardware_speed', 10)
        self.dir_pub = self.create_publisher(String, 'hardware_direction', 10)

        # Subscriber for commands
        self.create_subscription(
            String,
            'hardware_commands',
            self.command_callback,
            10
        )
        self.get_logger().info('Motor H-Bridge node initialized')

    def command_callback(self, msg: String):
        data = msg.data.strip().lower()
        parts = data.split(':')
        cmd = parts[0]
        duty = 0.0
        if len(parts) > 1:
            try:
                duty = float(parts[1])
            except ValueError:
                self.get_logger().error(f"Invalid duty cycle: {parts[1]}")
                return

        if cmd == 'f':
            self.hbridge.set_direction(True)
            self.hbridge.set_speed(duty)
            self.current_direction = 'forward'
            self.current_speed = duty
            self.get_logger().info(f'Moving forward at {duty}%')
        elif cmd == 'b':
            self.hbridge.set_direction(False)
            self.hbridge.set_speed(duty)
            self.current_direction = 'backward'
            self.current_speed = duty
            self.get_logger().info(f'Moving backward at {duty}%')
        elif cmd == 's':
            self.hbridge.stop()
            self.current_direction = 'stopped'
            self.current_speed = 0.0
            self.get_logger().info('Motor stopped')
        else:
            self.get_logger().warn(f'Unknown command: {cmd}')
            return

        # Publish status
        speed_msg = Float32()
        speed_msg.data = self.current_speed
        dir_msg = String()
        dir_msg.data = self.current_direction
        self.speed_pub.publish(speed_msg)
        self.dir_pub.publish(dir_msg)

    def destroy_node(self):
        self.hbridge.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorHBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()