import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from drone_interfaces.action import SetActuatorTest
from px4_msgs.msg import VehicleCommand

# REFS
# https://mavlink.io/en/messages/common.html#MAV_CMD_DO_MOTOR_TEST

class ActuatorTestServer(Node):
    def __init__(self):
        super().__init__('actuator_test_server')
        self._action_server = ActionServer(
            self,
            SetActuatorTest,
            'set_actuator_test',
            self.execute_callback
        )
        self.command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.get_logger().info('Actuator Test Server initialized')

    async def execute_callback(self, goal_handle):
        req = goal_handle.request
        self.get_logger().info(f'Received goal: actuator_id={goal_handle.request.actuator_id}, value={goal_handle.request.value}, timeout={goal_handle.request.timeout}')

        command_msg = VehicleCommand()
        command_msg.timestamp = int(time.time() * 1e6)
        command_msg.command = 310  # VEHICLE_CMD_ACTUATOR_TEST now
        command_msg.param1 = float(req.value)          # value -1..1
        command_msg.param2 = float(req.timeout)        # timeout [s]
        command_msg.param3 = 33 + req.actuator_id  

        command_msg.target_system = 1
        command_msg.target_component = 1
        command_msg.source_system = 255
        command_msg.source_component = 0
        command_msg.confirmation = 0
        command_msg.from_external = True

        self.command_publisher.publish(command_msg)

        time.sleep(goal_handle.request.timeout)

        goal_handle.succeed()
        result = SetActuatorTest.Result()
        result.success = True
        result.message = f'Actuator {req.actuator_id} set to {req.value} for {req.timeout} seconds.'
        return result
    
def main(args=None):
    rclpy.init(args=args)
    actuator_test_server = ActuatorTestServer()
    rclpy.spin(actuator_test_server)
    actuator_test_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()