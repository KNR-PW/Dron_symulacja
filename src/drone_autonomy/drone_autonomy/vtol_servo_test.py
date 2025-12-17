import time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from .drone_controller import DroneController
from drone_interfaces.action import (SetActuatorTest)

class ActuatorTestClient(DroneController):
    def __init__(self):
        super().__init__()
        self._client = ActionClient(self, SetActuatorTest, "set_actuator_test")

    def send_servo(self, actuator_id, value, timeout=1.0):
        goal = SetActuatorTest.Goal()
        goal.actuator_id = actuator_id
        goal.value = value
        goal.timeout = timeout

        self._client.wait_for_server()
        self._send_goal_future = self._client.send_goal_async(
            goal,
            feedback_callback=self.feedback_cb
        )
        self._send_goal_future.add_done_callback(self.goal_response_cb)

    def feedback_cb(self, feedback):
        self.get_logger().info(f'Feedback: {feedback.feedback.state}')

    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return

        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self.result_cb)

    def result_cb(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.message}')

def main(args=None):
    rclpy.init(args=args)
    mission = ActuatorTestClient()

    mission.send_servo(actuator_id=4, value=0.5, timeout=2.0)  
    time.sleep(2)
    mission.send_servo(actuator_id=4, value=-0.5, timeout=2.0)
    time.sleep(2)
    mission.send_servo(actuator_id=4, value=1, timeout=2.0)
    time.sleep(2)
    mission.send_servo(actuator_id=4, value=-1, timeout=2.0)

    mission.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()