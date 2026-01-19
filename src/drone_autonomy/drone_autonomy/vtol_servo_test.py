import time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from .drone_comunication import DroneController
from drone_interfaces.action import (SetActuatorTest)

class ActuatorTestClient(DroneController):
    def __init__(self):
        super().__init__()
        self._client = ActionClient(self, SetActuatorTest, "set_actuator_test")

    def send_servo(self, actuator_id: int, value: float, timeout: float = 1.0):
        if not -1.0 <= value <= 1.0:
            self.get_logger().error("Value must be between -1 and 1")
            return

        goal = SetActuatorTest.Goal()
        goal.actuator_id = actuator_id
        goal.value = value
        goal.timeout = timeout
        if not self._client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("set_actuator_test action server not available")
            return

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

    def sweep_actuators(self, actuator_ids: list[int], points_num: int, timeout: float = 0.2):
        step = 2.0 / (points_num - 1)  # -1 <---> 1
        for actuator_id in actuator_ids:
            self.get_logger().info(f"Sweeping actuator {actuator_id}")
            for i in range(points_num):
                value = -1.0 + i * step
                self.send_servo(actuator_id, value, timeout)
                rclpy.spin_once(self, timeout_sec=0.1)
                time.sleep(timeout)

def main(args=None):
    rclpy.init(args=args)
    mission = ActuatorTestClient()

    print("Starting VTOL servo test...")
   
    actuator_ids = list(range(8)) 
    mission.sweep_actuators(actuator_ids, points_num=8, timeout=0.2)

    mission.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()