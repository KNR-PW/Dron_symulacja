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

    # METHODS THAT GENERATE TWO DIFFERENT SERVO OUTPUTS FOR FURTHER TESTING

    def send_servo_pwm_and_pct(
        self, 
        actuator_id: int,  
        value:float,
        timeout:float=1.0,
        pwm_enabled:bool=True,
    ):

        if pwm_enabled:
            param2 = 1  # PWM
            value = float(value)

            if value < self.PWM_MIN or value > self.PWM_MAX:
                self.get_logger().error(f"pwm_value must be between {self.PWM_MIN} and {self.PWM_MAX}")
                return
        else:
            param2 = 0  # percentage
            value = float(value)
            value = max(0.0, min(100.0, value))

        goal = SetActuatorTest.Goal()
        goal.actuator_id = actuator_id
        goal.value = value
        goal.timeout = timeout

        if not self._client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("set_actuator_test action server not available")
            return
        
        self._client.wait_for_server()
        self._send_goal_future = self._client.send_goal_async(
            goal, feedback_callback=self.feedback_cb)
        self._send_goal_future.add_done_callback(self.goal_response_cb)


    def send_commands_in_loop(
        self, 
        actuator_ids: list[int],  
        points_num:int,
        timeout:float,
        use_pwm:bool=True,
    ):
        if points_num < 2:
            self.get_logger().error("points_num must be >= 2")
            return

        if not self._client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("set_actuator_test action server not available")
            return
        
        step = (self.PWM_MAX - self.PWM_MIN) / (points_num - 1)

        for actuator_id in actuator_ids:
                self.get_logger().info(
                    f"Sweeping actuator {actuator_id} from {self.PWM_MIN} to {self.PWM_MAX} "
                    f"in {points_num} points"
                )

                for i in range(points_num):
                    if use_pwm:
                        value = self.PWM_MIN + i * step
                    else:
                        value = 0.0 + i * (100.0 / (points_num - 1))

                    self.get_logger().info(
                        f"Actuator {actuator_id}: value={value:.3f}"
                    )

                    self.send_servo_pwm_and_pct(actuator_id, value, timeout, pwm_enabled=use_pwm)

                    rclpy.spin_once(self, timeout_sec=0.1)
                    time.sleep(timeout)

def main(args=None):
    rclpy.init(args=args)
    mission = ActuatorTestClient()

    print("Starting VTOL servo test...")
   
    mission.send_commands_in_loop(
        actuator_ids=[0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16],
        points_num=8,
        timeout=0.2,
        use_pwm=True
    )


    mission.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()