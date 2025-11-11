import rclpy
import time
from drone_comunication import DroneController
from rclpy.action import ActionClient

from rclpy.node import Node
from drone_interfaces.srv import SetMode
from drone_interfaces.action import (
    Arm,
    Takeoff)

class Test_PX4(Node):
    def __init__(self):
        super().__init__('minimal_client_async')
        NAMESPACE_HARDWARE = 'knr_hardware/'

        self._mode_cli = self.create_client(SetMode, NAMESPACE_HARDWARE+'set_mode')
        while not self._mode_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
        self._arm_client    = ActionClient(self, Arm, NAMESPACE_HARDWARE+'Arm')

    def send_mode(self, mode: str):
        req = SetMode.Request()
        req.mode = mode
        return self._mode_cli.call_async(req)
    
    def arm(self) -> bool:
        self.get_logger().info('Seting mode to guided...')
        if not self.send_mode('GUIDED'):
            return False
        self.get_logger().info('Arming drone...')
        return self._send_action(self._arm_client, Arm.Goal())
    
    def _send_action(self, client: ActionClient, goal_msg) -> bool:
        # Wait for action server
        while not client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn(f'Waiting for {client._action_name} server...')
        # Lock state and send
        self._busy = True
        send_future = client.send_goal_async(goal_msg)
        send_future.add_done_callback(lambda f: self._on_action_response(f, client))

        # Spin until result or emergency
        while True:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self._alarm:
                self.get_logger().error('Emergency, aborting')
                return False
            if not self._busy:
                break
        return True

    def _on_action_response(self, send_future, client: ActionClient):
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'{client.action_name} goal rejected')
            self._busy = False
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self._on_action_result(f))

    def _on_action_result(self, result_future):
        status = result_future.result().status
        print(status)
        self._busy = False


def main(args=None):
    rclpy.init(args=args)
    # mission = Test_PX4()
    # mission.arm()
    # mission.send_mode('LAND')
    mission = DroneController()
    mission.arm()
    # mission.takeoff(5.0)
    # # mission.send_goto_global(-35.363319396972656, 149.16531372070312, 5.0)
    # time.sleep(5)
    # # mission.send_goto_relative( 8.0, 0.0, 0.0)
    # # mission.send_set_yaw(3.14/2)
    # # mission.toggle_control()
    # # mission.send_vectors(1.0,0.0,0.0)
    # # time.sleep(1)
    # # mission.send_vectors(1.0,0.0,0.0)
    # # time.sleep(1)
    # # mission.send_vectors(1.0,0.0,0.0)
    # # time.sleep(1)
    # # mission.toggle_control()
    # mission.land()
    # mission.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()