import rclpy
import time
from drone_comunication import DroneController

from rclpy.node import Node
from drone_interfaces.srv import SetMode

class Test_PX4(Node):
    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(SetMode, 'knr_hardware/set_mode')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        

    def send_request(self, mode: str):
        req = SetMode.Request()
        req.mode = mode
        return self.cli.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    mission = Test_PX4()
    mission.send_request('GUIDED')
    # mission = DroneController()
    # mission.arm()
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