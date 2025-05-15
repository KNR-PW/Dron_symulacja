from drone_comunication.drone_controller import DroneController


import rclpy
from drone_interfaces.srv import Dropper
import time
class Mission(DroneController):
    def __init__(self):
        super().__init__()

        self.beacon = self.create_client(Dropper,"dropper")

    def drop_beacon(self, number_of_beacon: int = 1):
        req = Dropper.Request()
        req.beacon_number = number_of_beacon
        fut = self._mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
        if fut.result() is None:
            self.get_logger().error(f'Failed to drop beacon')
            return False
        self.get_logger().info(f'wainting a sec for a drop')
        time.sleep(1.5)
        self.get_logger().info(f'beacon sucsefully drop')
        return True
  
def main(args=None):
    rclpy.init(args=args)
    mission = Mission()
    mission.arm()
    mission.start_video()
    mission.takeoff(7.0)
    mission.send_goto_relative( 0.0, 0.0, 0.0)
    mission.set_speed(0.5)
    time.sleep(2)
    mission.send_goto_relative( -10.0, 0.0, 0.0)
    mission.stop_video()
    mission.land()
    mission.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
