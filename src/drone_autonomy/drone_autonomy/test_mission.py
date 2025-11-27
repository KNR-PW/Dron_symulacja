import rclpy
import time
from drone_comunication import DroneController
from rclpy.action import ActionClient

from rclpy.node import Node
from drone_interfaces.srv import SetMode, VtolServoCalib
from drone_interfaces.action import (
    Arm,
    Takeoff)

class TestClient(DroneController):
    def __init__(self):
        super().__init__()
        self.cli = self.create_client(VtolServoCalib, 'knr_hardware/calib_servo')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for calib_servo service...')
    def set_servo(self, angle_4: float, angle_5: float):
        req = VtolServoCalib.Request()
        req.angle_4 = angle_4  
        req.angle_5 = angle_5  
        self.future = self.cli.call_async(req)



def main(args=None):
    rclpy.init(args=args)
    # mission = Test_PX4()
    # mission.arm()
    # mission.send_mode('LAND')
    mission = TestClient()
    mission.arm()
    mission.set_servo(45.0,45.0)  # Value in degrees, calculated from the OZ to the axis of rotation of the motor 
    # time.sleep(30)

    # mission.send_goto_global(47.398183, 8.54611, 5.0)

    # # time.sleep(30)

    # # mission.send_goto_global(47.39797127066047, 8.54616274676383, 5.0)
    # mission.send_goto_relative( 8.0, 0.0, 0.0)
    # time.sleep(5)
    # mission.send_goto_relative( 0.0, 8.0, 0.0)
    # time.sleep(5)
    # mission.send_goto_relative( -8.0, 0.0, 0.0)
    # time.sleep(5)
    # mission.send_goto_relative( 0.0, -8.0, 0.0)
    # time.sleep(5)

    # mission.send_set_yaw(3.14/2)
    # time.sleep(5)
    # mission.send_set_yaw(-3.14/2)
    # time.sleep(5)

    #mission.toggle_control()
    #for x in range(100):
        #mission.send_vectors(1.0,-1.0,0.0)
        #time.sleep(0.1)

    #mission.toggle_control()
    #mission.land()
    # mission.rtl()
    mission.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()