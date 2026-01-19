import rclpy
import time
from drone_comunication import DroneController
from rclpy.action import ActionClient

from rclpy.node import Node
from drone_interfaces.srv import SetMode, PreflightCalibrationControlService
from drone_interfaces.action import (
    Arm,
    Takeoff)

class TestClient(DroneController):
    def __init__(self):
        super().__init__()
        self.cli = self.create_client(PreflightCalibrationControlService, 'knr_hardware/preflight_calibration_control_service')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for preflight_calibration_control_service...')
    
    def start_calib(self, action: int):
        req = PreflightCalibrationControlService.Request()
        req.action = action
        self.future = self.cli.call_async(req)
        self.get_logger().info(f'Sent calibration request with action: {action}')



def main(args=None):
    rclpy.init(args=args)
    try:
        print("Creating TestClient...")
        mission = TestClient()
        print("TestClient created - service is ready!")
        
        print("Sending calibration START (action=1)...")
        mission.start_calib(1)  # Valid actions: 0=STOP, 1=START
        
        print("Waiting for response...")
        time.sleep(2)
        
        print("Done!")
    except Exception as e:
        print(f"ERROR: {e}")
        import traceback
        traceback.print_exc()
    finally:
        mission.destroy_node()
        rclpy.shutdown()

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