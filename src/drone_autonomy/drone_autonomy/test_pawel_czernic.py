from drone_comunication import DroneController
import rclpy
import time

class Mission(DroneController):
    def __init__(self):
        super().__init__()

def main(args=None):
    rclpy.init(args=args)
    mission = Mission()
    mission.arm()
    mission.takeoff(10.0)
    mission.send_goto_relative( 5.0, 0.0, 0.0)
    mission.start_video()
    time.sleep(5)
    mission.send_goto_relative(0.0, 5.0, 0.0)
    time.sleep(5)
    mission.send_goto_relative(-5.0, 0.0, 0.0)
    time.sleep(5)
    mission.send_goto_relative(0.0, -5.0, 0.0)
    time.sleep(5)
    mission.stop_video()
    mission.land()
    mission.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()