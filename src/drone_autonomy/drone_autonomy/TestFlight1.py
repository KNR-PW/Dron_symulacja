import rclpy
import time
from drone_comunication import DroneController

def main(args=None):
    rclpy.init(args=args)
    mission = DroneController()
    mission.arm()
    mission.takeoff(5.0)
    time.sleep(5)
    mission.land()
    mission.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()