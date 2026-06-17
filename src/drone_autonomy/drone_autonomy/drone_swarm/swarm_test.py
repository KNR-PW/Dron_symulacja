import rclpy
import time
from drone_autonomy.drone_comunication import DroneController

ALT = 5.0


def main(args=None):
    rclpy.init(args=args)
    mission = DroneController()

    mission.arm()
    mission.takeoff(ALT)
    time.sleep(5)
    mission.send_goto_relative(10, -10, -6)
    time.sleep(3)

    mission.land()
    mission.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
