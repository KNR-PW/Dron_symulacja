import rclpy
import time
from drone_comunication import DroneController

def main(args=None):
    rclpy.init(args=args)
    mission = DroneController()
    mission.arm()

    mission.takeoff(5.0)

    mission.send_goto_relative(5.0, 0.0, 0.0)
    mission.send_goto_relative(0.0, 5.0, 0.0)
    mission.send_goto_relative(-5.0, 0.0, 0.0)
    mission.send_goto_relative(0.0, -5.0, 0.0)

    # time.sleep(30)

    # mission.toggle_control()
    # for x in range(100):
    #     mission.send_vectors(1.0,-1.0,0.0)
    #     time.sleep(0.1)
    # mission.toggle_control()


    mission.land()
    # mission.rtl()
    mission.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()