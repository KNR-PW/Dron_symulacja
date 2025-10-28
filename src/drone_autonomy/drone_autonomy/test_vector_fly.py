
import rclpy
import time
from drone_comunication import DroneController


def main(args=None):
    rclpy.init(args=args)
    mission = DroneController
    mission.arm()
    mission.takeoff(5.0)
    # mission.send_goto_relative( 8.0, 0.0, 0.0)
    # mission.send_set_yaw(3.14/2)
    mission.toggle_control()
    mission.send_vectors(1.0,0.0,0.0)
    time.sleep(1)
    mission.send_vectors(1.0,0.0,0.0)
    time.sleep(1)
    mission.send_vectors(1.0,0.0,0.0)
    time.sleep(1)
    mission.toggle_control()
    mission.land()
    mission.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()