import rclpy
import time
from drone_comunication import DroneController
from drone_hardware import LoggerControl

def main(args=None):
    rclpy.init(args=args)
    mission = DroneController()
    logger = LoggerControl()
    logger.start()
    mission.arm()
    mission.takeoff(5.0)
    mission._set_mode("FIXED_WING")
    time.sleep(1)
    mission.send_goto_global(47.398136, 8.549139, 10.0)
    mission.send_goto_global(47.397321, 8.546019, 10.0)
    time.sleep(1)
    mission._set_mode("MULTICOPTER")
    time.sleep(1)
    mission.send_goto_global(47.398136, 8.549139, 5.0)
    mission.land()
    # mission.rtl()
    mission.destroy_node()
    logger.stop()
    rclpy.shutdown()


if __name__ == "__main__":
    main()