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
    mission.takeoff(4.0)
    mission.send_goto_relative( 0.0, 0.0, 0.0)
    mission.set_speed(0.5)
    mission.send_goto_relative(0.0, 17.0,0.0)
    for x in range(10):
        mission.send_goto_relative(1.0, 0.0, 0.0)
        mission.send_goto_relative(0.0, -16.0,0.0)
        mission.send_goto_relative(1.0, 0.0, 0.0)
        mission.send_goto_relative(0.0, 16.0,0.0)


    mission.land()
    mission.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()