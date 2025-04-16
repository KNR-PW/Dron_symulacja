from drone_comunication import Hardware_com
import rclpy

class Mission(Hardware_com):
    def __init__(self):

        
def main(args=None):
    rclpy.init(args=args)
    mission = Mission()
    mission.arm()
    mission.takeoff(2.0)

    mission.send_goto_relative( 2, 0, 0)

    mission.land()
    mission.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
