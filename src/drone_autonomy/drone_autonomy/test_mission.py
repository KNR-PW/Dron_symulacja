from drone_comunication.drone_controller import DroneController


import rclpy

class Mission(DroneController):
    def __init__(self):
        super().__init__()


        
def main(args=None):
    rclpy.init(args=args)
    mission = Mission()
    mission.arm()
    mission.takeoff(2.0)
    mission.start_video()

    mission.send_goto_relative( 1.0, 1.0, 0.0)
    mission.stop_video()
    mission.land()
    mission.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
