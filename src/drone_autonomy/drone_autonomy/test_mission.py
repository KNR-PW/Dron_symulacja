from drone_comunication import Hardware_com
import rclpy

class Mission(Hardware_com):
    def __init__(self):
        super().__init__("test")


def main():
    rclpy.init()
    mission = Mission()
    mission.arm()
    mission.takeoff(2.0)
    #mission.set_yaw_action(3.14)
    #mission.set_yaw_action(3.14)

    mission.send_goto_relative(3.0,0.0,0.0)
    mission.set_yaw_action(3.14)
    mission.land()
    mission.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
