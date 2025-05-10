from drone_comunication import Hardware_com
from drone_interfaces.msg import MiddleofAruco
import rclpy
import time

class Mission(Hardware_com):
    def __init__(self):
        super().__init__()
        self.midl = self.create_subscription(MiddleofAruco, 'aruco_markers', self.point_of_aruco ,10)


    
    def point_of_aruco(self, msg):
        self.get_logger().info(f"Point of middle [{msg.x}, {msg.y}]")
        self.middle_of_aruco = [msg.x, msg.y]
        
def main(args=None):
    '''   rclpy.init(args=args)
    rclpy.init()
    node = Mission()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
    '''
    rclpy.init(args=args)
    mission = Mission()
    mission.arm()
    mission.takeoff(10.0)
    time.sleep(3)
   # mission.send_goto_relative( 2, 0, 0)

    mission.land()
    mission.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
