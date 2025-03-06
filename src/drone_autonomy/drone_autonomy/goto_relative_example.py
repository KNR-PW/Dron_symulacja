import rclpy  # Python Client Library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
# from detection import Detection
from drone_interfaces.action import GotoRelative
import time
from rclpy.action import ActionClient



class GotoRelativeExample(Node):

    def __init__(self):
        super().__init__('goto_detection_client')
        self.goto_rel_action_client = ActionClient(self, GotoRelative, 'goto_relative')



    def send_goto_relative(self, north, east, down):
        self.get_logger().info("Sending goto relative action goal")
        goal_msg = GotoRelative.Goal()
        goal_msg.north = float(north)
        goal_msg.east = float(east)
        goal_msg.down = float(down)
        while not self.goto_rel_action_client.wait_for_server():
            self.get_logger().info('waiting for goto server...')

        self.goto_rel_action_client.send_goal_async(goal_msg)
        self.get_logger().info("Goto action sent")

def main(args=None):
    rclpy.init(args=args)

    goto_relative_example = GotoRelativeExample()
    goto_relative_example.send_goto_relative(2.0, 0.0, 0.0)

    goto_relative_example.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
