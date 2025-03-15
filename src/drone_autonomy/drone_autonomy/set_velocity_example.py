import rclpy
from rclpy.node import Node
from drone_interfaces.srv import SetVelocity
import time
from rclpy.action import ActionClient

class SetVelocity(Node):

    def __init__(self):
        super().__init__('goto_detection_client')
        self.set_velocity = ActionClient(self, SetVelocity, 'set_velocity')



    def set_velocity_frd(self, front, right, down):
        self.get_logger().info("Sending goto frd relative action goal")
        goal_msg = SetVelocity.Goal()
        goal_msg.vx = float(front)
        goal_msg.vy = float(right)
        goal_msg.vz = float(down)
        while not self.goto_rel_action_client.wait_for_server():
            self.get_logger().info('waiting for goto server...')

        self.goto_rel_action_client.send_goal_async(goal_msg)
        self.get_logger().info("Goto action sent")

def main(args=None):
    rclpy.init(args=args)

    set_velocity_example = SetVelocity()
    set_velocity_example.set_velocity_frd(1.0,1.0,0.0)

    set_velocity_example.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()