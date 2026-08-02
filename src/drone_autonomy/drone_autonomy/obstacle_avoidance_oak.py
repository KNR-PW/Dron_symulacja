#!/usr/bin/env python3
"""

Na tym etapie node tylko:
  - subskrybuje obraz(y) glebi z jednej lub wielu kamer stereo,

"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from drone_interfaces.msg import VelocityVectors
from drone_interfaces.srv import ToggleVelocityControl, SetMode
from cv_bridge import CvBridge


class ObstacleAvoidanceOak(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_oak')

        self.declare_parameter('depth_topics', ['oak/stereo/image_raw'])
        depth_topics = self.get_parameter('depth_topics').value

        self.bridge = CvBridge()
        self.latest_depth = {}

        self.depth_subscribers = []
        for topic in depth_topics:
            sub = self.create_subscription(
                Image, topic,
                lambda msg, t=topic: self.depth_callback(msg, t),
                10)
            self.depth_subscribers.append(sub)

        self.velocity_vectors_publisher = self.create_publisher(
            VelocityVectors, 'knr_hardware/velocity_vectors', 10)
        self.toggle_velocity_control_client = self.create_client(
            ToggleVelocityControl, 'knr_hardware/toggle_v_control')
        self.guard_srv = self.create_service(
            SetMode, 'set_brake_on_obstacle', self.set_guard_callback)

        self.guard_active = False

        self.get_logger().info(
            f"ObstacleAvoidanceOak (szkielet) gotowy. Depth topics: {list(depth_topics)}")

    def depth_callback(self, msg: Image, topic: str):
        self.latest_depth[topic] = self.bridge.imgmsg_to_cv2(msg, '16UC1')
        # TODO: logika unikania przeszkod

    def set_guard_callback(self, request, response):
        self.guard_active = request.mode.upper() in ['ON', 'TRUE', '1', 'ENABLE']
        self.get_logger().info(f"Guard active: {self.guard_active}")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceOak()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
