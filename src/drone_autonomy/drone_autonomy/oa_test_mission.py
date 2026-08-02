import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from drone_autonomy.drone_comunication.drone_controller import DroneController


class DepthMonitor(Node):
    def __init__(self, depth_topic='oak/stereo/image_raw'):
        super().__init__('oa_depth_monitor')
        self.bridge = CvBridge()
        self.create_subscription(Image, depth_topic, self.depth_callback, 10)
        self._last_log = 0.0
        self.get_logger().info(f"Monitor glebi nasluchuje na: {depth_topic}")

    def depth_callback(self, msg: Image):
        now = time.time()
        if now - self._last_log < 0.5:
            return
        self._last_log = now

        depth_mm = self.bridge.imgmsg_to_cv2(msg, '16UC1')
        valid = depth_mm[depth_mm > 0]
        if valid.size == 0:
            self.get_logger().info("Brak waznej glebi (0 pikseli) - sprawdz swap_lr")
            return

        nearest_m = float(valid.min()) / 1000.0
        coverage = 100.0 * valid.size / depth_mm.size
        self.get_logger().info(
            f"Najblizsza przeszkoda: {nearest_m:.2f} m | pokrycie: {coverage:.0f}%")


def main(args=None):
    rclpy.init(args=args)

    monitor = DepthMonitor()
    monitor_exec = SingleThreadedExecutor()
    monitor_exec.add_node(monitor)
    monitor_thread = threading.Thread(target=monitor_exec.spin, daemon=True)
    monitor_thread.start()

    mission = DroneController()

    mission.arm()
    mission.takeoff(4.0)

    try:
        while rclpy.ok():
            mission.send_goto_relative(8.0, 0.0, 0.0)
            time.sleep(3)
            mission.send_goto_relative(-8.0, 0.0, 0.0)
            time.sleep(2)
    except KeyboardInterrupt:
        mission.get_logger().info("Przerwano - laduje")

    mission.land()

    mission.destroy_node()
    monitor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
