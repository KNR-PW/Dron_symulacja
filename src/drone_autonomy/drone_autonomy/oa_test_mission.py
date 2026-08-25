import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge

from drone_autonomy.drone_comunication.drone_controller import DroneController


class DepthMonitor(Node):
    def __init__(self, depth_topic='oak/stereo/image_raw'):
        super().__init__('oa_depth_monitor')
        self.bridge = CvBridge()
        self.avoiding = False
        self.create_subscription(Image, depth_topic, self.depth_callback, 10)
        self.create_subscription(
            Bool, 'obstacle_avoidance_oak/avoiding', self.avoiding_callback, 10)
        self._last_log = 0.0
        self.get_logger().info(f"Monitor glebi nasluchuje na: {depth_topic}")

    def avoiding_callback(self, msg: Bool):
        self.avoiding = msg.data

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
        flag = ' | OMIJANIE AKTYWNE' if self.avoiding else ''
        self.get_logger().info(
            f"Najblizsza przeszkoda: {nearest_m:.2f} m | pokrycie: {coverage:.0f}%{flag}")


def cruise_distance(mission: DroneController, monitor: DepthMonitor, direction: float,
                     distance_m: float, speed: float = 1.5, rate_hz: float = 5.0):

    period = 1.0 / rate_hz
    start = mission.get_gps()
    if start is None:
        mission.get_logger().warn('Brak odczytu GPS - lece na czas (fallback)')
        elapsed = 0.0
        fallback_duration = distance_m / speed
        while elapsed < fallback_duration and rclpy.ok():
            if not monitor.avoiding:
                mission.send_vectors(direction * speed, 0.0, 0.0)
            time.sleep(period)
            elapsed += period
        return

    start_north, start_east, _ = start
    traveled = 0.0
    while traveled < distance_m and rclpy.ok():
        if not monitor.avoiding:
            mission.send_vectors(direction * speed, 0.0, 0.0)
        time.sleep(period)

        pos = mission.get_gps()
        if pos is not None:
            north, east, _ = pos
            traveled = math.hypot(north - start_north, east - start_east)


def main(args=None):
    rclpy.init(args=args)

    monitor = DepthMonitor()
    monitor_exec = SingleThreadedExecutor()
    monitor_exec.add_node(monitor)
    monitor_thread = threading.Thread(target=monitor_exec.spin, daemon=True)
    monitor_thread.start()

    mission = DroneController()

    mission.arm()
    mission.takeoff(2.2)
    mission.toggle_control()

    try:
        while rclpy.ok():
            mission.get_logger().info('Lot 20m do przodu w strone przeszkod...')
            cruise_distance(mission, monitor, direction=1.0, distance_m=20.0)
            mission.send_vectors(0.0, 0.0, 0.0)
            time.sleep(2)

            mission.get_logger().info('Lot 20m do tylu (powrot)...')
            cruise_distance(mission, monitor, direction=-1.0, distance_m=20.0)
            mission.send_vectors(0.0, 0.0, 0.0)
            time.sleep(2)
    except KeyboardInterrupt:
        mission.get_logger().info("Przerwano - laduje")

    mission.send_vectors(0.0, 0.0, 0.0)
    mission.toggle_control()
    mission.land()

    mission.destroy_node()
    monitor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
