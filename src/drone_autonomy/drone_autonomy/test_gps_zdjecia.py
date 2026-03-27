#!/usr/bin/env python3
import sys
import rclpy
import argparse
import cv2
from cv_bridge import CvBridge
from drone_comunication.drone_controller import DroneController
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
)
from px4_msgs.msg import SensorGps
from sensor_msgs.msg import Image
import math

ALTITUDE = 30.0
CAMERA_TOPIC = '/oak/rgb/image_raw'

def _point(text):
    lat, lon = text.split(",")
    return float(lat), float(lon)
    
def parse_cli_args(argv):
    parser = argparse.ArgumentParser(
        prog="hydrolab_basic",
        description="Hydrolab mission, 4 pool gps points"
    )
    parser.add_argument("--p1", required=True)
    parser.add_argument("--p2", required=True)
    parser.add_argument("--p3", required=True)
    parser.add_argument("--p4", required=True)

    args, _ = parser.parse_known_args(argv)
    points = [_point(args.p1),
              _point(args.p2),
              _point(args.p3),
              _point(args.p4)]
    return points


class hydro_basic(DroneController):
    def __init__(self, pool_gps):
        super().__init__("hydro_basic")

        self.pool_gps = pool_gps
        self.lat0 = None
        self.lon0 = None
        self.lat_current = None
        self.lon_current = None
        self.photo_idx = 0
        self.frame = None
        self.br = CvBridge()

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self.sub_gps = self.create_subscription(
            SensorGps,
            "/fmu/out/vehicle_gps_position",
            self.gps_cb,
            qos
        )

        self.sub_camera = self.create_subscription(
            Image,
            CAMERA_TOPIC,
            self.camera_cb,
            10
        )

        self.get_logger().info("Waiting for GPS...")

    # ----------------------------------------------------------------

    def camera_cb(self, msg: Image):
        self.frame = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    def gps_cb(self, msg: SensorGps):
        if hasattr(msg, "fix_type") and msg.fix_type < 3:
            return
        if self.lat0 is None:
            self.lat0 = float(msg.latitude_deg)
            self.lon0 = float(msg.longitude_deg)
            self.get_logger().info(f"Start GPS: lat={self.lat0:.8f}, lon={self.lon0:.8f}")
        self.lat_current = float(msg.latitude_deg)
        self.lon_current = float(msg.longitude_deg)

    def wait_for_gps(self, timeout=30.0):
        start = self.get_clock().now()
        while rclpy.ok() and self.lat0 is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (self.get_clock().now() - start).nanoseconds / 1e9 > timeout:
                return False
        return True

    def take_mission_photo(self):
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.05)

        if self.frame is None:
            self.get_logger().error(f"Brak klatki z kamery – zdjęcie {self.photo_idx} pominięte!")
            return

        filename = f"Basen_{self.photo_idx}.jpg"
        cv2.imwrite(filename, self.frame)
        self.get_logger().info(f"Zdjęcie zapisane: {filename}")
        self.photo_idx += 1

    # ----------------------------------------------------------------

    def run(self):
        lat0 = self.lat0
        lon0 = self.lon0
        visited_points = []

        self.arm()
        self.takeoff(ALTITUDE)

        for lat, lon in self.pool_gps[:-1]:
            self.get_logger().info(f"Flying to GPS: {lat:.8f}, {lon:.8f}")
            self.send_goto_global(lat, lon, ALTITUDE)
            self.take_mission_photo()
            visited_points.append((self.lat_current, self.lon_current))

        latf, lonf = self.pool_gps[-1]
        self.send_goto_global(latf, lonf, ALTITUDE)
        visited_points.append((self.lat_current, self.lon_current))

        self.send_goto_global(lat0, lon0, ALTITUDE)

        self.get_logger().info("=" * 20)
        self.get_logger().info("Visited coordinates:")
        for i, (lat, lon) in enumerate(visited_points, start=1):
            self.get_logger().info(f"  Point {i}: lat={lat:.8f}, lon={lon:.8f}")
        self.get_logger().info("=" * 20)


# ----------------------------------------------------------------
# MAIN
# ----------------------------------------------------------------

def main():
    rclpy.init(args=sys.argv)

    pool_gps = parse_cli_args(sys.argv[1:])
    mission = hydro_basic(pool_gps)

    if not mission.wait_for_gps():
        mission.get_logger().error("No GPS connection")
        rclpy.shutdown()
        return

    mission.run()
    rclpy.spin(mission)
    mission.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()