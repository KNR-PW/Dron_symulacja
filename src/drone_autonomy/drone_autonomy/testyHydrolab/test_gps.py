#!/usr/bin/env python3
import sys
import rclpy
import argparse
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from drone_comunication.drone_controller import DroneController
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
)
from px4_msgs.msg import SensorGps
import math

ALTITUDE = 10.0

def _point(text):
    lat, lon = text.split(",")
    return float(lat), float(lon)
    
def parse_cli_args(argv):
    parser = argparse.ArgumentParser(
        prog="gps test",
        description="Test to check how accurate the gps is"
    )
    parser.add_argument("--p1", required=True)
    parser.add_argument("--p2", required=True)
    parser.add_argument("--p3", required=True)

    args, _ = parser.parse_known_args(argv)
    points = [_point(args.p1),
              _point(args.p2),
              _point(args.p3)]
    return points


class hydro_basic(DroneController):
    def __init__(self, gps_points):
        super().__init__("hydro_basic")

        self.gps_points = gps_points
        self.lat0 = None
        self.lon0 = None
        self.lat_current = None
        self.lon_current = None
        # self.frame = None

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

        self.get_logger().info("Waiting for GPS...")


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


    def run(self):
        lat0 = self.lat0
        lon0 = self.lon0
        visited_points = []

        self.arm()
        self.takeoff(ALTITUDE)

        for lat, lon in self.gps_points[:-1]:
            self.get_logger().info(f"Flying to GPS: {lat:.8f}, {lon:.8f}")
            self.send_goto_global(lat, lon, ALTITUDE)
            visited_points.append((self.lat_current, self.lon_current))

        latf, lonf = self.gps_points[-1]
        self.send_goto_global(latf, lonf, ALTITUDE)
        visited_points.append((self.lat_current, self.lon_current))

        self.send_goto_global(lat0, lon0, ALTITUDE)

        self.get_logger().info("=" * 20)
        self.get_logger().info("Visited coordinates:")
        for i, (lat, lon) in enumerate(visited_points, start=1):
            self.get_logger().info(f"  Point {i}: lat={lat:.8f}, lon={lon:.8f}")
        self.get_logger().info("=" * 20)
        self.land()


def main():
    rclpy.init(args=sys.argv)

    gps_points = parse_cli_args(sys.argv[1:])
    mission = hydro_basic(gps_points)

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