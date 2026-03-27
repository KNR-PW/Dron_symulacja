#!/usr/bin/env python3
import sys
import rclpy
import argparse
from drone_comunication.drone_controller import DroneController
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
)
from px4_msgs.msg import SensorGps
import math
from drone_interfaces.srv import (
    MakePhoto
)

ALTITUDE = 30.0

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

        self.pool_gps  = pool_gps

        self.lat0 = None
        self.lon0 = None

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self.photo_idx = 0 
        self.photo_client = self.create_client(MakePhoto, 'nazwa_serwisu')
        
        self.sub = self.create_subscription(
            SensorGps,
            "/fmu/out/vehicle_gps_position",
            self.gps_cb,
            qos
        )

        self.get_logger().info("Waiting for GPS...")

        self.lat_current = None
        self.lon_current = None

# =====================================================================
#
# =====================================================================

    def calculate_distance(self, lat1, lon1, lat2, lon2):
        d_lat = (lat2 - lat1) * 111320.0
        d_lon = (lon2 - lon1) * 111320.0 * math.cos(math.radians(lat1))
        return math.sqrt(d_lat**2 + d_lon**2)

    def gps_cb(self, msg: SensorGps):
        if hasattr(msg, "fix_type") and msg.fix_type < 3:
            return

        # lat0/lon0 – tylko raz (punkt startowy)
        if self.lat0 is None:
            self.lat0 = float(msg.latitude_deg)
            self.lon0 = float(msg.longitude_deg)
            self.get_logger().info(f"Start GPS: lat={self.lat0:.8f}, lon={self.lon0:.8f}")

        # current – zawsze aktualizowane
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
        # self.lat0, self.lon0 = self.get_gps()
        lat0 = self.lat0
        lon0 = self.lon0

        visited_points = []

        self.arm()
        self.takeoff(ALTITUDE)
        self.photo_client.wait_for_service(timeout_sec=5.0)
        for lat, lon in self.pool_gps[:-1]:
            self.get_logger().info(
                f"Flying to GPS: {lat:.8f}, {lon:.8f}"
            )
            self.send_goto_global(lat, lon, ALTITUDE)
            self.take_mission_photo()

            # gps = self.get_gps()
            # if gps is not None:
            #     actual_lat, actual_lon = gps
            #     visited_points.append((actual_lat, actual_lon))
            #     self.get_logger().info(f"Reached: {actual_lat:.8f}, {actual_lon:.8f}")
            # else:
            #     self.get_logger().warn("Could not get GPS, saving planned coordinates instead")
            #     visited_points.append((lat, lon))

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

# ================================================================
# MAIN
# ================================================================

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