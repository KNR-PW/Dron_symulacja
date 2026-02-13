#!/usr/bin/env python3
import sys
import argparse
import math

import rclpy
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
)

from px4_msgs.msg import SensorGps
from .drone_comunication.drone_controller import DroneController


# ============================================================
# GEO / METRY
# ============================================================
ALTITUDE = 30.0  # metry AGL


def meters_to_gps(lat0, lon0, d_north, d_east):
    """
    Proste i wystarczające przybliżenie lokalne:
    d_north, d_east w metrach → delta GPS
    """
    lat_rad = math.radians(lat0)
    d_lat = d_north / 111_320.0
    d_lon = d_east / (111_320.0 * math.cos(lat_rad))
    return lat0 + d_lat, lon0 + d_lon


# ============================================================
# GEOMETRIA
# ============================================================

def rotate_point(p, angle_deg, origin):
    angle = math.radians(angle_deg)
    ox, oy = origin
    px, py = p
    qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
    qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
    return qx, qy


def polygon_centroid(points):
    x = sum(p[0] for p in points) / len(points)
    y = sum(p[1] for p in points) / len(points)
    return x, y


def line_segment_intersection_y(p1, p2, y):
    x1, y1 = p1
    x2, y2 = p2

    if (y1 < y and y2 < y) or (y1 > y and y2 > y):
        return None
    if y1 == y2:
        return None

    t = (y - y1) / (y2 - y1)
    if 0 <= t <= 1:
        return x1 + t * (x2 - x1)
    return None


def lawnmower_path(points, step, angle_deg=0.0):
    """
    points: lista punktów (E, N) w metrach
    zwraca listę punktów ABSOLUTNYCH (E, N)
    """
    centroid = polygon_centroid(points)
    rotated = [rotate_point(p, -angle_deg, centroid) for p in points]

    ys = [p[1] for p in rotated]
    miny, maxy = min(ys), max(ys)

    waypoints_rot = []
    direction = 1
    y = miny

    while y <= maxy:
        xs = []
        for i in range(len(rotated)):
            p1 = rotated[i]
            p2 = rotated[(i + 1) % len(rotated)]
            x = line_segment_intersection_y(p1, p2, y)
            if x is not None:
                xs.append(x)

        xs.sort()
        if len(xs) >= 2:
            seg = [(xs[0], y), (xs[-1], y)]
            if direction < 0:
                seg.reverse()
            waypoints_rot.extend(seg)
            direction *= -1

        y += step

    return [rotate_point(p, angle_deg, centroid) for p in waypoints_rot]


# ============================================================
# CLI
# ============================================================

def _parse_point(text):
    lat, lon = text.split(",")
    return float(lat), float(lon)


def parse_cli_args(argv):
    parser = argparse.ArgumentParser(
        prog="coverage_node",
        description="Coverage node – 4 punkty GPS"
    )

    parser.add_argument("--p1", required=True)
    parser.add_argument("--p2", required=True)
    parser.add_argument("--p3", required=True)
    parser.add_argument("--p4", required=True)

    parser.add_argument("--step", type=float, default=14.0,
                        help="Odstęp pasów (m)")
    parser.add_argument("--angle", type=float, default=0.0,
                        help="Rotacja trasy (deg)")

    args, _ = parser.parse_known_args(argv)
    points = [_parse_point(args.p1),
              _parse_point(args.p2),
              _parse_point(args.p3),
              _parse_point(args.p4)]
    return points, args.step, args.angle


# ============================================================
# NODE
# ============================================================

class CoverageNode(DroneController):

    def __init__(self, gps_points, step, angle):
        super().__init__("coverage_node")

        self.gps_points = gps_points
        self.step = step
        self.angle = angle

        self.lat0 = None
        self.lon0 = None

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self.sub = self.create_subscription(
            SensorGps,
            "/fmu/out/vehicle_gps_position",
            self.gps_cb,
            qos
        )

        self.get_logger().info("Czekam na GPS fix...")

    def gps_cb(self, msg: SensorGps):
        if self.lat0 is not None:
            return
        if hasattr(msg, "fix_type") and msg.fix_type < 3:
            return

        self.lat0 = float(msg.latitude_deg)
        self.lon0 = float(msg.longitude_deg)

        self.get_logger().info(
            f"Start GPS: lat={self.lat0:.8f}, lon={self.lon0:.8f}"
        )
        self.destroy_subscription(self.sub)

    def wait_for_gps(self, timeout=30.0):
        start = self.get_clock().now()
        while rclpy.ok() and self.lat0 is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (self.get_clock().now() - start).nanoseconds / 1e9 > timeout:
                return False
        return True

    def run(self):
        # === GPS → metry (ENU)
        lat_ref, lon_ref = self.gps_points[0]
        lat_rad = math.radians(lat_ref)

        poly_m = []
        for lat, lon in self.gps_points:
            d_n = (lat - lat_ref) * 111_320.0
            d_e = (lon - lon_ref) * 111_320.0 * math.cos(lat_rad)
            poly_m.append((d_e, d_n))

        # === planowanie
        waypoints_m = lawnmower_path(poly_m, self.step, self.angle)

        # === lot
        self.arm()
        self.takeoff(ALTITUDE)

        for e, n in waypoints_m:
            lat, lon = meters_to_gps(lat_ref, lon_ref, n, e)
            self.get_logger().info(
                f"Lecę do GPS: {lat:.8f}, {lon:.8f}"
            )
            self.send_goto_global(lat, lon,ALTITUDE)


# ============================================================
# MAIN
# ============================================================

def main():
    rclpy.init(args=sys.argv)

    gps_points, step, angle = parse_cli_args(sys.argv[1:])
    node = CoverageNode(gps_points, step, angle)

    if not node.wait_for_gps():
        node.get_logger().error("Brak GPS fixa.")
        rclpy.shutdown()
        return

    node.run()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
