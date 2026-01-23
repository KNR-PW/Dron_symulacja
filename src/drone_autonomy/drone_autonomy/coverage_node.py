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

from .drone_comunication.drone_controller import DroneController
from px4_msgs.msg import SensorGps


try:
    from geographiclib.geodesic import Geodesic
    _HAS_GEOGRAPHICLIB = True
except Exception:
    _HAS_GEOGRAPHICLIB = False
    

def gps_to_local(gps_points, start_point):
    lat0, lon0 = start_point
    if not _HAS_GEOGRAPHICLIB:
        raise ImportError("Brak biblioteki geographiclib")
    
    geod = Geodesic.WGS84

    for lat, lon in gps_points:
        p=geod.Inverse(lat0, lon0, lat, lon)
        local_points = []

        distance = p['s12']
        azi_deg = p['azi1']
        azi_rad = math.radians(azi_deg)

        xn = distance * math.cos(azi_rad)
        yn = distance * math.sin(azi_rad)
        local_points.append((xn, yn))
    return local_points

def get_points_to_meters(gps_points, start_point, coef_lat, coef_lon):
    lat0, lon0 = start_point
    calculated_points = []
    for lat, lon in gps_points:
        dy = (lat - lat0)*1000/coef_lat
        dx = (lon - lon0)*1000/coef_lon
        calculated_points.append((dx, dy))

    return calculated_points

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


def nearest_vertex(start_point, polygon_points):
    sx, sy = start_point
    best = None
    best_d2 = float("inf")
    for (x, y) in polygon_points:
        d2 = (x - sx) ** 2 + (y - sy) ** 2
        if d2 < best_d2:
            best_d2 = d2
            best = (x, y)
    return best


def lawnmower_path(points, step, angle_deg=0, start_point=(0.0, 0.0)):
    centroid = polygon_centroid(points)

    # obrót wielokąta
    rotated = [rotate_point(p, -angle_deg, centroid) for p in points]

    ys = [p[1] for p in rotated]
    miny, maxy = min(ys), max(ys)

    waypoints_rot = []
    direction = 1
    y = miny

    while y <= maxy:
        intersections = []

        for i in range(len(rotated)):
            p1 = rotated[i]
            p2 = rotated[(i + 1) % len(rotated)]

            x = line_segment_intersection_y(p1, p2, y)
            if x is not None:
                intersections.append(x)

        intersections.sort()

        if len(intersections) >= 2:
            x1, x2 = intersections[0], intersections[-1]
            segment = [(x1, y), (x2, y)]
            if direction == -1:
                segment.reverse()
            waypoints_rot.extend(segment)
            direction *= -1

        y += step


    waypoints = [rotate_point(p, angle_deg, centroid) for p in waypoints_rot]

    entry_vertex = nearest_vertex(start_point, points)

    full_points = [start_point]
    if entry_vertex is not None and entry_vertex != start_point:
        full_points.append(entry_vertex)

    full_points.extend(waypoints)

    return full_points

def local_to_gps(local_points, start_point):
    sx, sy = start_point
    gps_points=[]
    
    for points in local_points:
        x, y=points

        distance = math.sqrt(x**2 + y**2)
        azimuth = math.degrees(math.atan2(y, x))
    
        geod = Geodesic.WGS84
        result = geod.Direct(sx, sy, azimuth, distance)
        gps_points.append((result['lat2'], result['lon2']))
    return gps_points

def _parse_point(text: str):
    parts = [p.strip() for p in text.split(",")]
    if len(parts) != 2:
        raise ValueError(f"Zły format punktu '{text}'. Użyj: lat,lon")
    return (float(parts[0]), float(parts[1]))


def parse_cli_args(argv):
    parser = argparse.ArgumentParser(
        prog="coverage_node",
        description="Coverage node: podaj 4 punkty jako lat,lon (bez wysokości)."
    )

    parser.add_argument("--p1", required=True, help='Punkt 1 w formacie "lat,lon"')
    parser.add_argument("--p2", required=True, help='Punkt 2 w formacie "lat,lon"')
    parser.add_argument("--p3", required=True, help='Punkt 3 w formacie "lat,lon"')
    parser.add_argument("--p4", required=True, help='Punkt 4 w formacie "lat,lon"')

    parser.add_argument("--step", type=float, default=5.0, help="Odstęp pasów (m), > 0")
    parser.add_argument("--angle", type=float, default=0.0, help="Rotacja trasy (deg)")
    parser.add_argument(
        "--goto_mode",
        choices=["absolute_local", "delta_current"],
        default="absolute_local",
        help=(
            "absolute_local: wysyłaj waypointy ABSOLUTNE w lokalnym NED originu autopilota.\n"
            "delta_current: wysyłaj delty (dN,dE) od aktualnej pozycji.\n"
            "Jeśli nie wiesz, zacznij od absolute_local."
        )
    )

    args, _unknown = parser.parse_known_args(argv)
    points = [_parse_point(args.p1), _parse_point(args.p2), _parse_point(args.p3), _parse_point(args.p4)]
    return points, args.step, args.angle, args.goto_mode


class CoverageNode(DroneController):
    def __init__(self, points, step, angle, goto_mode):
        super().__init__('gps_init')

        self.points = points
        self.step = float(step)
        self.angle = float(angle)
        self.goto_mode = goto_mode

        #Punkt startowy
        self.lat0 = None
        self.lon0 = None

        # elemnty do kalibracji:
        self.lat_coefficient = None
        self.lon_coefficient = None

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.sub = self.create_subscription(
            SensorGps,
            '/fmu/out/vehicle_gps_position',
            self.gps_cb,
            qos
        )

        self.get_logger().info("Czekam na pierwszy poprawny GPS (fix_type >= 3)...")

    def gps_cb(self, msg: SensorGps):
        if self.lat0 is not None:
            return
        if hasattr(msg, 'fix_type') and msg.fix_type < 3:
            return

        self.lat0 = float(msg.latitude_deg)
        self.lon0 = float(msg.longitude_deg)

        self.get_logger().info(
            f"\033[94mStartowe GPS: lat={self.lat0:.8f}, lon={self.lon0:.8f}\033[0m"
        )

        # tylko pierwsza próbka
        self.destroy_subscription(self.sub)

    def wait_for_gps_fix(self, timeout_sec=20.0):
        start = self.get_clock().now()
        while rclpy.ok() and self.lat0 is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
            if elapsed >= timeout_sec:
                return False
        return self.lat0 is not None


    def _read_gps_once(self, timeout_sec: float = 10.0):
        self._tmp_lat = None
        self._tmp_lon = None

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        def _cb(msg: SensorGps):
            if hasattr(msg, "fix_type") and msg.fix_type < 3:
                return
            self._tmp_lat = float(msg.latitude_deg)
            self._tmp_lon = float(msg.longitude_deg)

        tmp_sub = self.create_subscription(
            SensorGps,
            "/fmu/out/vehicle_gps_position",
            _cb,
            qos
        )

        try:
            start = self.get_clock().now()
            while rclpy.ok() and (self._tmp_lat is None or self._tmp_lon is None):
                rclpy.spin_once(self, timeout_sec=0.1)
                elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
                if elapsed >= timeout_sec:
                    self.get_logger().warn("Timeout: nie odebrałem próbki GPS.")
                    return (None, None)

            return (self._tmp_lat, self._tmp_lon)

        finally:
            try:
                self.destroy_subscription(tmp_sub)
            except Exception:
                pass

    def _log_points(self, title, pts):
        ORANGE = "\033[38;5;208m"
        RESET = "\033[0m"
        self.get_logger().info(
            ORANGE +
            f"\n=== {title} ===\n" +
            "\n".join([f"{i:02d}: {p}" for i, p in enumerate(pts)]) +
            "\n====================" +
            RESET
        )

    def send_go_round(self, gps_points, start_point, step, angle):
        for point in gps_points:
            x,y = point
            self.get_logger().info(f"\033[35m  {x}, {y} \033[0m")
        trajectory = lawnmower_path(gps_points, step, angle, start_point)
        local_to_gps(trajectory, start_point)
        for vector in local_to_gps:
            lat, lon = vector
            z = 0.0
            self.get_logger().info(f'Lece do: N:{lat}, E:{lon}, D:{z}')
            self.send_goto_global(lat, lon, z)
    


def main():
    rclpy.init(args=sys.argv)
    points, step, angle, goto_mode = parse_cli_args(sys.argv[1:])
    node = CoverageNode(points, step, angle, goto_mode)
    ok = node.wait_for_gps_fix(timeout_sec=30.0)
    if not ok:
        node.get_logger().error("Nie udało się złapać GPS fix w czasie. Kończę.")
        node.destroy_node()
        rclpy.shutdown()
        return
    
    gps_start_point = (node.lat0, node.lon0)

    local_points = gps_to_local(points, gps_start_point)

    node.arm()
    node.takeoff(9.0)
    try:
        node.send_go_round(local_points, gps_start_point, step, angle)

        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
