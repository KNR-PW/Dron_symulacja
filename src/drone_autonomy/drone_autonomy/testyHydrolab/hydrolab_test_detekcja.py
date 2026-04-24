#!/usr/bin/env python3
import sys
import argparse
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from drone_comunication.drone_controller import DroneController
from Hydrolab import PoolDetector, center_over_pool
from rclpy.executors import MultiThreadedExecutor
import rclpy
import threading
import time

ALTITUDE = 10.0

def _point(text):
    lat, lon = text.split(",")
    return float(lat), float(lon)

def parse_cli_args(argv):
    parser = argparse.ArgumentParser(
        prog="hydrolab live",
        description="Leci do basenu po GPS, wykrywa i centruje"
    )
    parser.add_argument("--p1", required=True)
    parser.add_argument("--alt", type=float, default=ALTITUDE)

    args, _ = parser.parse_known_args(argv)
    return _point(args.p1), args.alt


def run(mission: DroneController, detector: PoolDetector, pool_point, altitude):
    lat, lon = pool_point

    mission.arm()
    mission.takeoff(altitude)

    mission.get_logger().info(f"Flying to pool GPS: {lat:.8f}, {lon:.8f}")
    mission.send_goto_global(lat, lon, altitude)
    time.sleep(5.0)

    found = detector.scan(3.0)
    if not found:
        mission.get_logger().warn("Pool not detected — returning.")
        mission.rtl()
        return

    mission.get_logger().info("Pool detected — centering.")
    center_over_pool(mission, detector, tag="pool1")

    mission.get_logger().info("Descending to 3 m.")
    mission.send_goto_relative(0.0, 0.0, 7.0)
    time.sleep(5.0)

    mission.get_logger().info("Hovering at 3 m...")
    time.sleep(3.0)

    det = detector.get_live_offset()
    if det is not None:
        _, _, frame, boxes = det
        detector.save_annotated(frame, boxes, tag="lowalt")

    mission.get_logger().info("Ascending back to 10 m.")
    mission.send_goto_relative(0.0, 0.0, -7.0)
    time.sleep(5.0)

    mission.get_logger().info("Returning home.")
    mission.rtl()


def main():
    rclpy.init(args=sys.argv)

    pool_point, altitude = parse_cli_args(sys.argv[1:])
    mission = DroneController("hydrolab_live")
    detector = PoolDetector(mission)

    executor = MultiThreadedExecutor()
    executor.add_node(mission)
    executor.add_node(detector)

    t = threading.Thread(target=run, args=(mission, detector, pool_point, altitude), daemon=True)
    t.start()

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        t.join(timeout=2.0)
        mission.destroy_node()
        detector.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
