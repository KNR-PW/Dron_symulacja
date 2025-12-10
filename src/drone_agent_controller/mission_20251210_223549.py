#!/usr/bin/env python3
"""
Auto-generated mission: mission_20251210_223549
Summary: Square flight pattern: takeoff to 10m, fly forward 10m, right 10m, backward 10m, left 10m, then land
"""

import rclpy
import time
from drone_comunication import DroneController


def main(args=None):
    rclpy.init(args=args)
    mission = DroneController()
    
    mission.arm()
    mission.takeoff(10.0)
    time.sleep(2)
    mission.send_goto_relative(10.0, 0.0, 0.0)
    time.sleep(2)
    mission.send_goto_relative(0.0, 10.0, 0.0)
    time.sleep(2)
    mission.send_goto_relative(-10.0, 0.0, 0.0)
    time.sleep(2)
    mission.send_goto_relative(0.0, -10.0, 0.0)
    time.sleep(2)
    mission.land()
    
    mission.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
