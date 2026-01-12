#!/usr/bin/env python3
"""
Auto-generated mission: mission_20260107_042515
Summary: Ascend to 10 meters, fly to relative coordinates (25,-5,0), take photo, and return home. (1 photo(s))
Generated: 2026-01-07 04:25:15

DroneController methods used:
- arm() - Arm and set GUIDED mode
- takeoff(altitude) - Take off to altitude
- land() - Land at current position
- rtl() - Return to launch
- send_goto_relative(north, east, down) - Move relative
- send_goto_global(lat, lon, alt) - Move to GPS
- send_set_yaw(yaw_rad, relative) - Set heading
- set_speed(speed) - Set flight speed
"""
import rclpy
import time
from drone_autonomy.drone_comunication import DroneController
from drone_interfaces.srv import MakePhoto


def take_photo(node, prefix='photo'):
    """Robi zdjęcie przez serwis /make_photo"""
    client = node.create_client(MakePhoto, '/make_photo')
    if not client.wait_for_service(timeout_sec=2.0):
        node.get_logger().warn('Photo service not available!')
        return False
    
    req = MakePhoto.Request()
    req.prefix = prefix
    req.ext = 'jpg'
    
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    
    if future.result() is not None:
        result = future.result().success
        node.get_logger().info(f'Photo: {result}')
        return 'saved' in result.lower()
    return False


def main(args=None):
    rclpy.init(args=args)
    mission = DroneController()
    
    print("🚁 Starting mission: mission_20260107_042515")
    
    # Arm drone
    mission.arm()
    time.sleep(2)
    
    # === MISSION WAYPOINTS ===
    # Step 1: Takeoff to 10.0m
    mission.takeoff(10.0)
    time.sleep(3)
    # Step 2: Move relative N:25.0, E:-5.0, D:0.0
    mission.send_goto_relative(25.0, -5.0, 0.0)
    time.sleep(2)
    take_photo(mission, 'photo')
    time.sleep(1)
    # Step 4: Return to launch
    mission.rtl()
    time.sleep(2)
    
    print("✅ Mission complete!")
    
    mission.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
