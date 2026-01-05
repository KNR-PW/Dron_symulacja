#!/usr/bin/env python3
"""
Auto-generated mission: mission_20260102_163926
Summary: Drone launches 10m up, flies 15m east, takes photo, then lands. (1 photo(s))
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
    
    mission.arm()
    mission.takeoff(10.0)
    time.sleep(2)
    mission.send_goto_relative(0.0, 15.0, 0.0)
    time.sleep(2)
    take_photo(mission, 'photo_1')
    time.sleep(1)
    mission.land()
    
    mission.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
