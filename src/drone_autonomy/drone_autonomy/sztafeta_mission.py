import threading
import time
from datetime import datetime
import os

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from drone_interfaces.msg import ArucoMarkers, Telemetry
from drone_interfaces.srv import GetLocationRelative, GetAttitude, SetMode, SetSpeed, PostLog, Dropper
from drone_interfaces.action import Arm, Takeoff, GotoRelative, GotoGlobal, SetYawAction

from drone_comunication.drone_controller import DroneController

import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import json

import sys
from pathlib import Path
import cv2
import numpy as np

class MissionRunner(DroneController):
    def __init__(self, waypoints, beacons_nums, droppers_nums, flight_alt):
        super().__init__()

        self.mission_id = None
        self.waypoints = waypoints
        self.flight_alt = flight_alt
        self.beacons_nums = beacons_nums
        self.droppers_nums = droppers_nums

        if len(self.beacons_nums) != len(self.waypoints):
            self.get_logger().error(f"Invalid input data ")

        self._latest_image = None
        self._cv_bridge = CvBridge()

        self._mission_started = False

        self.dropper_client = self.create_client(Dropper, 'dropper')


        threading.Thread(target=self._delayed_start, daemon=True).start()

    def _delayed_start(self):
        time.sleep(2.0)
        self._run_mission()


    def test_beacon(self, num):
        self.send_beacon_msg("b"+str(i)+"r")
        time.sleep(2.0)
        self.send_beacon_msg("d"+str(i))

    def send_beacon_msg(self, beacon_msg):
        self.get_logger().info(f"Senting beacon msg: {beacon_msg}")
        try:
            
            while not self.dropper_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Waiting for dropper service...')
            request = Dropper.Request()
            request.beacon_msg = beacon_msg
            future = self.dropper_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                if future.result().error:
                    self.get_logger().error(f"Failed to send msg to beacon {beacon_msg}")
                else:
                    self.get_logger().info(f"Beacon msg sent successfully: {beacon_msg}")
            else:
                self.get_logger().error('Service call failed')
        except Exception as e:
            self.get_logger().error(f"Exception in beacon msg: {e}")
    
    def _run_mission(self):

        if self._mission_started:
            return
        self._mission_started = True


        if not self.arm():
            self.get_logger().error("Arm failed; aborting mission.")
            return
        if not self.takeoff(self.flight_alt):
            self.get_logger().error("Takeoff failed; aborting mission.")
            return

        time.sleep(2.0)  

        for idx, (north, east, down) in enumerate(self.waypoints, start=1):
            self.get_logger().info(f"Heading to waypoint {idx}: N={north}, E={east}, D={down}")

            if not self.send_goto_global(north, east, down):
                self.get_logger().error(f"Goto waypoint {idx} failed; skipping.")
                continue

            time.sleep(3.0)
            self.get_logger().info(f"Arrived to waypoint {idx}")

            beacon = self.beacons_nums[idx-1]
            dropper = self.droppers_nums[idx-1]

            self.get_logger().info(f"Sending beacon message for beacon {beacon} and dropper {dropper}")
            self.send_beacon_msg("b"+str(beacon)+"r")
            time.sleep(2.0)
            self.send_beacon_msg("d"+str(dropper))
            time.sleep(2.0)

            
        self.rtl()
        return



def main(args=None):
    rclpy.init(args=args)
    alt = 10.0
    # waypoints = [(2.0, 0.0, 0.0), (0.0, 3.0, 0.0), (-2.0, 0.0, 0.0), (0.0, -3.0, 0.0)]
    waypoints = [
        (-35.363319396972656, 149.16531372070312, 10),
        (-35.36327258544922, 149.16510009765625, 10)
        # (50.2715662, 18.6443051, alt),
        # (50.2714623, 18.6442565, alt),
        # (50.2717372, 18.6440945, alt),
        # (50.2719005, 18.6444969, alt)
        
        ]
    beacons = [1, 2]
    droppers = [1, 2]
    
    node = MissionRunner(waypoints, beacons, alt)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
