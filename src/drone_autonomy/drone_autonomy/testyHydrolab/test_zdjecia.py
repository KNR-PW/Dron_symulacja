#!/usr/bin/env python3
import sys
import rclpy
import argparse
import cv2
import os
from cv_bridge import CvBridge
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from drone_comunication.drone_controller import DroneController
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
)
from sensor_msgs.msg import Image
import math

ALTITUDE = 10.0
SAVE_DIR = "/root/ros_ws/photos"
os.makedirs(SAVE_DIR, exist_ok=True)
    

class hydro_basic(DroneController):
    def __init__(self, pool_gps):
        super().__init__("hydro_basic")
        self.declare_parameter('camera_topic', '/oak/rgb/image_raw')
        self.declare_parameter('save_dir', SAVE_DIR)

        self.pool_gps = pool_gps
        self.photo_idx = 0
        self.frame = None
        self.br = CvBridge()

        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.save_dir = self.get_parameter('save_dir').get_parameter_value().string_value
        os.makedirs(self.save_dir,exist_ok=True)
        # qos = QoSProfile(
        #     history=QoSHistoryPolicy.KEEP_LAST,
        #     depth=10,
        #     reliability=QoSReliabilityPolicy.BEST_EFFORT,
        #     durability=QoSDurabilityPolicy.VOLATILE,
        # )

        self.sub_camera = self.create_subscription(
            Image,
            camera_topic,
            self.camera_cb,
            10
        )


    def camera_cb(self, msg: Image):
        self.frame = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # self.frame = self.br.imgmsg_to_cv2(msg, desired_encoding='passthrough')


    def take_mission_photo(self):
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.05)

        if self.frame is None:
            self.get_logger().error(f"Brak klatki z kamery – zdjęcie {self.photo_idx} pominięte!")
            return
        
        filename = os.path.join(self.save_dir, f"zdjecie_{self.photo_idx}.jpg")
        cv2.imwrite(filename, self.frame)
        self.get_logger().info(f"Zdjęcie zapisane: {filename}")
        self.photo_idx += 1


    def run(self):
        self.arm()
        self.takeoff(ALTITUDE)

        for lat, lon in self.pool_gps[:-1]:
            self.get_logger().info(f"Flying to point: {lat:.8f}, {lon:.8f}")
            self.send_goto_relative(lat, lon, 0)
            self.take_mission_photo()

        latf, lonf = self.pool_gps[-1]
        self.send_goto_relative(latf, lonf, 0)
        self.take_mission_photo()

        self.rtl()


def main(args=None):
    rclpy.init(args=args)
    pool_gps = [
            (-2.0, -8.0),
            ( 0.0,  8.0),
            ( 2.0, -8.0),
            ( 0.0,  8.0),
        ]
    mission = hydro_basic(pool_gps)

    mission.run()
    rclpy.spin(mission)
    mission.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()