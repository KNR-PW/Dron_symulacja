#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import rclpy
from rclpy.node import Node

from drone_interfaces.msg import MiddleOfAruco, VelocityVectors
from drone_interfaces.srv import ToggleVelocityControl
from drone_autonomy.drone_comunication.drone_controller import DroneController


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class FollowArucoCentroid(DroneController):
    def __init__(self):
        super().__init__()

        # ---- Parametry ----
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('aruco_topic', '/aruco_markers')
        self.declare_parameter('target_alt', 2.0)
        self.declare_parameter('kp', 0.005)
        self.declare_parameter('deadband_px', 20)
        self.declare_parameter('max_vel', 1.0)
        self.declare_parameter('lowpass', 0.3)
        self.declare_parameter('lost_timeout', 0.8)  # s

        self.img_w = int(self.get_parameter('image_width').value)
        self.img_h = int(self.get_parameter('image_height').value)
        self.cx = self.img_w / 2.0
        self.cy = self.img_h / 2.0
        self.kp = float(self.get_parameter('kp').value)
        self.deadband_px = int(self.get_parameter('deadband_px').value)
        self.max_vel = float(self.get_parameter('max_vel').value)
        self.lowpass = float(self.get_parameter('lowpass').value)
        self.lost_timeout = float(self.get_parameter('lost_timeout').value)
        self.aruco_topic = str(self.get_parameter('aruco_topic').value)
        self.target_alt = float(self.get_parameter('target_alt').value)

        self.state = "OK"

        # ---- Subskrypcja markera ----
        self.sub = self.create_subscription(MiddleOfAruco, self.aruco_topic, self.on_marker, 10)

        # Stan filtrowany błędu (normalized)
        self.ex_f = 0.0
        self.ey_f = 0.0
        self.last_seen = time.time()

        self.get_logger().info(
            f"Start follow_aruco_centroid: img=({self.img_w}x{self.img_h}) "
            f"kp={self.kp} max_vel={self.max_vel} deadband={self.deadband_px}px"
        )

        #Definiowanie misji

        # Arm & takeoff — jak w repo
        # try:
        #     self.arm()
        #     self.takeoff(self.target_alt)
        #     self.get_logger().info(f"Arm & takeoff to {self.target_alt} m OK")
        # except Exception as e:
        #     self.get_logger().warn(f"arm/takeoff failed: {e}")

        # Spróbuj włączyć tryb sterowania wektorami (jeśli serwis istnieje)



    # ──────────────────────────────────────────────────────────
    def on_marker(self, msg: MiddleOfAruco):
        # błąd w pikselach względem środka obrazu
        ex_px = float(msg.x) - self.cx
        ey_px = float(msg.y) - self.cy

        # martwa strefa
        if abs(ex_px) < self.deadband_px:
            ex_px = 0.0
        if abs(ey_px) < self.deadband_px:
            ey_px = 0.0

        # normalizacja do [-1,1]
        ex = ex_px / (self.img_w / 2.0)
        ey = ey_px / (self.img_h / 2.0)

        # low-pass
        a = clamp(self.lowpass, 0.0, 1.0)
        self.ex_f = (1 - a) * self.ex_f + a * ex
        self.ey_f = (1 - a) * self.ey_f + a * ey

        self.last_seen = time.time()

    # ──────────────────────────────────────────────────────────
    def control_loop(self):
        # brak markera niedawno → wyhamuj
        if (time.time() - self.last_seen) > self.lost_timeout:
            self.send_vectors(0.0, 0.0, 0.0)
            return

        # Proste P: ex -> vy, ey -> vx (kamera do przodu)
        vx = -self.kp * self.ey_f
        vx = clamp(vx, -self.max_vel, self.max_vel)
        vy = +self.kp * self.ex_f
        vy = clamp(vy, -self.max_vel, self.max_vel)
        if abs(self.ey_f) < 0.1 and abs(self.ex_f) < 0.1:
            self.send_vectors(0.0, 0.0, 0.0)
            self.state = "OK"
            self.timer.cancel()

        self.send_vectors(vx, vy, 0.0)

    # ──────────────────────────────────────────────────────────
    def fly_to_aruco(self):
        self.state = "BUSY"
        self.get_logger().info("wlaczam nadlatywanie do aruco")
        # Timer sterowania 10 Hz
        self.timer = self.create_timer(0.1, self.control_loop)
        self.wait_busy()


    def wait_busy(self):
        self.get_logger().info("busy")
        while self.state == "BUSY":
            rclpy.spin_once(self, timeout_sec=0.05)

    def destroy_node(self):
        # Zatrzymaj na wyjściu
        try:
            self.send_vectors(0.0, 0.0, 0.0)
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    mission = FollowArucoCentroid()
    mission.arm()
    mission.takeoff(5.0)
    mission.toggle_control()
    mission.fly_to_aruco()
    mission.toggle_control()
    mission.land()


if __name__ == '__main__':
    main()
