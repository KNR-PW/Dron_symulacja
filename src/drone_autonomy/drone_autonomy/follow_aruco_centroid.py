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


class FollowArucoCentroid(Node):
    def __init__(self):
        super().__init__('follow_aruco_centroid')

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

        # ---- Sterowanie dronem (jak w repo) ----
        self.ctrl = DroneController()

        # Publisher na wektory prędkości
        self.vel_pub = self.create_publisher(VelocityVectors, 'velocity_vectors', 10)

        # Opcjonalny serwis przełączający kontrolę prędkości
        self.toggle_cli = self.create_client(ToggleVelocityControl, 'toggle_v_control')

        # ---- Subskrypcja markera ----
        self.sub = self.create_subscription(MiddleOfAruco, self.aruco_topic, self.on_marker, 10)

        # Stan filtrowany błędu (normalized)
        self.ex_f = 0.0
        self.ey_f = 0.0
        self.last_seen = time.time()

        # Timer sterowania 10 Hz
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(
            f"Start follow_aruco_centroid: img=({self.img_w}x{self.img_h}) "
            f"kp={self.kp} max_vel={self.max_vel} deadband={self.deadband_px}px"
        )

        # Arm & takeoff — jak w repo
        try:
            self.ctrl.arm()
            self.ctrl.takeoff(self.target_alt)
            self.get_logger().info(f"Arm & takeoff to {self.target_alt} m OK")
        except Exception as e:
            self.get_logger().warn(f"arm/takeoff failed: {e}")

        # Spróbuj włączyć tryb sterowania wektorami (jeśli serwis istnieje)
        if self.toggle_cli.wait_for_service(timeout_sec=2.0):
            try:
                req = ToggleVelocityControl.Request()
                future = self.toggle_cli.call_async(req)
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
                res = future.result()
                self.get_logger().info(f"toggle_v_control: {getattr(res, 'result', None)}")
            except Exception as e:
                self.get_logger().warn(f"toggle_v_control failed: {e}")
        else:
            self.get_logger().warn("toggle_v_control service not available (continue anyway)")

    # ──────────────────────────────────────────────────────────
    def send_vectors(self, vx, vy, vz):
        msg = VelocityVectors()
        msg.vx = float(clamp(vx, -self.max_vel, self.max_vel))
        msg.vy = float(clamp(vy, -self.max_vel, self.max_vel))
        msg.vz = float(vz)  # zwykle 0
        self.vel_pub.publish(msg)

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
        vy = +self.kp * self.ex_f
        self.send_vectors(vx, vy, 0.0)

    # ──────────────────────────────────────────────────────────
    def destroy_node(self):
        # Zatrzymaj na wyjściu
        try:
            self.send_vectors(0.0, 0.0, 0.0)
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = FollowArucoCentroid()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
