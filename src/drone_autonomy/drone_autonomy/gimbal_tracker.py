#!/usr/bin/env python3
"""
Minimalny tester gimbala — sledzi namiot TYLKO pitchem gimbala (bez ruchu drona).
Logika gimbala odwzorowana 1:1 ze stanu APPROACH w tent_tracker:
    ey = (srodek_namiotu_y - srodek_obrazu_y) / (polowa_wysokosci)
    gimbal += kp_gimbal * ey / control_rate
Publikuje kat pitch na gz topic /gimbal/cmd_pitch (przez subprocess),
dokladnie jak tent_tracker. Sluzy do izolowanego testu sledzenia gimbalem.
"""

import subprocess
import threading
import time

import rclpy
from rclpy.node import Node

from drone_interfaces.msg import TentDetection


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class GimbalTracker(Node):
    # Zakres gimbala (rad): 0.0 = prosto, 1.05 = max w dol (~60 deg)
    GIMBAL_MIN = 0.0
    GIMBAL_MAX = 1.57  # ~prosto w dol (zweryfikowane empirycznie)
    GIMBAL_SEARCH = 0.30  # ~lekko w dol/przod — pozycja gdy brak namiotu

    def __init__(self):
        super().__init__('gimbal_tracker')

        # Rozdzielczosc kamery 640x480 (nie 1920x1080 jak domyslnie w tent_tracker!)
        self.declare_parameter('kp_gimbal', 1.5)
        self.declare_parameter('img_h', 1024)
        self.declare_parameter('control_rate', 10.0)
        self.declare_parameter('lost_timeout', 2.0)
        self.declare_parameter('deadzone', 0.06)   # martwa strefa wokol srodka

        self.kp_gimbal    = self.get_parameter('kp_gimbal').value
        self.img_h        = self.get_parameter('img_h').value
        self.control_rate = self.get_parameter('control_rate').value
        self.lost_timeout = self.get_parameter('lost_timeout').value
        self.deadzone     = self.get_parameter('deadzone').value

        self.tent_detected = False
        self.tent_cy = 0.0
        self.last_det_time = 0.0

        self.gimbal_rad = self.GIMBAL_SEARCH
        self._gimbal_target = self.GIMBAL_SEARCH
        self._gimbal_lock = threading.Lock()

        self.create_subscription(TentDetection, '/tent_detections', self._det_cb, 10)

        # Watek wysylajacy komendy gimbala na gz topic (jak w tent_tracker)
        self._gimbal_thread = threading.Thread(target=self._gimbal_worker, daemon=True)
        self._gimbal_thread.start()

        self._timer = self.create_timer(1.0 / self.control_rate, self._loop)
        self.get_logger().info(
            f"GimbalTracker gotowy | kp_gimbal={self.kp_gimbal} img_h={self.img_h}")

    def _det_cb(self, msg: TentDetection):
        if msg.detected:
            bb = msg.bounding_box  # [x, y, w, h]
            self.tent_cy = bb[1] + bb[3] / 2.0
            self.tent_detected = True
            self.last_det_time = time.time()
        else:
            self.tent_detected = False

    def _set_gimbal(self, rad):
        with self._gimbal_lock:
            self._gimbal_target = clamp(rad, self.GIMBAL_MIN, self.GIMBAL_MAX)

    def _gimbal_worker(self):
        last_sent = -1.0
        while True:
            with self._gimbal_lock:
                target = self._gimbal_target
            if abs(target - last_sent) > 0.01:
                cmd = [
                    "gz", "topic",
                    "-t", "/gimbal/cmd_pitch",
                    "-m", "gz.msgs.Double",
                    "-p", f"data: {target:.3f}",
                ]
                try:
                    subprocess.run(cmd, capture_output=True, timeout=2.0)
                except Exception:
                    pass
                last_sent = target
                self.gimbal_rad = target
            time.sleep(0.1)

    def _loop(self):
        now = time.time()
        dt_lost = now - self.last_det_time if self.last_det_time > 0 else 999.0

        # Do SEARCH dopiero po dluzszym braku detekcji (lost_timeout),
        # a nie przy pojedynczej pustej klatce — inaczej gimbal "skacze".
        if dt_lost > self.lost_timeout:
            self._set_gimbal(self.GIMBAL_SEARCH)
            self.get_logger().info("[SZUKAM] namiot zgubiony → gimbal SEARCH",
                                   throttle_duration_sec=2.0)
            return

        # W oknie lost_timeout uzywamy OSTATNIEJ znanej pozycji namiotu
        # (jesli aktualna klatka byla pusta, trzymamy gimbal w miejscu).
        half_h = self.img_h / 2.0
        ey = (self.tent_cy - half_h) / half_h

        # Martwa strefa: blisko srodka nie ruszamy gimbala (koniec drgan)
        if abs(ey) < self.deadzone:
            self.get_logger().info(
                f"[CENTR] namiot wycentrowany ey={ey:+.2f} gimbal={self.gimbal_rad:.2f}",
                throttle_duration_sec=1.0)
            return

        # Ta sama formula co APPROACH w tent_tracker
        gimbal_delta = self.kp_gimbal * ey * (1.0 / self.control_rate)
        self._set_gimbal(self.gimbal_rad + gimbal_delta)

        self.get_logger().info(
            f"[SLEDZE] cy={self.tent_cy:.0f} ey={ey:+.2f} gimbal={self.gimbal_rad:.2f}",
            throttle_duration_sec=0.5)


def main(args=None):
    rclpy.init(args=args)
    node = GimbalTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
