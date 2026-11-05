#!/usr/bin/env python3
"""
Minimalny tester gimbala — sledzi namiot pitchem gimbala (bez ruchu drona).
Steruje przez ROS topic knr_hardware/gimbal_pitch -> drone_handler -> MAVLink
(DO_MOUNT_CONTROL) -> ArduPilot -> serwo montazu. TA SAMA sciezka co na realu.

Konwencja katow (real, MNT1_PITCH_MIN/MAX = -90/45 wg docs/gimbal setup.md):
      0 deg = poziomo (w przod)
    -45 deg = pod katem w dol/przod  (pozycja SEARCH)
    -90 deg = prosto w dol
Wiec: namiot ponizej srodka (ey>0) => patrz bardziej w dol => pitch maleje.

UWAGA: domyslne parametry sa dla REALU (OAK-D PRO W: img_h 1024, vfov 64.4)
i w Gazebo dzialaja BEZ ZMIAN. Symulacja ma teraz te sama kamere
(gimbal_small_3d: 1024x1024, FOV 64.4 deg) i te sama sciezke gimbala
(DO_SET_SERVO(7) -> kanal 6 w modelu iris_with_gimbal, kalibracja
1100 us = -90 deg, 1600 us = -45 deg). Nic nie nadpisujemy.
"""

import time

import rclpy
from drone_autonomy.drone_comunication.drone_controller import DroneController

from drone_interfaces.msg import TentDetection


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class SuasGimbalController(DroneController):

    def __init__(self):
        super().__init__('suas_gimbal_controller')

        # Zakres pracy gimbala w STOPNIACH — parametry, bo Gazebo i real maja
        # rozne kalibracje (patrz docstring modulu).
        self.declare_parameter('pitch_search', -45.0)  # gdy brak namiotu: pod katem w dol/przod
        self.declare_parameter('pitch_min', -90.0)     # najnizej: prosto w dol
        self.declare_parameter('pitch_max', -45.0)     # najwyzej: pod katem w dol/przod

        # Sterowanie geometryczne: kat liczony WPROST z bledu i pionowego FOV kamery,
        # z tlumieniem. Bez "magicznego" kp.
        self.declare_parameter('vfov_deg', 64.4)    # zmierzony pionowy FOV OAK-D PRO W
        self.declare_parameter('damping', 0.4)      # 1.0 = deadbeat (1 krok), mniej = lagodniej
        self.declare_parameter('img_h', 1024)       # wysokosc klatki: kwadratowe preview 1024x1024
        self.declare_parameter('control_rate', 10.0)
        self.declare_parameter('lost_timeout', 2.0)
        self.declare_parameter('deadzone', 0.06)

        self.pitch_search = self.get_parameter('pitch_search').value
        self.pitch_min    = self.get_parameter('pitch_min').value
        self.pitch_max    = self.get_parameter('pitch_max').value
        self.vfov_deg     = self.get_parameter('vfov_deg').value
        self.damping      = self.get_parameter('damping').value
        self.img_h        = self.get_parameter('img_h').value
        self.control_rate = self.get_parameter('control_rate').value
        self.lost_timeout = self.get_parameter('lost_timeout').value
        self.deadzone     = self.get_parameter('deadzone').value

        self.tent_detected = False
        self.tent_cy = 0.0
        self.last_det_time = 0.0
        self.pitch_deg = self.pitch_search
        self._new_det = False   # korekta tylko raz na SWIEZA detekcje

        self.create_subscription(TentDetection, '/tent_detections', self._det_cb, 10)
        self._timer = self.create_timer(1.0 / self.control_rate, self._loop)
        self.get_logger().info(
            f"SuasGimbalController (MAVLink mount) gotowy | vfov={self.vfov_deg} "
            f"damping={self.damping} | pitch {self.pitch_min}..{self.pitch_max} "
            f"search={self.pitch_search}")

    def _det_cb(self, msg: TentDetection):
        if msg.detected:
            bb = msg.bounding_box  # [x, y, w, h]
            self.tent_cy = bb[1] + bb[3] / 2.0
            self.tent_detected = True
            self.last_det_time = time.time()
            self._new_det = True
        else:
            self.tent_detected = False

    def _set_pitch(self, deg):
        self.pitch_deg = clamp(deg, self.pitch_min, self.pitch_max)
        self.set_gimbal_pitch(self.pitch_deg)

    def _loop(self):
        now = time.time()
        dt_lost = now - self.last_det_time if self.last_det_time > 0 else 999.0

        # Do SEARCH dopiero po dluzszym braku detekcji (nie przy 1 pustej klatce).
        if dt_lost > self.lost_timeout:
            self._set_pitch(self.pitch_search)
            self.get_logger().info("[SZUKAM] namiot zgubiony -> SEARCH",
                                   throttle_duration_sec=2.0)
            return

        # Detekcja jest duzo wolniejsza niz ta petla (np. 1.4 FPS vs 10 Hz).
        # Korekte aplikujemy TYLKO raz na nowa klatke detekcji — inaczej ta sama
        # (stara) pozycja namiotu bylaby liczona kilkukrotnie i gimbal oscyluje.
        if not self._new_det:
            return
        self._new_det = False

        half_h = self.img_h / 2.0
        ey = (self.tent_cy - half_h) / half_h

        # Martwa strefa: blisko srodka nie ruszamy gimbala (koniec drgan).
        if abs(ey) < self.deadzone:
            self.get_logger().info(
                f"[CENTR] ey={ey:+.2f} pitch={self.pitch_deg:+.1f}deg",
                throttle_duration_sec=1.0)
            return

        # Geometria: namiot jest ey*(vfov/2) stopni od osi kamery -> pochyl o tyle
        # (z tlumieniem). ey>0 (ponizej srodka) -> pitch maleje (bardziej w dol).
        correction = ey * (self.vfov_deg / 2.0) * self.damping
        self._set_pitch(self.pitch_deg - correction)
        self.get_logger().info(
            f"[SLEDZE] cy={self.tent_cy:.0f} ey={ey:+.2f} pitch={self.pitch_deg:+.1f}deg")


def main(args=None):
    rclpy.init(args=args)
    node = SuasGimbalController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
