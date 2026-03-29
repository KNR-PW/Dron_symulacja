"""
tent_follower — node sterowania dronem nad namiot z użyciem YOLO detekcji.

Kamera patrzy prosto w dół. Node subskrybuje /tent_detections (TentDetection)
i telemetrię, oblicza błąd pozycji namiotu w kadrze i steruje dronem
za pomocą wektorów prędkości (velocity control) aby wycentrować namiot
w polu widzenia i zawisać nad nim.

Sterowanie yaw: dron obraca się w kierunku namiotu — przygotowane pod
przyszłą integrację kamery z pochyleniem.
"""

import math
import time

import rclpy
from rclpy.node import Node

from drone_autonomy.drone_comunication.drone_controller import DroneController
from drone_interfaces.msg import Telemetry, TentDetection


def clamp(value, min_val, max_val):
    return max(min_val, min(max_val, value))


class TentFollower(DroneController):
    """Steruje dronem aby podlecial nad namiot i zawisl nad nim."""

    def __init__(self):
        super().__init__('tent_follower')

        # ─── Parametry ───────────────────────────────────────
        self.declare_parameter('target_alt', 55.0)
        self.declare_parameter('kp_xy', 1.0)
        self.declare_parameter('kp_yaw', 0.5)
        self.declare_parameter('kp_alt', 0.5)
        self.declare_parameter('max_vel', 1.5)
        self.declare_parameter('max_yaw_rate', 0.5)
        self.declare_parameter('max_vz', 1.5)
        self.declare_parameter('deadzone', 0.05)
        self.declare_parameter('lost_timeout', 5.0)
        self.declare_parameter('img_w', 1920)
        self.declare_parameter('img_h', 1080)
        self.declare_parameter('control_rate', 10.0)

        self.target_alt = self.get_parameter('target_alt').value
        self.kp_xy = self.get_parameter('kp_xy').value
        self.kp_yaw = self.get_parameter('kp_yaw').value
        self.kp_alt = self.get_parameter('kp_alt').value
        self.max_vel = self.get_parameter('max_vel').value
        self.max_yaw_rate = self.get_parameter('max_yaw_rate').value
        self.max_vz = self.get_parameter('max_vz').value
        self.deadzone = self.get_parameter('deadzone').value
        self.lost_timeout = self.get_parameter('lost_timeout').value
        self.img_w = self.get_parameter('img_w').value
        self.img_h = self.get_parameter('img_h').value
        self.control_rate = self.get_parameter('control_rate').value

        # ─── Stan wewnętrzny ─────────────────────────────────
        self.last_detection_time = 0.0
        self.tent_detected = False
        self.tent_cx = 0.0  # centroid x w pikselach
        self.tent_cy = 0.0  # centroid y w pikselach

        # Telemetria
        self.altitude = 0.0
        self.drone_yaw = 0.0
        self.drone_roll = 0.0
        self.drone_pitch = 0.0

        # ─── Subskrypcje ─────────────────────────────────────
        self.create_subscription(
            TentDetection,
            '/tent_detections',
            self._detection_cb,
            10
        )
        self.create_subscription(
            Telemetry,
            'knr_hardware/telemetry',
            self._telemetry_ext_cb,
            10
        )

        # ─── Timer sterowania ────────────────────────────────
        period = 1.0 / self.control_rate
        self.control_timer = self.create_timer(period, self._control_loop)
        self.control_timer.cancel()  # nie startujemy petli od razu

        self.hovering = False
        self.velocity_mode_active = False

        self.get_logger().info(
            f"TentFollower init: alt={self.target_alt}m, kp_xy={self.kp_xy}, "
            f"kp_yaw={self.kp_yaw}, max_vel={self.max_vel}m/s, "
            f"img={self.img_w}x{self.img_h}"
        )

    # ═══════════════════════════════════════════════════════════
    #  Callbacki
    # ═══════════════════════════════════════════════════════════

    def _detection_cb(self, msg: TentDetection):
        """Callback detekcji namiotu z yolo_detector."""
        if msg.detected:
            bb = msg.bounding_box  # [x, y, w, h]
            self.tent_cx = bb[0] + bb[2] / 2.0
            self.tent_cy = bb[1] + bb[3] / 2.0
            self.tent_detected = True
            self.last_detection_time = time.time()
        else:
            self.tent_detected = False

    def _telemetry_ext_cb(self, msg: Telemetry):
        """Rozszerzony callback telemetrii (roll/pitch/yaw)."""
        self.altitude = msg.alt
        self.drone_yaw = msg.yaw
        self.drone_roll = msg.roll
        self.drone_pitch = msg.pitch

    # ═══════════════════════════════════════════════════════════
    #  Pętla sterowania
    # ═══════════════════════════════════════════════════════════

    def _compute_vz(self):
        """Regulator P wysokości — utrzymuje target_alt."""
        alt_error = self.target_alt - self.altitude  # >0 = za nisko
        vz = clamp(-self.kp_alt * alt_error, -self.max_vz, self.max_vz)
        # vz w body frame: + = w dół, - = w górę
        # alt_error > 0 (za nisko) → vz < 0 (leć w górę)
        return vz

    def _control_loop(self):
        """Główna pętla sterowania ~10 Hz."""
        now = time.time()
        dt_since_last = now - self.last_detection_time

        # Zawsze utrzymuj wysokość
        vz = self._compute_vz()

        # --- Brak detekcji → hover z altitude hold ---
        if not self.tent_detected or dt_since_last > self.lost_timeout:
            if not self.hovering:
                self.get_logger().info(
                    f"Namiot utracony — hover (alt={self.altitude:.1f}m, "
                    f"target={self.target_alt:.1f}m)")
                self.hovering = True
            self.send_vectors(0.0, 0.0, vz, 0.0)
            return

        self.hovering = False

        # --- Obliczenie błędu w pikselach ---
        half_w = self.img_w / 2.0
        half_h = self.img_h / 2.0

        # Błąd: dodatni ex = namiot po prawej, dodatni ey = namiot poniżej centrum
        ex_px = self.tent_cx - half_w
        ey_px = self.tent_cy - half_h

        # Normalizacja do [-1, 1]
        ex_norm = ex_px / half_w
        ey_norm = ey_px / half_h

        # --- Deadzone → hover z altitude hold ---
        if abs(ex_norm) < self.deadzone and abs(ey_norm) < self.deadzone:
            self.send_vectors(0.0, 0.0, vz, 0.0)
            if not self.hovering:
                self.get_logger().info("Namiot wycentrowany — hover nad celem")
                self.hovering = True
            return

        # --- Regulator P: wektory prędkości w body frame ---
        # ŻELAZNA LOGIKA Z LOGÓW:
        # Rozwiązaliśmy skrzyżowanie osi (X lotu to teraz faktyczne Y obrazu, tak jak powinno być).
        # Skoro teraz dron idealnie "ucieka" zamiast "gonić" to zwroty fizyczne PX4 / kamery są zanegowane. 
        # Odwracamy oba wektory o 180 stopni (zmiana znaków na minus i plus):
        vx_target = -self.kp_xy * ey_norm
        vy_target =  self.kp_xy * ex_norm

        vx = clamp(vx_target, -self.max_vel, self.max_vel)
        vy = clamp(vy_target, -self.max_vel, self.max_vel)

        # --- Sterowanie yaw ---
        yaw_rate = clamp(
            self.kp_yaw * ex_norm,
            -self.max_yaw_rate,
            self.max_yaw_rate
        )

        self.send_vectors(vx, vy, vz, yaw_rate)
        self.get_logger().info(
            f"CTRL: ex={ex_norm:+.2f} ey={ey_norm:+.2f} → "
            f"vx={vx:+.2f} vy={vy:+.2f} vz={vz:+.2f} yaw_r={yaw_rate:+.2f} "
            f"alt={self.altitude:.1f}m",
            throttle_duration_sec=1.0
        )

    # ═══════════════════════════════════════════════════════════
    #  Sekwencja misji
    # ═══════════════════════════════════════════════════════════

    def run_mission(self):
        """Velocity control → pętla sterowania.

        Zakłada, że dron jest już uzbrojony i w powietrzu
        (np. przez start_px4_sim.sh + test_mission / QGC).
        Altitude hold w pętli sterowania sam wzniesie/zniży do target_alt.
        """
        self.get_logger().info("=== START MISJI: tent_follower ===")
        self.get_logger().info(
            f"Target alt: {self.target_alt}m | "
            f"kp_xy={self.kp_xy} kp_alt={self.kp_alt} max_vel={self.max_vel}")

        # Upewnij się, że velocity control jest WŁĄCZONY
        self.get_logger().info("Wlaczanie velocity control...")
        result = self.toggle_control()
        if not result.result:
            # Toggle wyłączył zamiast włączyć — togglujemy ponownie
            self.get_logger().warn(
                "Velocity control byl juz ON — toggle go wylaczyl — wlaczam ponownie")
            self.toggle_control()
        self.velocity_mode_active = True
        self.get_logger().info("Velocity control AKTYWNY")

        # Start petli sterowania
        self.get_logger().info("Start petli sterowania — szukam namiotu...")
        self.last_detection_time = 0.0
        self.control_timer.reset()  # uruchom timer

    def stop_mission(self):
        """Zatrzymuje misję i przywraca position control."""
        self.get_logger().info("=== STOP MISJI ===")
        self.control_timer.cancel()

        try:
            self.send_vectors(0.0, 0.0, 0.0, 0.0)

            if self.velocity_mode_active:
                self.get_logger().info("Wylaczam velocity control...")
                result = self.toggle_control()
                if result.result:
                    # Nadal ON — toggle jeszcze raz
                    self.toggle_control()
                self.velocity_mode_active = False
        except Exception:
            pass  # kontekst ROS już zamknięty (Ctrl+C)


def main(args=None):
    rclpy.init(args=args)
    node = TentFollower()

    try:
        # Uruchom misję z opóźnieniem dając czas na połączenie serwisów
        node.get_logger().info("Czekam 2s na inicjalizacje serwisow...")
        time.sleep(2.0)
        node.run_mission()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_mission()
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
