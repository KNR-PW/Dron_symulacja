"""
tent_tracker — Maszyna stanów: SEARCH → APPROACH → HOVER.

Steruje gimbalem kamery (pitch) przez gz topic i dronem (vx, vy, vz)
przez velocity control, aby podlecieć nad wykryty namiot i nad nim zawisać.

API drona (FRD body frame):
  send_vectors(vx, vy, vz, yaw)
    vx > 0 = do przodu
    vy > 0 = w prawo
    vz > 0 = w dół
    yaw > 0 = obrót w prawo (clockwise)
"""

import csv
import math
import os
import subprocess
import threading
import time
from enum import Enum, auto

import rclpy

from drone_autonomy.drone_comunication.drone_controller import DroneController
from drone_interfaces.msg import Telemetry, TentDetection
from std_msgs.msg import Float32


# ────────────────────── Helpers ──────────────────────

def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class State(Enum):
    SEARCH = auto()
    APPROACH = auto()
    HOVER = auto()


# ────────────────────── Node ──────────────────────

class TentTracker(DroneController):
    """Podlot nad namiot z użyciem gimbala i regulatorów P."""

    # Zakres gimbala w Gazebo (rad): 0.0 = prosto, 1.05 = max w dół (~60°)
    GIMBAL_MIN = 0.0
    GIMBAL_MAX = 1.05  # ~60 deg
    GIMBAL_SEARCH = 0.30  # ~17 deg — kamera patrzy bardziej do przodu
    GIMBAL_DOWN = 1.0   # prawie pionowo w dół

    def __init__(self):
        super().__init__('tent_tracker')

        # ─── Parametry ROS ────────────────────────────────────
        self.declare_parameter('target_alt', 55.0)
        self.declare_parameter('kp_gimbal', 1.5)
        self.declare_parameter('kp_vx', 2.0)
        self.declare_parameter('kp_vy', 3.0)
        self.declare_parameter('kp_alt', 0.5)
        self.declare_parameter('kp_yaw', 0.3)
        self.declare_parameter('max_vel', 3.0)
        self.declare_parameter('max_vz', 1.5)
        self.declare_parameter('max_yaw_rate', 0.3)
        self.declare_parameter('ema_alpha', 0.4)
        self.declare_parameter('lost_timeout', 3.0)
        self.declare_parameter('img_w', 1920)
        self.declare_parameter('img_h', 1080)
        self.declare_parameter('control_rate', 10.0)
        self.declare_parameter('hover_deadzone', 0.08)

        self.target_alt    = self.get_parameter('target_alt').value
        self.kp_gimbal     = self.get_parameter('kp_gimbal').value
        self.kp_vx         = self.get_parameter('kp_vx').value
        self.kp_vy         = self.get_parameter('kp_vy').value
        self.kp_alt        = self.get_parameter('kp_alt').value
        self.kp_yaw        = self.get_parameter('kp_yaw').value
        self.max_vel       = self.get_parameter('max_vel').value
        self.max_vz        = self.get_parameter('max_vz').value
        self.max_yaw_rate  = self.get_parameter('max_yaw_rate').value
        self.ema_alpha     = self.get_parameter('ema_alpha').value
        self.lost_timeout  = self.get_parameter('lost_timeout').value
        self.img_w         = self.get_parameter('img_w').value
        self.img_h         = self.get_parameter('img_h').value
        self.control_rate  = self.get_parameter('control_rate').value
        self.hover_deadzone = self.get_parameter('hover_deadzone').value

        # ─── Stan ─────────────────────────────────────────────
        self.state = State.SEARCH
        self.tent_detected = False
        self.tent_cx = 0.0
        self.tent_cy = 0.0
        self.last_det_time = 0.0

        self.altitude = 0.0
        self.drone_yaw = 0.0

        # Gimbal
        self.gimbal_rad = self.GIMBAL_SEARCH
        self._gimbal_target = self.GIMBAL_SEARCH
        self._gimbal_lock = threading.Lock()

        # EMA
        self.sm_vx = 0.0
        self.sm_vy = 0.0
        self.sm_yaw = 0.0

        # Velocity control
        self.velocity_mode_active = False

        # ─── Subskrypcje ──────────────────────────────────────
        self.create_subscription(TentDetection, '/tent_detections',
                                 self._det_cb, 10)
        self.create_subscription(Telemetry, 'knr_hardware/telemetry',
                                 self._tel_cb, 10)

        # ─── Timer sterowania ─────────────────────────────────
        period = 1.0 / self.control_rate
        self._timer = self.create_timer(period, self._control_loop)
        self._timer.cancel()

        # ─── Wątek gimbala ────────────────────────────────────
        self._gimbal_thread = threading.Thread(
            target=self._gimbal_worker, daemon=True)
        self._gimbal_thread.start()

        # ─── Debug publishers (rqt_plot) ──────────────────────
        self.pub_vx     = self.create_publisher(Float32, '~/debug/vx', 10)
        self.pub_vy     = self.create_publisher(Float32, '~/debug/vy', 10)
        self.pub_yaw    = self.create_publisher(Float32, '~/debug/yaw_rate', 10)
        self.pub_vz     = self.create_publisher(Float32, '~/debug/vz', 10)
        self.pub_gimbal = self.create_publisher(Float32, '~/debug/gimbal_rad', 10)
        self.pub_ex     = self.create_publisher(Float32, '~/debug/error_x', 10)
        self.pub_ey     = self.create_publisher(Float32, '~/debug/error_y', 10)

        # ─── Logowanie CSV (Black Box) ────────────────────────
        self.csv_path = os.path.expanduser('~/Dron_symulacja/tent_tracker_log.csv')
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'time_rel', 'state', 'ex', 'ey', 'gimbal_rad', 'vx_cmd', 'vy_cmd', 'yaw_cmd', 'vz_cmd', 'alt'
        ])
        self.node_start_time = time.time()

        self.get_logger().info(f"TentTracker init: alt={self.target_alt}m  kp_gimbal={self.kp_gimbal}  kp_vx={self.kp_vx}  kp_vy={self.kp_vy}  img={self.img_w}x{self.img_h}")
        self.get_logger().info(f"Logi CSV zapisywane do: {self.csv_path}")

    # ═══════════════════════════════════════════════════════════
    #  Gimbal (osobny wątek)
    # ═══════════════════════════════════════════════════════════

    def _set_gimbal(self, rad):
        """Ustaw docelowy kąt gimbala (thread-safe)."""
        rad = clamp(rad, self.GIMBAL_MIN, self.GIMBAL_MAX)
        with self._gimbal_lock:
            self._gimbal_target = rad

    def _gimbal_worker(self):
        """Wątek wysyłający komendy gz topic co ~100ms."""
        last_sent = -1.0
        while True:
            with self._gimbal_lock:
                target = self._gimbal_target

            if abs(target - last_sent) > 0.01:
                cmd = [
                    "gz", "topic",
                    "-t", "/model/knr_tiltrotor_0/servo_7",
                    "-m", "gz.msgs.Double",
                    "-p", f"data: {target:.3f}"
                ]
                try:
                    subprocess.run(cmd, capture_output=True, timeout=2.0)
                except Exception:
                    pass
                last_sent = target
                self.gimbal_rad = target

            time.sleep(0.1)

    # ═══════════════════════════════════════════════════════════
    #  Callbacki
    # ═══════════════════════════════════════════════════════════

    def _det_cb(self, msg: TentDetection):
        if msg.detected:
            bb = msg.bounding_box  # [x, y, w, h]
            self.tent_cx = bb[0] + bb[2] / 2.0
            self.tent_cy = bb[1] + bb[3] / 2.0
            self.tent_detected = True
            self.last_det_time = time.time()
        else:
            self.tent_detected = False

    def _tel_cb(self, msg: Telemetry):
        self.altitude = msg.alt
        self.drone_yaw = msg.yaw

    # ═══════════════════════════════════════════════════════════
    #  Pętla sterowania
    # ═══════════════════════════════════════════════════════════

    def _control_loop(self):
        now = time.time()
        rel_time = now - self.node_start_time
        dt_lost = now - self.last_det_time if self.last_det_time > 0 else 999.0

        # Altitude hold (zawsze aktywny)
        alt_err = self.target_alt - self.altitude
        vz = clamp(-self.kp_alt * alt_err, -self.max_vz, self.max_vz)

        # Znormalizowane uchyby obrazu (-1..+1)
        half_w = self.img_w / 2.0
        half_h = self.img_h / 2.0
        ex = (self.tent_cx - half_w) / half_w if self.tent_detected else 0.0
        ey = (self.tent_cy - half_h) / half_h if self.tent_detected else 0.0

        # ─── Przejścia stanów ─────────────────────────────────
        if dt_lost > self.lost_timeout:
            if self.state != State.SEARCH:
                self.get_logger().info("Namiot ZGUBIONY → SEARCH")
                self.state = State.SEARCH
                self._set_gimbal(self.GIMBAL_SEARCH)

        elif self.tent_detected:
            if self.state == State.SEARCH:
                self.get_logger().info("Namiot WIDZIANY → APPROACH")
                self.state = State.APPROACH

            # APPROACH → HOVER: gimbal prawie pionowo
            if self.state == State.APPROACH:
                if self.gimbal_rad >= self.GIMBAL_DOWN:
                    self.get_logger().info(
                        "NAD NAMIOTEM! → HOVER (gimbal locked)")
                    self.state = State.HOVER

        # ─── Logika stanów ────────────────────────────────────
        vx_target = 0.0
        vy_target = 0.0
        yaw_target = 0.0

        if self.state == State.SEARCH:
            # Nie wysyłamy zer — użytkownik steruje ręcznie
            # Publikujemy tylko debug/csv i wychodzimy
            self.pub_vx.publish(Float32(data=0.0))
            self.pub_vy.publish(Float32(data=0.0))
            self.pub_yaw.publish(Float32(data=0.0))
            self.pub_vz.publish(Float32(data=0.0))
            self.pub_gimbal.publish(Float32(data=float(self.gimbal_rad)))
            self.pub_ex.publish(Float32(data=float(ex)))
            self.pub_ey.publish(Float32(data=float(ey)))
            self.csv_writer.writerow([
                f"{rel_time:.3f}", self.state.name, f"{ex:.3f}", f"{ey:.3f}", f"{self.gimbal_rad:.3f}",
                "0.000", "0.000", "0.000", "0.000", f"{self.altitude:.2f}"
            ])
            self.csv_file.flush()
            self.get_logger().info(
                f"[SEARCH] czekam na namiot... gimbal={self.gimbal_rad:.2f} alt={self.altitude:.1f}m",
                throttle_duration_sec=2.0
            )
            return  # NIE wysyłamy send_vectors — ręczne sterowanie

        elif self.state == State.APPROACH:
            # 1. Gimbal pitch — śledź namiot w osi Y obrazu
            gimbal_delta = self.kp_gimbal * ey * (1.0 / self.control_rate)
            new_gimbal = self.gimbal_rad + gimbal_delta
            self._set_gimbal(new_gimbal)

            # 2. vx — leć do przodu proporcjonalnie do kąta gimbala
            #    gimbal=0.0 → namiot daleko → leć szybko (forward_ratio=1.0)
            #    gimbal=1.05 → namiot pod spodem → hamuj (forward_ratio=0.0)
            forward_ratio = 1.0 - (self.gimbal_rad / self.GIMBAL_MAX)
            vx_target = self.kp_vx * forward_ratio

            # 3. vy — WYŁĄCZONE w APPROACH (używamy tylko yaw do celowania)
            vy_target = 0.0

            # 4. yaw — obracaj nos drona w kierunku namiotu (uchyb X)
            yaw_target = self.kp_yaw * ex

        elif self.state == State.HOVER:
            # Gimbal zablokowany pionowo
            self._set_gimbal(self.GIMBAL_MAX)

            # Korekcja mikroruchów z uchybów obrazu (tylko translacja, bez obrotu)
            # Gdy kamera patrzy w dół: ex→vy, ey→-vx (obraz jest "odwrócony" vs body)
            vx_target = -self.kp_vy * ey
            vy_target =  self.kp_vy * ex
            yaw_target = 0.0

        # ─── Clamp + EMA ─────────────────────────────────────
        vx_target = clamp(vx_target, -self.max_vel, self.max_vel)
        vy_target = clamp(vy_target, -self.max_vel, self.max_vel)
        yaw_target = clamp(yaw_target, -self.max_yaw_rate, self.max_yaw_rate)

        a = self.ema_alpha
        self.sm_vx  = a * vx_target  + (1 - a) * self.sm_vx
        self.sm_vy  = a * vy_target  + (1 - a) * self.sm_vy
        self.sm_yaw = a * yaw_target + (1 - a) * self.sm_yaw

        self.send_vectors(self.sm_vx, self.sm_vy, vz, self.sm_yaw)

        # ─── Debug publish ────────────────────────────────────
        self.pub_vx.publish(Float32(data=float(self.sm_vx)))
        self.pub_vy.publish(Float32(data=float(self.sm_vy)))
        self.pub_yaw.publish(Float32(data=float(self.sm_yaw)))
        self.pub_vz.publish(Float32(data=float(vz)))
        self.pub_gimbal.publish(Float32(data=float(self.gimbal_rad)))
        self.pub_ex.publish(Float32(data=float(ex)))
        self.pub_ey.publish(Float32(data=float(ey)))

        # ─── Zapis do CSV ─────────────────────────────────────
        self.csv_writer.writerow([
            f"{rel_time:.3f}", self.state.name, f"{ex:.3f}", f"{ey:.3f}", f"{self.gimbal_rad:.3f}",
            f"{self.sm_vx:.3f}", f"{self.sm_vy:.3f}", f"{self.sm_yaw:.3f}", f"{vz:.3f}", f"{self.altitude:.2f}"
        ])
        self.csv_file.flush() # upewnij się, że dane są na dysku

        # ─── Logi ─────────────────────────────────────────────
        self.get_logger().info(
            f"[{self.state.name}] ex={ex:+.2f} ey={ey:+.2f} "
            f"gimbal={self.gimbal_rad:.2f} | "
            f"vx={self.sm_vx:+.2f} vy={self.sm_vy:+.2f} "
            f"yr={self.sm_yaw:+.2f} alt={self.altitude:.1f}m",
            throttle_duration_sec=1.0
        )

    # ═══════════════════════════════════════════════════════════
    #  Misja
    # ═══════════════════════════════════════════════════════════

    def run_mission(self):
        self.get_logger().info("=== START MISJI: tent_tracker ===")
        self.get_logger().info(
            f"Target alt: {self.target_alt}m | "
            f"kp_vx={self.kp_vx} kp_vy={self.kp_vy} max_vel={self.max_vel}")

        # Włącz velocity control
        result = self.toggle_control()
        if not result.result:
            self.get_logger().warn("toggle_control OFF→ON retry")
            self.toggle_control()
        self.velocity_mode_active = True
        self.get_logger().info("Velocity control AKTYWNY")

        # Gimbal do pozycji startowej (szukanie)
        self._set_gimbal(self.GIMBAL_SEARCH)

        # Start pętli
        self.state = State.SEARCH
        self.last_det_time = 0.0
        self._timer.reset()
        self.get_logger().info("Szukam namiotu...")

    def stop_mission(self):
        self.get_logger().info("=== STOP MISJI ===")
        self._timer.cancel()
        try:
            self.send_vectors(0.0, 0.0, 0.0, 0.0)
            if self.velocity_mode_active:
                result = self.toggle_control()
                if result.result:
                    self.toggle_control()
                self.velocity_mode_active = False
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = TentTracker()

    try:
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
