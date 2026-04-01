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
        self.declare_parameter('kp_forward', 0.15)
        self.declare_parameter('kp_hover', 5.0)
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
        self.kp_forward = self.get_parameter('kp_forward').value
        self.kp_hover = self.get_parameter('kp_hover').value
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

        # --- Zmienne Gimbala ---
        self.gimbal_pwm = 500.0  # domyślnie 30 stopni do przodu-do dołu
        self.kp_gimbal = 100.0   # Przywrócone 100 wg kroku 672, gdzie "działało prawie dobrze"
        
        # --- Wygładzanie prędkości (EMA) ---
        self.smooth_vx = 0.0
        self.smooth_vy = 0.0
        self.smooth_yaw = 0.0
        self.ema_alpha = 0.3  # 0.0=bardzo gładko, 1.0=brak filtrowania
        
        from drone_interfaces.srv import SetServo
        self.servo_client = self.create_client(SetServo, '/knr_hardware/set_servo')

        self.get_logger().info(
            f"TentFollower init: alt={self.target_alt}m, kp_xy={self.kp_xy}, "
            f"kp_yaw={self.kp_yaw}, max_vel={self.max_vel}m/s, "
            f"img={self.img_w}x{self.img_h}"
        )

    # ═══════════════════════════════════════════════════════════
    #  Callbacki
    # ═══════════════════════════════════════════════════════════

    def _detection_cb(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9
        self.last_detection_time = now

        if msg.detected:
            self.tent_detected = True
            bb = msg.bounding_box  # [x, y, w, h]
            self.tent_cx = bb[0] + bb[2] / 2.0
            self.tent_cy = bb[1] + bb[3] / 2.0
            
            # --- KONTROLA GIMBALA (Tylko na nowej klatce!) ---
            ey_px = self.tent_cy - (self.img_h / 2.0)
            ey_norm = ey_px / (self.img_h / 2.0)
            
            # Tryb zatrzasku poosiągnięciu maxa (żeby nie uciekał z powrotem do trybu Yaw):
            if self.gimbal_pwm >= 1020.0:
                # Jeśli dron dojechał już kamerą pod siebie, zablokuj ją dołem.
                self.gimbal_pwm = 1023.0
            else:
                # W przeciwnym razie pracujemy normalnie (skrypt z kroku 672, który działał dobrze)
                self.gimbal_pwm += self.kp_gimbal * ey_norm
                self.gimbal_pwm = max(500.0, min(1023.0, self.gimbal_pwm))
                
            self.set_camera_pitch(self.gimbal_pwm)
            
            # Wznawiamy timer
            if self.control_timer.is_canceled():
                self.control_timer.reset()
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

        # 1. Obliczenie uchybu na obrazie (z ostatniej klatki)
        half_w = self.img_w / 2.0
        half_h = self.img_h / 2.0

        ex_px = self.tent_cx - half_w
        ey_px = self.tent_cy - half_h

        ex_norm = ex_px / half_w
        ey_norm = ey_px / half_h

        # Gimbal jest teraz aktualizowany WŁĄCZNIE w _detection_cb, by uniknąć over-shootów!
        # Mamy tylko dostęp do jego aktualnego kąta.em (Forward) 
        gimbal_angle_deg = 30.0 + ((self.gimbal_pwm - 500.0) / 523.0) * 60.0 # od 30 do 90 stopni dół
        theta_v = 90.0 - gimbal_angle_deg # kąt do promienia od pionu (straight down = 0)
        
        # dist_x > 0 oznacza, ze cel leży z przodu
        dist_x = self.altitude * math.tan(math.radians(theta_v))

        # 3. Lot drona (Pościg vs Precyzyjny hover)
        kp_forward_val = self.kp_forward  # Parametryzowane
        kp_hover_val = self.kp_hover      # Parametryzowane
        
        # Kiedy gimbal schodzi pionowo w dół (> 800 PWM),
        # płynnie przestajemy obracać drona (yaw) i zaczynamy używać sterowania bocznego (vy)
        hover_blend = clamp((self.gimbal_pwm - 800.0) / 200.0, 0.0, 1.0)
        
        # Pościg z "marchewki" gimbala
        # Dodajemy dynamiczny mnożnik (boost): im bardziej patrzy w przód (blend zbliża się do 0),
        # tym mocniej szarżujemy (np. x3). Jak patrzy prosto w dół (blend=1), boost znika.
        approach_boost = 1.0 + (1.0 - hover_blend) * 2.0 
        vx_target = kp_forward_val * dist_x * approach_boost
        
        # Gdy kamera prosto w dół (blend=1), obraz jest odwrócony vs body:
        #   ey > 0 (dół obrazu) = namiot ZA dronem → vx < 0
        vx_target += hover_blend * (-kp_hover_val * ey_norm)
        
        # Sterowanie boczne (tylko hover)
        vy_target = hover_blend * (kp_hover_val * ex_norm)
        
        # Yaw tylko w fazie pościgu (zgodnie z życzeniem wyłączane, gdy kamera patrzy pionowo w dół, blend=1)
        # Przywrócony ORYGINALNY znak (z plusem)! W PX4 oś Z idzie w dół (FRD),
        # więc dodatnie Yaw oznacza obrót w PRAWO. Wcześniejszy minus zepsuł pościg.
        yaw_rate_target = (1.0 - hover_blend) * self.kp_yaw * ex_norm

        vx = clamp(vx_target, -self.max_vel, self.max_vel)
        vy = clamp(vy_target, -self.max_vel, self.max_vel)
        yaw_rate = clamp(yaw_rate_target, -self.max_yaw_rate, self.max_yaw_rate)

        # Osiągnięcie celu
        if abs(ex_norm) < self.deadzone and abs(ey_norm) < self.deadzone and self.gimbal_pwm > 1000.0:
            if not self.hovering:
                self.get_logger().info("Namiot wycentrowany — stabilny hover!")
                self.hovering = True
            vx = 0.0
            vy = 0.0
            yaw_rate = 0.0
        else:
            self.hovering = False

        # --- EMA wygładzanie (eliminuje szarpanie przy wolnym YOLO) ---
        a = self.ema_alpha
        self.smooth_vx = a * vx + (1.0 - a) * self.smooth_vx
        self.smooth_vy = a * vy + (1.0 - a) * self.smooth_vy
        self.smooth_yaw = a * yaw_rate + (1.0 - a) * self.smooth_yaw

        self.send_vectors(self.smooth_vx, self.smooth_vy, vz, self.smooth_yaw)
        self.get_logger().info(
            f"CTRL V1: ex={ex_norm:+.2f} ey={ey_norm:+.2f} -> "
            f"pwm={self.gimbal_pwm:.0f} dx={dist_x:.1f} bl={hover_blend:.1f} | "
            f"OUT: vx={self.smooth_vx:+.2f} vy={self.smooth_vy:+.2f} yr={self.smooth_yaw:+.2f} alt={self.altitude:.1f}m",
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

    def _gimbal_worker(self):
        """Pojedynczy wątek w tle wykonujący komendy gz topic, zabezpieczający przed zalaniem OS."""
        import subprocess
        import time
        while True:
            if abs(self.target_gimbal_pwm - self.last_sent_gimbal_pwm) > 1.0:
                pwm = self.target_gimbal_pwm
                self.last_sent_gimbal_pwm = pwm
                rad = max(0.0, min(1.05, (pwm - 500.0) / 523.0 * 1.05))
                cmd = [
                    "gz", "topic", 
                    "-t", "/model/knr_tiltrotor_0/servo_7", 
                    "-m", "gz.msgs.Double", 
                    "-p", f"data: {rad}"
                ]
                try:
                    res = subprocess.run(cmd, capture_output=True, text=True)
                    if res.returncode != 0:
                         self.get_logger().error(f"[Gimbal Worker] Błąd GZ: {res.stderr.strip()}")
                    else:
                         self.get_logger().info(f"[Gimbal Worker] Wysłano GZ Topic rad={rad:.2f} (pwm={pwm:.0f})")
                except Exception as e:
                     self.get_logger().error(f"[Gimbal Worker] Wyjątek BASH: {e}")
            time.sleep(0.1)  # Sztywne 10Hz na sprzętówkę!

    def set_camera_pitch(self, pwm):
        # Aktualizujemy cel, a worker przechwyci to w swoim tempie (chroni system)
        self.target_gimbal_pwm = pwm

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
