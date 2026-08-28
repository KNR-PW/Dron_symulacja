"""
suas_flight_controller — Maszyna stanów: SEARCH → APPROACH → HOVER.

Steruje gimbalem kamery (pitch) przez MAVLink mount (set_gimbal_pitch,
ta sama sciezka co na realnym dronie) i dronem (vx, vy, vz) przez velocity
control, aby podlecieć nad wykryty namiot i nad nim zawisać.

Gimbal liczony geometrycznie (dokladnie jak w suas_gimbal_controller):
  korekta = ey * (vfov/2) * damping, aplikowana RAZ na swieza detekcje,
  z martwa strefa (gimbal_deadzone) zeby nie drgal przy namiocie w srodku.

Konwencja katow (real, MNT1_PITCH_MIN/MAX = -90/45 wg docs/gimbal setup.md):
      0 deg = poziomo (w przod)
    -45 deg = pod katem w dol/przod  (pozycja SEARCH)
    -90 deg = prosto w dol           (HOVER)
Namiot ponizej srodka (ey>0) => patrz bardziej w dol => pitch maleje.

UWAGA: domyslne parametry sa dla realu (OAK-D PRO W: img 1024x1024, vfov 64.4).
Gazebo ma inna kalibracje gimbala (-45 = prosto w dol, +45 = przod) i szerszy FOV
— do symulacji podaj parametry:
  pitch_min:=-45.0 pitch_max:=45.0 pitch_search:=30.0 pitch_hover_thr:=-38.0
  vfov_deg:=114.6
Rozmiar klatki jest teraz ten sam (1024x1024) w Gazebo i na realu.

Wejscie w APPROACH dopiero po potwierdzeniu detekcji: M trafien (det_confirm_frames)
z ostatnich N klatek (det_window_frames) i — o ile tracker podaje ID — nalezacych
do tej samej sciezki. Filtruje falszywki, a przy tym toleruje pojedyncze
przegapienia detektora. Prog pewnosci ustawia sam detektor (parametr 'conf').

API drona (FRD body frame):
  send_vectors(vx, vy, vz, yaw)
    vx > 0 = do przodu
    vy > 0 = w prawo
    vz > 0 = w dół
    yaw > 0 = obrót w prawo (clockwise)
"""

import csv
import os
import time
from collections import deque
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

class SuasFlightController(DroneController):
    """Podlot nad namiot z użyciem gimbala i regulatorów P."""

    def __init__(self):
        super().__init__('suas_flight_controller')

        # ─── Parametry ROS ────────────────────────────────────
        # UWAGA: ta sama wartosc domyslna co w launchu — nie rozjezdzac ich.
        self.declare_parameter('target_alt', 5.0)
        # Zakres pracy gimbala w STOPNIACH — parametry (jak w suas_gimbal_controller),
        # bo Gazebo i real maja rozne kalibracje (patrz docstring modulu).
        self.declare_parameter('pitch_search', -45.0)     # brak namiotu: pod katem w dol/przod
        self.declare_parameter('pitch_min', -90.0)        # najnizej: prosto w dol (HOVER)
        # Wyzej niz pitch_search (inaczej vx startuje od razu na 100% kp_vx, bo
        # forward_ratio liczy sie wzgledem tego zakresu) i zeby tracker mogl siegnac
        # namiotu widocznego POWYZEJ linii szukania. Limit sprzetowy to ok. -18 deg
        # (PWM 1900, patrz drone_handler.gimbal_pitch_callback).
        self.declare_parameter('pitch_max', -30.0)        # najwyzej: pod katem w dol/przod
        self.declare_parameter('pitch_hover_thr', -83.0)  # prog APPROACH->HOVER (7 deg od pionu)
        # Gimbal geometrycznie: kat wprost z bledu i pionowego FOV kamery + tlumienie
        self.declare_parameter('vfov_deg', 64.4)   # zmierzony pionowy FOV OAK-D PRO W
        self.declare_parameter('damping', 0.6)
        self.declare_parameter('gimbal_deadzone', 0.06)
        # Filtr falszywych detekcji: okno M z N klatek (odporne na pojedyncze
        # przegapienia detektora, dalej odrzuca pojedyncze falszywki).
        self.declare_parameter('det_confirm_frames', 6)   # M — ile trafien wymagamy
        self.declare_parameter('det_window_frames', 8)    # N — dlugosc okna
        self.declare_parameter('det_confirm_gap', 0.5)    # dluzsza przerwa = okno nieaktualne [s]
        # Progu pewnosci tu NIE ma — filtruje juz detektor (parametr 'conf' w
        # launchach detekcji). Jedno pokretlo, jedno miejsce.
        # Wymagaj tego samego ID sciezki z trackera w calym oknie
        self.declare_parameter('require_same_track', True)
        self.declare_parameter('kp_vx', 2.0)
        self.declare_parameter('kp_vy', 3.0)
        self.declare_parameter('kp_alt', 0.5)
        self.declare_parameter('kp_yaw', 0.3)
        self.declare_parameter('max_vel', 3.0)
        self.declare_parameter('max_vz', 1.5)
        self.declare_parameter('max_yaw_rate', 0.3)
        self.declare_parameter('ema_alpha', 0.4)
        self.declare_parameter('lost_timeout', 3.0)
        # Rozmiar oryginalnej klatki (w niej sa wspolrzedne bounding_box):
        # OAK-D PRO W, kwadratowe preview 1024x1024 (patrz config/oak_rgb.yaml).
        # Kwadrat, bo MODEL4.engine ma sztywne wejscie 1024x1024 - klatka 4:3
        # dokladalaby szare pasy i marnowala 25% tensora.
        self.declare_parameter('img_w', 1024)
        self.declare_parameter('img_h', 1024)
        self.declare_parameter('control_rate', 10.0)
        self.declare_parameter('hover_deadzone', 0.08)

        self.target_alt    = self.get_parameter('target_alt').value
        self.pitch_search  = self.get_parameter('pitch_search').value
        self.pitch_min     = self.get_parameter('pitch_min').value
        self.pitch_max     = self.get_parameter('pitch_max').value
        self.pitch_hover_thr = self.get_parameter('pitch_hover_thr').value
        self.vfov_deg      = self.get_parameter('vfov_deg').value
        self.damping       = self.get_parameter('damping').value
        self.gimbal_deadzone = self.get_parameter('gimbal_deadzone').value
        self.det_confirm_frames = self.get_parameter('det_confirm_frames').value
        self.det_window_frames  = self.get_parameter('det_window_frames').value
        self.det_confirm_gap    = self.get_parameter('det_confirm_gap').value
        self.require_same_track = self.get_parameter('require_same_track').value
        # M > N = warunek nie do spelnienia (dron nigdy nie wszedlby w APPROACH).
        if self.det_confirm_frames > self.det_window_frames:
            self.get_logger().warn(
                f"det_confirm_frames({self.det_confirm_frames}) > det_window_frames"
                f"({self.det_window_frames}) — przycinam do {self.det_window_frames}")
            self.det_confirm_frames = self.det_window_frames
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
        self.tent_cx = 0.0
        self.tent_cy = 0.0
        self.last_det_time = 0.0
        self.last_conf = 0.0

        self.altitude = 0.0
        self.drone_yaw = 0.0

        # Gimbal (kat pitch w stopniach; korekta raz na swieza detekcje)
        self.pitch_deg = self.pitch_search
        self._new_det = False

        # Potwierdzanie detekcji: przesuwne okno N ostatnich klatek (1 = trafienie)
        self._det_window = deque(maxlen=self.det_window_frames)
        self._prev_frame_time = 0.0
        self._cand_id = -1          # ID sledzonego kandydata (-1 = brak/tracker milczy)

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

        # ─── Debug publishers (rqt_plot) ──────────────────────
        self.pub_vx     = self.create_publisher(Float32, '~/debug/vx', 10)
        self.pub_vy     = self.create_publisher(Float32, '~/debug/vy', 10)
        self.pub_yaw    = self.create_publisher(Float32, '~/debug/yaw_rate', 10)
        self.pub_vz     = self.create_publisher(Float32, '~/debug/vz', 10)
        self.pub_gimbal = self.create_publisher(Float32, '~/debug/gimbal_deg', 10)
        self.pub_ex     = self.create_publisher(Float32, '~/debug/error_x', 10)
        self.pub_ey     = self.create_publisher(Float32, '~/debug/error_y', 10)

        # ─── Logowanie CSV (Black Box) ────────────────────────
        self.csv_path = os.path.expanduser('~/suas_flight_controller_log.csv')
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'time_rel', 'state', 'ex', 'ey', 'gimbal_deg', 'vx_cmd', 'vy_cmd', 'yaw_cmd', 'vz_cmd', 'alt',
            'conf', 'det_hits', 'det_window', 'track_id'
        ])
        self.node_start_time = time.time()

        self.get_logger().info(f"SuasFlightController init: alt={self.target_alt}m  vfov={self.vfov_deg} damping={self.damping}  kp_vx={self.kp_vx}  kp_vy={self.kp_vy}  img={self.img_w}x{self.img_h}")
        self.get_logger().info(
            f"Gimbal: pitch {self.pitch_min}..{self.pitch_max} search={self.pitch_search} "
            f"hover_thr={self.pitch_hover_thr} deadzone={self.gimbal_deadzone} | "
            f"potwierdzenie detekcji: {self.det_confirm_frames} z {self.det_window_frames} "
            f"klatek, max przerwa {self.det_confirm_gap}s, "
            f"same_track={self.require_same_track} (prog pewnosci: parametr 'conf' detektora)")
        self.get_logger().info(f"Logi CSV zapisywane do: {self.csv_path}")

    # ═══════════════════════════════════════════════════════════
    #  Gimbal
    # ═══════════════════════════════════════════════════════════

    def _set_gimbal(self, deg):
        """Ustaw kąt pitch gimbala [deg] i wyślij przez MAVLink mount
        (set_gimbal_pitch -> drone_handler -> serwo; ta sama sciezka co na realu)."""
        deg = clamp(deg, self.pitch_min, self.pitch_max)
        if abs(deg - self.pitch_deg) > 0.2:
            self.pitch_deg = deg
            self.set_gimbal_pitch(deg)

    # ═══════════════════════════════════════════════════════════
    #  Hamowanie
    # ═══════════════════════════════════════════════════════════

    def _brake(self):
        """Jawne wyzerowanie predkosci przy wyjsciu z APPROACH/HOVER.

        W SEARCH nie wysylamy setpointow, a ArduPilot w GUIDED trzyma OSTATNI
        zadany wektor jeszcze ~3 s — bez tego dron przy 3 m/s dryfowalby ~9 m
        po utracie celu. Zerujemy tez EMA, zeby po powrocie do APPROACH
        rozpedzac sie od zera, a nie od ostatniej predkosci sprzed utraty."""
        self.sm_vx = 0.0
        self.sm_vy = 0.0
        self.sm_yaw = 0.0
        try:
            self.send_vectors(0.0, 0.0, 0.0, 0.0)
        except Exception as e:
            self.get_logger().error(f"Nie udalo sie wyslac zerowego wektora: {e}")

    # ═══════════════════════════════════════════════════════════
    #  Callbacki
    # ═══════════════════════════════════════════════════════════

    @property
    def _det_hits(self):
        """Ile trafien w oknie N ostatnich klatek."""
        return sum(self._det_window)

    def _reset_det_window(self):
        self._det_window.clear()
        self._cand_id = -1

    def _det_cb(self, msg: TentDetection):
        """Detektor publikuje KAZDA klatke (takze detected=False), wiec okno
        liczy sie w klatkach, nie w czasie. Wymagamy M trafien z ostatnich N —
        pojedyncze przegapienie detektora nie kasuje dorobku, a pojedyncza
        falszywka nie wystarczy do startu podlotu.

        Pewnosci nie sprawdzamy — detektor publikuje detected=True dopiero
        powyzej swojego progu 'conf'."""
        now = time.time()

        # Detektor sie zaciol / kamera padla — stare okno przestaje byc aktualne.
        if self._prev_frame_time > 0.0 and (now - self._prev_frame_time) > self.det_confirm_gap:
            if self._det_window:
                self.get_logger().warn(
                    f"Przerwa w detekcjach {now - self._prev_frame_time:.2f}s "
                    f"(> {self.det_confirm_gap}s) — okno potwierdzania wyzerowane")
            self._reset_det_window()
        self._prev_frame_time = now

        good = bool(msg.detected)

        if good and self.require_same_track and msg.track_id >= 0:
            # Tracker widzi obiekt: liczymy tylko klatki tej samej sciezki.
            # Skok na inne ID = inny obiekt, wiec potwierdzamy go od zera.
            if self._cand_id < 0:
                self._cand_id = msg.track_id
            elif msg.track_id != self._cand_id:
                self.get_logger().info(
                    f"Zmiana sledzonego obiektu (ID {self._cand_id} → {msg.track_id}) "
                    f"— potwierdzam od nowa")
                self._det_window.clear()
                self._cand_id = msg.track_id

        self._det_window.append(1 if good else 0)

        if not good:
            return

        bb = msg.bounding_box  # [x, y, w, h]
        self.tent_cx = bb[0] + bb[2] / 2.0
        self.tent_cy = bb[1] + bb[3] / 2.0
        self.last_conf = msg.confidence
        self.last_det_time = now
        self._new_det = True

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

        # Znormalizowane uchyby obrazu (-1..+1).
        # W oknie lost_timeout uzywamy OSTATNIEJ znanej pozycji namiotu —
        # pojedyncza pusta klatka detekcji nie zeruje sterowania.
        have_target = dt_lost <= self.lost_timeout
        half_w = self.img_w / 2.0
        half_h = self.img_h / 2.0
        ex = (self.tent_cx - half_w) / half_w if have_target else 0.0
        ey = (self.tent_cy - half_h) / half_h if have_target else 0.0

        # ─── Przejścia stanów ─────────────────────────────────
        if dt_lost > self.lost_timeout:
            if self.state != State.SEARCH:
                self.get_logger().info("Namiot ZGUBIONY → SEARCH (hamowanie)")
                self.state = State.SEARCH
                self._set_gimbal(self.pitch_search)
                self._brake()
            self._reset_det_window()

        elif have_target:
            # SEARCH → APPROACH dopiero po M trafieniach z ostatnich N klatek —
            # pojedyncze falszywe wskazanie nie startuje podlotu.
            if self.state == State.SEARCH:
                if self._det_hits >= self.det_confirm_frames:
                    self.get_logger().info(
                        f"Namiot POTWIERDZONY ({self._det_hits}/{len(self._det_window)} "
                        f"klatek, ID={self._cand_id}) → APPROACH")
                    self.state = State.APPROACH
                else:
                    self.get_logger().info(
                        f"[SEARCH] potwierdzam namiot {self._det_hits}/"
                        f"{self.det_confirm_frames} (okno {len(self._det_window)}/"
                        f"{self.det_window_frames}, ID={self._cand_id})...",
                        throttle_duration_sec=1.0)

            # APPROACH → HOVER: gimbal prawie pionowo
            if self.state == State.APPROACH:
                if self.pitch_deg <= self.pitch_hover_thr:
                    self.get_logger().info(
                        "NAD NAMIOTEM! → HOVER (gimbal locked)")
                    self.state = State.HOVER

        # ─── Logika stanów ────────────────────────────────────
        vx_target = 0.0
        vy_target = 0.0
        yaw_target = 0.0

        if self.state == State.SEARCH:
            # Nie wysyłamy setpointów — wejście w SEARCH zahamowało już drona
            # (_brake), a dalej lot prowadzi operator.
            # Publikujemy tylko debug/csv i wychodzimy
            self.pub_vx.publish(Float32(data=0.0))
            self.pub_vy.publish(Float32(data=0.0))
            self.pub_yaw.publish(Float32(data=0.0))
            self.pub_vz.publish(Float32(data=0.0))
            self.pub_gimbal.publish(Float32(data=float(self.pitch_deg)))
            self.pub_ex.publish(Float32(data=float(ex)))
            self.pub_ey.publish(Float32(data=float(ey)))
            self.csv_writer.writerow([
                f"{rel_time:.3f}", self.state.name, f"{ex:.3f}", f"{ey:.3f}", f"{self.pitch_deg:.1f}",
                "0.000", "0.000", "0.000", "0.000", f"{self.altitude:.2f}",
                f"{self.last_conf:.2f}", self._det_hits, len(self._det_window), self._cand_id
            ])
            self.csv_file.flush()
            self.get_logger().info(
                f"[SEARCH] czekam na namiot... gimbal={self.pitch_deg:+.1f}deg "
                f"alt={self.altitude:.1f}m det={self._det_hits}/{self.det_confirm_frames}",
                throttle_duration_sec=2.0
            )
            return  # NIE wysyłamy send_vectors — steruje operator

        elif self.state == State.APPROACH:
            # 1. Gimbal pitch — geometrycznie, RAZ na swieza detekcje
            #    (detekcja jest wolniejsza niz ta petla; wielokrotne uzycie tej
            #    samej klatki powodowaloby oscylacje gimbala).
            if self._new_det:
                self._new_det = False
                # Martwa strefa jak w gimbal_controller: blisko srodka nie ruszamy
                # gimbala (koniec drgan).
                if abs(ey) >= self.gimbal_deadzone:
                    correction = ey * (self.vfov_deg / 2.0) * self.damping
                    self._set_gimbal(self.pitch_deg - correction)

            # 2. vx — leć do przodu proporcjonalnie do pochylenia gimbala
            #    pitch=pitch_max (przod/skos) → namiot daleko → leć szybko (ratio=1.0)
            #    pitch=pitch_min (prosto w dol) → namiot pod spodem → hamuj (ratio=0.0)
            span = self.pitch_max - self.pitch_min
            forward_ratio = (self.pitch_deg - self.pitch_min) / span if span else 0.0
            forward_ratio = clamp(forward_ratio, 0.0, 1.0)
            vx_target = self.kp_vx * forward_ratio

            # 3. vy — WYŁĄCZONE w APPROACH (używamy tylko yaw do celowania)
            vy_target = 0.0

            # 4. yaw — obracaj nos drona w kierunku namiotu (uchyb X)
            yaw_target = self.kp_yaw * ex

        elif self.state == State.HOVER:
            # Gimbal zablokowany pionowo w dol
            self._set_gimbal(self.pitch_min)
            self._new_det = False

            # Korekcja mikroruchów z uchybów obrazu (tylko translacja, bez obrotu)
            # Gdy kamera patrzy w dół: ex→vy, ey→-vx (obraz jest "odwrócony" vs body)
            # Martwa strefa: blisko środka nie ruszamy dronem.
            vx_target = -self.kp_vy * ey if abs(ey) >= self.hover_deadzone else 0.0
            vy_target =  self.kp_vy * ex if abs(ex) >= self.hover_deadzone else 0.0
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
        self.pub_gimbal.publish(Float32(data=float(self.pitch_deg)))
        self.pub_ex.publish(Float32(data=float(ex)))
        self.pub_ey.publish(Float32(data=float(ey)))

        # ─── Zapis do CSV ─────────────────────────────────────
        self.csv_writer.writerow([
            f"{rel_time:.3f}", self.state.name, f"{ex:.3f}", f"{ey:.3f}", f"{self.pitch_deg:.1f}",
            f"{self.sm_vx:.3f}", f"{self.sm_vy:.3f}", f"{self.sm_yaw:.3f}", f"{vz:.3f}", f"{self.altitude:.2f}",
            f"{self.last_conf:.2f}", self._det_hits, len(self._det_window), self._cand_id
        ])
        self.csv_file.flush() # upewnij się, że dane są na dysku

        # ─── Logi ─────────────────────────────────────────────
        self.get_logger().info(
            f"[{self.state.name}] ex={ex:+.2f} ey={ey:+.2f} "
            f"gimbal={self.pitch_deg:+.1f}deg | "
            f"vx={self.sm_vx:+.2f} vy={self.sm_vy:+.2f} "
            f"yr={self.sm_yaw:+.2f} alt={self.altitude:.1f}m",
            throttle_duration_sec=1.0
        )

    # ═══════════════════════════════════════════════════════════
    #  Misja
    # ═══════════════════════════════════════════════════════════

    def run_mission(self):
        self.get_logger().info("=== START MISJI: suas_flight_controller ===")
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

        # Gimbal do pozycji startowej (szukanie) — wyslij bezwarunkowo,
        # bo pitch_deg startuje juz na pitch_search i _set_gimbal by pominal
        self.pitch_deg = self.pitch_search
        self.set_gimbal_pitch(self.pitch_deg)

        # Start pętli
        self.state = State.SEARCH
        self.last_det_time = 0.0
        self._prev_frame_time = 0.0
        self._reset_det_window()
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
    node = SuasFlightController()

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
