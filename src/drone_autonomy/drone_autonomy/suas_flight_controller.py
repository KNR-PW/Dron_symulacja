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

UWAGA: domyslne parametry sa dla REALU (OAK-D PRO W: img 1024x1024, vfov 64.4)
i w Gazebo dzialaja BEZ ZMIAN — symulacja zostala dociagnieta do realu:
  * kamera: model gimbal_small_3d ma 1024x1024 i FOV 64.4 deg,
  * gimbal: pitch idzie przez SERVO7 (kanal 6 w ArduPilotPlugin modelu
    iris_with_gimbal), czyli ta sama sciezka DO_SET_SERVO(7) co na realu,
    z ta sama kalibracja 1100 us = -90 deg, 1600 us = -45 deg.
Nie nadpisuj wiec w symulacji ani vfov_deg/img_*, ani katow pitch_*.
Jedyne, co warto podniesc w Gazebo, to det_confirm_gap (detektor chodzi tam
~2-4 FPS zamiast 7-14).

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
import math
import os
import select
import sys
import termios
import time
import tty
from collections import deque
from enum import Enum, auto

import rclpy

from drone_autonomy.drone_comunication.drone_controller import DroneController
from drone_autonomy.geometry import _project_pixel
from drone_interfaces.msg import Telemetry, TentDetection
from std_msgs.msg import Empty, Float32


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

    def __init__(self, node_name='suas_flight_controller'):
        super().__init__(node_name)

        # ─── Parametry ROS ────────────────────────────────────
        # UWAGA: ta sama wartosc domyslna co w launchu — nie rozjezdzac ich.
        self.declare_parameter('target_alt', 5.0)
        # Zakres pracy gimbala w STOPNIACH — parametry (jak w suas_gimbal_controller),
        # bo Gazebo i real maja rozne kalibracje (patrz docstring modulu).
        # Ten kat dziala TYLKO dopoki dron nic nie widzi — po pierwszej swiezej
        # detekcji gimbal i tak koryguje sie na cel. Bardziej w dol = lepsze
        # pokrycie terenu tuz pod dronem i wokol waypointu.
        self.declare_parameter('pitch_search', -55.0)     # brak celu: pod katem w dol/przod
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
        # Przerwa miedzy KLATKAMI detektora, po ktorej okno M z N jest zerowane
        # w calosci. Detektor publikuje kazda klatke — takze pusta — wiec to
        # mierzy JITTER DETEKTORA, a nie utrate celu.
        # 1.5 s, bo 0.5 zerowalo okno przy normalnym jitterze: w symulacji
        # detektor chodzi ~5 Hz z przerwami do 0.7 s i cel 31x9 px NIGDY sie
        # nie potwierdzal — dron stal nad czlowiekiem i go "nie widzial".
        # Na realu (7-14 FPS) 1.5 jest bezpieczne: w oknie akwizycji dron wisi,
        # wiec starsza o sekunde detekcja dotyczy tej samej sceny, a przed
        # falszywkami chroni i tak 4 z 8 klatek oraz require_same_track.
        self.declare_parameter('det_confirm_gap', 1.5)
        # Progu pewnosci tu NIE ma — filtruje juz detektor (parametr 'conf' w
        # launchach detekcji). Jedno pokretlo, jedno miejsce.
        # Wymagaj tego samego ID sciezki z trackera w calym oknie
        self.declare_parameter('require_same_track', True)
        self.declare_parameter('kp_vx', 2.0)
        # UWAGA na jednostke: kp_hover mnozy uchyb w METRACH, wiec ma wymiar 1/s
        # (m/s na metr uchybu). Poprzednik kp_vy mnozyl uchyb ZNORMALIZOWANY,
        # przez co to samo ustawienie dawalo inna reakcje na kazdej wysokosci:
        # ey=0.2 to 3.8 m na 30 m, ale 10 m na 80 m.
        #
        # 0.05 odtwarza sprawdzone w locie zachowanie starego kp_vy=1.0 na 30 m
        # (0.2 m/s przy ey=0.2). Roznica polega na tym, ze teraz ta sama reakcja
        # obowiazuje na kazdej wysokosci — na 80 m stary regulator dawal te same
        # 0.2 m/s przy uchybie 10 m, czyli dosuwal sie w nieskonczonosc.
        self.declare_parameter('kp_hover', 0.2)
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
        # Martwa strefa i tolerancja tez w METRACH — inaczej "wycentrowany"
        # znaczyloby co innego na kazdej wysokosci.
        self.declare_parameter('hover_deadzone_m', 0.5)
        # Topic detekcji: /tent_detections albo /people_detections. Misja
        # przelacza go pod cel, ktorym sie akurat zajmuje.
        self.declare_parameter('detection_topic', '/tent_detections')
        # Odejmowanie przechylu ramy od uchybu obrazu. Gimbal NIE jest
        # stabilizowany, wiec gdy dron pochyla sie zeby przyspieszyc, kamera
        # pochyla sie razem z nim i cel "ucieka" po kadrze. Regulator bierze to
        # za blad pozycji, dodaje gazu, dron pochyla sie mocniej — pętla domyka
        # sie DODATNIM sprzezeniem i rozbuja sie tym szybciej, im wieksze kp.
        # Zmierzone w locie: przy kp_hover=0.2 uchyb narastal z 4 do 17 m przy
        # predkosci nieprzekraczajacej 2.5 m/s, czyli cel skakal po kadrze,
        # a nie dron po swiecie.
        # false = stare zachowanie (goly piksel razy skala), do porownania.
        self.declare_parameter('tilt_comp', True)
        self.declare_parameter('cam_yaw_offset_deg', 0.0)

        # ─── Zrzut ladunku ────────────────────────────────────
        # Kanaly serw, po jednym na ladunek. 0 = kanal nieskonfigurowany, czyli
        # tryb symulacji: zrzut tylko sie loguje i nic nie jedzie na sprzet.
        # ROS 2 nie przyjmuje pustej listy jako wartosci domyslnej (nie zna
        # wtedy typu), stad zera zamiast [].
        # Na realu np. drop_servo_ch:=[9,10] — i pamietaj o SERVOn_FUNCTION=0.
        self.declare_parameter('drop_servo_ch', [0, 0])
        self.declare_parameter('drop_pwm_open', 1900)
        self.declare_parameter('drop_pwm_close', 1100)
        self.declare_parameter('drop_hold_s', 1.0)

        # Czy przy zgubieniu celu odstawiac gimbal na pitch_search.
        # W fazie DOLOTU tak — -55 st. patrzy przed siebie i pomaga cel znalezc.
        # W finalnym CENTROWANIU odwrotnie: wisimy nad celem, czyli cel jest
        # w nadirze, a -55 odchyla os kamery o 35 st. przy polowie FOV 32,2 st.
        # — punkt pod dronem wypada POZA kadrem i cel przestaje byc odzyskiwalny.
        # Dlatego suas_full_mission.center_over_target ustawia to na False.
        self.declare_parameter('search_gimbal_on_lost', True)

        # ─── Bramki czasowe misji ─────────────────────────────
        # Ile czekamy nad waypointem, az detektor potwierdzi klase (tym samym
        # oknem M z N klatek, co przy SEARCH->APPROACH). Wynik decyduje, czy
        # idziemy scenariuszem A (widzi) czy B (nie widzi).
        self.declare_parameter('acquire_timeout', 5.0)
        # Ile czekamy na spacje operatora. Brak reakcji = sciezka domyslna,
        # czyli zrzut na waypoint bez centrowania.
        self.declare_parameter('confirm_timeout', 15.0)
        # Zamiatanie gimbalem: dron STOI, gimbal przejezdza zakres. Na 50 m te
        # katy odpowiadaja punktom 0 / 13 / 23 / 35 m przed dronem, wiec skanuja
        # pas wzdluz nosa bez ruszania maszyna — taniej i szybciej niz spirala.
        self.declare_parameter('sweep_pitches', [-90.0, -75.0, -65.0, -55.0])
        self.declare_parameter('sweep_dwell', 2.0)

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
        self.kp_hover      = self.get_parameter('kp_hover').value
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
        self.hover_deadzone_m = self.get_parameter('hover_deadzone_m').value
        self.detection_topic = self.get_parameter('detection_topic').value
        self.tilt_comp = self.get_parameter('tilt_comp').value
        self.cam_yaw_offset = math.radians(
            self.get_parameter('cam_yaw_offset_deg').value)
        self.drop_servo_ch = list(self.get_parameter('drop_servo_ch').value)
        self.drop_pwm_open = self.get_parameter('drop_pwm_open').value
        self.drop_pwm_close = self.get_parameter('drop_pwm_close').value
        self.drop_hold_s = self.get_parameter('drop_hold_s').value
        self.search_gimbal_on_lost = self.get_parameter('search_gimbal_on_lost').value
        self.acquire_timeout = self.get_parameter('acquire_timeout').value
        self.confirm_timeout = self.get_parameter('confirm_timeout').value
        self.sweep_pitches = list(self.get_parameter('sweep_pitches').value)
        self.sweep_dwell = self.get_parameter('sweep_dwell').value
        # Ogniskowa z FOV i wysokosci kadru — ta sama liczba, ktorej uzywa
        # geolokator. Sluzy do zamiany uchybu w pikselach na metry.
        self.focal_px = (self.img_h / 2.0) / math.tan(
            math.radians(self.vfov_deg) / 2.0)

        # ─── Stan ─────────────────────────────────────────────
        self.state = State.SEARCH
        self.tent_cx = 0.0
        self.tent_cy = 0.0
        self.last_det_time = 0.0
        self.last_conf = 0.0

        self.altitude = 0.0
        self.drone_yaw = 0.0
        # Pozycja globalna — get_gps() z DroneController oddaje NED wzgledem
        # home, a do goto_global i do liczenia namiaru na cel potrzebny jest GPS.
        self.global_lat = 0.0
        self.global_lon = 0.0
        self.flight_mode = ''
        # Przechyly ramy — potrzebne, bo gimbal ich nie kompensuje mechanicznie.
        self.veh_roll = 0.0
        self.veh_pitch = 0.0

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
        # Uchwyt zapamietany, bo pelna misja przepina detekcje miedzy klasami
        # (/tent_detections <-> /people_detections) w trakcie lotu.
        self._det_sub = self.create_subscription(
            TentDetection, self.detection_topic, self._det_cb, 10)
        self.create_subscription(Telemetry, 'knr_hardware/telemetry',
                                 self._tel_cb, 10)
        # Zapasowe wejscie potwierdzenia: wezel odpalony z launcha nie ma stdin
        # podpietego do terminala i raw termios by sie wywalil. Wtedy zostaje
        #   ros2 topic pub --once /mission_confirm std_msgs/msg/Empty {}
        self._confirmed = False
        self.create_subscription(Empty, '/mission_confirm',
                                 self._confirm_cb, 10)

        # ─── Timer sterowania ─────────────────────────────────
        period = 1.0 / self.control_rate
        self._timer = self.create_timer(period, self._control_loop)
        # WYLACZONA do czasu run_mission(). Inaczej petla chodzi juz od momentu
        # powstania wezla i w trakcie dolotu potrafi ruszac gimbalem (przejscie
        # SEARCH->APPROACH ustawia kat), mimo ze misja jeszcze nie prosila
        # o sterowanie. run_mission() ja wznawia przez _timer.reset().
        self._timer.cancel()

        # Przerwanie z zewnatrz (Ctrl+C w misji) — bramki ponizej sprawdzaja to
        # w swoich petlach, zeby nie czekaly na spacje ani nie zamiatały gimbalem
        # po tym, jak operator juz przerwal.
        self._abort = False
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
        # Nazwa pliku od nazwy wezla — misja nadpisujaca ten kontroler
        # (suas_simple_mission) pisze do wlasnego CSV, a nie do tego samego pliku.
        self.csv_path = os.path.expanduser(f'~/{self.get_name()}_log.csv')
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'time_rel', 'state', 'ex', 'ey', 'gimbal_deg', 'vx_cmd', 'vy_cmd', 'yaw_cmd', 'vz_cmd', 'alt',
            'conf', 'det_hits', 'det_window', 'track_id'
        ])
        self.node_start_time = time.time()

        self.get_logger().info(f"SuasFlightController init: alt={self.target_alt}m  vfov={self.vfov_deg} damping={self.damping}  kp_vx={self.kp_vx}  kp_hover={self.kp_hover}  img={self.img_w}x{self.img_h}")
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

        if good and self.require_same_track:
            if msg.track_id < 0:
                # BRAK SCIEZKI = nie liczymy trafienia.
                # Tracker nadaje ID dopiero, gdy obiekt utrzyma sie miedzy
                # klatkami. Prawdziwy cel dostaje je od razu (zmierzone:
                # namiot ID=53, lezacy czlowiek ID=151), a track_id=-1 to
                # podpis MIGOCZACEJ FALSZYWKI — drzewa albo cienia, ktory
                # detektor widzi raz tu, raz tam.
                # Wczesniej warunek brzmial "... and msg.track_id >= 0", wiec
                # przy braku ID cala kontrola sciezki byla POMIJANA, a trafienie
                # i tak wpadalo do okna. Zabezpieczenie nie dzialalo dokladnie
                # w przypadku, przed ktorym mialo chronic: 2026-09-02 grid
                # potwierdzil drzewo (4/6 klatek, ID=-1) i dron gonil duch
                # z pozycja skaczaca o 20 m miedzy probkami.
                good = False
            elif self._cand_id < 0:
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
        self.veh_roll = msg.roll
        self.veh_pitch = msg.pitch
        self.flight_mode = msg.flight_mode
        if msg.global_lat != 0.0 or msg.global_lon != 0.0:
            self.global_lat = msg.global_lat
            self.global_lon = msg.global_lon

    def _target_offset(self):
        """Gdzie lezy cel wzgledem drona: (do przodu, w prawo) w METRACH.

        Liczone ta sama, zweryfikowana geometria co w geolokatorze — z yaw=0,
        wiec wynik wychodzi od razu w osiach drona, a nie w polnoc/wschod.
        Dzieki podaniu roll/pitch ramy z uchybu znika skladowa pochodzaca
        z pochylenia kamery, ktora jest zrodlem dodatniego sprzezenia.

        tilt_comp=False podstawia zera zamiast przechylow, co daje dokladnie
        stare zachowanie (piksel razy wysokosc przez ogniskowa) — do porownania
        w jednym locie.
        """
        if self.focal_px <= 0.0:
            return 0.0, 0.0
        roll = self.veh_roll if self.tilt_comp else 0.0
        pitch = self.veh_pitch if self.tilt_comp else 0.0
        proj = _project_pixel(self.tent_cx, self.tent_cy, self.altitude,
                              roll, pitch, 0.0,
                              self.pitch_deg, self.cam_yaw_offset,
                              self.focal_px, self.img_w / 2.0, self.img_h / 2.0,
                              False)
        if proj is None:
            return 0.0, 0.0
        fwd, right, _ = proj
        return fwd, right

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
        # Pozycja celu wzgledem drona w METRACH. Gimbal zostaje przy uchybie
        # znormalizowanym, bo jego korekta jest KATOWA (ey * vfov/2) — tam metry
        # nie maja sensu. Translacja w HOVER liczy sie juz w metrach.
        fwd_m, right_m = self._target_offset() if have_target else (0.0, 0.0)

        # ─── Przejścia stanów ─────────────────────────────────
        if dt_lost > self.lost_timeout:
            if self.state != State.SEARCH:
                self.get_logger().info("Namiot ZGUBIONY → SEARCH (hamowanie)")
                self.state = State.SEARCH
                if self.search_gimbal_on_lost:
                    self._set_gimbal(self.pitch_search)
                else:
                    # Centrowanie: cel jest pod nami, gimbal ZOSTAJE w pionie,
                    # zeby wrocil przy pierwszej dobrej klatce.
                    self.get_logger().info(
                        "gimbal zostaje w pionie (search_gimbal_on_lost=false)")
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
            # Martwa strefa i wzmocnienie w METRACH, wiec zachowanie jest takie
            # samo na 20 i na 80 m.
            vx_target = (self.kp_hover * fwd_m
                         if abs(fwd_m) >= self.hover_deadzone_m else 0.0)
            vy_target = (self.kp_hover * right_m
                         if abs(right_m) >= self.hover_deadzone_m else 0.0)
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
            f"(przod {fwd_m:+.1f} prawo {right_m:+.1f} m) "
            f"gimbal={self.pitch_deg:+.1f}deg | "
            f"vx={self.sm_vx:+.2f} vy={self.sm_vy:+.2f} "
            f"yr={self.sm_yaw:+.2f} alt={self.altitude:.1f}m",
            throttle_duration_sec=1.0
        )

    # ═══════════════════════════════════════════════════════════
    #  Bramki misji: akwizycja, potwierdzenie, zamiatanie, zrzut
    # ═══════════════════════════════════════════════════════════

    def _confirm_cb(self, _msg):
        self._confirmed = True

    def set_detection_topic(self, topic):
        """Przepnij detekcje na inna klase celu w trakcie lotu."""
        if topic == self.detection_topic and self._det_sub is not None:
            return
        if self._det_sub is not None:
            self.destroy_subscription(self._det_sub)
        self.detection_topic = topic
        self._det_sub = self.create_subscription(
            TentDetection, topic, self._det_cb, 10)
        self.last_det_time = 0.0
        self._reset_det_window()
        self.get_logger().info(f"detekcje z {topic}")

    def wait_acquire(self, timeout=None):
        """Czekaj, az detektor potwierdzi cel oknem M z N klatek.

        Ten sam mechanizm, ktory przepuszcza SEARCH -> APPROACH, wiec pojedyncza
        falszywka nie wystarczy, a pojedyncze przegapienie nie kasuje dorobku.
        Zwraca True = cel potwierdzony (scenariusz A), False = trzeba szukac (B).
        """
        timeout = self.acquire_timeout if timeout is None else timeout
        self._reset_det_window()
        end = time.time() + timeout
        while time.time() < end and rclpy.ok() and not self._abort:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._det_hits >= self.det_confirm_frames:
                self.get_logger().info(
                    f"cel potwierdzony ({self._det_hits}/{len(self._det_window)} "
                    f"klatek, ID={self._cand_id})")
                return True
        self.get_logger().warn(
            f"okno akwizycji {timeout:.0f}s minelo bez potwierdzenia")
        return False

    def wait_confirm(self, prompt, timeout=None):
        """Czekaj na SPACJE operatora. Brak reakcji = sciezka domyslna.

        Domyslne dzialanie jest bezpieczne: bez operatora dron NIE goni detekcji,
        ktora moze byc falszywa. Przy padnietym SSH zawsze idzie domyslna.

        print(), a nie logger — monit ma byc widoczny niezaleznie od tego, jak
        ustawione jest logowanie.
        """
        timeout = self.confirm_timeout if timeout is None else timeout
        print(f"\n{prompt}", flush=True)
        self._confirmed = False
        deadline = time.time() + timeout
        tty_ok = sys.stdin.isatty()
        old = termios.tcgetattr(sys.stdin) if tty_ok else None
        if tty_ok:
            tty.setcbreak(sys.stdin.fileno())
        try:
            while time.time() < deadline and rclpy.ok() and not self._abort:
                rclpy.spin_once(self, timeout_sec=0.05)
                if self._confirmed:
                    print("  -> POTWIERDZONE (topic)", flush=True)
                    return True
                if tty_ok and select.select([sys.stdin], [], [], 0)[0]:
                    if sys.stdin.read(1) == ' ':
                        print("  -> POTWIERDZONE (spacja)", flush=True)
                        return True
        finally:
            if old is not None:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        print(f"  -> brak potwierdzenia w {timeout:.0f}s, ide sciezka domyslna",
              flush=True)
        return False

    def sweep_for_target(self):
        """Zamiatanie gimbalem: dron STOI, gimbal przejezdza zakres.

        Tansze i szybsze niz spirala — skanuje pas wzdluz nosa bez ruszania
        maszyna. Rzutowanie detekcji z pochylonego gimbala jest poprawne,
        bo _target_offset bierze aktualny kat montazu.
        Zwraca True, gdy cel zostal potwierdzony po drodze.
        """
        self.get_logger().info(
            f"ZAMIATANIE GIMBALEM {self.sweep_pitches} "
            f"(po {self.sweep_dwell:.0f}s na krok)")
        self._brake()
        for pitch in self.sweep_pitches:
            if self._abort:
                return False
            self._set_gimbal(pitch)
            self._reset_det_window()
            end = time.time() + self.sweep_dwell
            while time.time() < end and rclpy.ok() and not self._abort:
                rclpy.spin_once(self, timeout_sec=0.05)
                if self._det_hits >= self.det_confirm_frames:
                    self.get_logger().info(
                        f"cel znaleziony przy gimbalu {pitch:+.0f} st.")
                    return True
        self.get_logger().warn("zamiatanie nic nie znalazlo")
        return False

    def drop(self, idx: int) -> bool:
        """Zwolnij ladunek nr idx (0-based).

        Idzie ta sama droga co gimbal — DO_SET_SERVO przez drone_handler.
        Kanal 0 = symulacja: tylko log, nic nie jedzie na sprzet.
        """
        ch = self.drop_servo_ch[idx] if idx < len(self.drop_servo_ch) else 0
        if ch <= 0:
            self.get_logger().warn(
                f"ZRZUT {idx + 1} [SYMULACJA] — kanal serwa nieskonfigurowany "
                f"(drop_servo_ch={self.drop_servo_ch}), alt={self.altitude:.1f} m")
            return False
        self.get_logger().info(
            f"ZRZUT {idx + 1}: serwo {ch} -> {self.drop_pwm_open} us "
            f"(alt={self.altitude:.1f} m)")
        self.set_servo(ch, self.drop_pwm_open)
        end = time.time() + self.drop_hold_s
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
        self.set_servo(ch, self.drop_pwm_close)
        return True

    # ═══════════════════════════════════════════════════════════
    #  Misja
    # ═══════════════════════════════════════════════════════════

    def run_mission(self, start_state=None, fresh=True):
        """Wlacz sterowanie wizualne.

        fresh=True (domyslnie): start od zera — gimbal na pitch_search, stan
        SEARCH, wyzerowane okno potwierdzania. Tak startuje suas_simple_mission.

        fresh=False: KONTYNUACJA. Nie ruszamy gimbala ani okna detekcji, bo cel
        zostal juz znaleziony (np. przez zamiatanie) i przestawienie gimbala na
        pitch_search natychmiast by go zgubilo — dron stoi nad celem, a -55 st.
        patrzy 35 m przed siebie. Stan domyslnie APPROACH, bo istniejaca logika
        sama przejdzie do HOVER, gdy gimbal dojdzie do pionu.
        """
        self.get_logger().info("=== START MISJI: suas_flight_controller ===")
        self.get_logger().info(
            f"Target alt: {self.target_alt}m | "
            f"kp_vx={self.kp_vx} kp_hover={self.kp_hover} max_vel={self.max_vel} "
            f"| focal={self.focal_px:.0f} px, detekcje z {self.detection_topic}")

        # Włącz velocity control
        result = self.toggle_control()
        if not result.result:
            self.get_logger().warn("toggle_control OFF→ON retry")
            self.toggle_control()
        self.velocity_mode_active = True
        self.get_logger().info("Velocity control AKTYWNY")

        if fresh:
            # Gimbal do pozycji startowej (szukanie) — wyslij bezwarunkowo,
            # bo pitch_deg startuje juz na pitch_search i _set_gimbal by pominal
            self.pitch_deg = self.pitch_search
            self.set_gimbal_pitch(self.pitch_deg)
            self.state = State.SEARCH if start_state is None else start_state
            self.last_det_time = 0.0
            self._prev_frame_time = 0.0
            self._reset_det_window()
            self.get_logger().info("Szukam celu...")
        else:
            self.state = State.APPROACH if start_state is None else start_state
            self.get_logger().info(
                f"Kontynuuje ze stanu {self.state.name}, gimbal "
                f"{self.pitch_deg:+.0f} st. (cel juz znaleziony)")
        self._timer.reset()

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
