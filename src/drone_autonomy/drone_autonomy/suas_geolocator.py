#!/usr/bin/env python3
"""
suas_geolocator — zapis wspolrzednych GPS namiotu podczas przelotu ortofoto.

Node PASYWNY: nie steruje dronem, tylko slucha detekcji i telemetrii. Jedyne co
wysyla, to kat gimbala (-90 = prosto w dol), zeby kadr byl na pewno w nadirze.

Zasada dzialania: kazda detekcja jest rzutowana z piksela na punkt na ziemi
(wysokosc z telemetrii + ogniskowa kamery), a punkty sa grupowane przestrzennie.
Prawdziwy namiot widziany z wielu pozycji drona daje 100+ trafien zbieznych
w jednym miejscu; false positive daje kilka rozrzuconych. To mocniejszy filtr
niz cokolwiek liczonego na pojedynczej klatce.

Zalozenia (patrz docs/suas_geolocator.md):
  * gimbal ma JEDNA OS (pitch) i NIE jest stabilizowany - MNT1_TYPE=0 oraz
    SERVO7_FUNCTION=0 w SITL_param/gazebo_iris.parm, a drone_handler steruje
    surowym DO_SET_SERVO. Kamera pochyla sie razem z rama, wiec rzutowanie
    ODEJMUJE roll/pitch rachunkiem (parametr gimbal_stabilized na wypadek,
    gdyby kiedys wszedl stabilizowany mount),
  * kadr zorientowany "gora = przod drona" (cam_yaw_offset_deg = 0).

Wyjscie w save_dir:
  tent_target.json           staly plik dla misji zrzutu (nadpisywany atomowo)
  <data-godzina>/target.json      kopia z tego lotu
  <data-godzina>/observations.csv KAZDA przyjeta detekcja, dane surowe
  <data-godzina>/kandydat_NN.jpg  klatka podgladu, 1 na kandydata
"""

import bisect
import csv
import json
import math
import os
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Float32

from drone_autonomy.geometry import _project_pixel
from drone_interfaces.msg import Telemetry, TentDetections, OperatorMark


# ────────────────────── Helpers ──────────────────────

def meters_to_gps(lat0, lon0, d_north, d_east):
    """
    Proste przybliżenie lokalne: metry -> delta GPS. Wzor przepisany z
    coverage_node.meters_to_gps; NIE importujemy go, bo tamten modul ciagnie
    na starcie px4_msgs, a my jestesmy na ArduPilocie.
    """
    lat_rad = math.radians(lat0)
    d_lat = d_north / 111_320.0
    d_lon = d_east / (111_320.0 * math.cos(lat_rad))
    return lat0 + d_lat, lon0 + d_lon


CLASS_NAMES = {0: 'namiot', 1: 'czlowiek'}
CLASS_KEYS = {0: 'tent', 1: 'people'}


class Candidate:
    """Jeden klaster detekcji = jeden domniemany cel danej klasy."""

    def __init__(self, cid, class_id, north, east, conf, t, drone_ne,
                 source='auto'):
        self.id = cid
        self.class_id = class_id
        # 'operator' = wskazany klikniejciem w GUI. Ma pierwszenstwo nad
        # automatem niezaleznie od liczby obserwacji: czlowiek widzi wiecej
        # niz model, ktory na 80 m ma na celu 11 pikseli.
        self.source = source
        # Srednia wazona pewnoscia — pewniejsze detekcje ciagna punkt mocniej.
        self._sum_n = north * conf
        self._sum_e = east * conf
        self._sum_w = conf
        self.n_obs = 1
        self.sum_conf = conf
        self.first_t = t
        self.last_t = t
        self.n_passes = 1
        # Skrajne pozycje drona przy obserwacjach — im dalej od siebie, tym
        # lepsza triangulacja i tym mniejsza szansa, ze to artefakt jednego ujecia.
        self._drone_pts = [drone_ne]
        # Rozrzut samych rzutowanych punktow = realna dokladnosc pomiaru.
        self._pts = [(north, east)]

    @property
    def north(self):
        return self._sum_n / self._sum_w

    @property
    def east(self):
        return self._sum_e / self._sum_w

    @property
    def mean_conf(self):
        return self.sum_conf / self.n_obs

    @property
    def score(self):
        return self.n_obs * self.mean_conf

    def add(self, north, east, conf, t, drone_ne, pass_gap, source='auto'):
        if t - self.last_t > pass_gap:
            self.n_passes += 1
        if source == 'operator':
            self.source = 'operator'
        self._sum_n += north * conf
        self._sum_e += east * conf
        self._sum_w += conf
        self.n_obs += 1
        self.sum_conf += conf
        self.last_t = t
        self._drone_pts.append(drone_ne)
        self._pts.append((north, east))

    def drone_spread(self):
        """Najwieksza odleglosc miedzy pozycjami drona przy obserwacjach [m]."""
        return _max_pairwise(self._drone_pts)

    def point_spread(self):
        """Odchylenie rzutowanych punktow od sredniej [m] — realna dokladnosc."""
        n0, e0 = self.north, self.east
        if len(self._pts) < 2:
            return 0.0
        return math.sqrt(sum((n - n0) ** 2 + (e - e0) ** 2
                             for n, e in self._pts) / len(self._pts))


def _max_pairwise(pts, cap=200):
    """Maksymalna odleglosc w zbiorze. Przy duzej liczbie punktow probkujemy,
    bo pelne O(n^2) na 10 tys. obserwacji zablokowaloby watek."""
    if len(pts) < 2:
        return 0.0
    if len(pts) > cap:
        step = len(pts) // cap
        pts = pts[::step]
    best = 0.0
    for i in range(len(pts)):
        ax, ay = pts[i]
        for j in range(i + 1, len(pts)):
            bx, by = pts[j]
            d = math.hypot(ax - bx, ay - by)
            if d > best:
                best = d
    return best


class SuasGeolocator(Node):

    def __init__(self):
        super().__init__('suas_geolocator')

        # ── 1. PARAMETRY ────────────────────────────────────────────
        self.declare_parameter('save_dir', os.path.expanduser('~/suas_targets'))
        # Gimbal: node sam pilnuje nadiru, zeby kat byl pewny takze po restarcie
        # czegokolwiek. Przy lock_nadir=true suas_gimbal_controller NIE MOZE chodzic.
        self.declare_parameter('lock_nadir', True)
        self.declare_parameter('mount_pitch_deg', -90.0)
        # Obrot kadru wzgledem osi drona: 0 = gora kadru to przod drona.
        self.declare_parameter('cam_yaw_offset_deg', 0.0)
        # false = gimbal jednoosiowy bez stabilizacji (nasz przypadek: MNT1_TYPE=0,
        # SERVO7_FUNCTION=0), wiec rzutowanie samo odejmuje roll/pitch z telemetrii.
        # true = mechanika juz je zdjela i kompensacja liczylaby sie podwojnie.
        self.declare_parameter('gimbal_stabilized', False)

        # Geometria kamery — preview OAK 1024x1024, VFOV 64.4 st.
        self.declare_parameter('focal_px', 813.0)
        self.declare_parameter('img_w', 1024)
        self.declare_parameter('img_h', 1024)

        # Zapasowy szacunek opoznienia detekcji: uzywany TYLKO wtedy, gdy
        # header.stamp klatki chodzi w innym zegarze niz time.time()
        # (most z Gazebo podaje czas symulacji od zera) - patrz _frame_time.
        self.declare_parameter('det_latency', 0.20)

        # Bramki na pojedynczej detekcji
        self.declare_parameter('min_conf', 0.0)      # 0 = ufaj progowi detektora
        self.declare_parameter('min_alt', 10.0)      # nie zbieraj smieci przy ziemi
        # Przechyl jest teraz KOMPENSOWANY w rzutowaniu, wiec ta bramka nie sluzy
        # juz do ratowania dokladnosci - zostaje tylko po to, zeby odrzucic ostre
        # zakrety, gdzie i tak dochodzi rozmycie ruchem. Wczesniej bylo 10 st.,
        # co odrzucalo prawie kazda klatke z prostego galsu przy 8 m/s.
        self.declare_parameter('max_tilt', 25.0)     # st.
        self.declare_parameter('center_frac', 0.8)   # tylko srodkowe 80% kadru
        self.declare_parameter('tent_size_m', 3.0)   # do bramki na rozmiar boxa
        self.declare_parameter('person_size_m', 0.6)  # czlowiek z nadiru
        self.declare_parameter('size_tol_lo', 0.3)
        self.declare_parameter('size_tol_hi', 3.0)
        self.declare_parameter('telemetry_max_age', 0.5)
        # Ile stempel klatki moze sie roznic od zegara sciennego, zeby uznac go
        # za ten sam zegar. Most z Gazebo podaje czas symulacji liczony od zera,
        # wiec bez tej bramki mieszalibysmy dwa zegary i rzutowanie sypaloby sie
        # po cichu. Powyzej progu wracamy do stalego szacunku det_latency.
        self.declare_parameter('stamp_max_skew', 5.0)

        # Klastrowanie
        self.declare_parameter('cluster_radius', 10.0)   # m; z grubsza 0.12 * H
        self.declare_parameter('min_obs', 10)            # tyle trafien = kandydat na zwyciezce
        # Czlowiek jest mniejszy i gorzej wykrywany, wiec zbiera mniej trafien.
        # Za wysoki prog oznaczalby, ze nigdy nie awansuje na 'best'.
        self.declare_parameter('min_obs_person', 5)
        # Powyzej tej wysokosci NIE przyjmujemy automatycznych detekcji czlowieka.
        # 50 m to wysokosc zrzutu, czyli najnizszy pulap misji — automat dostaje
        # szanse dokladnie tam, gdzie ma jej najwiecej, a caly przelot ortofoto
        # (80 m) jest odciety.
        #
        # Powod: czlowiek ma z nadiru ok. 0.5 m, czyli 8 px z 50 m i 5 px z 80 m —
        # YOLO ma stride 8, wiec z pulapu ortofoto nie ma czego wykrywac i kazda
        # detekcja stamtad jest smieciem. W symulacji model bral znacznik ArUco za
        # czlowieka przez 166 klatek przy conf 0.53; taki klaster awansowalby na
        # 'best' i misja poleciałaby zrzucic ladunek na znacznik.
        #
        # UWAGA: bramka dotyczy WYLACZNIE automatu. Klikniecie operatora dziala
        # na kazdej wysokosci — to jest caly sens recznego oznaczania.
        self.declare_parameter('person_max_alt', 50.0)
        self.declare_parameter('pass_gap', 3.0)          # przerwa cos>tyle s = kolejny przelot

        self.declare_parameter('snapshots', True)
        self.declare_parameter('report_period', 5.0)

        p = self.get_parameter
        self.lock_nadir = p('lock_nadir').value
        self.mount_pitch = p('mount_pitch_deg').value
        # mount_pitch = kat, ktorym RZUTUJEMY (aktualnie zadany gimbalowi).
        # _nadir_target = kat, ktory sami wymuszamy, gdy trzymamy blokade nadiru.
        # Rozdzielone celowo: po przejeciu lotu misja pochyla gimbal w APPROACH,
        # a my musimy rzutowac tym samym katem — inaczej kazde pochylenie
        # psuloby obserwacje (rzutowanie zakladaloby -90, kamera patrzy -70).
        self._nadir_target = float(self.mount_pitch)
        self.cam_yaw_offset = math.radians(p('cam_yaw_offset_deg').value)
        self.gimbal_stabilized = p('gimbal_stabilized').value
        self.focal_px = p('focal_px').value
        self.cx = p('img_w').value / 2.0
        self.cy = p('img_h').value / 2.0
        self.img_w = p('img_w').value
        self.img_h = p('img_h').value
        self.det_latency = p('det_latency').value
        self.min_conf = p('min_conf').value
        self.min_alt = p('min_alt').value
        self.max_tilt = math.radians(p('max_tilt').value)
        self.center_frac = p('center_frac').value
        self.size_m = {0: p('tent_size_m').value,
                       1: p('person_size_m').value}
        self.size_tol_lo = p('size_tol_lo').value
        self.size_tol_hi = p('size_tol_hi').value
        self.telemetry_max_age = p('telemetry_max_age').value
        self.stamp_max_skew = p('stamp_max_skew').value
        self.cluster_radius = p('cluster_radius').value
        self.min_obs = {0: p('min_obs').value,
                        1: p('min_obs_person').value}
        self.max_alt_auto = {0: float('inf'),
                             1: p('person_max_alt').value}
        self.pass_gap = p('pass_gap').value
        self.snapshots = p('snapshots').value
        report_period = p('report_period').value

        # ── 2. STAN ─────────────────────────────────────────────────
        # Bufor telemetrii: potrzebny, bo detekcja przychodzi ~det_latency po
        # zrobieniu zdjecia, a telemetria leci 10 Hz. Interpolujemy wstecz.
        self._telem = deque(maxlen=64)     # (t, lat, lon, alt, roll, pitch, yaw)
        self._telem_t = deque(maxlen=64)   # te same t, osobno pod bisect
        self.origin = None                 # (lat, lon) pierwszego fixa
        self.candidates = []
        self._next_cid = 1
        self._best_id = {}      # class_id -> id kandydata
        self._stamp_mode = None  # log raz: 'stamp klatki' czy 'det_latency'
        self._last_cid = -1      # do kolumny cluster_id w CSV
        self._last_jpeg = None
        self._n_det = 0
        self._rejects = {'telemetria': 0, 'wysokosc': 0, 'przechyl': 0,
                         'brzeg': 0, 'rozmiar': 0, 'pewnosc': 0, 'promien': 0,
                         'czlowiek_wysoko': 0}

        # ── 3. PLIKI ────────────────────────────────────────────────
        base = os.path.expanduser(p('save_dir').value)
        self.flight_dir = os.path.join(base, time.strftime('%Y-%m-%d_%H-%M'))
        os.makedirs(self.flight_dir, exist_ok=True)
        # targets.json - nowy format z sekcjami obu klas; to czyta misja.
        self.targets_json = os.path.join(base, 'targets.json')
        self.flight_json = os.path.join(self.flight_dir, 'targets.json')
        # tent_target.json - stary format, tylko namiot. Zostaje dla zgodnosci
        # z tym, co juz bylo uruchamiane w polu.
        self.target_json = os.path.join(base, 'tent_target.json')
        self._csv_f = open(os.path.join(self.flight_dir, 'observations.csv'),
                           'w', newline='')
        self._csv = csv.writer(self._csv_f)
        self._csv.writerow(['t', 'class', 'u', 'v', 'w', 'h', 'conf', 'track_id',
                            'lat', 'lon', 'alt', 'roll', 'pitch', 'yaw',
                            'det_lat', 'det_lon', 'cluster_id'])

        # ── 4. PUB / SUB ────────────────────────────────────────────
        # /detections zamiast /tent_detections: komplet boxow z klatki, obie
        # klasy naraz, i - co wazniejsze - z naglowkiem czasu klatki.
        self.create_subscription(TentDetections, '/detections', self._det_cb, 10)
        self.create_subscription(OperatorMark, '/operator_mark', self._mark_cb, 10)
        self.create_subscription(Telemetry, 'knr_hardware/telemetry',
                                 self._telem_cb, 10)
        if self.snapshots:
            # Podglad detektora leci BEST_EFFORT i jest publikowany tylko przy
            # aktywnej subskrypcji — trzymamy ostatnia klatke, zapisujemy na dysk
            # dopiero przy nowym kandydacie.
            img_qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                                 reliability=ReliabilityPolicy.BEST_EFFORT)
            self.create_subscription(CompressedImage,
                                     '/tent_detections/image/compressed',
                                     self._img_cb, img_qos)
        self._gimbal_pub = self.create_publisher(
            Float32, 'knr_hardware/gimbal_pitch', 10)
        # Sledzimy KAZDA komende gimbala — takze cudza (misja w APPROACH) —
        # i rzutujemy aktualnie zadanym katem. Serwo nie ma sprzezenia, wiec
        # komenda jest najlepsza wiedza, jaka mamy; slew ~0.5 s akceptujemy.
        self.create_subscription(Float32, 'knr_hardware/gimbal_pitch',
                                 self._gimbal_cmd_cb, 10)
        # Misja zwalnia blokade nadiru przy przejeciu lotu (false). Od tej
        # chwili gimbal nalezy do niej i nie wolno nam go szarpac do pionu.
        self.create_subscription(Bool, '/geolocator/lock_nadir', self._lock_cb, 10)

        # ── 5. TIMERY ───────────────────────────────────────────────
        # Timer zawsze istnieje; publikuje tylko gdy blokada jest wlaczona,
        # bo misja moze ja zwolnic (i teoretycznie przywrocic) w locie.
        if self.lock_nadir:
            self._set_nadir()
        self.create_timer(2.0, self._set_nadir)
        self.create_timer(report_period, self._report)

        self.get_logger().info(
            f"suas_geolocator gotowy | zapis: {self.flight_dir} | "
            f"nadir={'TAK' if self.lock_nadir else 'NIE'} "
            f"({self.mount_pitch:+.0f} st.) | promien klastra "
            f"{self.cluster_radius:.0f} m | min_obs={self.min_obs}")
        self.get_logger().info(
            f"klasy: {CLASS_NAMES} | rozmiary [m]: {self.size_m} | "
            f"kompensacja przechylu: {'WYLACZONA (mount stabilizowany)' if self.gimbal_stabilized else 'WLACZONA'}")
        self.get_logger().info(
            f"automat dla czlowieka tylko ponizej "
            f"{self.max_alt_auto[1]:.0f} m (wyzej ma za malo pikseli); "
            f"znaczniki operatora dzialaja na kazdej wysokosci")
        if self.lock_nadir:
            self.get_logger().warn(
                "lock_nadir=true — trzymam gimbal w pionie do czasu, az misja "
                "zwolni blokade (/geolocator/lock_nadir). NIE uruchamiaj "
                "rownolegle suas_gimbal_controller")

    # ────────────────────── Gimbal ──────────────────────

    def _set_nadir(self):
        if self.lock_nadir:
            self._gimbal_pub.publish(Float32(data=float(self._nadir_target)))

    def _gimbal_cmd_cb(self, msg: Float32):
        new = float(msg.data)
        if abs(new - self.mount_pitch) > 0.5:
            self.get_logger().info(
                f"gimbal zadany {new:+.0f} st. — rzutuje tym katem")
        self.mount_pitch = new

    def _lock_cb(self, msg: Bool):
        want = bool(msg.data)
        if want != self.lock_nadir:
            self.get_logger().info(
                "blokada nadiru " + ("WLACZONA" if want else
                                     "ZWOLNIONA — gimbalem steruje misja"))
        self.lock_nadir = want

    # ────────────────────── Telemetria ──────────────────────

    def _telem_cb(self, msg: Telemetry):
        # global_lat/global_lon sa float64; msg.lat/msg.lon to float32,
        # czyli ~0.7 m kwantyzacji na naszej szerokosci — nie uzywamy ich.
        if msg.global_lat == 0.0 and msg.global_lon == 0.0:
            return                      # brak fixa GPS
        t = time.time()
        self._telem.append((t, msg.global_lat, msg.global_lon, msg.alt,
                            msg.roll, msg.pitch, msg.yaw))
        self._telem_t.append(t)
        if self.origin is None:
            self.origin = (msg.global_lat, msg.global_lon)

    def _telem_at(self, t):
        """Telemetria w chwili t, interpolowana liniowo miedzy probkami."""
        if not self._telem:
            return None
        ts = list(self._telem_t)
        if t <= ts[0]:
            return self._telem[0] if ts[0] - t < self.telemetry_max_age else None
        if t >= ts[-1]:
            return self._telem[-1] if t - ts[-1] < self.telemetry_max_age else None
        i = bisect.bisect_left(ts, t)
        a, b = self._telem[i - 1], self._telem[i]
        span = b[0] - a[0]
        f = 0.0 if span <= 0 else (t - a[0]) / span
        out = [a[0] + (b[0] - a[0]) * f]
        for k in range(1, 4):                       # lat, lon, alt
            out.append(a[k] + (b[k] - a[k]) * f)
        for k in range(4, 7):                       # roll, pitch, yaw
            out.append(_lerp_angle(a[k], b[k], f))
        return tuple(out)

    # ────────────────────── Podglad ──────────────────────

    def _img_cb(self, msg: CompressedImage):
        self._last_jpeg = msg.data

    # ────────────────────── Detekcje ──────────────────────

    def _frame_time(self, header, now):
        """Czas klatki uzywany do wyszukania telemetrii.

        Wolimy header.stamp, bo niesie moment powstania klatki i zdejmuje
        zgadywanie opoznienia inferencji. Ale stamp moze chodzic w INNYM zegarze
        niz time.time() - most z Gazebo podaje czas symulacji liczony od zera -
        wiec akceptujemy go tylko, gdy jest zbiezny z zegarem sciennym.
        Inaczej wracamy do stalego szacunku det_latency.
        """
        ts = header.stamp.sec + header.stamp.nanosec * 1e-9
        if ts > 0.0 and abs(ts - now) < self.stamp_max_skew:
            mode = 'stamp klatki'
            t = ts
        else:
            mode = f'det_latency={self.det_latency:.2f}s (stamp w innym zegarze)'
            t = now - self.det_latency
        if self._stamp_mode != mode:
            self._stamp_mode = mode
            self.get_logger().info(f"czas detekcji brany z: {mode}")
        return t

    def _det_cb(self, msg: TentDetections):
        """Cala klatka naraz: wspolne bramki liczymy raz, boxy po kolei."""
        if not msg.detections:
            return
        now = time.time()

        tel = self._telem_at(self._frame_time(msg.header, now))
        if tel is None:
            self._rejects['telemetria'] += len(msg.detections)
            return
        _, lat, lon, alt, roll, pitch, yaw = tel

        if alt < self.min_alt:
            self._rejects['wysokosc'] += len(msg.detections)
            return
        # Przechyl jest kompensowany w rzutowaniu, wiec ta bramka odcina juz
        # tylko ostre zakrety, gdzie dochodzi rozmycie ruchem.
        if abs(roll) > self.max_tilt or abs(pitch) > self.max_tilt:
            self._rejects['przechyl'] += len(msg.detections)
            return

        img_w = msg.img_w or self.img_w
        img_h = msg.img_h or self.img_h

        for det in msg.detections:
            if not det.detected:
                continue
            self._n_det += 1

            if det.confidence < self.min_conf:
                self._rejects['pewnosc'] += 1
                continue

            size_m = self.size_m.get(det.class_id)
            if size_m is None:
                continue                      # klasa spoza mapowania - ignoruj

            # Limit wysokosci PER KLASA, tylko dla automatu. Z pulapu ortofoto
            # czlowiek ma kilka pikseli, wiec to co model tam "widzi" to niemal
            # na pewno smiec — a raz zbudowany klaster fałszywki trafilby do misji.
            if alt > self.max_alt_auto.get(det.class_id, float('inf')):
                self._rejects['czlowiek_wysoko'] += 1
                continue

            x, y, w, h = det.bounding_box
            u = x + w / 2.0
            v = y + h / 2.0

            # Brzegi kadru maja najwiekszy blad rzutowania i dystorsje obiektywu.
            mx = img_w * (1.0 - self.center_frac) / 2.0
            my = img_h * (1.0 - self.center_frac) / 2.0
            if not (mx <= u <= img_w - mx and my <= v <= img_h - my):
                self._rejects['brzeg'] += 1
                continue

            proj = _project_pixel(u, v, alt, roll, pitch, yaw,
                                  self.mount_pitch, self.cam_yaw_offset,
                                  self.focal_px, self.cx, self.cy,
                                  self.gimbal_stabilized)
            if proj is None:
                self._rejects['promien'] += 1
                continue
            d_north, d_east, slant = proj

            # Obiekt o znanym rozmiarze ma na danej odleglosci przewidywalna
            # wielkosc w pikselach. Prog jest PER KLASA - namiot 3 m i czlowiek
            # 0.6 m maja zupelnie inne oczekiwane boxy, wiec jeden wspolny prog
            # albo wycinalby ludzi, albo przepuszczal smieci zamiast namiotow.
            # Liczymy z odleglosci SKOSNEJ, nie z wysokosci: przy pochylonej
            # kamerze punkt przy krawedzi kadru jest wyraznie dalej.
            expected_px = size_m * self.focal_px / slant
            got_px = max(w, h)
            if not (self.size_tol_lo * expected_px <= got_px
                    <= self.size_tol_hi * expected_px):
                self._rejects['rozmiar'] += 1
                continue

            self._add_observation(det.class_id, lat, lon, d_north, d_east,
                                  det.confidence, now, 'auto')

            det_lat, det_lon = meters_to_gps(lat, lon, d_north, d_east)
            self._csv.writerow([f"{now:.3f}", det.class_id,
                                f"{u:.1f}", f"{v:.1f}", f"{w:.1f}", f"{h:.1f}",
                                f"{det.confidence:.3f}", det.track_id,
                                f"{lat:.7f}", f"{lon:.7f}", f"{alt:.2f}",
                                f"{roll:.4f}", f"{pitch:.4f}", f"{yaw:.4f}",
                                f"{det_lat:.7f}", f"{det_lon:.7f}",
                                self._last_cid])

    def _mark_cb(self, msg: OperatorMark):
        """Klikniecie operatora w zamrozona klatke.

        Rzutujemy DOKLADNIE ten piksel telemetria z chwili, w ktorej powstala
        klatka - stad header.stamp w wiadomosci. Dlatego opoznienie podgladu
        przestaje byc zrodlem bledu: operator klika w obraz, ktory widzi,
        a nie w to, co dron widzi teraz.
        """
        now = time.time()
        tel = self._telem_at(self._frame_time(msg.header, now))
        if tel is None:
            self.get_logger().warn("znacznik operatora bez telemetrii — pomijam")
            return
        _, lat, lon, alt, roll, pitch, yaw = tel

        proj = _project_pixel(msg.u, msg.v, alt, roll, pitch, yaw,
                              self.mount_pitch, self.cam_yaw_offset,
                              self.focal_px, self.cx, self.cy,
                              self.gimbal_stabilized)
        if proj is None:
            self.get_logger().warn("znacznik operatora: promien nie trafia w ziemie")
            return
        d_north, d_east, _ = proj

        # conf=1.0: klikniecie jest pewne z definicji, a wagi w klastrze sa
        # pewnoscia, wiec znacznik reczny ciagnie srodek klastra najmocniej.
        cand = self._add_observation(int(msg.class_id), lat, lon,
                                     d_north, d_east, 1.0, now, 'operator')
        # Logujemy DWIE rzeczy, bo to co innego: surowy punkt z klikniecia
        # (tym weryfikujesz celnosc piksela) i srodek klastra, do ktorego klik
        # wpadl (to jest adres, ktory pojdzie do misji). Przy trafieniu w ten
        # sam cel co automat beda prawie identyczne — i o to chodzi.
        mlat, mlon = meters_to_gps(lat, lon, d_north, d_east)
        clat, clon = self._latlon(cand)
        self.get_logger().info(
            f"ZNACZNIK OPERATORA [{CLASS_NAMES.get(msg.class_id, msg.class_id)}] "
            f"piksel ({msg.u:.0f},{msg.v:.0f}) z {alt:.1f} m -> "
            f"{d_east:+.1f} m wschod, {d_north:+.1f} m polnoc od drona "
            f"= {mlat:.6f} {mlon:.6f}")
        self.get_logger().info(
            f"   wpadl do klastra #{cand.id} ({cand.n_obs} obs) -> "
            f"{clat:.6f} {clon:.6f}")

    def _add_observation(self, class_id, lat, lon, d_north, d_east,
                         conf, t, source):
        """Wspolna sciezka dla detekcji automatu i klikniec operatora."""
        det_lat, det_lon = meters_to_gps(lat, lon, d_north, d_east)
        obj_n, obj_e = self._to_local(det_lat, det_lon)
        drone_ne = self._to_local(lat, lon)
        cand = self._assign(class_id, obj_n, obj_e, conf, t, drone_ne, source)
        self._last_cid = cand.id
        return cand

    def _to_local(self, lat, lon):
        """GPS -> metry (polnoc, wschod) wzgledem pierwszego fixa."""
        lat0, lon0 = self.origin
        return ((lat - lat0) * 111_320.0,
                (lon - lon0) * 111_320.0 * math.cos(math.radians(lat0)))

    def _assign(self, class_id, north, east, conf, t, drone_ne, source='auto'):
        """Dopisz do najblizszego klastra TEJ SAMEJ KLASY albo zaloz nowy.

        Dopasowanie w obrebie klasy, bo czlowiek stojacy 5 m od namiotu to nie
        jest ten sam obiekt - a przy wspolnym klastrowaniu wpadlby do jednego
        worka i usredniloby oba cele w punkt lezacy miedzy nimi.
        """
        best, best_d = None, self.cluster_radius
        for c in self.candidates:
            if c.class_id != class_id:
                continue
            d = math.hypot(c.north - north, c.east - east)
            if d < best_d:
                best, best_d = c, d
        if best is not None:
            best.add(north, east, conf, t, drone_ne, self.pass_gap, source)
            return best

        cand = Candidate(self._next_cid, class_id, north, east, conf, t,
                         drone_ne, source)
        self._next_cid += 1
        self.candidates.append(cand)
        lat, lon = meters_to_gps(self.origin[0], self.origin[1], north, east)
        self.get_logger().info(
            f"NOWY KANDYDAT #{cand.id} [{CLASS_NAMES.get(class_id, class_id)}]"
            f"  {lat:.6f} {lon:.6f}  conf={conf:.2f}  zrodlo={source}")
        self._save_snapshot(cand)
        return cand

    def _best_for(self, class_id):
        """Zwyciezca danej klasy.

        PRIORYTET OPERATORA: klaster oznaczony recznie wygrywa niezaleznie od
        liczby obserwacji. Automat na 80 m ma na czlowieku 11 pikseli, wiec
        czlowiek przy ekranie widzi wiecej niz model - i to jego wskazanie ma
        byc adresem zrzutu. Dopiero gdy nikt nie kliknal, decyduje automat
        i jego prog min_obs.
        """
        mine = [c for c in self.candidates if c.class_id == class_id]
        if not mine:
            return None, []
        ranked = sorted(mine, key=lambda c: c.score, reverse=True)
        oper = [c for c in ranked if c.source == 'operator']
        if oper:
            return max(oper, key=lambda c: c.last_t), ranked
        need = self.min_obs.get(class_id, 10)
        return next((c for c in ranked if c.n_obs >= need), None), ranked

    def _save_snapshot(self, cand):
        if not self.snapshots or self._last_jpeg is None:
            return
        name = CLASS_NAMES.get(cand.class_id, cand.class_id)
        path = os.path.join(self.flight_dir, f"kandydat_{cand.id:02d}_{name}.jpg")
        try:
            with open(path, 'wb') as f:
                f.write(self._last_jpeg)
        except OSError as e:
            self.get_logger().warn(f"nie zapisano {path}: {e}")

    # ────────────────────── Raport i zapis ──────────────────────

    def _report(self):
        if self.origin is None:
            self.get_logger().warn(
                "brak telemetrii z fixem GPS — detekcje odrzucane "
                f"(dostalem {self._n_det} detekcji)")
            return

        by_class = {cid: self._best_for(cid) for cid in sorted(self.size_m)}
        drops = ', '.join(f"{k}={v}" for k, v in self._rejects.items() if v)

        if not self.candidates:
            self.get_logger().info(
                f"brak kandydatow | detekcji: {self._n_det}"
                + (f" | odrzucone: {drops}" if drops else ""))
            return

        lines = []
        for class_id, (best, ranked) in by_class.items():
            if not ranked:
                continue
            name = CLASS_NAMES.get(class_id, class_id).upper()
            lines.append(f"--- KANDYDACI ({name}) ---------------------------")
            for c in ranked[:5]:
                lat, lon = self._latlon(c)
                mark = "   <-- ZAPISANY" if best is not None and c.id == best.id else ""
                src = "  [OPERATOR]" if c.source == 'operator' else ""
                lines.append(
                    f" #{c.id}  {lat:.6f}  {lon:.6f}   obs={c.n_obs:4d}  "
                    f"conf={c.mean_conf:.2f}  przeloty={c.n_passes}"
                    f"  rozrzut={c.point_spread():.1f}m{src}{mark}")
        if drops:
            lines.append(f" (detekcji: {self._n_det}, odrzucone: {drops})")
        self.get_logger().info("\n".join(lines))

        for class_id, (best, _) in by_class.items():
            if best is None:
                continue
            if self._best_id.get(class_id) not in (None, best.id):
                self.get_logger().warn(
                    f"ZMIANA LIDERA [{CLASS_NAMES.get(class_id, class_id)}] "
                    f"-> #{best.id}")
            self._best_id[class_id] = best.id

        self._write_json(by_class)

    def _latlon(self, c):
        return meters_to_gps(self.origin[0], self.origin[1], c.north, c.east)

    def _as_dict(self, c):
        lat, lon = self._latlon(c)
        return {
            'id': c.id,
            'class': CLASS_NAMES.get(c.class_id, c.class_id),
            'class_id': c.class_id,
            'source': c.source,
            'lat': round(lat, 7),
            'lon': round(lon, 7),
            'n_obs': c.n_obs,
            'mean_conf': round(c.mean_conf, 3),
            'n_passes': c.n_passes,
            'score': round(c.score, 2),
            'point_spread_m': round(c.point_spread(), 2),
            'drone_spread_m': round(c.drone_spread(), 1),
            'first_seen': round(c.first_t, 1),
            'last_seen': round(c.last_t, 1),
        }

    def _write_json(self, by_class):
        created = time.strftime('%Y-%m-%dT%H:%M:%S')
        data = {
            'created': created,
            'min_obs': {CLASS_KEYS.get(c, str(c)): n
                        for c, n in self.min_obs.items()},
        }
        for class_id, (best, ranked) in by_class.items():
            data[CLASS_KEYS.get(class_id, str(class_id))] = {
                'best': self._as_dict(best) if best is not None else None,
                'candidates': [self._as_dict(c) for c in ranked],
            }
        blob = json.dumps(data, indent=2, ensure_ascii=False)
        _atomic_write(self.targets_json, blob)
        _atomic_write(self.flight_json, blob)

        # Stary format, tylko namiot — zeby to, co juz lata w polu, nie padlo.
        tent_best, tent_ranked = by_class.get(0, (None, []))
        legacy = {
            'class': 'namiot',
            'created': created,
            'min_obs': self.min_obs.get(0),
            'best': self._as_dict(tent_best) if tent_best is not None else None,
            'candidates': [self._as_dict(c) for c in tent_ranked],
        }
        _atomic_write(self.target_json,
                      json.dumps(legacy, indent=2, ensure_ascii=False))

    def close(self):
        """Domkniecie plikow — wywolywane takze przy Ctrl+C."""
        try:
            if self.candidates and self.origin is not None:
                by_class = {cid: self._best_for(cid) for cid in sorted(self.size_m)}
                self._write_json(by_class)
                for class_id, (best, _) in by_class.items():
                    name = CLASS_NAMES.get(class_id, class_id)
                    if best is not None:
                        lat, lon = self._latlon(best)
                        self.get_logger().info(
                            f"ZAPISANO [{name}] #{best.id}  {lat:.6f} {lon:.6f}  "
                            f"({best.n_obs} obserwacji, zrodlo={best.source}) "
                            f"-> {self.targets_json}")
                    else:
                        self.get_logger().warn(
                            f"[{name}] zaden kandydat nie zebral "
                            f"{self.min_obs.get(class_id)} obserwacji — "
                            f"w {self.targets_json} jest sam ranking, bez 'best'")
        finally:
            if not self._csv_f.closed:
                self._csv_f.close()


def _lerp_angle(a, b, f):
    """Interpolacja katow z owinieciem przez +-pi (yaw skacze na polnocy)."""
    d = (b - a + math.pi) % (2 * math.pi) - math.pi
    return a + d * f


def _atomic_write(path, text):
    """Zapis przez plik tymczasowy + rename — wylaczenie zasilania w trakcie
    nie zostawi obcietego JSON-a."""
    tmp = path + '.tmp'
    with open(tmp, 'w') as f:
        f.write(text)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)


def main(args=None):
    rclpy.init(args=args)
    node = SuasGeolocator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
