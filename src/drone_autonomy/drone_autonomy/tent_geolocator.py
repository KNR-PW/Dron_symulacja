#!/usr/bin/env python3
"""
tent_geolocator — zapis wspolrzednych GPS namiotu podczas przelotu ortofoto.

Node PASYWNY: nie steruje dronem, tylko slucha detekcji i telemetrii. Jedyne co
wysyla, to kat gimbala (-90 = prosto w dol), zeby kadr byl na pewno w nadirze.

Zasada dzialania: kazda detekcja jest rzutowana z piksela na punkt na ziemi
(wysokosc z telemetrii + ogniskowa kamery), a punkty sa grupowane przestrzennie.
Prawdziwy namiot widziany z wielu pozycji drona daje 100+ trafien zbieznych
w jednym miejscu; false positive daje kilka rozrzuconych. To mocniejszy filtr
niz cokolwiek liczonego na pojedynczej klatce.

Zalozenia (patrz docs/tent_geolocator.md):
  * gimbal trzyma -90 st. niezaleznie od pochylen ramy -> kamera zawsze w pionie,
    wiec rzutowanie nie potrzebuje korekty roll/pitch,
  * kadr zorientowany "gora = przod drona" (cam_yaw_offset_deg = 0),
  * detektor chodzi z classes:=0, czyli publikuje TYLKO namioty.

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
from std_msgs.msg import Float32

from drone_interfaces.msg import Telemetry, TentDetection


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


class Candidate:
    """Jeden klaster detekcji = jeden domniemany namiot."""

    def __init__(self, cid, north, east, conf, t, drone_ne):
        self.id = cid
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

    def add(self, north, east, conf, t, drone_ne, pass_gap):
        if t - self.last_t > pass_gap:
            self.n_passes += 1
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


class TentGeolocator(Node):

    def __init__(self):
        super().__init__('tent_geolocator')

        # ── 1. PARAMETRY ────────────────────────────────────────────
        self.declare_parameter('save_dir', os.path.expanduser('~/suas_targets'))
        # Gimbal: node sam pilnuje nadiru, zeby kat byl pewny takze po restarcie
        # czegokolwiek. Przy lock_nadir=true suas_gimbal_controller NIE MOZE chodzic.
        self.declare_parameter('lock_nadir', True)
        self.declare_parameter('mount_pitch_deg', -90.0)
        # Obrot kadru wzgledem osi drona: 0 = gora kadru to przod drona.
        self.declare_parameter('cam_yaw_offset_deg', 0.0)

        # Geometria kamery — preview OAK 1024x1024, VFOV 64.4 st.
        self.declare_parameter('focal_px', 813.0)
        self.declare_parameter('img_w', 1024)
        self.declare_parameter('img_h', 1024)

        # TentDetection nie ma naglowka z czasem klatki, wiec cofamy sie
        # w buforze telemetrii o staly szacunek opoznienia detekcji.
        self.declare_parameter('det_latency', 0.20)

        # Bramki na pojedynczej detekcji
        self.declare_parameter('min_conf', 0.0)      # 0 = ufaj progowi detektora
        self.declare_parameter('min_alt', 10.0)      # nie zbieraj smieci przy ziemi
        self.declare_parameter('max_tilt', 10.0)     # st.; przy zakretach odpuszczamy
        self.declare_parameter('center_frac', 0.8)   # tylko srodkowe 80% kadru
        self.declare_parameter('tent_size_m', 3.0)   # do bramki na rozmiar boxa
        self.declare_parameter('size_tol_lo', 0.3)
        self.declare_parameter('size_tol_hi', 3.0)
        self.declare_parameter('telemetry_max_age', 0.5)

        # Klastrowanie
        self.declare_parameter('cluster_radius', 10.0)   # m; z grubsza 0.12 * H
        self.declare_parameter('min_obs', 10)            # tyle trafien = kandydat na zwyciezce
        self.declare_parameter('pass_gap', 3.0)          # przerwa cos>tyle s = kolejny przelot

        self.declare_parameter('snapshots', True)
        self.declare_parameter('report_period', 5.0)

        p = self.get_parameter
        self.lock_nadir = p('lock_nadir').value
        self.mount_pitch = p('mount_pitch_deg').value
        self.cam_yaw_offset = math.radians(p('cam_yaw_offset_deg').value)
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
        self.tent_size_m = p('tent_size_m').value
        self.size_tol_lo = p('size_tol_lo').value
        self.size_tol_hi = p('size_tol_hi').value
        self.telemetry_max_age = p('telemetry_max_age').value
        self.cluster_radius = p('cluster_radius').value
        self.min_obs = p('min_obs').value
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
        self._best_id = None
        self._last_jpeg = None
        self._n_det = 0
        self._rejects = {'telemetria': 0, 'wysokosc': 0, 'przechyl': 0,
                         'brzeg': 0, 'rozmiar': 0, 'pewnosc': 0}

        # ── 3. PLIKI ────────────────────────────────────────────────
        base = os.path.expanduser(p('save_dir').value)
        self.flight_dir = os.path.join(base, time.strftime('%Y-%m-%d_%H-%M'))
        os.makedirs(self.flight_dir, exist_ok=True)
        self.target_json = os.path.join(base, 'tent_target.json')
        self.flight_json = os.path.join(self.flight_dir, 'target.json')
        self._csv_f = open(os.path.join(self.flight_dir, 'observations.csv'),
                           'w', newline='')
        self._csv = csv.writer(self._csv_f)
        self._csv.writerow(['t', 'u', 'v', 'w', 'h', 'conf', 'track_id',
                            'lat', 'lon', 'alt', 'roll', 'pitch', 'yaw',
                            'det_lat', 'det_lon', 'cluster_id'])

        # ── 4. PUB / SUB ────────────────────────────────────────────
        self.create_subscription(TentDetection, '/tent_detections', self._det_cb, 10)
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

        # ── 5. TIMERY ───────────────────────────────────────────────
        if self.lock_nadir:
            self._set_nadir()
            self.create_timer(2.0, self._set_nadir)
        self.create_timer(report_period, self._report)

        self.get_logger().info(
            f"tent_geolocator gotowy | zapis: {self.flight_dir} | "
            f"nadir={'TAK' if self.lock_nadir else 'NIE'} "
            f"({self.mount_pitch:+.0f} st.) | promien klastra "
            f"{self.cluster_radius:.0f} m | min_obs={self.min_obs}")
        if self.lock_nadir:
            self.get_logger().warn(
                "lock_nadir=true — NIE uruchamiaj rownolegle suas_gimbal_controller")

    # ────────────────────── Gimbal ──────────────────────

    def _set_nadir(self):
        self._gimbal_pub.publish(Float32(data=float(self.mount_pitch)))

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

    def _det_cb(self, msg: TentDetection):
        if not msg.detected:
            return
        self._n_det += 1
        now = time.time()

        if msg.confidence < self.min_conf:
            self._rejects['pewnosc'] += 1
            return

        tel = self._telem_at(now - self.det_latency)
        if tel is None:
            self._rejects['telemetria'] += 1
            return
        _, lat, lon, alt, roll, pitch, yaw = tel

        if alt < self.min_alt:
            self._rejects['wysokosc'] += 1
            return
        # Rzutowanie zaklada kamere w pionie. Na prostych galsach przechyl jest
        # bliski zeru; odrzucamy klatki z zakretow, ktorych i tak nie potrzebujemy.
        if abs(roll) > self.max_tilt or abs(pitch) > self.max_tilt:
            self._rejects['przechyl'] += 1
            return

        x, y, w, h = msg.bounding_box
        u = x + w / 2.0
        v = y + h / 2.0

        # Brzegi kadru maja najwiekszy blad rzutowania i dystorsje obiektywu.
        mx = self.img_w * (1.0 - self.center_frac) / 2.0
        my = self.img_h * (1.0 - self.center_frac) / 2.0
        if not (mx <= u <= self.img_w - mx and my <= v <= self.img_h - my):
            self._rejects['brzeg'] += 1
            return

        # Namiot o znanym rozmiarze ma na danej wysokosci przewidywalna wielkosc
        # w pikselach. Wycina i piksele-smieci, i box na pol kadru.
        expected_px = self.tent_size_m * self.focal_px / alt
        got_px = max(w, h)
        if not (self.size_tol_lo * expected_px <= got_px
                <= self.size_tol_hi * expected_px):
            self._rejects['rozmiar'] += 1
            return

        # ── Rzutowanie piksel -> ziemia ──────────────────────────
        # Kamera w nadirze: offset w metrach to offset w pikselach * GSD,
        # gdzie GSD = alt / focal_px. Gora kadru = przod drona.
        right = (u - self.cx) * alt / self.focal_px
        fwd = -(v - self.cy) * alt / self.focal_px

        # Obrot o kurs drona (gimbal ma jedna os, wiec kadr krazy razem z rama).
        hdg = yaw + self.cam_yaw_offset
        d_north = fwd * math.cos(hdg) - right * math.sin(hdg)
        d_east = fwd * math.sin(hdg) + right * math.cos(hdg)

        det_lat, det_lon = meters_to_gps(lat, lon, d_north, d_east)

        # ── Klastrowanie w metrach wzgledem pierwszego fixa ──────
        obj_n, obj_e = self._to_local(det_lat, det_lon)
        drone_ne = self._to_local(lat, lon)
        cand = self._assign(obj_n, obj_e, msg.confidence, now, drone_ne)

        self._csv.writerow([f"{now:.3f}", f"{u:.1f}", f"{v:.1f}",
                            f"{w:.1f}", f"{h:.1f}", f"{msg.confidence:.3f}",
                            msg.track_id, f"{lat:.7f}", f"{lon:.7f}",
                            f"{alt:.2f}", f"{roll:.4f}", f"{pitch:.4f}",
                            f"{yaw:.4f}", f"{det_lat:.7f}", f"{det_lon:.7f}",
                            cand.id])

    def _to_local(self, lat, lon):
        """GPS -> metry (polnoc, wschod) wzgledem pierwszego fixa."""
        lat0, lon0 = self.origin
        return ((lat - lat0) * 111_320.0,
                (lon - lon0) * 111_320.0 * math.cos(math.radians(lat0)))

    def _assign(self, north, east, conf, t, drone_ne):
        """Dopisz do najblizszego klastra w promieniu albo zaloz nowy."""
        best, best_d = None, self.cluster_radius
        for c in self.candidates:
            d = math.hypot(c.north - north, c.east - east)
            if d < best_d:
                best, best_d = c, d
        if best is not None:
            best.add(north, east, conf, t, drone_ne, self.pass_gap)
            return best

        cand = Candidate(self._next_cid, north, east, conf, t, drone_ne)
        self._next_cid += 1
        self.candidates.append(cand)
        lat, lon = meters_to_gps(self.origin[0], self.origin[1], north, east)
        self.get_logger().info(
            f"NOWY KANDYDAT #{cand.id}  {lat:.6f} {lon:.6f}  conf={conf:.2f}")
        self._save_snapshot(cand)
        return cand

    def _save_snapshot(self, cand):
        if not self.snapshots or self._last_jpeg is None:
            return
        path = os.path.join(self.flight_dir, f"kandydat_{cand.id:02d}.jpg")
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
        if not self.candidates:
            drops = ', '.join(f"{k}={v}" for k, v in self._rejects.items() if v)
            self.get_logger().info(
                f"brak kandydatow | detekcji: {self._n_det}"
                + (f" | odrzucone: {drops}" if drops else ""))
            return

        ranked = sorted(self.candidates, key=lambda c: c.score, reverse=True)
        best = next((c for c in ranked if c.n_obs >= self.min_obs), None)

        lines = ["--- KANDYDACI (namiot) -------------------------------"]
        for i, c in enumerate(ranked[:8], 1):
            lat, lon = self._latlon(c)
            mark = "   <-- ZAPISANY" if best is not None and c.id is best.id else ""
            lines.append(
                f" #{c.id}  {lat:.6f}  {lon:.6f}   obs={c.n_obs:4d}  "
                f"conf={c.mean_conf:.2f}  przeloty={c.n_passes}"
                f"  rozrzut={c.point_spread():.1f}m{mark}")
        drops = ', '.join(f"{k}={v}" for k, v in self._rejects.items() if v)
        if drops:
            lines.append(f" (detekcji: {self._n_det}, odrzucone: {drops})")
        self.get_logger().info("\n".join(lines))

        if best is not None and best.id != self._best_id:
            if self._best_id is not None:
                self.get_logger().warn(f"ZMIANA LIDERA -> #{best.id}")
            self._best_id = best.id
        self._write_json(ranked, best)

    def _latlon(self, c):
        return meters_to_gps(self.origin[0], self.origin[1], c.north, c.east)

    def _as_dict(self, c):
        lat, lon = self._latlon(c)
        return {
            'id': c.id,
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

    def _write_json(self, ranked, best):
        data = {
            'class': 'namiot',
            'created': time.strftime('%Y-%m-%dT%H:%M:%S'),
            'min_obs': self.min_obs,
            'best': self._as_dict(best) if best is not None else None,
            'candidates': [self._as_dict(c) for c in ranked],
        }
        blob = json.dumps(data, indent=2, ensure_ascii=False)
        for path in (self.target_json, self.flight_json):
            _atomic_write(path, blob)

    def close(self):
        """Domkniecie plikow — wywolywane takze przy Ctrl+C."""
        try:
            if self.candidates and self.origin is not None:
                ranked = sorted(self.candidates, key=lambda c: c.score, reverse=True)
                best = next((c for c in ranked if c.n_obs >= self.min_obs), None)
                self._write_json(ranked, best)
                if best is not None:
                    lat, lon = self._latlon(best)
                    self.get_logger().info(
                        f"ZAPISANO #{best.id}  {lat:.6f} {lon:.6f}  "
                        f"({best.n_obs} obserwacji) -> {self.target_json}")
                else:
                    self.get_logger().warn(
                        f"zaden kandydat nie zebral {self.min_obs} obserwacji — "
                        f"w {self.target_json} jest sam ranking, bez 'best'")
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
    node = TentGeolocator()
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
