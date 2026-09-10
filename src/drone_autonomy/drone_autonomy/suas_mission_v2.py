#!/usr/bin/env python3
"""
suas_mission_v2 — misja SUAS: adres albo skan. Wersja uproszczona.

Pelny opis i uzasadnienie liczb: docs/misja.md.

ZASADA
    Dla kazdej klasy osobno: "czy mam adres w targets.json?"
        NAMIOT   TAK -> DOWIEZ   NIE -> SZUKAJ skanem
        CZLOWIEK TAK -> DOWIEZ   NIE -> SZUKAJ skanem po namiocie
    Kolejnosc sztywna: najpierw NAMIOT, potem CZLOWIEK.

DWIE SCIEZKI
    B (mam adres)   DOLOT z wyciszonym detektorem -> okno M z N (wp_acquire)
                    -> ZAWIS + ZRZUT  albo  ZRZUT NA WSPOLRZEDNE.
                    Zawsze konczy sie zrzutem.
    A (skan)        STOP -> SPRAWDZ na stojaco -> CELUJ -> PODLOT -> ZAWIS
                    -> ZRZUT. Bez wycentrowania NIE ma zrzutu, bo nie ma
                    adresu, w ktory mozna by trafic w ciemno.

ZACHOWANIA SA ZASZYTE W KODZIE (stale ponizej). Yaml stroi wylacznie liczby —
progi, limity czasu, katy. Wyjatkiem jest auto_takeoff, ktory dotyczy startu,
a nie przebiegu misji.

Wymaga: drone_handler + detektor (/tent_detections, /people_detections).
Geolokator opcjonalny — bez niego nie ma adresow i obie klasy ida skanem.

    ros2 run drone_autonomy suas_mission_v2 --ros-args \\
        --params-file ~/Dron_symulacja/src/drone_bringup/config/misja_v2.yaml
"""

import json
import math
import os
import signal
import time
from collections import deque

import rclpy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool

from drone_autonomy.suas_flight_controller import State, SuasFlightController, clamp
from drone_interfaces.msg import TentDetection

TENT, PERSON = 0, 1
M_LAT = 111_320.0

# ── Zachowania misji ────────────────────────────────────────────────
# Celowo stale, nie parametry: yaml ma stroic liczby, a nie zmieniac przebieg.
#
# CENTRUJ_NAD_ADRESEM — czy majac adres i widzac cel pod dronem centrowac sie
# na nim, czy zrzucac wprost na wspolrzedne. Namiot TAK: bramka 4 z 8
# z ciagloscia track_id, 49 px na 50 m — jak detektor mowi "widze", to widzi.
# Czlowiek NIE: bramka 2 z 8 bez ciaglosci ID (inaczej lezacy nie przeszedlby
# jej nigdy), wiec "widzialem" moze znaczyc dwie klatki cienia, a adres to
# klaster setek obserwacji albo klik operatora — mocniejsza przeslanka.
CENTRUJ_NAD_ADRESEM = {TENT: True, PERSON: False}
# Po ostatnim punkcie trasy wroc na pierwszy.
SKAN_WRACA_NA_PIERWSZY = True
# Po zrzucie na namiot szukaj czlowieka tym samym skanem. W trakcie skanu na
# namiot nikt nie obserwowal klasy CZLOWIEK, wiec teren jest nieogladany.
SZUKAJ_CZLOWIEKA_PO_NAMIOCIE = True


def _m_per_deg_lon(lat):
    return M_LAT * math.cos(math.radians(lat))


def _wrap(a):
    """Kat do [-pi, pi]."""
    return (a + math.pi) % (2 * math.pi) - math.pi


class ScanWatcher:
    """Okno M z N dla jednej klasy, niezalezne od okna kontrolera.

    Skan ma inne (czulsze) progi i musi dac sie zerowac na kazdej komorce bez
    ruszania stanu kontrolera. Zerowanie jest konieczne: okno jest kroczace,
    wiec dwa trafienia z komorki k i dwa z k+1 domknelyby je na celu, ktorego
    nie ma w zadnej.
    """

    def __init__(self, node, topic, m, n):
        self.node = node
        self.topic = topic
        self.m = m
        self.n = n
        self.mute_until = 0.0
        self._win = deque(maxlen=n)
        self._prev_frame = 0.0
        self._track = -1
        self.sub = node.create_subscription(TentDetection, topic, self._cb, 10)

    def _cb(self, msg: TentDetection):
        now = time.time()
        # Detektor sie zaciol / kamera padla — stare okno nie jest aktualne.
        if (self._prev_frame > 0.0
                and now - self._prev_frame > self.node.det_confirm_gap):
            self.reset()
        self._prev_frame = now

        good = bool(msg.detected)
        if good and self.node.require_same_track:
            if msg.track_id < 0:
                good = False        # brak ID = podpis migoczacej falszywki
            elif self._track < 0:
                self._track = msg.track_id
            elif msg.track_id != self._track:
                self._win.clear()
                self._track = msg.track_id
        self._win.append(1 if good else 0)

    @property
    def hits(self):
        return sum(self._win)

    @property
    def window(self):
        return len(self._win)

    @property
    def track_id(self):
        return self._track

    def confirmed(self) -> bool:
        if time.time() < self.mute_until:
            return False
        return self.hits >= self.m

    def reset(self):
        self._win.clear()
        self._track = -1

    def set_gate(self, m, n):
        if (m, n) == (self.m, self.n):
            return
        self.m, self.n = m, n
        self._win = deque(self._win, maxlen=n)   # nowa dlugosc, ta sama tresc

    def mute(self, seconds):
        self.mute_until = time.time() + seconds
        self.reset()


class SuasMission(SuasFlightController):

    def __init__(self):
        super().__init__('suas_mission')

        p = self.declare_parameter

        # ── Pliki ───────────────────────────────────────────────────
        p('targets_json',
          '~/Dron_symulacja/src/drone_bringup/config/targets.json')
        # Trasa skanu: .waypoints z Mission Plannera ("QGC WPL 110"),
        # "lat, lon" w linii, albo linia OFFSETS i pary "wschod, polnoc"
        # w metrach od punktu przejecia lotu.
        p('scan_waypoints',
          '~/Dron_symulacja/src/drone_bringup/config/misja_skan.waypoints')

        # ── Start ───────────────────────────────────────────────────
        p('takeover_timeout', 1200.0)
        p('auto_takeoff', False)     # TRYB TESTOWY: sam uzbraja i startuje

        # ── Sciezka B: dolot na adres ───────────────────────────────
        p('wp_acquire', 5.0)         # okno M z N nad waypointem

        # ── Zawis (wspolny ogon obu sciezek) ────────────────────────
        p('approach_timeout', 90.0)  # PODLOT + centrowanie (sciezka A)
        # Limit samego zawisu (sciezka B — dron stoi juz nad celem). Osobny,
        # bo w sciezce A w limicie miesci sie jeszcze przelot kilkudziesieciu
        # metrow. Gdy zawis nie domknie sie w tym czasie, regulator oscyluje
        # wokol center_tol_m i czekanie dluzej nic nie da.
        p('hover_timeout', 10.0)
        p('center_lost_timeout', 15.0)   # ciagly SEARCH tak dlugo = cel znikl
        p('center_tol_m', 3.0)           # kiedy cel jest "pod dronem"
        p('hover_hold_time', 2.0)

        # ── Sciezka A: CELUJ ────────────────────────────────────────
        # Tolerancja w PIKSELACH, nie w metrach: przy podniesionym gimbalu
        # gorne wiersze kadru patrza ponad horyzont, rzutowanie zwraca None,
        # a uchyb metrowy odczytalby sie jako "wycentrowany".
        p('aim_tol_px', 0.06)
        p('aim_hold', 1.0)
        p('aim_timeout', 15.0)

        # ── Skan ────────────────────────────────────────────────────
        p('scan_yaw_step', 60.0)
        p('scan_arc_deg', [180.0, 360.0])     # luk per punkt trasy
        # Bezwzgledny kurs, na ktorym luk ma byc wysrodkowany. Bez tego luk
        # zaczynalby sie od kursu przylotu — a na WP1 wracamy tylem do pola.
        # Przy luku 360 pomijany.
        p('scan_heading_deg', [0.0, 0.0])
        p('scan_pitch_nadir', -90.0)          # raz na punkt, bez obracania
        p('scan_pitches', [-65.0, -40.0])     # na kazdej pozycji yaw
        p('scan_dwell', 1.5)
        p('scan_yaw_rate', 0.4)               # rad/s, tylko obrot skanu
        p('scan_settle', 0.8)
        p('scan_confirm_frames', 3)           # bramka skanu: czulsza
        p('scan_window_frames', 6)
        # Bramka w PRZELOCIE ostrzejsza niz skanowa: obraz jest rozmyty ruchem,
        # a kazde wyzwolenie kosztuje hamowanie, sprawdzenie i cisze.
        p('transit_confirm_frames', 4)
        p('transit_window_frames', 8)
        p('scan_timeout', 600.0)
        p('person_scan_timeout', 300.0)
        p('pitch_transit', -55.0)             # kat w przelocie miedzy punktami
        p('arrive_tol', 3.0)

        # ── Nizsze bramki dla czlowieka ─────────────────────────────
        # Namiot ma na 50 m 49 px i widac go w kazdej klatce. Czlowiek lezacy
        # 30x22 px, stojacy 8 px — te same progi bywaja nieprzechodzalne.
        p('person_scan_confirm_frames', 2)
        p('person_scan_window_frames', 8)
        p('person_transit_confirm_frames', 3)
        p('person_transit_window_frames', 8)
        p('person_det_confirm_frames', 2)
        p('person_det_window_frames', 8)
        # Najwazniejsze dla czlowieka. require_same_track odrzuca track_id < 0
        # ORAZ czysci okno przy zmianie ID. Przy celu widzianym w ~28% klatek
        # tracker gubi sciezke i nadaje nowe ID przy kazdym powrocie, wiec okno
        # nigdy sie nie domyka. Zdejmuje to zabezpieczenie przed falszywka —
        # chroni juz tylko reszta lancucha (SPRAWDZ, CELUJ, centrowanie).
        p('person_require_same_track', False)
        p('person_lost_timeout', 5.0)    # przerwy w detekcji sadaja 2 s i wiecej

        # ── Cisza po falszywce ──────────────────────────────────────
        p('det_cooldown', 20.0)          # falszywka z PRZELOTU
        # Falszywka ze SKANU: cisza musi byc krotka, bo skan trwa ~50 s i 20 s
        # oslepia polowe komorek. Dluga niepotrzebna — po nieudanym sprawdzeniu
        # wznawiamy od NASTEPNEJ komorki, wiec ta sama falszywka nie wraca.
        p('scan_det_cooldown', 5.0)
        p('reconfirm_timeout', 3.0)
        p('brake_settle_time', 3.0)

        g = self.get_parameter
        self.targets_json = os.path.expanduser(str(g('targets_json').value))
        self.scan_waypoints = os.path.expanduser(str(g('scan_waypoints').value))
        self.takeover_timeout = g('takeover_timeout').value
        self.auto_takeoff = g('auto_takeoff').value
        self.wp_acquire = g('wp_acquire').value
        self.approach_timeout = g('approach_timeout').value
        self.hover_timeout = g('hover_timeout').value
        self.center_lost_timeout = g('center_lost_timeout').value
        self.center_tol_m = g('center_tol_m').value
        self.hover_hold_time = g('hover_hold_time').value
        self.aim_tol_px = g('aim_tol_px').value
        self.aim_hold = g('aim_hold').value
        self.aim_timeout = g('aim_timeout').value
        self.scan_yaw_step = g('scan_yaw_step').value
        self.scan_arc_deg = list(g('scan_arc_deg').value)
        self.scan_heading_deg = list(g('scan_heading_deg').value)
        self.scan_pitch_nadir = g('scan_pitch_nadir').value
        self.scan_pitches = list(g('scan_pitches').value)
        self.scan_dwell = g('scan_dwell').value
        self.scan_yaw_rate = g('scan_yaw_rate').value
        self.scan_settle = g('scan_settle').value
        self.scan_m = g('scan_confirm_frames').value
        self.scan_n = g('scan_window_frames').value
        self.transit_m = g('transit_confirm_frames').value
        self.transit_n = g('transit_window_frames').value
        self.scan_timeout = g('scan_timeout').value
        self.person_scan_timeout = g('person_scan_timeout').value
        self.pitch_transit = g('pitch_transit').value
        self.arrive_tol = g('arrive_tol').value
        self.p_scan_m = g('person_scan_confirm_frames').value
        self.p_scan_n = g('person_scan_window_frames').value
        self.p_transit_m = g('person_transit_confirm_frames').value
        self.p_transit_n = g('person_transit_window_frames').value
        self.p_det_m = g('person_det_confirm_frames').value
        self.p_det_n = g('person_det_window_frames').value
        self.p_same_track = g('person_require_same_track').value
        self.p_lost_timeout = g('person_lost_timeout').value
        self.det_cooldown = g('det_cooldown').value
        self.scan_det_cooldown = g('scan_det_cooldown').value
        self.reconfirm_timeout = g('reconfirm_timeout').value
        self.brake_settle_time = g('brake_settle_time').value
        # Bramka kontrolera jest mutowana per klasa (_apply_class), wiec
        # wartosci wyjsciowe trzeba zapamietac.
        self.tent_det_m = self.det_confirm_frames
        self.tent_det_n = self.det_window_frames
        self.tent_same_track = self.require_same_track
        self.tent_lost_timeout = self.lost_timeout

        # "Wycentrowany" nie moze byc ciasniejszy niz martwa strefa regulatora:
        # ponizej hover_deadzone_m HOVER przestaje korygowac i warunek nigdy
        # by sie nie domknal.
        if self.center_tol_m < self.hover_deadzone_m:
            self.get_logger().warn(
                f"center_tol_m({self.center_tol_m}) < hover_deadzone_m"
                f"({self.hover_deadzone_m}) — podnosze")
            self.center_tol_m = self.hover_deadzone_m

        for opis, m, n in (('person_scan', self.p_scan_m, self.p_scan_n),
                           ('person_transit', self.p_transit_m, self.p_transit_n),
                           ('person_det', self.p_det_m, self.p_det_n)):
            if m > n:
                self.get_logger().warn(
                    f"{opis}: M({m}) > N({n}) — warunek nie do spelnienia")
        if self.scan_m > self.scan_n:
            self.get_logger().warn(
                f"scan_confirm_frames({self.scan_m}) > scan_window_frames"
                f"({self.scan_n}) — przycinam")
            self.scan_m = self.scan_n

        self.klasy = {
            TENT: ('NAMIOT', 'tent', '/tent_detections'),
            PERSON: ('CZLOWIEK', 'people', '/people_detections'),
        }

        self._abort = False
        self.home = None
        self._scan_hit = False
        self.watcher = None
        self._deadline = 0.0

        # Po przejeciu lotu gimbal nalezy do misji — geolokator ma przestac
        # trzymac pion, inaczej dwoch pisze na jeden silownik.
        self._nadir_pub = self.create_publisher(Bool, '/geolocator/lock_nadir', 10)

        self.get_logger().info(
            f"suas_mission: alt={self.target_alt} m | adresy z {self.targets_json} "
            f"| trasa skanu {self.scan_waypoints}")
        # Trase sprawdzamy TERAZ, a nie gdy okaze sie potrzebna — inaczej brak
        # pliku wychodzi po starcie, dolocie i zejsciu, czyli po dwoch minutach.
        znaleziona = self._resolve_asset(self.scan_waypoints)
        if os.path.isfile(znaleziona):
            self.get_logger().info(f"trasa skanu: {znaleziona}")
        else:
            self.get_logger().error(
                "BRAK PLIKU TRASY SKANU — bez adresu z geolokatora misja nie "
                "bedzie miala gdzie szukac")

        self.get_logger().info(
            "centrowanie nad celem z adresu: "
            + ", ".join(f"{self.klasy[c][0]}="
                        + ("TAK" if CENTRUJ_NAD_ADRESEM[c] else
                           "NIE, zrzut wprost na wspolrzedne")
                        for c in (TENT, PERSON))
            + " | misja nie pyta o potwierdzenie")
        self.get_logger().info(
            f"skan: krok {self.scan_yaw_step:.0f} st., luki {self.scan_arc_deg}, "
            f"nadir {self.scan_pitch_nadir:.0f} raz + {self.scan_pitches} na pozycje, "
            f"dwell {self.scan_dwell:.1f}s")
        self.get_logger().info(
            f"bramki M z N — NAMIOT: skan {self.scan_m}/{self.scan_n}, "
            f"przelot {self.transit_m}/{self.transit_n}, "
            f"kontroler {self.tent_det_m}/{self.tent_det_n} | "
            f"CZLOWIEK: skan {self.p_scan_m}/{self.p_scan_n}, "
            f"przelot {self.p_transit_m}/{self.p_transit_n}, "
            f"kontroler {self.p_det_m}/{self.p_det_n}")

    # ═══════════════════════════════════════════════════════════
    #  Pomocnicze
    # ═══════════════════════════════════════════════════════════

    def _install_signals(self):
        """SIGHUP rownie wazny co SIGINT: dostajemy go przy zerwanym terminalu
        (SSH, Tailscale). Bez obslugi proces ginie bez RTL, a dron zostaje
        w GUIDED. Nie zwalnia to z uruchamiania pod tmux — tam SIGHUP w ogole
        nie dolatuje."""
        def handler(signum, _frame):
            if self._abort:
                raise KeyboardInterrupt
            self._abort = True
            self._alarm = True
            powod = {signal.SIGINT: "Ctrl+C", signal.SIGTERM: "SIGTERM",
                     signal.SIGHUP: "SIGHUP (zerwany terminal)"}.get(
                         signum, f"sygnal {signum}")
            self.get_logger().warn(
                f"{powod} — przerywam i wracam (kolejny sygnal = twarde wyjscie)")
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            signal.signal(sig, handler)

    def _spin(self, seconds, stop_on_abort=True):
        end = time.time() + seconds
        while rclpy.ok() and time.time() < end:
            if stop_on_abort and self._abort:
                return
            rclpy.spin_once(self, timeout_sec=0.05)

    def _dist_to(self, lat, lon):
        return math.hypot((lat - self.global_lat) * M_LAT,
                          (lon - self.global_lon) * _m_per_deg_lon(lat))

    def _offset_gps(self, lat, lon, d_north, d_east):
        return lat + d_north / M_LAT, lon + d_east / _m_per_deg_lon(lat)

    def _alt_hold_vz(self):
        """Skladowa pionowa przy sterowaniu recznym (skan, CELUJ) — petla
        kontrolera jest wtedy wylaczona i nikt nie trzyma wysokosci."""
        return clamp(-self.kp_alt * (self.target_alt - self.altitude),
                     -self.max_vz, self.max_vz)

    def _vel_on(self):
        """Sterowanie predkosciowe BEZ petli wizualnej — skan i CELUJ wysylaja
        wektory same i petla tylko by sie z nimi bila."""
        if self.velocity_mode_active:
            return
        r = self.toggle_control()
        if not r.result:
            self.toggle_control()
        self.velocity_mode_active = True

    def _vel_off(self):
        if not self.velocity_mode_active:
            return
        try:
            self.send_vectors(0.0, 0.0, 0.0, 0.0)
            r = self.toggle_control()
            if r.result:
                self.toggle_control()
        except Exception:
            pass
        self.velocity_mode_active = False

    def _resolve_asset(self, path):
        """Znajdz plik repo niezaleznie od tego, gdzie stoi workspace: Jetson
        (~/Dron_symulacja), kontener (/root/ros_ws) albo katalog share. Docker
        montuje tylko src/, wiec '~' znaczy co innego po kazdej stronie."""
        if not path:
            return ''
        wprost = os.path.expanduser(path)
        if os.path.isfile(wprost):
            return wprost
        nazwa = os.path.basename(path)
        alt = ['/root/ros_ws/src/drone_bringup/config',
               os.path.expanduser('~/Dron_symulacja/src/drone_bringup/config')]
        try:
            from ament_index_python.packages import get_package_share_directory
            alt.append(os.path.join(
                get_package_share_directory('drone_bringup'), 'config'))
        except Exception:
            pass
        for katalog in alt:
            kandydat = os.path.join(katalog, nazwa)
            if os.path.isfile(kandydat):
                self.get_logger().warn(
                    f"{wprost} nie istnieje — biore {kandydat}")
                return kandydat
        self.get_logger().error(
            f"nie znalazlem '{nazwa}'. Sprawdzilem:\n  "
            + "\n  ".join([wprost] + [os.path.join(k, nazwa) for k in alt]))
        return wprost

    def _gimbal(self, deg, force=False):
        """force=True omija filtr 'kat sie nie zmienil' — potrzebne na starcie,
        bo pitch_deg jest zainicjowany na pitch_search i serwo nigdy nie
        dostaloby pierwszej komendy."""
        if force:
            self.pitch_deg = clamp(deg, self.pitch_min, self.pitch_max)
            self.set_gimbal_pitch(self.pitch_deg)
        else:
            self._set_gimbal(deg)

    # ═══════════════════════════════════════════════════════════
    #  Przejecie lotu, wysokosc, adresy
    # ═══════════════════════════════════════════════════════════

    def wait_for_guided(self) -> bool:
        """Nic tu nie wysylamy: dopoki jest AUTO, ArduPilot i tak ignoruje
        setpointy predkosci."""
        self.get_logger().info(
            "=== FAZA_ORTO === czekam na AUTO -> GUIDED (nic nie wysylam)")
        end = time.time() + self.takeover_timeout
        last = None
        while rclpy.ok() and not self._abort:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.flight_mode != last:
                last = self.flight_mode
                self.get_logger().info(f"tryb lotu: {last}")
            if self.flight_mode == 'GUIDED':
                self.get_logger().info("=== PRZEJECIE === GUIDED, przejmuje lot")
                return True
            if time.time() > end:
                self.get_logger().error(
                    f"TIMEOUT {self.takeover_timeout:.0f}s bez GUIDED")
                return False
        return False

    def descend(self, alt, tol=2.0, timeout=60.0) -> bool:
        """goto_global konczy sie po odleglosci POZIOMEJ < 2 m, a tu podajemy
        te sama lat/lon — bez wlasnego czekania zejscie trwaloby 0 s."""
        self.get_logger().info(
            f"zejscie na {alt:.0f} m (jestem na {self.altitude:.0f} m)")
        self.target_alt = alt
        self.send_goto_global(self.global_lat, self.global_lon, alt)
        end = time.time() + timeout
        while rclpy.ok() and not self._abort and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if abs(self.altitude - alt) <= tol:
                self.get_logger().info(f"na {self.altitude:.1f} m")
                return True
        self.get_logger().warn(
            f"zejscie: po {timeout:.0f}s jestem na {self.altitude:.1f} m — lece dalej")
        return False

    def read_targets(self):
        """targets.json -> {class_id: (lat, lon, zrodlo, n_obs)}.

        Bierzemy tylko 'best' kazdej klasy — adres uznajemy za wiarygodny.
        Filtr min_obs zostaje, bo geolokator wypisuje takze klastry z 3-4
        obserwacji (krzaki). Wskazanie operatora przechodzi bez progu.
        """
        try:
            with open(self.targets_json) as fh:
                data = json.load(fh)
            # Wiek pliku to jedyny sygnal, ze geolokator padl i zostal adres
            # z poprzedniego lotu.
            wiek = time.time() - os.path.getmtime(self.targets_json)
            (self.get_logger().warn if wiek > 300 else self.get_logger().info)(
                f"{self.targets_json}: zapisany {wiek:.0f} s temu")
        except FileNotFoundError:
            self.get_logger().warn(
                f"brak {self.targets_json} — zadna klasa nie ma adresu, "
                f"obie ida skanem")
            inne = self._resolve_asset(self.targets_json)
            if inne != os.path.expanduser(self.targets_json):
                self.get_logger().warn(
                    f"UWAGA: plik o tej nazwie LEZY w {inne}. Jesli to on ma "
                    f"byc zrodlem adresow, popraw targets_json w yamlu — "
                    f"NIE biore go sam, bo moglby byc z innego lotu.")
            return {}
        except Exception as e:
            self.get_logger().warn(f"nie moge odczytac {self.targets_json}: {e}")
            return {}

        mins = data.get('min_obs') or {}
        out = {}
        for cid, (name, key, _topic) in self.klasy.items():
            sec = data.get(key) or {}
            best = sec.get('best')
            if not best:
                self.get_logger().warn(f"{name}: BRAK adresu")
                continue
            need = mins.get(key, 10)
            src = best.get('source', '?')
            n_obs = best.get('n_obs', 0)
            if src != 'operator' and n_obs < need:
                self.get_logger().warn(
                    f"{name}: adres odrzucony — {n_obs} obserwacji przy progu "
                    f"{need} (to poziom krzaka, nie celu)")
                continue
            try:
                blat, blon = float(best['lat']), float(best['lon'])
            except (KeyError, TypeError, ValueError):
                self.get_logger().error(
                    f"{name}: 'best' bez poprawnych wspolrzednych — pomijam")
                continue
            out[cid] = (blat, blon, src, n_obs)
            self.get_logger().info(
                f"{name}: adres {blat:.7f} {blon:.7f} "
                f"(zrodlo={src}, obs={n_obs})")
        return out

    def _load_waypoints(self, path):
        """Trasa skanu -> [(lat, lon), ...]. Trzy formaty, zeby nie trzeba bylo
        nic konwertowac: QGC WPL (lat w kol. 8, lon w 9; HOME i komendy bez
        wspolrzednych pomijane), "lat, lon" w linii, albo OFFSETS + pary
        "wschod, polnoc" w metrach od punktu przejecia lotu."""
        try:
            raw = open(os.path.expanduser(path)).read().splitlines()
        except Exception as e:
            self.get_logger().error(f"nie moge wczytac {path}: {e}")
            return []

        qgc = bool(raw) and raw[0].startswith('QGC WPL')
        offsets = any(l.strip().upper() == 'OFFSETS' for l in raw)
        pts = []
        for line in raw:
            line = line.split('#')[0].strip()
            if not line or line.startswith('QGC') or line.upper() == 'OFFSETS':
                continue
            if qgc:
                f = line.split('\t')
                if len(f) < 10 or f[0] == '0':
                    continue
                try:
                    lat, lon = float(f[8]), float(f[9])
                except ValueError:
                    continue
                if abs(lat) < 1e-7 and abs(lon) < 1e-7:
                    continue
                pts.append((lat, lon))
            else:
                try:
                    a, b = (float(x) for x in line.replace(',', ' ').split()[:2])
                except ValueError:
                    continue
                pts.append(self._offset_gps(self.home[0], self.home[1], b, a)
                           if offsets else (a, b))
        if not pts:
            self.get_logger().error(f"{path}: nie znalazlem zadnych punktow")
        return pts

    # ═══════════════════════════════════════════════════════════
    #  Wspolny ogon: CELUJ, PODLOT, ZAWIS
    # ═══════════════════════════════════════════════════════════

    def aim_at_target(self) -> bool:
        """CELUJ: dron STOI, obraca sie nosem na cel i domyka gimbal.

        Bez tego kroku podlot bylby lukiem — vy jest w APPROACH wylaczone, wiec
        cala korekta boczna idzie przez obrot, a dron leci przy tym do przodu.
        """
        self.get_logger().info("CELUJ: obracam sie na cel, bez ruchu do przodu")
        self._vel_on()
        t0 = time.time()
        ok_since = None
        half_w = self.img_w / 2.0
        half_h = self.img_h / 2.0
        try:
            while rclpy.ok() and not self._abort:
                rclpy.spin_once(self, timeout_sec=0.05)
                now = time.time()
                if self.last_det_time <= 0 or now - self.last_det_time > self.lost_timeout:
                    self.get_logger().warn("CELUJ: cel zniknal")
                    return False

                ex = (self.tent_cx - half_w) / half_w
                ey = (self.tent_cy - half_h) / half_h

                # Gimbal RAZ na swieza detekcje — detekcja jest wolniejsza niz
                # ta petla i uzycie tej samej klatki dwa razy rozbujaloby go.
                if self._new_det:
                    self._new_det = False
                    if abs(ey) >= self.gimbal_deadzone:
                        self._set_gimbal(
                            self.pitch_deg - ey * (self.vfov_deg / 2.0) * self.damping)

                yaw_rate = clamp(self.kp_yaw * ex,
                                 -self.max_yaw_rate, self.max_yaw_rate)
                self.send_vectors(0.0, 0.0, self._alt_hold_vz(), yaw_rate)

                # Gimbal na ograniczniku nie domknie juz uchybu pionowego —
                # czekanie na ey w tolerancji byloby czekaniem na cos, co nie
                # moze nastapic. Od tego jest PODLOT, w trakcie ktorego cel sam
                # schodzi w dol kadru.
                na_ograniczniku = self.pitch_deg >= self.pitch_max - 0.5
                pion_ok = abs(ey) <= self.aim_tol_px or na_ograniczniku
                if abs(ex) <= self.aim_tol_px and pion_ok:
                    if ok_since is None:
                        ok_since = now
                        powod = ("gimbal na ograniczniku, reszte domknie podlot"
                                 if na_ograniczniku and abs(ey) > self.aim_tol_px
                                 else "cel w srodku kadru")
                        self.get_logger().info(
                            f"CELUJ: {powod} (ex={ex:+.3f} ey={ey:+.3f}), "
                            f"trzymam {self.aim_hold:.1f}s")
                    elif now - ok_since >= self.aim_hold:
                        self.get_logger().info(
                            f"WYCELOWANY (gimbal {self.pitch_deg:+.0f} st.)")
                        return True
                else:
                    ok_since = None

                if now - t0 > self.aim_timeout:
                    self.get_logger().warn(
                        f"CELUJ: timeout {self.aim_timeout:.0f}s "
                        f"(ex={ex:+.3f} ey={ey:+.3f})")
                    return False
        finally:
            self.send_vectors(0.0, 0.0, 0.0, 0.0)
            self._vel_off()
        return False

    def approach_and_center(self, over_target=False) -> bool:
        """PODLOT + ZAWIS — sterowanie oddane kontrolerowi.

        over_target=True (sciezka B): dron stoi juz nad celem, gimbal w pionie,
        kontroler wchodzi od razu w HOVER. Przy zgubieniu gimbal ma ZOSTAC
        w pionie — cel jest pod nami. Limit: hover_timeout.

        over_target=False (sciezka A): cel jest przed dronem, wiec przy
        zgubieniu gimbal wraca na kat przeszukiwania. Limit: approach_timeout,
        bo w nim miesci sie jeszcze przelot.

        Zwraca True TYLKO przy udanym wycentrowaniu.
        """
        self.search_gimbal_on_lost = not over_target
        # fresh=False: nie ruszamy gimbala ani okna detekcji — cel wlasnie
        # zostal potwierdzony.
        self.run_mission(fresh=False)
        limit = self.hover_timeout if over_target else self.approach_timeout
        t0 = time.time()
        hold_since = None
        search_since = None
        hovered = over_target
        try:
            while rclpy.ok() and not self._abort:
                rclpy.spin_once(self, timeout_sec=0.05)
                now = time.time()

                if self.state == State.HOVER and not hovered:
                    hovered = True
                    self.search_gimbal_on_lost = False
                    self.get_logger().info(
                        "nad celem — gimbal zostaje w pionie takze przy utracie")

                if self.state == State.SEARCH:
                    if search_since is None:
                        search_since = now
                    elif now - search_since > self.center_lost_timeout:
                        self.get_logger().warn(
                            f"cel nie wrocil przez {self.center_lost_timeout:.0f}s "
                            f"— przerywam centrowanie")
                        return False
                else:
                    search_since = None

                err = None
                if self.last_det_time > 0 and \
                        now - self.last_det_time <= self.lost_timeout:
                    err = self._target_offset()
                close = (err is not None
                         and abs(err[0]) <= self.center_tol_m
                         and abs(err[1]) <= self.center_tol_m)
                if self.state == State.HOVER and close:
                    if hold_since is None:
                        hold_since = now
                        self.get_logger().info(
                            f"cel pod dronem (przod {err[0]:+.1f} "
                            f"prawo {err[1]:+.1f} m) — trzymam "
                            f"{self.hover_hold_time:.0f}s")
                    elif now - hold_since >= self.hover_hold_time:
                        self.get_logger().info("WYCENTROWANY")
                        return True
                else:
                    hold_since = None

                if now - t0 > limit:
                    ostatni = ("" if err is None else
                               f", ostatni uchyb przod {err[0]:+.1f} "
                               f"prawo {err[1]:+.1f} m")
                    self.get_logger().error(
                        f"TIMEOUT {'zawisu' if over_target else 'centrowania'} "
                        f"{limit:.0f}s (stan {self.state.name}{ostatni})")
                    return False
        finally:
            self.stop_mission()
            self.search_gimbal_on_lost = True
        return False

    # ═══════════════════════════════════════════════════════════
    #  SCIEZKA B — mam adres
    # ═══════════════════════════════════════════════════════════

    def _zrzut_na_wspolrzedne(self, cid, lat, lon) -> bool:
        """Sciezka domyslna. Wracamy nad sam waypoint, bo centrowanie moglo nas
        z niego zsunac, a adres jest tym, w co naprawde wierzymy."""
        name = self.klasy[cid][0]
        self.get_logger().warn(f"{name}: ZRZUT NA WSPOLRZEDNE")
        self.action_interrupt = None
        self._gimbal(self.pitch_min)
        self.send_goto_global(lat, lon, self.target_alt)
        self._spin(1.0)
        self.drop(cid)
        return True

    def deliver(self, cid, lat, lon, src, n_obs) -> bool:
        """Dolot na waypoint i zrzut. ZAWSZE konczy sie zrzutem.

        Detektor MILCZY przez cala droge: adres to klaster setek obserwacji
        albo klik operatora, a zatrzymanie sie dla krzaka po drodze oznaczaloby
        zrzut obok niego i porzucenie zweryfikowanego adresu.
        """
        name, _key, topic = self.klasy[cid]
        self._apply_class(cid)
        self.set_detection_topic(topic)
        self.action_interrupt = None
        self.get_logger().info(
            f"╔══ {name} ══ DOWIEZ: {self._dist_to(lat, lon):.0f} m stad "
            f"(zrodlo={src}, obs={n_obs}), detektor milczy w drodze")

        self._gimbal(self.pitch_min)
        self.send_goto_global(lat, lon, self.target_alt)

        # Nad waypointem cel ma byc POD nami — gimbal w pion, inaczej okno
        # akwizycji nie mialoby szans.
        self._gimbal(self.pitch_min)
        self._spin(1.0)
        self.get_logger().info(
            f"nad waypointem (blad {self._dist_to(lat, lon):.1f} m), gimbal w pionie")

        widoczny = self.wait_acquire(timeout=self.wp_acquire)
        if widoczny and CENTRUJ_NAD_ADRESEM[cid]:
            self.get_logger().info(f"{name}: centruje na widocznym celu")
            if self.approach_and_center(over_target=True):
                self.get_logger().info(f"{name}: ZRZUT NAD WYCENTROWANYM CELEM")
                self.drop(cid)
                return True
            self.get_logger().warn(
                f"{name}: centrowanie sie nie udalo — schodze na wspolrzedne")
        elif widoczny:
            self.get_logger().info(
                f"{name}: cel w kadrze, ale ta klasa nie centruje sie nad "
                f"adresem — zrzut na wspolrzedne")

        return self._zrzut_na_wspolrzedne(cid, lat, lon)

    # ═══════════════════════════════════════════════════════════
    #  SCIEZKA A — skan
    # ═══════════════════════════════════════════════════════════

    def _gate(self, cid, faza):
        """Progi M z N dla danej KLASY i FAZY."""
        if cid == PERSON:
            return {'scan': (self.p_scan_m, self.p_scan_n),
                    'transit': (self.p_transit_m, self.p_transit_n),
                    'det': (self.p_det_m, self.p_det_n)}[faza]
        return {'scan': (self.scan_m, self.scan_n),
                'transit': (self.transit_m, self.transit_n),
                'det': (self.tent_det_m, self.tent_det_n)}[faza]

    def _apply_class(self, cid):
        """Przestaw kontroler pod dana klase: bramka M z N, wymog ciaglosci ID
        i lost_timeout. Klasa bazowa trzyma po jednej wartosci, wiec misja je
        mutuje; wartosci namiotowe siedza w tent_*. Okno jest buforem o stalej
        dlugosci, wiec zmiana N wymaga zbudowania go od nowa."""
        m, n = self._gate(cid, 'det')
        same = self.p_same_track if cid == PERSON else self.tent_same_track
        lost = self.p_lost_timeout if cid == PERSON else self.tent_lost_timeout
        if (m, n, same, lost) == (self.det_confirm_frames, self.det_window_frames,
                                  self.require_same_track, self.lost_timeout):
            return
        self.get_logger().info(
            f"kontroler pod {self.klasy[cid][0]}: bramka {m} z {n}, "
            f"same_track={same}, lost_timeout={lost:.1f}s")
        self.det_confirm_frames = m
        self.det_window_frames = n
        self.require_same_track = same
        self.lost_timeout = lost
        self._det_window = deque(maxlen=n)
        self._cand_id = -1

    def _mute(self, seconds):
        if self.watcher is not None:
            self.watcher.mute(seconds)

    def _watch_scan(self) -> bool:
        """Haczyk dla akcji goto: czy watcher wlasnie potwierdzil cel."""
        if self.watcher is not None and self.watcher.confirmed():
            self._scan_hit = True
            self.get_logger().info(
                f"DETEKTOR w przelocie: {self.watcher.hits}/{self.watcher.window} "
                f"klatek, ID={self.watcher.track_id}"
                f"{self._opis_odleglosci()} — przerywam dolot")
            return True
        return False

    def _opis_odleglosci(self):
        """Jak daleko lezy potwierdzony cel — to jest POMIAR zasiegu detektora,
        a od niego zalezy, ile punktow skanu potrzebuje pole."""
        if self.last_det_time <= 0 or time.time() - self.last_det_time > self.lost_timeout:
            return ""
        fwd, right = self._target_offset()
        d = math.hypot(fwd, right)
        if d <= 0.1:
            return ""
        px = self.focal_px * 3.0 / math.hypot(self.altitude, d)
        return f", ~{d:.0f} m stad (namiot 3 m ~ {px:.0f} px)"

    def _scan_cell(self, pitch) -> bool:
        """Jedna komorka skanu: ustaw gimbal, odczekaj, patrz.

        Okno zerujemy PO dojechaniu gimbala — inaczej klatki z przejazdu
        (kadr rozmazany po terenie spoza komorki) liczylyby sie do niej.
        """
        self._gimbal(pitch)
        self._spin(self.scan_settle)
        self.watcher.reset()
        end = time.time() + self.scan_dwell
        while rclpy.ok() and not self._abort and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            # Petla kontrolera jest w skanie wylaczona, wiec bez tego nikt nie
            # trzyma wysokosci.
            self.send_vectors(0.0, 0.0, self._alt_hold_vz(), 0.0)
            if self.watcher.confirmed():
                self.get_logger().info(
                    f"DETEKTOR w skanie (gimbal {pitch:+.0f} st.): "
                    f"{self.watcher.hits}/{self.watcher.window} klatek, "
                    f"ID={self.watcher.track_id}{self._opis_odleglosci()}")
                return True
        return False

    def _turn_to(self, cel_rad, timeout=12.0) -> bool:
        """Obrot do BEZWZGLEDNEGO kursu (0 = polnoc, rosnie w prawo).

        Nie uzywamy akcji Set_yaw: jej petla w drone_handler porownuje kat
        z [0,2pi) z katem z [-pi,pi], wiec przy ujemnym namiarze warunek konca
        nigdy sie nie spelnia. Wymaga wlaczonego sterowania predkosciowego.
        """
        cel = _wrap(cel_rad)
        t0 = time.time()
        try:
            while rclpy.ok() and not self._abort:
                rclpy.spin_once(self, timeout_sec=0.05)
                err = _wrap(cel - self.drone_yaw)
                if abs(err) < math.radians(3.0):
                    return True
                if time.time() - t0 > timeout:
                    self.get_logger().warn(
                        f"obrot: timeout, zostalo {math.degrees(err):+.0f} st.")
                    return False
                rate = clamp(2.0 * err, -self.scan_yaw_rate, self.scan_yaw_rate)
                self.send_vectors(0.0, 0.0, self._alt_hold_vz(), rate)
        finally:
            self.send_vectors(0.0, 0.0, 0.0, 0.0)
            self._spin(0.3)
        return False

    def _build_scan_cells(self, arc_deg, heading_deg):
        """Komorki skanu: [(kurs bezwzgledny albo None, pitch), ...].

        Kursy BEZWZGLEDNE, bo skan musi dac sie wznowic po tym, jak CELUJ
        obroci maszyna — kroki wzgledne liczylyby sie wtedy od przypadkowego
        kursu. Nadir jest komorka 0 i ma kurs None: przy -90 kadr otacza punkt
        ze wszystkich stron. Pochylenia na przemian, zeby przejazd gimbala
        dzial sie w trakcie obrotu.
        """
        n_yaw = max(1, int(round(arc_deg / self.scan_yaw_step)))
        # Pelny obrot pokrywa wszystko niezaleznie od kursu startowego.
        pelny = arc_deg >= 355.0
        if pelny or heading_deg is None:
            start = self.drone_yaw
        else:
            start = (math.radians(heading_deg)
                     - math.radians((n_yaw - 1) * self.scan_yaw_step / 2.0))
        cells = [(None, self.scan_pitch_nadir)]
        for k in range(n_yaw):
            kurs = start + math.radians(k * self.scan_yaw_step)
            seq = (self.scan_pitches if k % 2 == 0
                   else list(reversed(self.scan_pitches)))
            for pitch in seq:
                cells.append((kurs, pitch))
        self.get_logger().info(
            f"SKAN {arc_deg:.0f} st.: nadir + {n_yaw} pozycji yaw x "
            f"{len(self.scan_pitches)} pochylen = {len(cells)} komorek"
            + ("" if pelny else
               f", luk wysrodkowany na kursie {heading_deg:+.0f} st. "
               f"(od {math.degrees(_wrap(start)):+.0f})"))
        return cells

    def scan_at_point(self, cells, start_at=0):
        """Przejdz komorki od start_at. Zwraca INDEKS komorki, ktora zobaczyla
        cel, albo None. Indeks, a nie True/False, zeby po nieudanym sprawdzeniu
        dalo sie wznowic od NASTEPNEJ komorki."""
        # Sterowanie predkosciowe wlaczamy RAZ na caly przebieg — kazde
        # toggle_control to synchroniczne wywolanie serwisu.
        self._vel_on()
        biezacy_kurs = None
        try:
            for i in range(start_at, len(cells)):
                if self._abort or time.time() > self._deadline:
                    return None
                kurs, pitch = cells[i]
                if kurs is not None and kurs != biezacy_kurs:
                    self._turn_to(kurs)
                    biezacy_kurs = kurs
                    self.get_logger().info(
                        f"  kurs {math.degrees(_wrap(kurs)):+.0f} st. "
                        f"(komorka {i + 1}/{len(cells)})")
                if self._scan_cell(pitch):
                    return i
            return None
        finally:
            # Wylaczyc TRZEBA: nastepny krok to send_goto_global, a akcje GPS
            # nie dzialaja przy wlaczonym sterowaniu predkosciowym.
            self._vel_off()

    def handle_scan_target(self, w_locie=False, class_id=TENT) -> bool:
        """Cel zobaczony w skanie albo w przelocie.

        STOP jest konieczny, bo przerwanie akcji NIE zatrzymuje drona —
        ArduPilot w GUIDED trzyma ostatni zadany punkt.

        SPRAWDZ na stojaco idzie PELNYM oknem kontrolera, nie czulszym oknem
        skanu: skan ma nie przegapic, sprawdzenie ma sie nie dac oszukac.
        Zrzut idzie WYLACZNIE po udanym wycentrowaniu — nie ma tu adresu, wiec
        nie ma czego zrzucic w ciemno.
        """
        name, _key, topic = self.klasy[class_id]
        cisza = self.det_cooldown if w_locie else self.scan_det_cooldown
        self.get_logger().info(f"╔══ {name} ══ STOP i sprawdzenie na stojaco")

        self.action_interrupt = None
        self.send_goto_global(self.global_lat, self.global_lon, self.target_alt)
        self._spin(self.brake_settle_time)

        self._apply_class(class_id)
        self.set_detection_topic(topic)
        if not self.wait_acquire(timeout=self.reconfirm_timeout):
            self._mute(cisza)
            self.get_logger().warn(
                f"{name}: na stojaco cel sie nie potwierdzil — falszywka, "
                f"cisza {cisza:.0f}s")
            return False

        if not self.aim_at_target():
            self._mute(cisza)
            return False

        if not self.approach_and_center(over_target=False):
            self._mute(cisza)
            self._gimbal(self.pitch_transit)
            self.get_logger().warn(
                f"{name}: cel zniknal w trakcie centrowania — falszywka, "
                f"NIE zrzucam (cisza {cisza:.0f}s)")
            return False

        self.get_logger().info(f"{name}: ZRZUT NAD WYCENTROWANYM CELEM")
        self.drop(class_id)
        return True

    def _skan_punktu(self, cid, arc, hdg) -> bool:
        """Skan z postoju na JEDNYM punkcie. True = ladunek poszedl."""
        self.watcher.set_gate(*self._gate(cid, 'scan'))
        # Cisza z PRZELOTU nie obowiazuje w skanie — inaczej falszywka zlapana
        # w drodze oslepia poczatek skanu na nowym punkcie.
        self.watcher.mute_until = 0.0
        self.watcher.reset()
        cells = self._build_scan_cells(arc, hdg)
        od = 0
        while od < len(cells) and not self._abort:
            trafil = self.scan_at_point(cells, od)
            if trafil is None:
                return False                   # przeszedl wszystko, nic
            if self.handle_scan_target(class_id=cid):
                return True
            od = trafil + 1                    # falszywka: od nastepnej komorki
        return False

    def search_class(self, cid, skan_w_miejscu=False, budzet=None) -> bool:
        """Skan trasy dla JEDNEJ klasy. True = ladunek poszedl. Detektor patrzy
        takze w przelocie — tu nie ma lepszego adresu niz to, co widac.

        skan_w_miejscu=True: zacznij pelnym obrotem TU, gdzie jestesmy, a trase
        od NAJBLIZSZEGO punktu. Uzywane przy szukaniu czlowieka po zrzucie na
        namiot: dron stoi wtedy gdzies posrodku pola, a nie na WP1.
        """
        name, _key, topic = self.klasy[cid]
        bazowe = self._load_waypoints(self._resolve_asset(self.scan_waypoints))
        if not bazowe:
            return False

        # Kazdy punkt niesie SWOJ indeks z konfiguracji, bo kolejnosc moze sie
        # zmienic, a luk i kurs sa przypisane do PUNKTU, nie do miejsca w kolejce.
        trasa = [(i, la, lo) for i, (la, lo) in enumerate(bazowe)]
        if skan_w_miejscu:
            # Najblizszy sasiad. Cykliczne przesuniecie listy byloby gorsze, bo
            # trasa jest LINIA, a nie petla: przy trzech punktach dawalo 425 m
            # przelotu zamiast 300 m.
            zostalo, trasa = list(trasa), []
            poz_lat, poz_lon = self.global_lat, self.global_lon
            while zostalo:
                j = min(range(len(zostalo)),
                        key=lambda z: math.hypot(
                            (zostalo[z][1] - poz_lat) * M_LAT,
                            (zostalo[z][2] - poz_lon) * _m_per_deg_lon(poz_lat)))
                trasa.append(zostalo[j])
                poz_lat, poz_lon = zostalo[j][1], zostalo[j][2]
                zostalo.pop(j)
        elif SKAN_WRACA_NA_PIERWSZY and len(bazowe) > 1:
            trasa.append((0, bazowe[0][0], bazowe[0][1]))

        # Watcher ZAWSZE swiezy i na wlasciwym topicu — poprzednia klasa
        # sluchala innego.
        if self.watcher is not None:
            try:
                self.destroy_subscription(self.watcher.sub)
            except Exception:
                pass
        self._apply_class(cid)
        self.set_detection_topic(topic)
        self.watcher = ScanWatcher(self, topic, *self._gate(cid, 'scan'))
        czas = budzet if budzet else self.scan_timeout
        self._deadline = time.time() + czas
        self.get_logger().info(
            f"{name}: SKAN — {len(trasa)} punktow, budzet {czas:.0f}s"
            + (f", start obrotem w miejscu, potem od najblizszego "
               f"(WP{trasa[0][0] + 1})" if skan_w_miejscu else ""))
        if skan_w_miejscu:
            self.get_logger().info(
                f"{name}: kolejnosc punktow "
                + " -> ".join(f"WP{i + 1}" for i, _la, _lo in trasa))

        try:
            if skan_w_miejscu:
                self.get_logger().info(
                    f"{name}: obrot 360 st. w miejscu, bez przelotu")
                if self._skan_punktu(cid, 360.0, None):
                    return True

            for nr, (idx, wlat, wlon) in enumerate(trasa, 1):
                if self._abort or time.time() > self._deadline:
                    self.get_logger().warn("skan: koniec budzetu czasu")
                    return False
                self.get_logger().info(
                    f"=== punkt {nr}/{len(trasa)} (WP{idx + 1}) === "
                    f"{self._dist_to(wlat, wlon):.0f} m stad")

                # Dolot z obserwacja. Przerwany w polowie -> po obsluzeniu celu
                # wracamy na kurs do TEGO SAMEGO punktu.
                while not self._abort and time.time() < self._deadline:
                    if self._dist_to(wlat, wlon) <= self.arrive_tol:
                        break
                    self._gimbal(self.pitch_transit)
                    self.watcher.set_gate(*self._gate(cid, 'transit'))
                    self.watcher.reset()
                    self._scan_hit = False
                    self.action_interrupt = self._watch_scan
                    try:
                        self.send_goto_global(wlat, wlon, self.target_alt)
                    finally:
                        self.action_interrupt = None
                    if not self._scan_hit:
                        break
                    self._scan_hit = False
                    if self.handle_scan_target(w_locie=True, class_id=cid):
                        return True

                # Luk i kurs per PUNKT (indeks z konfiguracji, nie z kolejki).
                arc = (self.scan_arc_deg[idx] if idx < len(self.scan_arc_deg)
                       else self.scan_arc_deg[-1])
                hdg = (self.scan_heading_deg[idx]
                       if idx < len(self.scan_heading_deg)
                       else self.scan_heading_deg[-1])
                if self._skan_punktu(cid, arc, hdg):
                    return True

            self.get_logger().error(
                f"{name}: skan przeleciany w calosci, nic nie znaleziono — "
                f"ladunek zostaje na pokladzie")
            return False
        finally:
            self.action_interrupt = None

    # ═══════════════════════════════════════════════════════════
    #  Misja
    # ═══════════════════════════════════════════════════════════

    def _finish(self, done):
        self.stop_mission()
        self._vel_off()
        self._spin(1.0, stop_on_abort=False)
        zostaly = [self.klasy[c][0] for c in self.klasy if c not in done]
        if zostaly:
            self.get_logger().warn(
                f"ladunki, ktore zostaly na pokladzie: {', '.join(zostaly)}")
        self.get_logger().info("=== POWROT: RTL ===")
        self.rtl()
        self._spin(2.0, stop_on_abort=False)
        self.get_logger().info("=== KONIEC MISJI ===")

    def run(self) -> bool:
        self.get_logger().info("=== START: suas_mission ===")
        self._install_signals()

        if self.auto_takeoff:
            self.get_logger().warn(
                "auto_takeoff=true — TRYB TESTOWY: sam startuje zamiast czekac "
                "na GUIDED (w prawdziwej misji dron juz leci)")
            if not self.arm():
                self.get_logger().error("ARM nieudany")
                return False
            if not self.takeoff(float(self.target_alt)):
                self.get_logger().error("TAKEOFF nieudany — LAND")
                self.land()
                return False
            self._spin(3.0)
        elif not self.wait_for_guided():
            return False

        self.home = (self.global_lat, self.global_lon)
        # Od tej chwili gimbal jest nasz — geolokator ma przestac trzymac pion.
        self._nadir_pub.publish(Bool(data=False))
        self._gimbal(self.pitch_transit, force=True)
        self.descend(self.target_alt)

        targets = self.read_targets()
        done = set()

        # Kolejnosc SZTYWNA: namiot ma na 50 m 49 px i wykrywa sie sam,
        # czlowiek 8 px i praktycznie nie.
        for cid in (TENT, PERSON):
            if self._abort:
                break
            name = self.klasy[cid][0]
            if cid in targets:
                lat, lon, src, n_obs = targets[cid]
                if self.deliver(cid, lat, lon, src, n_obs):
                    done.add(cid)
            elif cid == TENT:
                if self.search_class(TENT):
                    done.add(cid)
            elif SZUKAJ_CZLOWIEKA_PO_NAMIOCIE:
                # Szanse ograniczone: na 50 m czlowiek STOJACY ma 8 px, czyli
                # ponizej progu YOLO. Realnie liczy sie tylko lezacy.
                self.get_logger().info(
                    f"{name}: brak adresu — szukam skanem "
                    f"({'namiot zrzucony' if TENT in done else 'namiotu tez nie bylo'})")
                if self.search_class(PERSON, skan_w_miejscu=True,
                                     budzet=self.person_scan_timeout):
                    done.add(cid)

        self._finish(done)
        return True


def main(args=None):
    # SignalHandlerOptions.NO — Ctrl+C obsluguje sama misja, zeby zdazyla
    # wyslac RTL zanim kontekst ROS padnie.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = SuasMission()
    try:
        node.get_logger().info("Czekam 2s na inicjalizacje serwisow...")
        time.sleep(2.0)
        node.run()
    except KeyboardInterrupt:
        node.get_logger().warn("Twarde przerwanie — dron zostaje w GUIDED!")
    finally:
        for fn in (node.stop_mission, node.destroy_node, rclpy.shutdown):
            try:
                fn()
            except Exception:
                pass


if __name__ == '__main__':
    main()
