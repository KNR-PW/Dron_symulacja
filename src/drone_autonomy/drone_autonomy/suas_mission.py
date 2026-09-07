#!/usr/bin/env python3
"""
suas_mission — misja SUAS: adres albo skan.

Pelny opis dzialania i uzasadnienie kazdej liczby: docs/misja.md.
Ten wezel zastepuje suas_full_mission i suas_grid_mission.

JEDNA ZASADA
    Dla kazdej klasy osobno pada pytanie "czy mam adres?":
        NAMIOT   ma waypoint z geolokatora?  TAK -> DOWIEZ   NIE -> SZUKAJ
        CZLOWIEK ma waypoint z geolokatora?  TAK -> DOWIEZ   NIE -> nie rusza
    Kolejnosc klas jest sztywna: najpierw NAMIOT, potem CZLOWIEK.

DWIE SCIEZKI, WSPOLNY OGON
    B (cel POD dronem — dolot na waypoint)
        DOLOT (detektor MILCZY) -> STOP, gimbal w pion -> okno M z N (10 s)
        -> SPACJA raz (10 s) -> ZAWIS + ZRZUT  albo  ZRZUT NA WSPOLRZEDNE
        Wszystko od dolotu miesci sie w budzecie wp_budget (20 s), zeby
        migoczaca detekcja nie zapetlila pytania nad jednym punktem.

    A (cel PRZED dronem — skan albo wykrycie w przelocie)
        STOP -> SPRAWDZ na stojaco (pelne 4 z 8) -> CELUJ (yaw+gimbal, bez
        ruchu) -> SPACJA -> PODLOT -> ZAWIS -> ZRZUT

    W sciezce B nie ma CELUJ ani PODLOTU i to nie jest oszczednosc, tylko
    geometria: dron stoi NAD celem, wiec namiar jest nieokreslony, a jedyne,
    co zostaje, to korekta polozenia w metrach.

SKAN (tylko NAMIOT, gdy nie ma adresu)
    Na kazdym punkcie dron STOI i skanuje: obrot co scan_yaw_step (60 st.),
    na kazdej pozycji gimbal gora-dol. Nadir robi sie RAZ na punkt, bo przy
    -90 kadr otacza punkt pod dronem ze wszystkich stron.
    Trasa: WP1 -> WP2 -> WP1 (powrot wlacza scan_return_to_first).

    Bramka skanu (scan_confirm_frames z scan_window_frames, 3 z 6) jest
    CZULSZA niz bramka kontrolera (4 z 8) i to jest celowe: domkniecie okna
    w komorce niczego nie uruchamia poza sprawdzeniem, a po nim ida jeszcze
    cztery bramki. Falszywe wyzwolenie kosztuje kilka sekund, przegapiony cel
    kosztuje ladunek.

Wymaga: drone_handler + detektor (/tent_detections, /people_detections).
Geolokator jest opcjonalny — bez niego po prostu nie ma adresow i namiot
idzie sciezka skanu. Jesli chodzi, misja zdejmuje mu blokade nadiru.

Musi isc przez `ros2 run`, a NIE z launcha — wait_confirm czyta klawiature
i potrzebuje stdin podpietego do terminala.

    ros2 run drone_autonomy suas_mission --ros-args \\
        --params-file ~/Dron_symulacja/src/drone_bringup/config/misja.yaml
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


def _m_per_deg_lon(lat):
    return M_LAT * math.cos(math.radians(lat))


def _wrap(a):
    """Kat do [-pi, pi]."""
    return (a + math.pi) % (2 * math.pi) - math.pi


class ScanWatcher:
    """Okno M z N dla JEDNEJ klasy, liczone niezaleznie od kontrolera.

    Kontroler ma jedno okno i jeden topic naraz, bo w podejsciu sledzi
    konkretny obiekt. Skan potrzebuje wlasnego, bo:
      * ma INNE progi (czulsze — patrz docstring modulu),
      * musi dac sie wyzerowac przy kazdej zmianie komorki, nie ruszajac
        stanu kontrolera.

    Zerowanie na komorke jest KONIECZNE przy krotkim dwellu: okno jest
    kroczacym buforem, wiec dwa trafienia z komorki k i dwa z k+1 domknelyby
    je na celu, ktorego nie ma w zadnej z nich.
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
        # Detektor sie zaciol / kamera padla — stare okno przestaje byc aktualne.
        if (self._prev_frame > 0.0
                and now - self._prev_frame > self.node.det_confirm_gap):
            self.reset()
        self._prev_frame = now

        good = bool(msg.detected)
        if good and self.node.require_same_track:
            if msg.track_id < 0:
                # Brak ID = podpis migoczacej falszywki. Prawdziwy cel dostaje
                # ID od razu, bo utrzymuje sie miedzy klatkami.
                good = False
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
        """Przelacz progi. Misja uzywa innych w locie i innych na postoju."""
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

        # ── Zrodlo adresow ──────────────────────────────────────────
        p('targets_json',
          '~/Dron_symulacja/src/drone_bringup/config/targets.json')
        # Trasa skanu. Obslugiwane: .waypoints z Mission Plannera
        # ("QGC WPL 110"), "lat, lon" w linii, albo linia OFFSETS i pary
        # "wschod, polnoc" w metrach od punktu przejecia lotu.
        p('scan_waypoints',
          '~/Dron_symulacja/src/drone_bringup/config/misja_skan.waypoints')

        # ── Przejecie lotu ──────────────────────────────────────────
        p('takeover_timeout', 1200.0)
        p('auto_takeoff', False)
        p('finish_action', 'rtl')

        # ── SCIEZKA B: dolot na waypoint ────────────────────────────
        # Twardy budzet od dolotu do decyzji. Bez niego migoczaca detekcja
        # zapetlilaby pytanie: cel wraca -> pytamy -> znika -> wraca...
        p('wp_budget', 20.0)
        p('wp_acquire', 10.0)      # okno M z N nad waypointem (budzet - confirm)

        # ── ZAWIS (wspolny ogon obu sciezek) ────────────────────────
        # Te cztery zyja w suas_full_mission, a nie w kontrolerze bazowym —
        # bez wlasnej deklaracji byloby AttributeError w approach_and_center,
        # czyli w ostatnim kroku przed zrzutem.
        p('approach_timeout', 90.0)      # gorny limit calego centrowania
        p('center_lost_timeout', 15.0)   # ciagly SEARCH tak dlugo = cel zniknal
        p('center_tol_m', 1.5)           # kiedy cel jest "pod dronem"
        p('hover_hold_time', 5.0)        # jak w suas_simple_mission

        # ── SCIEZKA A: CELUJ ────────────────────────────────────────
        # Warunek w PIKSELACH, nie w metrach: przy wysoko podniesionym gimbalu
        # gorne wiersze kadru moga patrzec ponad horyzont, a rzutowanie zwraca
        # wtedy None i uchyb metrowy odczytalby sie jako "wycentrowany".
        p('aim_tol_px', 0.06)
        p('aim_hold', 1.0)
        p('aim_timeout', 15.0)

        # ── SKAN ────────────────────────────────────────────────────
        p('scan_yaw_step', 60.0)
        # Luk skanu per punkt trasy. WP1 lezy na brzegu, wiec pole jest
        # w calosci z przodu i wystarczy 180 st.; WP2 jest w srodku -> 360.
        p('scan_arc_deg', [180.0, 360.0])
        # BEZWZGLEDNY kurs (0 = polnoc, rosnie w prawo), na ktorym luk skanu ma
        # byc WYSRODKOWANY. Bez tego luk zaczynalby sie od kursu, z jakim dron
        # akurat przylecial — a na powrocie na WP1 przylatuje od strony WP2,
        # czyli TYLEM do pola, i przeskanowalby teren poza obszarem.
        # Przy luku 360 st. nie ma znaczenia i jest pomijany.
        p('scan_heading_deg', [0.0, 0.0])
        p('scan_pitch_nadir', -90.0)     # raz na punkt, bez obracania
        p('scan_pitches', [-65.0, -40.0])  # na kazdej pozycji yaw
        p('scan_dwell', 1.5)
        p('scan_yaw_rate', 1.0)          # rad/s TYLKO na obrot skanu
        p('scan_settle', 0.4)            # ile czekamy, az gimbal dojedzie
        p('scan_confirm_frames', 3)      # M — czulsza bramka skanu
        p('scan_window_frames', 6)       # N
        # Bramka na PRZELOT miedzy punktami — ostrzejsza niz skanowa. W skanie
        # dron STOI: obraz jest ostry, a falszywe wyzwolenie kosztuje tylko
        # ponowne sprawdzenie w miejscu. W locie jest odwrotnie — obraz jest
        # rozmyty ruchem (wiec falszywek jest wiecej), a kazde wyzwolenie
        # kosztuje hamowanie, sprawdzenie i cooldown.
        p('transit_confirm_frames', 4)
        p('transit_window_frames', 8)
        p('scan_return_to_first', True)  # po ostatnim punkcie wroc na pierwszy
        p('scan_timeout', 600.0)
        p('pitch_transit', -55.0)        # kat w przelocie miedzy punktami
        # Cisza dla klasy po falszywce — inaczej ten sam krzak zatrzymywalby
        # drona w kolko.
        p('det_cooldown', 20.0)
        # Cisza po falszywce ZNALEZIONEJ W SKANIE. Musi byc krotka, bo skan
        # trwa ~35 s i cisza 20 s oslepia polowe komorek. Zmierzone 2026-09-07:
        # falszywka z przelotu wyciszyla klase na 20 s, przez co 4 z 6 pozycji
        # yaw na WP2 przeleciano ze slepym detektorem — a namiot byl na 5/6.
        # W skanie dlugiej ciszy nie potrzeba: po nieudanym sprawdzeniu i tak
        # wznawiamy od NASTEPNEJ komorki, wiec ta sama falszywka nie wraca.
        p('scan_det_cooldown', 5.0)
        p('reconfirm_timeout', 3.0)
        p('brake_settle_time', 3.0)
        p('arrive_tol', 3.0)

        g = self.get_parameter
        self.targets_json = os.path.expanduser(str(g('targets_json').value))
        self.scan_waypoints = os.path.expanduser(str(g('scan_waypoints').value))
        self.takeover_timeout = g('takeover_timeout').value
        self.auto_takeoff = g('auto_takeoff').value
        self.finish_action = str(g('finish_action').value).lower()
        self.wp_budget = g('wp_budget').value
        self.wp_acquire = g('wp_acquire').value
        self.approach_timeout = g('approach_timeout').value
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
        self.scan_return_to_first = g('scan_return_to_first').value
        self.scan_timeout = g('scan_timeout').value
        self.pitch_transit = g('pitch_transit').value
        self.det_cooldown = g('det_cooldown').value
        self.scan_det_cooldown = g('scan_det_cooldown').value
        self.reconfirm_timeout = g('reconfirm_timeout').value
        self.brake_settle_time = g('brake_settle_time').value
        self.arrive_tol = g('arrive_tol').value

        # "Wycentrowany" nie moze byc ciasniejszy niz martwa strefa regulatora:
        # ponizej hover_deadzone_m HOVER przestaje korygowac, wiec warunek
        # nigdy by sie nie domknal i kazde centrowanie konczyloby sie timeoutem.
        if self.center_tol_m < self.hover_deadzone_m:
            self.get_logger().warn(
                f"center_tol_m({self.center_tol_m}) < hover_deadzone_m"
                f"({self.hover_deadzone_m}) — podnosze")
            self.center_tol_m = self.hover_deadzone_m

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

        # Obecnosc operatora. Potrzebna TYLKO w sciezce A: tam milczenie
        # operatora, ktory PATRZY, znaczy "to nie ten cel", a jego nieobecnosc
        # znaczy "decyduj sam". W sciezce B nie jest potrzebna, bo tam
        # milczenie i nieobecnosc daja to samo — zrzut na wspolrzedne.
        self._operator_online = False
        self._operator_stamp = 0.0
        self.create_subscription(Bool, '/operator_online', self._online_cb, 10)

        # Po przejeciu lotu gimbal nalezy do misji — geolokator ma przestac
        # trzymac pion, inaczej dwoch pisze na jeden silownik.
        self._nadir_pub = self.create_publisher(Bool, '/geolocator/lock_nadir', 10)

        self.get_logger().info(
            f"suas_mission: alt={self.target_alt} m | adresy z {self.targets_json} "
            f"| trasa skanu {self.scan_waypoints}")
        # Trase sprawdzamy TERAZ, a nie dopiero gdy okaze sie potrzebna. Inaczej
        # brakujacy plik wychodzi na jaw po starcie, dolocie i zejsciu na 50 m —
        # czyli po dwoch minutach lotu, ktory i tak nie ma jak sie udac.
        znaleziona = self._resolve_asset(self.scan_waypoints)
        if os.path.isfile(znaleziona):
            self.get_logger().info(f"trasa skanu: {znaleziona}")
        else:
            self.get_logger().error(
                "BRAK PLIKU TRASY SKANU — jesli namiot nie dostanie adresu "
                "z geolokatora, misja nie bedzie miala gdzie go szukac")

        self.get_logger().info(
            f"skan: krok {self.scan_yaw_step:.0f} st., luki {self.scan_arc_deg}, "
            f"nadir {self.scan_pitch_nadir:.0f} raz + {self.scan_pitches} na pozycje, "
            f"dwell {self.scan_dwell:.1f}s, bramka {self.scan_m} z {self.scan_n}")

    # ═══════════════════════════════════════════════════════════
    #  Pomocnicze
    # ═══════════════════════════════════════════════════════════

    def _install_signals(self):
        """SIGHUP jest tu rownie wazny co SIGINT: dostajemy go, gdy zerwie sie
        terminal (SSH, Tailscale). Bez obslugi proces ginie bez RTL, a dron
        zostaje wiszacy w GUIDED. To NIE zwalnia z uruchamiania pod tmux —
        tmux sprawia, ze SIGHUP w ogole nie dolatuje."""
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

    def _online_cb(self, msg: Bool):
        self._operator_online = bool(msg.data)
        self._operator_stamp = time.time()

    def operator_watching(self) -> bool:
        """Wymagamy SWIEZEJ wiadomosci, nie samej wartosci: gdyby wezel GUI
        padl, ostatnie 'true' zostaloby w pamieci na zawsze."""
        return self._operator_online and time.time() - self._operator_stamp < 5.0

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
        """Skladowa pionowa, gdy sterujemy recznie (skan, CELUJ). Ta sama
        formula co w _control_loop — bez niej dron dryfuje w pionie, bo petla
        kontrolera jest w tych fazach wylaczona."""
        return clamp(-self.kp_alt * (self.target_alt - self.altitude),
                     -self.max_vz, self.max_vz)

    def _vel_on(self):
        """Wlacz sterowanie predkosciowe BEZ uruchamiania petli wizualnej.
        Skan i CELUJ wysylaja wektory same, wiec petla tylko by sie z nimi bila."""
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
        """Znajdz plik repo niezaleznie od tego, gdzie stoi workspace.

        Ten sam yaml musi dzialac w TRZECH miejscach o roznych sciezkach:
          Jetson / host   ~/Dron_symulacja/src/drone_bringup/config/
          kontener Docker /root/ros_ws/src/drone_bringup/config/
          po instalacji   <share>/drone_bringup/config/
        Docker montuje TYLKO katalog src/, wiec '~' znaczy co innego po kazdej
        stronie i sciezka wpisana na sztywno dziala dokladnie w jednym z nich.

        Zwraca pierwszy istniejacy plik, a gdy nie ma zadnego — sciezke
        z konfiguracji, zeby komunikat bledu pokazal to, co wpisal uzytkownik.
        """
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
        """force=True omija filtr 'kat sie nie zmienil' z _set_gimbal.
        Potrzebne na starcie, bo pitch_deg jest zainicjowany na pitch_search
        i serwo nigdy nie dostaloby pierwszej komendy."""
        if force:
            self.pitch_deg = clamp(deg, self.pitch_min, self.pitch_max)
            self.set_gimbal_pitch(self.pitch_deg)
        else:
            self._set_gimbal(deg)

    # ═══════════════════════════════════════════════════════════
    #  Przejecie lotu, wysokosc, adresy
    # ═══════════════════════════════════════════════════════════

    def wait_for_guided(self) -> bool:
        """Nic tu nie wysylamy. Dopoki jest AUTO, ArduPilot i tak ignoruje
        setpointy predkosci — ROS jest w tym trybie fizycznie bezsilny."""
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
        """Akcja goto_global konczy sie po odleglosci POZIOMEJ < 2 m, a tu
        podajemy te sama lat/lon — bez wlasnego czekania "zejscie" trwaloby 0 s,
        a dron schodzilby dopiero w trakcie dolotu do celu."""
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

        Bierzemy TYLKO 'best' kazdej klasy. Nie ma puli kandydatow ani
        rankingu: adres uznajemy za wiarygodny i naszym zadaniem jest go
        dowiezc, a nie prowadzic sledztwo od nowa (docs/misja.md sekcja 1).

        Filtr min_obs zostaje, bo geolokator wypisuje takze klastry z 3-4
        obserwacjami — czyli krzaki i cienie. Wskazanie operatora przechodzi
        bez progu: klik powstal z ogladania obrazu, nie z liczenia trafien.
        """
        try:
            with open(self.targets_json) as fh:
                data = json.load(fh)
            wiek = time.time() - os.path.getmtime(self.targets_json)
            # targets.json to JEDYNY kanal miedzy geolokatorem a misja. Gdy
            # geolokator padnie albo w ogole nie wstal, zostaje stary plik
            # i misja poleciala by pod adres z poprzedniego lotu, nic nie
            # zauwazajac. Wiek pliku jest jedynym sygnalem, jaki mamy.
            (self.get_logger().warn if wiek > 300 else self.get_logger().info)(
                f"{self.targets_json}: zapisany {wiek:.0f} s temu")
        except FileNotFoundError:
            self.get_logger().warn(
                f"brak {self.targets_json} — zadna klasa nie ma adresu, "
                f"namiot idzie skanem")
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
        """Trasa skanu. Zwraca [(lat, lon), ...] albo [].

        Trzy formaty, zeby nie trzeba bylo niczego konwertowac:
          1. .waypoints z Mission Plannera ("QGC WPL 110") — lat w kolumnie 8,
             lon w 9. Wiersz 0 (HOME) i komendy bez wspolrzednych (TAKEOFF,
             RTL, DO_*) sa pomijane. To jest sciezka dla realu.
          2. "lat, lon" w kazdej linii.
          3. Linia OFFSETS, a po niej "wschod, polnoc" w METRACH wzgledem
             PUNKTU PRZEJECIA LOTU. Sciezka do testow — nic nie trzeba
             przeliczac na stopnie.
        """
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

        Po co osobny krok: gdyby podlot startowal od razu, dron lecialby
        do przodu i obracal sie jednoczesnie, czyli zatoczylby luk (vy jest
        w APPROACH wylaczone, cala korekta boczna idzie przez obrot). Po
        wycelowaniu na postoju podlot jest linia prosta, a operator dostaje
        do oceny obraz nieruchomy i wysrodkowany.

        Warunek w PIKSELACH, nie w metrach — patrz aim_tol_px w __init__.
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
                # ta petla, wiec uzycie tej samej klatki dwa razy rozbujaloby go.
                if self._new_det:
                    self._new_det = False
                    if abs(ey) >= self.gimbal_deadzone:
                        self._set_gimbal(
                            self.pitch_deg - ey * (self.vfov_deg / 2.0) * self.damping)

                yaw_rate = clamp(self.kp_yaw * ex,
                                 -self.max_yaw_rate, self.max_yaw_rate)
                self.send_vectors(0.0, 0.0, self._alt_hold_vz(), yaw_rate)

                # Gimbal na koncu zakresu NIE domknie juz uchybu pionowego.
                # Przy pitch_max -30 i 50 m najdalszy cel, na ktory da sie
                # wycelowac, lezy 87 m stad; cel ze 107 m wymagalby -25 st.
                # Czekanie na ey w tolerancji jest wtedy czekaniem na coś, co
                # nie moze nastapic — a od tego jest wlasnie PODLOT, w trakcie
                # ktorego cel sam schodzi w dol kadru i gimbal go dogania.
                # (Zmierzone 2026-09-07: CELUJ palil 15 s przy ex=-0.001,
                # ey=-0.184 i porzucal PRAWDZIWY, potwierdzony namiot.)
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

        over_target=True (sciezka B): dron juz stoi nad celem, gimbal w pionie.
        Kontroler wejdzie od razu w HOVER, bo pitch <= pitch_hover_thr, wiec
        podlotu faktycznie nie ma — zostaje sama korekta w metrach. Gimbal ma
        przy zgubieniu ZOSTAC w pionie: cel jest pod nami, a odstawienie go na
        kat przeszukiwania wyrzucilo by cel poza kadr na dobre.

        over_target=False (sciezka A): cel jest przed dronem, wiec przy
        zgubieniu gimbal ma wrocic na kat przeszukiwania — tam ma najwieksza
        szanse go odzyskac. Po pierwszym wejsciu w HOVER przelaczamy sie na
        zachowanie jak wyzej.

        Zwraca True TYLKO przy udanym wycentrowaniu.
        """
        self.search_gimbal_on_lost = not over_target
        # fresh=False: nie ruszamy gimbala ani okna detekcji — cel wlasnie
        # zostal potwierdzony i przestawienie czegokolwiek zgubiloby go.
        self.run_mission(fresh=False)
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

                # Cel zniknal na dobre: ciagly SEARCH dluzej niz
                # center_lost_timeout. Chwilowa utrata konczy sie w ulamku
                # sekundy, wiec to nie jest falszywy alarm.
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

                if now - t0 > self.approach_timeout:
                    self.get_logger().error(
                        f"TIMEOUT centrowania {self.approach_timeout:.0f}s "
                        f"(stan {self.state.name})")
                    return False
        finally:
            self.stop_mission()
            self.search_gimbal_on_lost = True
        return False

    # ═══════════════════════════════════════════════════════════
    #  SCIEZKA B — mam adres
    # ═══════════════════════════════════════════════════════════

    def deliver(self, cid, lat, lon, src, n_obs) -> bool:
        """Dolot na waypoint i zrzut. Zawsze konczy sie zrzutem.

        Detektor MILCZY przez cala droge (action_interrupt=None). Adres to
        klaster z setek obserwacji albo klik operatora; okno M z N zlapane
        w locie jest przy tym slaba przeslanka, a zatrzymanie sie dla krzaka
        po drodze oznaczaloby zrzut ladunku TEJ klasy obok niego i porzucenie
        adresu, ktory ktos juz zweryfikowal.
        """
        name, _key, topic = self.klasy[cid]
        self.set_detection_topic(topic)
        self.action_interrupt = None
        self.get_logger().info(
            f"╔══ {name} ══ DOWIEZ: {self._dist_to(lat, lon):.0f} m stad "
            f"(zrodlo={src}, obs={n_obs}), detektor milczy w drodze")

        self._gimbal(self.pitch_transit)
        self.send_goto_global(lat, lon, self.target_alt)

        # Nad waypointem cel ma byc POD nami — gimbal w pion, inaczej okno
        # akwizycji nie mialoby szans.
        self._gimbal(self.pitch_min)
        self._spin(1.0)
        blad = self._dist_to(lat, lon)
        self.get_logger().info(f"nad waypointem (blad {blad:.1f} m), gimbal w pionie")

        # ── Budzet 20 s: od tej chwili do decyzji ────────────────────
        t0 = time.time()
        zostalo = lambda: max(0.0, self.wp_budget - (time.time() - t0))

        widoczny = self.wait_acquire(timeout=min(self.wp_acquire, zostalo()))

        # Pytanie zadawane RAZ. Nawet jesli cel zniknie i wroci, nie pytamy
        # drugi raz — to jest zabezpieczenie przed petla nad jednym punktem.
        if widoczny:
            monit = ("Widze cel w kadrze.\n"
                     "[SPACJA] = wycentruj na nim i zrzuc     "
                     "[nic] = zrzut na wspolrzedne waypointu")
        else:
            monit = ("NIE widze celu w kadrze.\n"
                     "[SPACJA] = nic nie zmieni, i tak zrzucam na wspolrzedne     "
                     "[nic] = zrzut na wspolrzedne")
        czas = min(self.confirm_timeout, zostalo())
        approved = False
        if czas > 0.5:
            approved = self.wait_confirm(
                f"=== {name} (zrodlo: {src}, obs={n_obs}) ===\n{monit}   "
                f"({czas:.0f}s)", timeout=czas)
        else:
            self.get_logger().warn(
                f"{name}: budzet {self.wp_budget:.0f}s wyczerpany przed pytaniem")

        if approved and widoczny:
            if self.approach_and_center(over_target=True):
                self.get_logger().info(f"{name}: ZRZUT NAD WYCENTROWANYM CELEM")
                self.drop(cid)
                return True
            self.get_logger().warn(
                f"{name}: centrowanie sie nie udalo — schodze na wspolrzedne")

        # Sciezka domyslna. Wracamy nad sam waypoint, bo centrowanie moglo nas
        # z niego zsunac, a adres jest tym, w co naprawde wierzymy.
        self.get_logger().warn(f"{name}: ZRZUT NA WSPOLRZEDNE")
        self.action_interrupt = None
        self.send_goto_global(lat, lon, self.target_alt)
        self._spin(1.0)
        self.drop(cid)
        return True

    # ═══════════════════════════════════════════════════════════
    #  SCIEZKA A — skan
    # ═══════════════════════════════════════════════════════════

    def _watch_scan(self) -> bool:
        """Haczyk dla akcji goto: czy watcher wlasnie potwierdzil cel.
        Wolany z petli _send_action co ~0.2 s; sprawdzenie jest darmowe."""
        if self.watcher is not None and self.watcher.confirmed():
            self._scan_hit = True
            self.get_logger().info(
                f"DETEKTOR w przelocie: {self.watcher.hits}/{self.watcher.window} "
                f"klatek, ID={self.watcher.track_id}"
                f"{self._opis_odleglosci()} — przerywam dolot")
            return True
        return False

    def _opis_odleglosci(self):
        """Jak daleko lezy wlasnie potwierdzony cel — to jest POMIAR zasiegu.

        Bez tego log mowi tylko "wykryty" i nie da sie z niego odczytac, czy
        detektor siega 60 czy 110 m, a od tej liczby zalezy, ile punktow skanu
        potrzebuje pole (docs/misja.md sekcja 5).
        """
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

        Okno zerujemy PO dojechaniu gimbala, nie przed — inaczej klatki
        zrobione w trakcie przejazdu (kadr rozmazany po terenie, ktorego
        komorka nie dotyczy) liczylyby sie do niej.
        """
        self._gimbal(pitch)
        self._spin(self.scan_settle)
        self.watcher.reset()
        end = time.time() + self.scan_dwell
        while rclpy.ok() and not self._abort and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            # Zerowy wektor z korekta pionu: petla kontrolera jest w skanie
            # wylaczona, wiec bez tego nikt nie trzyma wysokosci.
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

        NIE uzywamy akcji Set_yaw: jej petla w drone_handler porownuje kat
        znormalizowany do [0,2pi) z katem z [-pi,pi], wiec przy ujemnym
        namiarze warunek konca nigdy sie nie spelnia i akcja wisi do timeoutu.
        Wymaga wlaczonego sterowania predkosciowego (robi to scan_at_point).
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
                if self._abort:
                    return False
                # Wzmocnienie 2.0 nasyca sie juz przy 30 st. uchybu, wiec
                # wiekszosc obrotu idzie z pelna scan_yaw_rate, a hamowanie
                # zaczyna sie dopiero na koncu.
                rate = clamp(2.0 * err, -self.scan_yaw_rate, self.scan_yaw_rate)
                self.send_vectors(0.0, 0.0, self._alt_hold_vz(), rate)
        finally:
            self.send_vectors(0.0, 0.0, 0.0, 0.0)
            self._spin(0.3)
        return False

    def _rotate_by(self, delta_deg, timeout=8.0) -> bool:
        """Obrot WZGLEDNY o zadany kat."""
        return self._turn_to(self.drone_yaw + math.radians(delta_deg), timeout)

    def _build_scan_cells(self, arc_deg, heading_deg):
        """Lista komorek skanu: [(kurs_bezwzgledny albo None, pitch), ...].

        Liczona RAZ na punkt i trzymana w KURSACH BEZWZGLEDNYCH — dzieki temu
        skan da sie WZNOWIC po tym, jak dron obroci sie w trakcie sprawdzania
        celu (CELUJ obraca maszyna). Kroki wzgledne tego nie potrafily: po
        powrocie ze sprawdzenia kolejne pozycje byly liczone od nowego,
        przypadkowego kursu.

        Nadir jest komorka nr 0 i ma kurs None, bo przy -90 kadr otacza punkt
        pod dronem ze wszystkich stron — obracanie sie do niego nic nie zmienia.

        Pochylenia na przemian (boustrofedon): gimbal nigdy nie wraca na pusto,
        bo jego przejazd miedzy pozycjami yaw dzieje sie w trakcie obrotu.
        """
        n_yaw = max(1, int(round(arc_deg / self.scan_yaw_step)))
        # Pelny obrot pokrywa wszystko niezaleznie od kursu startowego, wiec
        # ustawianie sie kosztowaloby do 180 st. obrotu za darmo.
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
        """Przejdz komorki skanu od start_at. Zwraca INDEKS komorki, ktora
        zobaczyla cel, albo None gdy przeszla wszystkie i nic nie zobaczyla.

        Indeks zamiast True/False po to, zeby po nieudanym sprawdzeniu celu
        dalo sie wznowic od NASTEPNEJ komorki. Wczesniej jedna falszywka
        konczyla skan calego punktu i reszta terenu nie byla ogladana wcale.
        """
        # Sterowanie predkosciowe wlaczamy RAZ na caly przebieg. Kazde
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

    def handle_scan_target(self, w_locie=False) -> bool:
        """Cel zobaczony w skanie albo w przelocie — sciezka A z sekcji 2.

        STOP jest konieczny, bo przerwanie akcji NIE zatrzymuje drona:
        ArduPilot w GUIDED trzyma ostatni zadany punkt. W samym skanie dron
        juz stoi i to wyjdzie na to samo, ale w przelocie nie.

        SPRAWDZ na stojaco idzie PELNYM oknem kontrolera (4 z 8), a nie
        czulszym oknem skanu — skan ma nie przegapic, sprawdzenie ma sie nie
        dac oszukac.
        """
        name, _key, topic = self.klasy[TENT]
        self.get_logger().info(f"╔══ {name} ══ STOP i sprawdzenie na stojaco")

        self.action_interrupt = None
        self.send_goto_global(self.global_lat, self.global_lon, self.target_alt)
        self._spin(self.brake_settle_time)

        self.set_detection_topic(topic)
        if not self.wait_acquire(timeout=self.reconfirm_timeout):
            self.watcher.mute(self.det_cooldown if w_locie else self.scan_det_cooldown)
            self.get_logger().warn(
                f"{name}: na stojaco cel sie nie potwierdzil — falszywka, "
                f"cisza {self.det_cooldown if w_locie else self.scan_det_cooldown:.0f}s")
            return False

        if not self.aim_at_target():
            self.watcher.mute(self.det_cooldown if w_locie else self.scan_det_cooldown)
            return False

        # SPACJA bramkuje PODLOT. Gdy operatora NIE MA, pytanie poszloby
        # w prozni — wtedy decyduje sam detektor, ale zrzut idzie WYLACZNIE
        # po udanym wycentrowaniu. Nie ma tu adresu, wiec nie ma czego zrzucic
        # w ciemno, a cel znikajacy w centrowaniu to prawie na pewno falszywka.
        if self.operator_watching():
            approved = self.wait_confirm(
                f"=== {name} (znaleziony skanem) ===\n"
                f"[SPACJA] = podlec i zrzuc     "
                f"[nic] = to nie ten cel, skanuje dalej   "
                f"({self.confirm_timeout:.0f}s)")
            if not approved:
                self.watcher.mute(self.det_cooldown if w_locie else self.scan_det_cooldown)
                self.get_logger().warn(
                    f"{name}: operator NIE potwierdzil — wracam do skanu")
                return False
        else:
            self.get_logger().warn(
                f"{name}: operatora nie ma — decyduje detektor, zrzut tylko "
                f"po udanym wycentrowaniu")

        if not self.approach_and_center(over_target=False):
            self.watcher.mute(self.det_cooldown if w_locie else self.scan_det_cooldown)
            self._gimbal(self.pitch_transit)
            self.get_logger().warn(
                f"{name}: cel zniknal w trakcie centrowania — falszywka, "
                f"NIE zrzucam (cisza {self.det_cooldown:.0f}s)")
            return False

        self.get_logger().info(f"{name}: ZRZUT NAD WYCENTROWANYM CELEM")
        self.drop(TENT)
        return True

    def search_tent(self) -> bool:
        """Trasa skanu: WP1 -> WP2 -> (powrot na WP1). Detektor patrzy
        rowniez w przelocie, bo tu nie ma lepszego adresu niz to, co widac."""
        name, _key, topic = self.klasy[TENT]
        pts = self._load_waypoints(self._resolve_asset(self.scan_waypoints))
        if not pts:
            return False
        if self.scan_return_to_first and len(pts) > 1:
            pts = pts + [pts[0]]

        self.set_detection_topic(topic)
        self.watcher = ScanWatcher(self, topic, self.scan_m, self.scan_n)
        self._deadline = time.time() + self.scan_timeout
        self.get_logger().info(
            f"{name}: SKAN — {len(pts)} punktow, budzet {self.scan_timeout:.0f}s")

        try:
            for i, (wlat, wlon) in enumerate(pts):
                if self._abort or time.time() > self._deadline:
                    self.get_logger().warn("skan: koniec budzetu czasu")
                    return False
                self.get_logger().info(
                    f"=== punkt {i + 1}/{len(pts)} === {self._dist_to(wlat, wlon):.0f} m stad")

                # Dolot z obserwacja. Przerwany w polowie -> po obsluzeniu celu
                # wracamy na kurs do TEGO SAMEGO punktu, zeby nie zgubic
                # kawalka trasy.
                while not self._abort and time.time() < self._deadline:
                    if self._dist_to(wlat, wlon) <= self.arrive_tol:
                        break
                    self._gimbal(self.pitch_transit)
                    self.watcher.set_gate(self.transit_m, self.transit_n)
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
                    if self.handle_scan_target(w_locie=True):
                        return True

                # Skan z postoju. Luk per punkt; powrot na WP1 dostaje ten sam
                # luk co WP1.
                idx = i if i < len(pts) - 1 or not self.scan_return_to_first else 0
                arc = (self.scan_arc_deg[idx] if idx < len(self.scan_arc_deg)
                       else self.scan_arc_deg[-1])
                self.watcher.set_gate(self.scan_m, self.scan_n)
                # Cisza z PRZELOTU nie obowiazuje w skanie. Inaczej falszywka
                # zlapana w drodze oslepia poczatek skanu na nowym punkcie —
                # 2026-09-07 wyciela 4 z 6 pozycji yaw, a namiot byl na 5/6.
                self.watcher.mute_until = 0.0
                self.watcher.reset()
                hdg = (self.scan_heading_deg[idx]
                       if idx < len(self.scan_heading_deg)
                       else self.scan_heading_deg[-1])
                cells = self._build_scan_cells(arc, hdg)
                od = 0
                while od < len(cells):
                    trafil = self.scan_at_point(cells, od)
                    if trafil is None:
                        break                      # przeszedl wszystko, nic
                    if self.handle_scan_target():
                        return True
                    # Falszywka: wznawiamy od NASTEPNEJ komorki, zamiast
                    # porzucac reszte punktu.
                    od = trafil + 1

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
        if self.finish_action == 'rtl':
            self.get_logger().info("=== POWROT: RTL ===")
            self.rtl()
        elif self.finish_action == 'land':
            self.land()
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
        # czlowiek 8 px i praktycznie nie. Robimy najpierw to, co na pewno
        # wyjdzie — nie sortujemy po score.
        for cid in (TENT, PERSON):
            if self._abort:
                break
            name = self.klasy[cid][0]
            if cid in targets:
                lat, lon, src, n_obs = targets[cid]
                if self.deliver(cid, lat, lon, src, n_obs):
                    done.add(cid)
            elif cid == TENT:
                if self.search_tent():
                    done.add(cid)
            else:
                self.get_logger().warn(
                    f"{name}: brak adresu — nie szukam. Na 50 m czlowiek ma "
                    f"8 px, wiec skan i tak by go nie znalazl. Ladunek wraca.")

        self._finish(done)
        return True


def main(args=None):
    # SignalHandlerOptions.NO — Ctrl+C obsluguje sama misja, zeby zdazyla
    # jeszcze wyslac RTL zanim kontekst ROS padnie.
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
