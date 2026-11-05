#!/usr/bin/env python3
"""
suas_grid_mission — UPROSZCZONA misja SUAS: grid z Mission Plannera, dwa cele,
dwa zrzuty, RTL.

Czym rozni sie od suas_full_mission
    * BEZ geolokatora — nie ma targets.json, nie ma kandydatow, nie ma score.
      Jedyne zrodlo wiedzy o celu to DETEKTOR, tu i teraz, w kadrze.
    * BEZ operatora — nie ma spacji, nie ma /operator_online, nie ma GUI.
      Cel potwierdza wylacznie okno M z N klatek (det_confirm_frames z
      det_window_frames), to samo, ktore przepuszcza SEARCH -> APPROACH.
    * BEZ liczonego gridu i bez spirali — trase rysujesz w Mission Plannerze
      i zapisujesz jako .waypoints. Misja leci dokladnie po niej.

PRZEBIEG
    1. FAZA_ORTO   dron leci trasa ortofoto w AUTO, prowadzi Mission Planner.
                   Ta misja NIC nie wysyla — czeka. Dopoki jest AUTO, ROS jest
                   fizycznie bezsilny, bo ArduPilot ignoruje w tym trybie
                   setpointy predkosci.
    2. PRZEJECIE   operator przelacza AUTO -> GUIDED. To jedyny sygnal, jakiego
                   misja potrzebuje. Wtedy: zejscie na target_alt (50 m),
                   gimbal na pitch_search (-75 st.).
    3. GRID        lot po punktach z pliku .waypoints. Przez CALY czas obie
                   klasy sa obserwowane rownolegle, kazda wlasnym oknem M z N.
    4. CEL         okno domkniete = przerywamy dolot TAM, GDZIE JESTESMY:
                   hamowanie -> sprawdzenie na stojaco -> APPROACH -> HOVER
                   (to juz robi suas_flight_controller) -> zrzut ladunku
                   tej klasy -> powrot na kurs do przerwanego punktu.
    5. RTL         gdy oba ladunki poszly, gdy grid sie skonczyl albo gdy
                   skonczyl sie budzet czasu.

DLACZEGO OBSERWACJA DWOCH KLAS NARAZ
Kontroler lotu ma jedno okno detekcji i jeden topic naraz (set_detection_topic),
bo w fazie podejscia sledzi konkretny obiekt. W gridzie tak sie nie da: namiot
i czlowiek moga wejsc w kadr w dowolnej kolejnosci, a przelatywanie trasy dwa
razy (raz na klase) kosztuje dwa razy tyle czasu. Dlatego misja trzyma WLASNE,
lekkie okna M z N (ClassWatcher) — po jednym na klase — i dopiero gdy ktores sie
domknie, przepina kontroler na ten topic i oddaje mu sterowanie.

DLACZEGO GIMBAL ZOSTAJE NA -75 W GRIDZIE
Nadir (-90) widzi tylko to, co jest DOKLADNIE pod dronem, wiec cel wpada w kadr
na sekunde i wypada. -75 przy 50 m przesuwa srodek kadru 13 m przed drona, wiec
cel jest widoczny dluzej i okno M z N ma sie z czego zapelnic, zanim dron nad
nim przeleci. Pion wlacza sie sam, dopiero w HOVER — robi to kontroler.

BEZ POTWIERDZENIA CZLOWIEKA = OSTROZNIEJ, NIE ODWAZNIEJ
Skoro nikt nie ogląda obrazu, jedyna bramka przed zrzutem to detektor. Dlatego
zrzut idzie WYLACZNIE po udanym wycentrowaniu: cel, ktory znika w trakcie
centrowania, jest falszywka (2026-09-02 grid potwierdzil w ten sposob drzewo)
i wtedy ladunek zostaje na pokladzie, a klasa dostaje det_cooldown ciszy, zeby
ta sama falszywka nie zatrzymywala drona w kolko. Zrzutu "na wspolrzedne
w ciemno" tu nie ma, bo nie ma zadnych wspolrzednych — nie ma geolokatora.

Wymaga: drone_handler + detektora (/tent_detections, /people_detections).
Geolokator NIE jest potrzebny; jesli chodzi (bo wstal z suas_bringup), misja
zdejmuje mu blokade nadiru, zeby dwoch nie pisalo na jeden silownik.
Stdin nie jest potrzebny (nie ma spacji), wiec ta misja moze isc takze z launcha.

    ros2 run drone_autonomy suas_grid_mission --ros-args \\
        --params-file ~/Dron_symulacja/src/drone_bringup/config/suas_grid_mission.yaml
"""

import math
import os
import signal
import time
from collections import deque

import rclpy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool

from drone_autonomy.suas_flight_controller import State, SuasFlightController
from drone_interfaces.msg import TentDetection

M_LAT = 111_320.0


def _m_per_deg_lon(lat):
    return M_LAT * math.cos(math.radians(lat))


class ClassWatcher:
    """Okno M z N klatek dla JEDNEJ klasy, liczone niezaleznie od kontrolera.

    Ta sama logika co _det_cb w suas_flight_controller (przerwa w klatkach
    zeruje okno, zmiana ID trackera zaczyna liczenie od nowa, track_id < 0 nie
    liczy sie jako trafienie), tylko bez sterowania — watcher niczego nie
    steruje, wylacznie odpowiada na pytanie "czy TERAZ widac ta klase".

    Progi bierze wprost z kontrolera (det_confirm_frames / det_window_frames /
    det_confirm_gap / require_same_track), zeby bylo jedno pokretlo w jednym
    miejscu — grid i podejscie potwierdzaja cel dokladnie tak samo.
    """

    def __init__(self, node, class_id, name, topic):
        self.node = node
        self.class_id = class_id
        self.name = name
        self.topic = topic
        self.done = False           # ladunek tej klasy juz poszedl
        self.mute_until = 0.0       # cisza po falszywce
        self._win = deque(maxlen=node.det_window_frames)
        self._prev_frame = 0.0
        self._track = -1
        self.sub = node.create_subscription(
            TentDetection, topic, self._cb, 10)

    # ── odbior klatek ────────────────────────────────────────────
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
                # Brak sciezki = podpis migoczacej falszywki. Prawdziwy cel
                # dostaje ID od trackera od razu.
                good = False
            elif self._track < 0:
                self._track = msg.track_id
            elif msg.track_id != self._track:
                self._win.clear()
                self._track = msg.track_id
        self._win.append(1 if good else 0)

    # ── odpytywanie ──────────────────────────────────────────────
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
        if self.done or time.time() < self.mute_until:
            return False
        return self.hits >= self.node.det_confirm_frames

    def reset(self):
        self._win.clear()
        self._track = -1

    def mute(self, seconds):
        """Nie sluchaj tej klasy przez chwile. Okno tez czyscimy, zeby po
        odcisze zaczac od zera, a nie od trafien sprzed falszywki."""
        self.mute_until = time.time() + seconds
        self.reset()


class SuasGridMission(SuasFlightController):

    def __init__(self):
        super().__init__('suas_grid_mission')

        # ── Trasa ────────────────────────────────────────────────
        # Plik .waypoints zapisany z Mission Plannera. Bez niego misja nie ma
        # dokad leciec i konczy sie od razu — celowo nie ma tu awaryjnego
        # "policz sobie grid sam", bo obszar zawodow rysuje czlowiek na mapie.
        self.declare_parameter('waypoints_file', '')
        # Ile razy przelecec cala trase, jesli po pierwszym razie zostal
        # niezrzucony ladunek. Kazdy kolejny przelot idzie w odwrotna strone,
        # zeby nie wracac pusto na poczatek trasy.
        self.declare_parameter('grid_passes', 2)
        # Twardy budzet czasu na CALE przeszukiwanie (wszystkie przeloty).
        self.declare_parameter('grid_timeout', 900.0)
        # Ile stoimy nad kazdym punktem trasy. Dron wyhamowuje, obraz przestaje
        # byc rozmyty ruchem i okno M z N ma szanse sie domknac na celu, ktory
        # w locie mignal tylko w rogu kadru.
        self.declare_parameter('point_dwell', 2.0)

        # Predkosc przelotu miedzy punktami gridu [m/s]. 0 = nie ruszaj, czyli
        # zostaje WPNAV_SPEED z ArduPilota. Dotyczy TYLKO lotu po trasie
        # (goto_global); podejscie do celu chodzi na velocity control i tam
        # predkosc wyznacza kp_vx z max_vel.
        # Wyzej = szybszy grid, ale i wieksze rozmycie ruchem: przy 8 m/s cel
        # przelatuje przez kadr o 63 m w 8 s, wiec okno M z N (4 z 8 klatek,
        # detektor ~5-12 Hz) zdazy sie domknac. Ponizej ok. 10 m/s to bezpieczne.
        self.declare_parameter('cruise_speed', 0.0)

        # ── Przejecie lotu ───────────────────────────────────────
        # Ile czekamy na przelaczenie AUTO -> GUIDED, zanim uznamy, ze cos
        # poszlo nie tak. Zawieszona misja nie moze wisiec w nieskonczonosc.
        self.declare_parameter('takeover_timeout', 1200.0)
        # TRYB TESTOWY. W prawdziwej misji dron jest juz w powietrzu po przelocie
        # ortofoto i czeka tylko na GUIDED. W symulacji nie ma trasy AUTO,
        # a start w GUIDED znaczylby, ze misja przejmuje drona stojacego na
        # ziemi. auto_takeoff:=true kaze jej samej uzbroic i wzniesc sie.
        self.declare_parameter('auto_takeoff', False)

        # ── Cel zauwazony w locie ────────────────────────────────
        # Ile czekamy na hamowanie po przerwaniu dolotu. Przy 5 m/s to ok. 2 s.
        self.declare_parameter('brake_settle_time', 3.0)
        # Ile czekamy na potwierdzenie NA STOJACO. To jest filtr rozmycia
        # ruchem: cel widziany w locie, ktorego nie widac po zatrzymaniu,
        # jest podejrzany. 3 s to ~15 klatek detektora, czyli okno M z N
        # napelnia sie dwukrotnie.
        self.declare_parameter('reconfirm_timeout', 3.0)
        # Po nieudanym sprawdzeniu (na stojaco albo w centrowaniu) tyle sekund
        # nie sluchamy TEJ klasy. Bez tego ta sama falszywka zatrzymywalaby
        # drona w kolko: wracamy na kurs, okno napelnia sie z tego samego
        # krzaka i znowu stoimy.
        self.declare_parameter('det_cooldown', 20.0)

        # ── Centrowanie nad celem ────────────────────────────────
        self.declare_parameter('approach_timeout', 90.0)
        # Ile ciaglego SEARCH w trakcie centrowania znaczy "cel zniknal na
        # dobre". Prawdziwy cel wraca w ulamku sekundy, wiec to nie jest
        # falszywy alarm — i nie ma po co czekac pelnego approach_timeout.
        self.declare_parameter('center_lost_timeout', 15.0)
        self.declare_parameter('center_tol_m', 1.5)
        self.declare_parameter('hover_hold_time', 3.0)

        # ── Klasy celow ──────────────────────────────────────────
        # POZYCJA NA LISCIE = class_id = numer ladunku (drop_pwm_by_class).
        # Nazwy sa tylko do logow, topici musza pasowac do detektora.
        self.declare_parameter('target_names', ['NAMIOT', 'CZLOWIEK'])
        self.declare_parameter('target_topics',
                               ['/tent_detections', '/people_detections'])

        self.declare_parameter('finish_action', 'rtl')

        p = self.get_parameter
        self.waypoints_file = str(p('waypoints_file').value).strip()
        self.grid_passes = max(1, int(p('grid_passes').value))
        self.grid_timeout = p('grid_timeout').value
        self.point_dwell = p('point_dwell').value
        self.cruise_speed = p('cruise_speed').value
        self.takeover_timeout = p('takeover_timeout').value
        self.auto_takeoff = p('auto_takeoff').value
        self.brake_settle_time = p('brake_settle_time').value
        self.reconfirm_timeout = p('reconfirm_timeout').value
        self.det_cooldown = p('det_cooldown').value
        self.approach_timeout = p('approach_timeout').value
        self.center_lost_timeout = p('center_lost_timeout').value
        self.center_tol_m = p('center_tol_m').value
        self.hover_hold_time = p('hover_hold_time').value
        self.target_names = list(p('target_names').value)
        self.target_topics = list(p('target_topics').value)
        self.finish_action = str(p('finish_action').value).lower()

        if len(self.target_names) != len(self.target_topics):
            self.get_logger().error(
                f"target_names ({len(self.target_names)}) i target_topics "
                f"({len(self.target_topics)}) maja rozna dlugosc — biore krotsza")
            n = min(len(self.target_names), len(self.target_topics))
            self.target_names = self.target_names[:n]
            self.target_topics = self.target_topics[:n]

        # "Wycentrowany" nie moze byc ciasniejszy niz martwa strefa regulatora,
        # bo wtedy dron przestaje korygowac, zanim uzna cel za wycentrowany.
        if self.center_tol_m < self.hover_deadzone_m:
            self.get_logger().warn(
                f"center_tol_m({self.center_tol_m}) < hover_deadzone_m"
                f"({self.hover_deadzone_m}) — podnosze")
            self.center_tol_m = self.hover_deadzone_m

        self._abort = False
        self.home = None            # (lat, lon) zapamietane przy przejeciu
        self.watchers = {}          # class_id -> ClassWatcher
        self._hit_cid = None        # klasa, ktora przerwala dolot
        self._deadline = 0.0        # koniec budzetu czasu na grid

        # Geolokatora nie uzywamy, ale jesli chodzi (wstal z suas_bringup),
        # to co 2 s stawia gimbal w pionie. Dwoch piszacych na jeden silownik
        # znaczy gimbal szarpany w trakcie podejscia — wiec zdejmujemy blokade.
        self._nadir_pub = self.create_publisher(Bool, '/geolocator/lock_nadir', 10)

        self.get_logger().info(
            f"suas_grid_mission: alt={self.target_alt} m | gimbal w gridzie "
            f"{self.pitch_search:+.0f} st. | potwierdzenie "
            f"{self.det_confirm_frames} z {self.det_window_frames} klatek "
            f"| trasa: {self.waypoints_file or 'BRAK!'}")

    # ═══════════════════════════════════════════════════════════
    #  Pomocnicze
    # ═══════════════════════════════════════════════════════════

    def _install_signals(self):
        """Zaden z tych sygnalow nie zabija kontekstu ROS od razu — RTL musi
        jeszcze przejsc.

        SIGHUP jest tu rownie wazny co SIGINT: dostajemy go, gdy zerwie sie
        terminal (SSH, Tailscale). Bez obslugi proces ginie natychmiast, bez
        stop_mission() i bez RTL, a dron zostaje wiszacy w GUIDED, bo
        drone_handler ma dalej wlaczony velocity control i nikt nie wysyla mu
        wektorow. Z obsluga — wraca do domu.

        To NIE zwalnia z uruchamiania misji pod tmux (patrz docs/misja_real.md):
        tmux sprawia, ze SIGHUP w ogole nie dolatuje.
        """
        def handler(signum, frame):
            if self._abort:
                raise KeyboardInterrupt
            self._abort = True          # widza to tez bramki w kontrolerze
            self._alarm = True
            powod = {signal.SIGINT: "Ctrl+C",
                     signal.SIGTERM: "SIGTERM",
                     signal.SIGHUP: "SIGHUP (zerwany terminal)"}.get(
                         signum, f"sygnal {signum}")
            self.get_logger().warn(f"{powod} — przerywam i wracam "
                                   "(kolejny sygnal = twarde wyjscie)")
        signal.signal(signal.SIGINT, handler)
        signal.signal(signal.SIGTERM, handler)
        signal.signal(signal.SIGHUP, handler)

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

    def _gimbal_search(self, force=False):
        """Gimbal do pozycji przeszukiwania (-75 st.).

        force=True wysyla bezwarunkowo. Potrzebne na starcie: pitch_deg jest
        zainicjowany wlasnie na pitch_search, wiec _set_gimbal uznalby, ze nic
        sie nie zmienilo, i serwo nigdy nie dostaloby komendy.
        """
        if force:
            self.pitch_deg = self.pitch_search
            self.set_gimbal_pitch(self.pitch_deg)
        else:
            self._set_gimbal(self.pitch_search)

    # ═══════════════════════════════════════════════════════════
    #  Obserwacja obu klas naraz
    # ═══════════════════════════════════════════════════════════

    def _make_watchers(self):
        for cid, (name, topic) in enumerate(
                zip(self.target_names, self.target_topics)):
            self.watchers[cid] = ClassWatcher(self, cid, name, topic)
            self.get_logger().info(
                f"obserwuje klase {cid} ({name}) na {topic}")

    def _reset_watchers(self):
        for w in self.watchers.values():
            w.reset()

    def _all_done(self) -> bool:
        return all(w.done for w in self.watchers.values())

    def _watch_classes(self) -> bool:
        """Haczyk dla akcji goto: czy ktoras klasa wlasnie sie potwierdzila.

        Wolany z petli _send_action co ~0.2 s. Sprawdzenie jest darmowe
        (liczniki w pamieci), wiec moze isc przy kazdym obrocie.
        """
        for w in self.watchers.values():
            if w.confirmed():
                self._hit_cid = w.class_id
                self.get_logger().info(
                    f"DETEKTOR: {w.name} potwierdzony w locie "
                    f"({w.hits}/{w.window} klatek, ID={w.track_id}) "
                    f"— przerywam dolot")
                return True
        return False

    def _spin_watch(self, seconds):
        """Stoj i patrz. Zwraca class_id potwierdzonej klasy albo None."""
        end = time.time() + seconds
        while rclpy.ok() and not self._abort and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            for w in self.watchers.values():
                if w.confirmed():
                    self.get_logger().info(
                        f"DETEKTOR: {w.name} potwierdzony na punkcie "
                        f"({w.hits}/{w.window} klatek, ID={w.track_id})")
                    return w.class_id
        return None

    # ═══════════════════════════════════════════════════════════
    #  Fazy
    # ═══════════════════════════════════════════════════════════

    def wait_for_guided(self) -> bool:
        """FAZA_ORTO: czekaj, az operator przelaczy AUTO -> GUIDED.

        Nic tu nie wysylamy — w AUTO trase prowadzi Mission Planner.
        """
        self.get_logger().info(
            "=== FAZA_ORTO === czekam na przelaczenie AUTO -> GUIDED "
            "(nic nie wysylam)")
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
        """Zejscie na zadana wysokosc nad biezaca pozycja — I CZEKANIE na nia.

        Akcja goto_global konczy sie, gdy odleglosc POZIOMA spadnie ponizej
        2 m. Przy tej samej lat/lon to natychmiast, wiec bez wlasnego czekania
        "zejscie" trwaloby 0 s, a dron schodzilby dopiero w trakcie pierwszego
        galsu — z celami ogladanymi z przypadkowej wysokosci.
        """
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
            f"zejscie: po {timeout:.0f}s jestem na {self.altitude:.1f} m "
            f"— lece dalej")
        return False

    def _load_waypoints(self, path):
        """Wczytaj trase. Zwraca [(lat, lon), ...] albo [].

        Trzy formaty, zeby nie trzeba bylo niczego konwertowac:

        1. .waypoints z Mission Plannera (naglowek "QGC WPL 110").
           Pola rozdzielone tabulatorem, lat w kolumnie 8, lon w 9.
           Wiersz 0 to HOME i jest pomijany, tak samo komendy bez wspolrzednych
           (lat i lon rowne 0) — czyli TAKEOFF, RTL, DO_*.
           To jest sciezka dla realu: rysujesz siatke w MP, zapisujesz plik.
           WYSOKOSC z pliku jest IGNOROWANA — lecimy na target_alt.

        2. "lat, lon" w kazdej linii — gdy masz gotowe wspolrzedne.

        3. Linia "OFFSETS", a po niej "wschod, polnoc" w METRACH wzgledem
           PUNKTU PRZEJECIA LOTU. Sciezka do testow w symulacji: nie trzeba
           przeliczac stopni, wpisujesz po prostu "-75, 125".

        Linie puste i zaczynajace sie od # sa pomijane.
        """
        try:
            raw = open(os.path.expanduser(path)).read().splitlines()
        except Exception as e:
            self.get_logger().error(f"nie moge wczytac {path}: {e}")
            return []

        qgc = any(l.startswith('QGC WPL') for l in raw[:1])
        offsets = any(l.strip().upper() == 'OFFSETS' for l in raw)
        pts = []
        for line in raw:
            line = line.split('#')[0].strip()
            if not line or line.startswith('QGC') or line.upper() == 'OFFSETS':
                continue
            if qgc:
                f = line.split('\t')
                if len(f) < 10 or f[0] == '0':          # wiersz 0 = HOME
                    continue
                try:
                    lat, lon = float(f[8]), float(f[9])
                except ValueError:
                    continue
                if abs(lat) < 1e-7 and abs(lon) < 1e-7:  # komenda bez pozycji
                    continue
                pts.append((lat, lon))
            else:
                try:
                    a, b = (float(x) for x in line.replace(',', ' ').split()[:2])
                except ValueError:
                    continue
                # OFFSETS: a=wschod, b=polnoc w metrach -> _offset_gps(dn, de)
                pts.append(self._offset_gps(self.home[0], self.home[1], b, a)
                           if offsets else (a, b))
        if not pts:
            self.get_logger().error(f"{path}: nie znalazlem zadnych punktow")
        return pts

    def goto_grid(self, lat, lon) -> bool:
        """Dolot do punktu trasy z wlaczonym podgladem obu klas.

        Gimbal ZOSTAJE na pitch_search — inaczej niz w suas_full_mission, gdzie
        po dolocie stawia sie pion. Tam waypoint BYL celem, wiec cel byl pod
        dronem; tu waypoint to tylko rog galsu, a cel moze byc gdziekolwiek
        wzdluz trasy, wiec kadr ma patrzec do przodu.
        """
        d = self._dist_to(lat, lon)
        self.get_logger().info(f"dolot: {d:.0f} m")
        self._gimbal_search()
        self.action_interrupt = self._watch_classes
        try:
            return self.send_goto_global(lat, lon, self.target_alt)
        finally:
            self.action_interrupt = None

    def approach_and_center(self) -> bool:
        """Oddaj sterowanie kontrolerowi: APPROACH -> HOVER, az cel pod dronem.

        Zwraca True dopiero, gdy cel jest blizej niz center_tol_m przez
        hover_hold_time. Wszystko inne (timeout, cel zniknal) = False, czyli
        BEZ ZRZUTU — bez operatora nie mamy drugiego zdania, wiec niepewny cel
        ma kosztowac przelot, a nie ladunek.
        """
        # W drodze do celu gimbal ma przy zgubieniu wracac na -75: cel jest
        # PRZED dronem, wiec ta pozycja daje najwieksza szanse odzyskania go.
        # Nad celem jest odwrotnie (patrz nizej).
        self.search_gimbal_on_lost = True
        # fresh=False: nie ruszamy gimbala ani okna detekcji — cel wlasnie
        # zostal potwierdzony i przestawienie czegokolwiek zgubiloby go.
        self.run_mission(fresh=False)
        t0 = time.time()
        hold_since = None
        search_since = None
        hovered = False
        try:
            while rclpy.ok() and not self._abort:
                rclpy.spin_once(self, timeout_sec=0.05)
                now = time.time()

                # Pierwsze wejscie w HOVER = jestesmy nad celem. Od tej chwili
                # cel jest w nadirze, a -75 odchylilby os kamery o 15 st. przy
                # polowie FOV 32 st. — punkt pod dronem zostaje w kadrze, ale
                # przy kazdym przechyle ramy wypada. Gimbal ma wiec zostac
                # w pionie i czekac na powrot celu.
                if self.state == State.HOVER and not hovered:
                    hovered = True
                    self.search_gimbal_on_lost = False
                    self.get_logger().info(
                        "nad celem — gimbal zostaje w pionie takze przy "
                        "chwilowej utracie")

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

    def handle_detection(self, class_id) -> bool:
        """Cala obsluga jednego celu: hamowanie -> sprawdzenie -> zrzut.

        Zwraca True tylko wtedy, gdy ladunek naprawde poszedl.

        Zatrzymanie jest konieczne, bo przerwanie akcji NIE zatrzymuje drona:
        cel zostaje otwarty po stronie serwera, a ArduPilot w GUIDED trzyma
        ostatni zadany punkt. Bez tego dron lecialby dalej przez cale
        sprawdzanie, czyli przy 5 m/s uciekloby mu kilkadziesiat metrow.

        Sprawdzenie NA STOJACO jest tez filtrem: cel potwierdzony w locie,
        ktory nie potwierdza sie po zatrzymaniu, jest podejrzany — rozmycie
        ruchem robi z krzakow dziwne rzeczy. Hamowanie przenosi nas kilkanascie
        metrow za cel, ale przy -75 st. i 50 m kadr obejmuje pas przed dronem
        i za nim, wiec cel zostaje widoczny.
        """
        w = self.watchers[class_id]
        self.get_logger().info(
            f"╔══ {w.name} ══ zatrzymuje sie i sprawdzam cel na stojaco")

        # 1. Hamowanie w miejscu. Haczyk zdjety, inaczej goto na wlasna pozycje
        #    przerwaloby sie natychmiast — okno detekcji jest dalej pelne.
        self.action_interrupt = None
        self.send_goto_global(self.global_lat, self.global_lon, self.target_alt)
        self._gimbal_search()
        self._spin(self.brake_settle_time)

        # 2. Potwierdzenie oknem KONTROLERA (to samo M z N). Od tego momentu
        #    kontroler patrzy na topic tej klasy i ma swieze last_det_time,
        #    wiec run_mission(fresh=False) zastanie gotowy, sledzony cel.
        self.set_detection_topic(w.topic)
        if not self.wait_acquire(timeout=self.reconfirm_timeout):
            w.mute(self.det_cooldown)
            self._reset_watchers()
            self.get_logger().warn(
                f"{w.name}: na stojaco cel sie nie potwierdzil — wracam na kurs, "
                f"tej klasy nie slucham przez {self.det_cooldown:.0f}s")
            return False

        # 3. Podejscie i centrowanie — to juz robi suas_flight_controller.
        if not self.approach_and_center():
            w.mute(self.det_cooldown)
            self._reset_watchers()
            self._gimbal_search()
            self.get_logger().warn(
                f"{w.name}: cel zniknal w trakcie centrowania — falszywka, "
                f"NIE zrzucam (cisza {self.det_cooldown:.0f}s)")
            return False

        # 4. Zrzut. Numer ladunku = numer klasy (0 = namiot, 1 = czlowiek).
        self.get_logger().info(f"{w.name}: ZRZUT NAD WYCENTROWANYM CELEM")
        self.drop(class_id)
        w.done = True
        self._reset_watchers()
        self._gimbal_search()
        return True

    def fly_points(self, pts) -> bool:
        """Jeden przelot calej trasy. False = przerwac przeszukiwanie
        (abort, koniec czasu albo oba ladunki poszly)."""
        for i, (wlat, wlon) in enumerate(pts, 1):
            if self._abort or self._all_done():
                return False
            if time.time() > self._deadline:
                self.get_logger().warn(
                    f"koniec budzetu czasu na punkcie {i}/{len(pts)}")
                return False
            self.get_logger().info(f"punkt {i}/{len(pts)}")

            # Dolot moze byc przerwany w polowie — wtedy po obsluzeniu celu
            # wracamy na kurs do TEGO SAMEGO punktu, zeby nie zgubic kawalka
            # galsu (i terenu, ktorego jeszcze nikt nie widzial).
            while not self._abort and time.time() < self._deadline:
                self.goto_grid(wlat, wlon)
                cid, self._hit_cid = self._hit_cid, None
                if cid is None:
                    break                       # doleciilismy do punktu
                self.handle_detection(cid)
                if self._all_done():
                    return False

            # Krotki postoj nad punktem: dron wyhamowal, obraz jest ostrzejszy.
            cid = self._spin_watch(self.point_dwell)
            if cid is not None:
                self.handle_detection(cid)
                if self._all_done():
                    return False
        return True

    # ═══════════════════════════════════════════════════════════
    #  Misja
    # ═══════════════════════════════════════════════════════════

    def _finish(self):
        self.stop_mission()
        self._spin(1.0, stop_on_abort=False)
        zostaly = [w.name for w in self.watchers.values() if not w.done]
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
        self.get_logger().info("=== START: suas_grid_mission ===")
        self._install_signals()

        if not self.waypoints_file:
            self.get_logger().error(
                "waypoints_file jest pusty — nie mam dokad leciec. "
                "Podaj sciezke do pliku .waypoints z Mission Plannera.")
            return False

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
        self._nadir_pub.publish(Bool(data=False))
        self._gimbal_search(force=True)
        # Predkosc przelotu ustawiamy RAZ, po przejeciu lotu. Dziala tylko na
        # goto_global (dronekit: vehicle.groundspeed), wiec podejscia do celu
        # to nie dotyczy — tam predkosc liczy kp_vx.
        if self.cruise_speed > 0.0:
            self.set_speed(self.cruise_speed)
        self.descend(self.target_alt)

        pts = self._load_waypoints(self.waypoints_file)
        if not pts:
            self._finish()
            return False

        self._make_watchers()
        if not self.watchers:
            self.get_logger().error(
                "target_names / target_topics sa puste — nie ma czego szukac")
            self._finish()
            return False

        self._deadline = time.time() + self.grid_timeout
        for p in range(1, self.grid_passes + 1):
            if self._abort or self._all_done() or time.time() > self._deadline:
                break
            # Kazdy kolejny przelot w druga strone — inaczej po ostatnim
            # punkcie trzeba by wracac przez cala trase na jej poczatek.
            order = pts if p % 2 else list(reversed(pts))
            self.get_logger().info(
                f"=== PRZELOT {p}/{self.grid_passes} === {len(order)} punktow, "
                f"pozostalo {self._deadline - time.time():.0f}s")
            if not self.fly_points(order):
                break

        self._finish()
        return True


def main(args=None):
    # SignalHandlerOptions.NO — Ctrl+C obsluguje sama misja, zeby zdazyla
    # jeszcze wyslac RTL zanim kontekst ROS padnie.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = SuasGridMission()
    try:
        node.get_logger().info("Czekam 2s na inicjalizacje serwisow...")
        time.sleep(2.0)
        node.run()
    except KeyboardInterrupt:
        node.get_logger().warn("Twarde przerwanie — dron zostaje w GUIDED!")
    finally:
        try:
            node.stop_mission()
        except Exception:
            pass
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
