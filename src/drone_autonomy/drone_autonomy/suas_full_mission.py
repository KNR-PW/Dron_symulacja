#!/usr/bin/env python3
"""
suas_full_mission — pelna misja SUAS: dwa cele, dwa zrzuty, RTL.

PRZEBIEG
    1. FAZA_ORTO   dron leci trasa ortofoto w AUTO, prowadzi Mission Planner.
                   Ta misja NIC nie wysyla — czeka. Dopoki jest AUTO, ROS jest
                   fizycznie bezsilny, bo ArduPilot ignoruje w tym trybie
                   setpointy predkosci.
    2. PRZEJECIE   operator przelacza AUTO -> GUIDED. To jedyny sygnal, jakiego
                   misja potrzebuje. Wtedy: zejscie na drop_alt, odczyt celow.
    3. CEL 1, 2    dolot nosem do przodu -> okno akwizycji -> ewentualnie
                   spacja operatora -> centrowanie -> zrzut.
    4. RTL

TRZY SCENARIUSZE NA CEL
    A. jest waypoint i detektor potwierdzil w oknie akwizycji
       -> pytanie o SPACJE. Potwierdzone: centrowanie i zrzut nad obiektem.
          Brak reakcji: zrzut na waypoint, bez centrowania.
    B. jest waypoint, ale detektor nic nie widzi
       -> spirala wokol waypointu. Znajdzie -> jak A.
          Nie znajdzie -> zrzut na waypoint.
          (Zamiatanie gimbalem jest tu domyslnie wylaczone — patrz sweep_enabled.)
    C. nie ma waypointu
       -> grid nad zadanym obszarem. Pierwsza potwierdzona detekcja -> jak A.
          Grid bez trafienia -> RTL z ladunkiem.

DLACZEGO POTWIERDZENIE BRAMKUJE PODEJSCIE, A NIE ZRZUT
Decyzja operatora zapada wtedy, gdy ma co ocenic: widzi obiekt na podgladzie
i mowi "tak, to jest cel". Od tej chwili dron pracuje sam. Gdyby druga bramka
stala przed samym zrzutem, utrata SSH w polowie centrowania zostawilaby drona
nad celem z ladunkiem, bo nie mialby od kogo dostac zgody.

Domyslne dzialanie jest bezpieczne w obie strony: bez operatora ladunek i tak
wychodzi (na wspolrzedne zamiast nad wycentrowany obiekt), a dron nigdy nie goni
detekcji, ktorej nikt nie potwierdzil — wiec falszywka nie sciagnie go z kursu.

Wymaga: drone_handler, detektora (/tent_detections, /people_detections),
suas_geolocator (pisze targets.json). Musi isc przez `ros2 run`, a NIE z launcha
— wait_confirm czyta klawiature i potrzebuje stdin podpietego do terminala.

    ros2 run drone_autonomy suas_full_mission --ros-args \\
        --params-file ~/ros_ws/src/drone_bringup/config/suas_mission.yaml
"""

import json
import math
import os
import signal
import time

import rclpy
from rclpy.signals import SignalHandlerOptions

from drone_autonomy.suas_flight_controller import State, SuasFlightController

# Klasa -> (nazwa, klucz w targets.json, topic detekcji)
TARGETS = {
    0: ('NAMIOT', 'tent', '/tent_detections'),
    1: ('CZLOWIEK', 'people', '/people_detections'),
}
M_LAT = 111_320.0


def _m_per_deg_lon(lat):
    return M_LAT * math.cos(math.radians(lat))


class SuasFullMission(SuasFlightController):

    def __init__(self):
        super().__init__('suas_full_mission')

        self.declare_parameter('targets_json',
                               os.path.expanduser('~/suas_targets/targets.json'))
        # Ile czekamy na przelaczenie AUTO -> GUIDED, zanim uznamy, ze cos poszlo
        # nie tak i wrocimy. Zawieszona misja nie moze wisiec w nieskonczonosc.
        self.declare_parameter('takeover_timeout', 1200.0)
        self.declare_parameter('arrive_tol', 3.0)       # [m] kiedy uznajemy dolot
        self.declare_parameter('approach_timeout', 90.0)
        self.declare_parameter('hover_hold_time', 3.0)
        self.declare_parameter('center_tol_m', 1.5)
        # Zamiatanie gimbalem przy ISTNIEJACYM waypoincie — domyslnie WYLACZONE.
        # Po dolocie gimbal patrzy w pion, wiec na 50 m kadr obejmuje 63 m terenu,
        # a zmierzone bledy waypointow to 0.45 m (automat) i ponizej 5 m (klik
        # operatora). Cel jest wiec gleboko w kadrze i jesli detektor go nie widzi,
        # to z braku pikseli (czlowiek ma na 50 m 8 px), a nie z braku kadru.
        # Przechylenie gimbala tylko odsuwa kadr od celu, ktory juz w nim byl.
        # Wlacz tylko, jesli spodziewasz sie waypointow mylnych o kilkadziesiat
        # metrow. Spirala zostaje, bo ona RUSZA DRONEM i pokrywa nowy teren.
        self.declare_parameter('sweep_enabled', False)
        # Spirala wokol waypointu, gdy cel nie zostal potwierdzony.
        self.declare_parameter('spiral_step', 20.0)
        self.declare_parameter('spiral_timeout', 45.0)
        # Grid dla scenariusza C. Prostokat wysrodkowany na pozycji startowej;
        # odstep galsow liczy sie sam ze sladu kadru i overlapu.
        self.declare_parameter('search_w', 150.0)
        self.declare_parameter('search_h', 250.0)
        self.declare_parameter('search_overlap', 0.3)
        self.declare_parameter('search_timeout', 300.0)
        self.declare_parameter('finish_action', 'rtl')
        # TRYB TESTOWY. W prawdziwej misji dron jest juz w powietrzu po przelocie
        # ortofoto i czeka tylko na przelaczenie AUTO -> GUIDED. W symulacji nie
        # ma trasy AUTO, a start w GUIDED spowodowalby, ze misja przejmie drona
        # stojacego na ziemi. auto_takeoff:=true kaze jej samej uzbroic i wzniesc
        # sie na target_alt, z pominieciem czekania na tryb.
        self.declare_parameter('auto_takeoff', False)
        # Jawny obrot nosem na cel przed dolotem. DOMYSLNIE WYLACZONY, bo akcja
        # Set_yaw w drone_handler wisi przy ujemnym namiarze: jej petla porownuje
        # kat znormalizowany do [0,2pi) z katem z zakresu [-pi,pi], wiec warunek
        # konca nigdy sie nie spelnia. ArduPilot w GUIDED i tak obraca drona
        # w strone zadanego punktu, wiec lot nosem do przodu mamy za darmo.
        # Wlacz dopiero, gdy yaw_callback w handlerze zostanie poprawiony.
        self.declare_parameter('yaw_to_target', False)

        p = self.get_parameter
        self.targets_json = p('targets_json').value
        self.takeover_timeout = p('takeover_timeout').value
        self.arrive_tol = p('arrive_tol').value
        self.approach_timeout = p('approach_timeout').value
        self.hover_hold_time = p('hover_hold_time').value
        self.center_tol_m = p('center_tol_m').value
        self.sweep_enabled = p('sweep_enabled').value
        self.spiral_step = p('spiral_step').value
        self.spiral_timeout = p('spiral_timeout').value
        self.search_w = p('search_w').value
        self.search_h = p('search_h').value
        self.search_overlap = p('search_overlap').value
        self.search_timeout = p('search_timeout').value
        self.finish_action = str(p('finish_action').value).lower()
        self.auto_takeoff = p('auto_takeoff').value
        self.yaw_to_target = p('yaw_to_target').value

        if self.center_tol_m < self.hover_deadzone_m:
            self.get_logger().warn(
                f"center_tol_m({self.center_tol_m}) < hover_deadzone_m"
                f"({self.hover_deadzone_m}) — podnosze")
            self.center_tol_m = self.hover_deadzone_m

        self._abort = False
        self.home = None            # (lat, lon) zapamietane przy przejeciu

        self.get_logger().info(
            f"suas_full_mission: drop_alt={self.target_alt} m | "
            f"cele z {self.targets_json} | akwizycja {self.acquire_timeout:.0f}s "
            f"| potwierdzenie {self.confirm_timeout:.0f}s")

    # ═══════════════════════════════════════════════════════════
    #  Pomocnicze
    # ═══════════════════════════════════════════════════════════

    def _install_signals(self):
        """Ctrl+C nie zabija kontekstu ROS od razu — RTL musi jeszcze przejsc."""
        def handler(signum, frame):
            if self._abort:
                raise KeyboardInterrupt
            self._abort = True          # widza to tez bramki w kontrolerze
            self._alarm = True
            self.get_logger().warn("Ctrl+C — przerywam i wracam "
                                   "(kolejne Ctrl+C = twarde wyjscie)")
        signal.signal(signal.SIGINT, handler)
        signal.signal(signal.SIGTERM, handler)

    def _spin(self, seconds, stop_on_abort=True):
        end = time.time() + seconds
        while rclpy.ok() and time.time() < end:
            if stop_on_abort and self._abort:
                return
            rclpy.spin_once(self, timeout_sec=0.05)

    def _dist_to(self, lat, lon):
        return math.hypot((lat - self.global_lat) * M_LAT,
                          (lon - self.global_lon) * _m_per_deg_lon(lat))

    def _bearing_to(self, lat, lon):
        """Namiar na cel w radianach, 0 = polnoc, rosnie zgodnie z ruchem zegara."""
        dn = (lat - self.global_lat) * M_LAT
        de = (lon - self.global_lon) * _m_per_deg_lon(lat)
        return math.atan2(de, dn)

    def _offset_gps(self, lat, lon, d_north, d_east):
        return lat + d_north / M_LAT, lon + d_east / _m_per_deg_lon(lat)

    # ═══════════════════════════════════════════════════════════
    #  Fazy
    # ═══════════════════════════════════════════════════════════

    def wait_for_guided(self) -> bool:
        """FAZA_ORTO: czekaj, az operator przelaczy AUTO -> GUIDED.

        Nic tu nie wysylamy. Geolokator w tym czasie zbiera cele, a operator
        moze je oznaczac klikami w GUI.
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

    def read_targets(self):
        """targets.json -> {class_id: (lat, lon, source)}.

        Czytamy przy KAZDYM wejsciu w GUIDED, nie raz na starcie: dzieki temu
        klik wykonany po przejeciu lotu wchodzi do gry — wychodzisz z GUIDED,
        klikasz, wracasz.
        """
        try:
            with open(self.targets_json) as fh:
                data = json.load(fh)
        except Exception as e:
            self.get_logger().warn(f"nie moge odczytac {self.targets_json}: {e}")
            return {}
        out = {}
        for cid, (name, key, _topic) in TARGETS.items():
            best = (data.get(key) or {}).get('best')
            if best:
                out[cid] = (best['lat'], best['lon'], best.get('source', '?'))
                self.get_logger().info(
                    f"cel {name}: {best['lat']:.6f} {best['lon']:.6f} "
                    f"(zrodlo={best.get('source')}, obs={best.get('n_obs')})")
            else:
                self.get_logger().warn(f"cel {name}: BRAK — bedzie przeszukiwanie")
        return out

    def descend(self, alt) -> bool:
        """Zejscie na zadana wysokosc nad biezaca pozycja."""
        self.get_logger().info(f"zejscie na {alt:.0f} m")
        self.target_alt = alt
        return self.send_goto_global(self.global_lat, self.global_lon, alt)

    def goto(self, lat, lon) -> bool:
        """Dolot nosem do przodu, nie bokiem.

        Kurs ustawiamy JAWNIE, zamiast liczyc na WP_YAW_BEHAVIOR w ArduPilocie —
        dzieki temu zachowanie jest takie samo w SITL i na realu.
        """
        d = self._dist_to(lat, lon)
        brg = self._bearing_to(lat, lon)
        self.get_logger().info(
            f"dolot: {d:.0f} m, namiar {math.degrees(brg):+.0f} st.")
        if self.yaw_to_target and d > self.arrive_tol:
            self.send_set_yaw(brg, relative=False)
            self._spin(2.0)
        ok = self.send_goto_global(lat, lon, self.target_alt)
        # Gimbal w PION po dolocie. Stoimy nad waypointem, wiec cel ma byc pod
        # nami — a gimbal zostalby tam, gdzie zostawil go poprzedni krok
        # (np. -55 po zamiataniu), czyli patrzylby 35 m przed siebie i okno
        # akwizycji nie mialoby szans.
        self._set_gimbal(self.pitch_min)
        self._spin(1.5)
        self.get_logger().info(
            f"nad waypointem (blad {self._dist_to(lat, lon):.1f} m), gimbal w pionie")
        return ok

    def center_over_target(self) -> bool:
        """Wizualne centrowanie: SEARCH -> APPROACH -> HOVER, az cel pod dronem.

        Zwraca True dopiero, gdy cel jest blizej niz center_tol_m przez
        hover_hold_time. Timeout = wracamy do sciezki domyslnej, czyli zrzutu
        na waypoint — zeby zgubienie celu w polowie nie zawiesilo misji.
        """
        # fresh=False: nie ruszamy gimbala ani okna detekcji. Cel wlasnie
        # zostal potwierdzony — przestawienie na pitch_search zgubiloby go
        # natychmiast, bo stoimy nad nim, a -55 st. patrzy daleko przed siebie.
        self.run_mission(fresh=False)
        t0 = time.time()
        hold_since = None
        try:
            while rclpy.ok() and not self._abort:
                rclpy.spin_once(self, timeout_sec=0.05)
                now = time.time()
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
        return False

    def spiral_search(self, lat, lon) -> bool:
        """Spirala wokol waypointu. Kwadratowa, bo prosciej i wystarcza."""
        self.get_logger().info(
            f"SPIRALA wokol waypointu, skok {self.spiral_step:.0f} m")
        end = time.time() + self.spiral_timeout
        ring = 1
        while rclpy.ok() and not self._abort and time.time() < end:
            r = ring * self.spiral_step
            for dn, de in ((r, 0), (0, r), (-r, 0), (0, -r)):
                if time.time() > end or self._abort:
                    break
                wlat, wlon = self._offset_gps(lat, lon, dn, de)
                self.send_goto_global(wlat, wlon, self.target_alt)
                if self.wait_acquire(timeout=1.5):
                    return True
            ring += 1
        self.get_logger().warn("spirala nic nie znalazla")
        return False

    def grid_search(self) -> bool:
        """Scenariusz C: grid nad obszarem wokol punktu startowego.

        Odstep galsow liczy sie sam ze sladu kadru na tej wysokosci
        i zadanego overlapu — nie trzeba go podawac recznie.
        """
        slad = self.img_w * self.altitude / self.focal_px
        spacing = max(5.0, slad * (1.0 - self.search_overlap))
        n = max(1, int(self.search_w / spacing) + 1)
        self.get_logger().info(
            f"GRID {self.search_w:.0f}x{self.search_h:.0f} m | slad {slad:.0f} m "
            f"| odstep {spacing:.0f} m | {n} galsow")
        lat0, lon0 = self.home
        end = time.time() + self.search_timeout
        for i in range(n):
            de = -self.search_w / 2 + i * spacing
            for dn in ((self.search_h / 2, -self.search_h / 2) if i % 2 == 0
                       else (-self.search_h / 2, self.search_h / 2)):
                if time.time() > end or self._abort:
                    self.get_logger().warn("grid: koniec czasu")
                    return False
                wlat, wlon = self._offset_gps(lat0, lon0, dn, de)
                self.goto(wlat, wlon)
                if self.wait_acquire(timeout=1.0):
                    self.get_logger().info(f"GRID: cel znaleziony na galsie {i+1}")
                    return True
        self.get_logger().warn("grid przeleciany, nic nie znaleziono")
        return False

    # ═══════════════════════════════════════════════════════════
    #  Obsluga jednego celu
    # ═══════════════════════════════════════════════════════════

    def handle_target(self, idx, class_id, wp):
        name, _key, topic = TARGETS[class_id]
        self.set_detection_topic(topic)
        self.get_logger().info(f"╔══ CEL {idx + 1}: {name} ══ ({topic})")

        if wp is None:
            # ── Scenariusz C ──
            self.get_logger().info("brak waypointu — przeszukuje teren")
            if not self.grid_search():
                self.get_logger().error(
                    f"{name}: nic nie znaleziono, ladunek zostaje na pokladzie")
                return False
            found = True
        else:
            lat, lon, src = wp
            self.goto(lat, lon)
            found = self.wait_acquire()
            if not found:
                # ── Scenariusz B ──
                if self.sweep_enabled:
                    found = self.sweep_for_target()
                if not found:
                    found = self.spiral_search(lat, lon)
                    if not found:
                        self.get_logger().warn(
                            f"{name}: nie widze celu — ZRZUT NA WAYPOINT")
                        self.goto(lat, lon)
                        self.drop(idx)
                        return True

        # ── Scenariusz A: cel widoczny, pytamy operatora ──
        approved = self.wait_confirm(
            f"=== CEL {idx + 1}/2: {name} ===\n"
            f"Wykryty w kadrze.\n"
            f"[SPACJA] = podejdz i wycentruj     "
            f"[nic] = zrzut na waypoint za {self.confirm_timeout:.0f}s")

        if approved and self.center_over_target():
            self.get_logger().info(f"{name}: ZRZUT NAD WYCENTROWANYM CELEM")
        elif wp is not None:
            self.get_logger().warn(
                f"{name}: bez centrowania — ZRZUT NA WAYPOINT")
            self.goto(wp[0], wp[1])
        else:
            self.get_logger().warn(f"{name}: zrzut w miejscu wykrycia")
        self.drop(idx)
        return True

    # ═══════════════════════════════════════════════════════════
    #  Misja
    # ═══════════════════════════════════════════════════════════

    def run(self) -> bool:
        self.get_logger().info("=== START: suas_full_mission ===")
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
        self.descend(self.target_alt)

        targets = self.read_targets()
        # Bliższy pierwszy — mniej latania i szybciej widac, czy dziala.
        order = sorted(TARGETS, key=lambda c: (
            self._dist_to(*targets[c][:2]) if c in targets else 1e9))

        for idx, class_id in enumerate(order):
            if self._abort:
                break
            self.handle_target(idx, class_id, targets.get(class_id))

        self.stop_mission()
        self._spin(1.0, stop_on_abort=False)
        if self.finish_action == 'rtl':
            self.get_logger().info("=== POWROT: RTL ===")
            self.rtl()
        elif self.finish_action == 'land':
            self.land()
        self._spin(2.0, stop_on_abort=False)
        self.get_logger().info("=== KONIEC MISJI ===")
        return True


def main(args=None):
    # SignalHandlerOptions.NO — Ctrl+C obsluguje sama misja, zeby zdazyla
    # jeszcze wyslac RTL zanim kontekst ROS padnie.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = SuasFullMission()
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
