"""
suas_simple_mission — prosta misja end-to-end na bazie suas_flight_controller.

Sekwencja (bez udzialu operatora):

    1. ARM            — tryb GUIDED + uzbrojenie
    2. TAKEOFF        — wzlot na `target_alt` (ta sama wysokosc, ktora
                        kontroler potem trzyma regulatorem `kp_alt`)
    3. SETTLE         — `settle_time` s zawisu na ustabilizowanie
    4. KONTROLER      — start maszyny stanow SEARCH -> APPROACH -> HOVER
                        (dokladnie ta z suas_flight_controller)
    5. CZEKANIE       — az dron ZAWISNIE NAD NAMIOTEM: stan HOVER **i** namiot
                        w srodku kadru (|ex|,|ey| <= `center_tol`) przez
                        `hover_hold_time` s. Sam HOVER nie wystarcza — kontroler
                        wchodzi w niego na progu kata gimbala, a dosuwanie nad
                        cel trwa jeszcze kilkanascie sekund.
                        Limit calosci: `search_timeout` s -> misja przerwana
    6. POWROT         — `finish_action`: rtl (domyslnie) / land / none

Klasa dziedziczy po SuasFlightController, wiec ma WSZYSTKIE jego parametry
(pitch_*, kp_*, img_w/h, vfov_deg, det_*, ...) — patrz docstring tamtego modulu,
zwlaszcza uwaga o innej kalibracji gimbala i FOV w Gazebo.

UWAGA: w stanie SEARCH dron NIE lata — stoi w zawisie i czeka, az detektor
zobaczy namiot. Misja nie robi zadnego przeszukiwania terenu, wiec namiot musi
byc w kadrze po starcie (w symulacji dron stoi ~30 m przed namiotem).
Jesli sie nie pojawi, po `search_timeout` dron wraca do domu.

Wymaga dzialajacego drone_handler + detektora publikujacego /tent_detections
(suas_detect_gazebo.launch.py albo suas_detect_jetson.launch.py).
"""

import signal
import time

import rclpy
from rclpy.signals import SignalHandlerOptions

from drone_autonomy.suas_flight_controller import State, SuasFlightController


class SuasSimpleMission(SuasFlightController):
    """arm -> takeoff -> podlot nad namiot -> powrot."""

    def __init__(self):
        super().__init__('suas_simple_mission')

        # ─── Parametry misji ──────────────────────────────────
        # UWAGA: te same wartosci domyslne co w launchu — nie rozjezdzac ich.
        self.declare_parameter('settle_time', 3.0)        # zawis po starcie [s]
        self.declare_parameter('hover_hold_time', 5.0)    # ile HOVER musi trwac [s]
        self.declare_parameter('search_timeout', 120.0)   # limit na caly podlot [s]
        self.declare_parameter('finish_action', 'rtl')    # rtl | land | none
        # Ile namiot moze byc od srodka kadru (0..1 polowy klatki), zeby uznac
        # zawis za "nad namiotem". Samo wejscie w HOVER tego NIE znaczy —
        # kontroler przelacza sie na progu kata gimbala, a dosuwanie nad cel
        # trwa jeszcze kilkanascie sekund.
        self.declare_parameter('center_tol', 0.12)

        self.settle_time     = self.get_parameter('settle_time').value
        self.hover_hold_time = self.get_parameter('hover_hold_time').value
        self.search_timeout  = self.get_parameter('search_timeout').value
        self.finish_action   = str(self.get_parameter('finish_action').value).lower()
        self.center_tol      = self.get_parameter('center_tol').value

        # Ponizej hover_deadzone kontroler przestaje korygowac, wiec ciasniejsza
        # tolerancja to warunek nie do spelnienia (czekalby do timeoutu).
        if self.center_tol < self.hover_deadzone:
            self.get_logger().warn(
                f"center_tol({self.center_tol}) < hover_deadzone({self.hover_deadzone}) "
                f"— podnosze do {self.hover_deadzone}")
            self.center_tol = self.hover_deadzone

        if self.finish_action not in ('rtl', 'land', 'none'):
            self.get_logger().warn(
                f"Nieznane finish_action='{self.finish_action}' — uzywam 'rtl'")
            self.finish_action = 'rtl'

        # Ctrl+C: NIE zabijamy kontekstu ROS od razu (patrz _install_signals) —
        # najpierw musi jeszcze przejsc RTL.
        self._abort = False

        self.get_logger().info(
            f"SuasSimpleMission: takeoff={self.target_alt}m settle={self.settle_time}s "
            f"hover_hold={self.hover_hold_time}s timeout={self.search_timeout}s "
            f"center_tol={self.center_tol} finish={self.finish_action}")

    # ═══════════════════════════════════════════════════════════
    #  Pomocnicze
    # ═══════════════════════════════════════════════════════════

    def _install_signals(self):
        """Wlasna obsluga Ctrl+C / SIGTERM.

        Domyslny handler rclpy natychmiast unicestwia kontekst ROS, wiec
        wszystko po KeyboardInterrupt (RTL, wylaczenie velocity control) leci
        w 'publisher's context is invalid' i dron zostaje w powietrzu w GUIDED.
        Tu tylko podnosimy flagi: `_abort` zatrzymuje petle misji, a `_alarm`
        (z DroneController) przerywa trwajaca akcje arm/takeoff/goto.
        Kontekst zostaje zywy, wiec RTL ma czym polecieć. Drugie Ctrl+C =
        twarde wyjscie."""
        def handler(signum, frame):
            if self._abort:
                raise KeyboardInterrupt
            self._abort = True
            self._alarm = True
            self.get_logger().warn(
                "Ctrl+C — przerywam misje i wracam (kolejne Ctrl+C = twarde wyjscie)")

        signal.signal(signal.SIGINT, handler)
        signal.signal(signal.SIGTERM, handler)

    def _spin_for(self, seconds):
        """Obsluguj callbacki (telemetria, detekcje, timer) przez `seconds`."""
        end = time.time() + seconds
        while rclpy.ok() and not self._abort and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def _tent_error(self):
        """Znormalizowany uchyb polozenia namiotu w kadrze (ex, ey) w zakresie
        -1..+1, albo None gdy detekcja jest przeterminowana (lost_timeout).
        Liczony tak samo jak w petli kontrolera."""
        if self.last_det_time <= 0.0:
            return None
        if time.time() - self.last_det_time > self.lost_timeout:
            return None
        half_w = self.img_w / 2.0
        half_h = self.img_h / 2.0
        return ((self.tent_cx - half_w) / half_w,
                (self.tent_cy - half_h) / half_h)

    def _wait_for_hover(self) -> bool:
        """Czekaj, az dron faktycznie zawisnie NAD namiotem.

        Sam stan HOVER nie wystarcza: kontroler wchodzi w niego na progu kata
        gimbala (pitch_hover_thr), a wtedy namiot potrafi byc jeszcze kilka
        metrow z boku — dosuwanie idzie dopiero mikrokorektami w HOVER.
        Dlatego warunkiem konca jest HOVER **i** namiot w srodku kadru
        (|ex|,|ey| <= center_tol) nieprzerwanie przez hover_hold_time.

        Zwraca True = zawis nad namiotem potwierdzony, False = timeout
        albo przerwanie."""
        t_start = time.time()
        hover_since = None

        while rclpy.ok():
            if self._abort:
                self.get_logger().warn("Przerwane — nie czekam dalej na HOVER")
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
            now = time.time()

            err = self._tent_error()
            centered = (err is not None
                        and abs(err[0]) <= self.center_tol
                        and abs(err[1]) <= self.center_tol)

            if self.state == State.HOVER and centered:
                if hover_since is None:
                    hover_since = now
                    self.get_logger().info(
                        f"Namiot w srodku kadru (ex={err[0]:+.2f} ey={err[1]:+.2f}) "
                        f"— trzymam {self.hover_hold_time}s na potwierdzenie")
                elif now - hover_since >= self.hover_hold_time:
                    self.get_logger().info(
                        f"NAD NAMIOTEM stabilnie przez {now - hover_since:.1f}s "
                        f"(ex={err[0]:+.2f} ey={err[1]:+.2f}) — podlot zakonczony")
                    return True
            else:
                if hover_since is not None:
                    powod = ("wypadl ze stanu HOVER" if self.state != State.HOVER
                             else "namiot uciekl ze srodka kadru")
                    self.get_logger().warn(f"{powod} — licze od nowa")
                    hover_since = None

                if self.state == State.HOVER:
                    # Jestesmy nad celem "z grubsza" — pokaz, ile brakuje.
                    opis = (f"ex={err[0]:+.2f} ey={err[1]:+.2f}" if err
                            else "brak swiezej detekcji")
                    self.get_logger().info(
                        f"[MISJA] HOVER — dosuwam sie nad namiot ({opis}, "
                        f"cel <= {self.center_tol}) t={now - t_start:.0f}/"
                        f"{self.search_timeout:.0f}s",
                        throttle_duration_sec=3.0)

            if now - t_start > self.search_timeout:
                self.get_logger().error(
                    f"TIMEOUT {self.search_timeout}s — nie zawislem nad namiotem "
                    f"(stan {self.state.name})")
                return False

            if self.state != State.HOVER:
                self.get_logger().info(
                    f"[MISJA] {self.state.name} t={now - t_start:.0f}/"
                    f"{self.search_timeout:.0f}s alt={self.altitude:.1f}m",
                    throttle_duration_sec=5.0)

        return False

    def _go_home(self):
        """Powrot wg finish_action. Velocity control musi byc juz WYLACZONY
        (stop_mission), inaczej drone_handler nadpisywalby tryb RTL/LAND."""
        if self.finish_action == 'rtl':
            self.get_logger().info("=== POWROT: RTL ===")
            self.rtl()
        elif self.finish_action == 'land':
            self.get_logger().info("=== POWROT: LAND (w miejscu) ===")
            self.land()
        else:
            self.get_logger().info("=== finish_action=none — zostawiam w zawisie ===")

    # ═══════════════════════════════════════════════════════════
    #  Misja
    # ═══════════════════════════════════════════════════════════

    def run(self) -> bool:
        self.get_logger().info("=== START MISJI: suas_simple_mission ===")
        self._install_signals()

        # ─── 1. ARM ───────────────────────────────────────────
        if not self.arm():
            if self._abort:
                self.get_logger().warn("Przerwane przed startem — dron zostaje na ziemi")
                return False
            self.get_logger().error("ARM nieudany — przerywam misje")
            return False

        # ─── 2. TAKEOFF ───────────────────────────────────────
        if not self.takeoff(float(self.target_alt)):
            if self._abort:
                self.get_logger().warn("Przerwane w trakcie startu — wracam")
                self.stop_mission()
                self._go_home()
                self._spin_for(2.0)
                return False
            self.get_logger().error("TAKEOFF nieudany — LAND")
            self.land()
            return False

        # ─── 3. Stabilizacja ──────────────────────────────────
        self.get_logger().info(f"Zawis {self.settle_time}s po starcie...")
        self._spin_for(self.settle_time)

        # ─── 4. Kontroler SEARCH -> APPROACH -> HOVER ─────────
        # run_mission() z rodzica: velocity control ON, gimbal na pitch_search,
        # start petli sterowania.
        self.run_mission()

        # ─── 5. Czekaj na zawis nad namiotem ──────────────────
        reached = self._wait_for_hover()

        # ─── 6. Powrot ────────────────────────────────────────
        # stop_mission zeruje wektory i wylacza velocity control — dopiero
        # potem wolno przelaczac tryb lotu. _spin_for konczy sie od razu przy
        # _abort, wiec tu spinujemy recznie (RTL ma dojsc takze po Ctrl+C).
        self.stop_mission()
        self._settle(1.0)
        self._go_home()
        self._settle(2.0)  # niech SetMode faktycznie pojdzie do handlera

        status = 'nad namiotem' if reached else ('PRZERWANA' if self._abort else 'BEZ namiotu')
        self.get_logger().info(f"=== KONIEC MISJI ({status}) ===")
        return reached

    def _settle(self, seconds):
        """Jak _spin_for, ale ignoruje _abort — uzywane w sciezce powrotu,
        ktora ma sie dokonczyc takze po Ctrl+C."""
        end = time.time() + seconds
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)


def main(args=None):
    # SignalHandlerOptions.NO — Ctrl+C obsluguje sama misja (_install_signals),
    # zeby zdazyla jeszcze wyslac RTL zanim kontekst ROS padnie.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = SuasSimpleMission()

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
