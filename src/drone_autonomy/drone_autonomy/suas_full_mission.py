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

REGULY (patrz docs/misja_scenariusze.md)
    * Pula kandydatow obu klas, ZAWSZE od najlepszego (score geolokatora).
    * Pytamy o SPACJE tylko, gdy operator patrzy (/operator_online z GUI).
      Spacja = "zrzucamy tutaj" (widoczny -> centrowanie; nie -> na wspolrzedne).
      Milczenie, gdy operator PATRZY = "to nie ten cel" -> nastepny kandydat,
      a gdy kandydatow brak -> grid bez konca (konczy spacja albo Ctrl+C).
    * Operatora NIE MA -> nie pytamy, zrzut na najlepszym kandydacie,
      grid tylko raz.

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
from std_msgs.msg import Bool

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
        # Ile ciaglego SEARCH w trakcie centrowania znaczy "cel zniknal na dobre".
        # Po naprawie gimbala (zostaje w pionie) prawdziwy cel wraca w ulamku
        # sekundy, wiec 15 s bez detekcji to nie chwilowe przeslonieciecie, tylko
        # brak celu — nie ma sensu czekac pelnego approach_timeout (90 s).
        self.declare_parameter('center_lost_timeout', 15.0)
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
        # DOMYSLNIE WYLACZONA. Kosztuje spiral_timeout (45 s) na kazdym
        # kandydacie, ktorego detektor nie potwierdzil, a przy waypoincie
        # z automatu (blad 0.45 m) albo z klikniecia operatora cel jest juz
        # gleboko w kadrze — jesli detektor go nie widzi, to z braku pikseli,
        # a nie dlatego, ze dron stoi 20 m obok. Wlacz tylko, gdy spodziewasz
        # sie waypointow mylnych o kilkadziesiat metrow.
        self.declare_parameter('spiral_enabled', False)
        self.declare_parameter('spiral_step', 20.0)
        self.declare_parameter('spiral_timeout', 45.0)
        # Grid dla scenariusza C. Prostokat wysrodkowany na pozycji startowej;
        # odstep galsow liczy sie sam ze sladu kadru i overlapu.
        self.declare_parameter('search_w', 150.0)
        self.declare_parameter('search_h', 250.0)
        self.declare_parameter('search_overlap', 0.3)
        # Wlasne punkty przeszukiwania zamiast liczonego gridu.
        # search_offsets: plaska lista [E1,N1, E2,N2, ...] w METRACH wzgledem
        #   punktu przejecia lotu. Wygodne w yamlu, bo nic nie trzeba przeliczac
        #   na stopnie ani pilnowac sciezki do pliku.
        # search_waypoints: sciezka do pliku (.waypoints z MP albo lat/lon).
        #   Ma pierwszenstwo nad search_offsets.
        # Oba puste = dron liczy siatke sam z search_w/search_h.
        self.declare_parameter('search_waypoints', '')
        self.declare_parameter('search_offsets', [0.0])
        self.declare_parameter('search_timeout', 300.0)
        # Co ile sekund w trakcie przeszukiwania zagladamy do targets.json.
        # Geolokator pracuje NIEPRZERWANIE, takze gdy dron leci gridem, wiec
        # nowy cel — z automatu albo z klikniecia operatora — moze sie pojawic
        # w kazdej chwili. Bez tego pollingu misja dowiedzialaby sie o nim
        # dopiero po zakonczeniu calego gridu.
        # Ile czekamy na potwierdzenie detektora po dolocie do punktu gridu.
        # NIE mylic z acquire_timeout (5 s), ktory obowiazuje nad kandydatem.
        # Bylo tu 1.0 s i to bylo za malo: wait_acquire zaczyna od wyzerowania
        # okna M z N, a przy detektorze chodzacym ~5 Hz z jitterem do 0.7 s
        # w sekundzie miesci sie 2-5 klatek — mniej niz 4 wymagane trafienia.
        # Bramka potrafila byc nieprzechodzalna niezaleznie od tego, czy cel
        # jest pod dronem. Przy 3 s to ~15 klatek, czyli okno napelnia sie
        # dwukrotnie. Koszt: 8 punktow x 2 s = 16 s przy budzecie search_timeout.
        self.declare_parameter('grid_acquire_timeout', 3.0)
        self.declare_parameter('watch_interval', 2.0)
        # Przerwanie przelotu gridem, gdy DETEKTOR domknie okno M z N juz
        # w locie — nie czekamy z tym do najblizszego waypointu. Okno wypelnia
        # sie caly czas (petla akcji kreci spin_once), wiec ta wiedza istnieje;
        # dotad byla tylko wyrzucana przez _reset_det_window() na starcie
        # wait_acquire. Cel wchodzacy w kadr w polowie 250-metrowego galsu
        # zatrzymuje teraz drona tam, gdzie jest, a nie 250 m dalej.
        self.declare_parameter('grid_det_interrupt', True)
        # Po nieudanym sprawdzeniu na stojaco tyle sekund nie sluchamy
        # detektora. Bez tego ta sama falszywka zatrzymywalaby drona w kolko:
        # wracamy na kurs, okno napelnia sie z tego samego krzaka i znowu stoimy.
        self.declare_parameter('det_interrupt_cooldown', 10.0)
        # Ile czekamy, az dron wyhamuje po przerwaniu dolotu, zanim uznamy
        # klatki za wiarygodne. Przy 5 m/s hamowanie trwa ok. 2 s.
        self.declare_parameter('brake_settle_time', 3.0)
        # KTO moze wyrwac drona z gridu:
        #   'operator' - tylko reczne oznaczenie z GUI (domyslnie)
        #   'any'      - takze kandydat z automatu, gdy przekroczy min_obs
        #   'off'      - nikt; grid leci do konca jak dawniej
        # Domyslnie 'operator', bo fałszywki produkuje klastrowanie, nie klik:
        # obiekt systematycznie brany za czlowieka (drzewo, cien, krzak) zbuduje
        # rownie zbiezny klaster co prawdziwy cel, a operator patrzy na obraz
        # i takiego bledu nie zrobi. Przy 'any' i NIEOBECNYM operatorze zrzut
        # idzie bez pytania, wiec fałszywka kosztuje ladunek — a nie jeden
        # niepotrzebny przelot, jak przy 'operator'.
        self.declare_parameter('watch_sources', 'operator')
        # Czy po PELNYM, nieudanym przelocie gridu siegac po kandydata z
        # automatu. To jest kompromis miedzy dwoma zlymi skrajnosciami:
        # automat wyrywajacy drona z trasy po kilku klatkach (fałszywki) i
        # automat calkiem ignorowany (marnujemy jedyna informacje, jaka mamy,
        # gdy grid przeleci wszystko i nic nie zobaczy). Po calym gridzie
        # klastry sa zbudowane z obserwacji z WIELU przelotow, wiec drzewo ma
        # duzo mniejsza szanse wygrac ze score prawdziwego celu.
        self.declare_parameter('auto_after_grid', True)
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
        self.center_lost_timeout = p('center_lost_timeout').value
        self.hover_hold_time = p('hover_hold_time').value
        self.center_tol_m = p('center_tol_m').value
        self.sweep_enabled = p('sweep_enabled').value
        self.spiral_enabled = p('spiral_enabled').value
        self.spiral_step = p('spiral_step').value
        self.spiral_timeout = p('spiral_timeout').value
        self.search_w = p('search_w').value
        self.search_h = p('search_h').value
        self.search_overlap = p('search_overlap').value
        self.search_waypoints = str(p('search_waypoints').value).strip()
        # [0.0] jako domyslna, bo ROS 2 nie przyjmuje pustej listy (nie zna typu).
        off = list(p('search_offsets').value)
        self.search_offsets = off if len(off) >= 4 else []
        if len(off) % 2:
            self.get_logger().warn(
                f"search_offsets ma nieparzysta liczbe wartosci ({len(off)}) — "
                f"ostatnia ignoruje")
        self.search_timeout = p('search_timeout').value
        self.grid_acquire_timeout = p('grid_acquire_timeout').value
        self.watch_interval = p('watch_interval').value
        self.grid_det_interrupt = p('grid_det_interrupt').value
        self.det_interrupt_cooldown = p('det_interrupt_cooldown').value
        self.brake_settle_time = p('brake_settle_time').value
        self.watch_sources = str(p('watch_sources').value).lower()
        self.auto_after_grid = p('auto_after_grid').value
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

        # Kandydaci, ktorych JUZ znamy: odwiedzeni, odrzuceni i ci widziani przy
        # pierwszym odczycie targets.json. Grid przerywa sie tylko dla celu spoza
        # tego zbioru, inaczej dron zawracalby w kolko do tego samego punktu.
        self._known_ids = set()
        self._interrupt_cand = None     # cel, ktory przerwal przeszukiwanie
        self._last_watch = 0.0
        self._det_interrupt = False     # detektor domknal okno jeszcze w locie
        self._det_suppress_until = 0.0

        # ── Obecnosc operatora ──────────────────────────────────────────
        # suas_marker_web publikuje tu true, dopoki przegladarka przysyla puls.
        # Pytamy o spacje TYLKO wtedy, gdy ktos naprawde patrzy — inaczej
        # czekalibysmy confirm_timeout w prozni na kazdym celu.
        # Brak wiadomosci (GUI nie chodzi) = operatora nie ma. To jest celowe:
        # domyslna sciezka jest bezpieczna, wiec milczenie ma znaczyc "decyduj sam".
        self._operator_online = False
        self._operator_stamp = 0.0
        self.create_subscription(Bool, '/operator_online', self._online_cb, 10)

        # ── Gimbal nalezy do misji od przejecia lotu ────────────────────
        # Geolokator w fazie ortofoto trzyma gimbal w pionie (co 2 s -90 st.).
        # Po przejeciu to MY sterujemy gimbalem (APPROACH go pochyla), wiec
        # blokade trzeba zwolnic — inaczej dwoch piszacych na jeden silownik
        # i gimbal szarpany do pionu co 2 s w trakcie podejscia.
        self._nadir_pub = self.create_publisher(Bool, '/geolocator/lock_nadir', 10)

        self.get_logger().info(
            f"suas_full_mission: drop_alt={self.target_alt} m | "
            f"cele z {self.targets_json} | akwizycja {self.acquire_timeout:.0f}s "
            f"| potwierdzenie {self.confirm_timeout:.0f}s")

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
        tmux sprawia, ze SIGHUP w ogole nie dolatuje i misja leci dalej sama.
        Ta obsluga jest siatka na wypadek, gdyby jednak dolecial.
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

    def _online_cb(self, msg: Bool):
        self._operator_online = bool(msg.data)
        self._operator_stamp = time.time()

    def operator_watching(self) -> bool:
        """Czy operator patrzy na obraz i moze cokolwiek potwierdzic.

        Wymagamy SWIEZEJ wiadomosci, nie samej wartosci: gdyby wezel GUI padl
        albo zerwalo sie polaczenie miedzy nim a misja, ostatnie 'true'
        zostaloby w pamieci na zawsze i dron pytalby w prozni.
        """
        return (self._operator_online
                and time.time() - self._operator_stamp < 5.0)

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

    def read_targets(self, verbose=True):
        """targets.json -> {class_id: [(lat, lon, source, n_obs, score), ...]}.

        Zwracamy CALA liste kandydatow, nie tylko 'best'. Gdy operator odrzuci
        cel #1 — bo z 50 m widzi, ze to atrapa, a waypoint powstal z 80 m —
        kandydat #2 jest gotowym, policzonym punktem i lot do niego kosztuje
        jeden przelot zamiast calego gridu.

        FILTR jest tu konieczny: geolokator wypisuje takze kandydatow z 3-4
        obserwacjami, czyli krzaki i cienie. Lot do takiego to kilkadziesiat
        sekund za nic. Przepuszczamy wiec tylko tych, ktorzy przeszli prog
        min_obs swojej klasy (ten sam, ktorym geolokator chroni 'best'),
        albo zostali wskazani recznie przez operatora.

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

        mins = data.get('min_obs') or {}
        out = {}
        for cid, (name, key, _topic) in TARGETS.items():
            sec = data.get(key) or {}
            best = sec.get('best')
            need = mins.get(key, 10)
            lst, seen = [], set()
            # 'best' na czele — geolokator wybral go wg obs*conf i priorytetu
            # operatora, wiec to nadal najlepszy pierwszy strzal.
            for c in ([best] if best else []) + (sec.get('candidates') or []):
                if not c or c.get('id') in seen:
                    continue
                if c.get('source') != 'operator' and c.get('n_obs', 0) < need:
                    continue
                seen.add(c.get('id'))
                lst.append((c['lat'], c['lon'],
                            c.get('source', '?'), c.get('n_obs', 0),
                            float(c.get('score', 0.0)), c.get('id')))
            out[cid] = lst
            if not verbose:
                continue
            if lst:
                opis = ", ".join(f"#{i+1} {s} obs={n} score={sc:.0f}"
                                 for i, (_la, _lo, s, n, sc, _id) in enumerate(lst))
                self.get_logger().info(f"cel {name}: {len(lst)} kandydat(ow) — {opis}")
            else:
                self.get_logger().warn(f"cel {name}: BRAK — bedzie przeszukiwanie")
        return out

    def descend(self, alt, tol=2.0, timeout=60.0) -> bool:
        """Zejscie na zadana wysokosc nad biezaca pozycja — I CZEKANIE na nia.

        Akcja goto_global w drone_handler konczy sie, gdy odleglosc POZIOMA
        (haversine) spadnie ponizej 2 m. Przy tej samej lat/lon to natychmiast,
        wiec bez wlasnego czekania "zejscie" trwaloby 0 s, a dron schodzilby
        dopiero w trakcie dolotu do celu i okno akwizycji otwieraloby sie na
        przypadkowej wysokosci.
        """
        self.get_logger().info(f"zejscie na {alt:.0f} m (jestem na {self.altitude:.0f} m)")
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
        # Przy zgubieniu celu gimbal ma ZOSTAC w pionie — cel jest pod nami,
        # a odstawienie na pitch_search wyrzuciloby go poza kadr na dobre.
        self.search_gimbal_on_lost = False
        self.run_mission(fresh=False)
        t0 = time.time()
        hold_since = None
        search_since = None
        try:
            while rclpy.ok() and not self._abort:
                rclpy.spin_once(self, timeout_sec=0.05)
                now = time.time()
                # Szybkie wyjscie, gdy cel zniknal na dobre: ciagly SEARCH dluzej
                # niz center_lost_timeout. Chwilowa utrata konczy sie w ulamku
                # sekundy (gimbal zostaje w pionie), wiec to nie jest falszywy alarm.
                if self.state == State.SEARCH:
                    if search_since is None:
                        search_since = now
                    elif now - search_since > self.center_lost_timeout:
                        self.get_logger().warn(
                            f"cel nie wrocil przez {self.center_lost_timeout:.0f}s "
                            f"— przerywam centrowanie, ide sciezka domyslna")
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

    def _load_waypoints(self, path):
        """Wczytaj wlasne punkty przeszukiwania. Zwraca [(lat, lon), ...] albo [].

        Obslugiwane sa trzy formaty, zeby nie trzeba bylo niczego konwertowac:

        1. .waypoints z Mission Plannera (naglowek "QGC WPL 110").
           Pola rozdzielone tabulatorem, lat w kolumnie 8, lon w 9.
           Wiersz 0 to HOME i jest pomijany, tak samo komendy bez wspolrzednych
           (lat i lon rowne 0) — czyli TAKEOFF, RTL, DO_*.
           To jest sciezka dla realu: rysujesz siatke w MP, zapisujesz plik.

        2. "lat, lon" w kazdej linii — gdy masz gotowe wspolrzedne.

        3. Linia "OFFSETS", a po niej "wschod, polnoc" w METRACH wzgledem
           PUNKTU PRZEJECIA LOTU. To jest sciezka do testow: nie trzeba
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

    def _grid_waypoints(self):
        """Punkty liczonego gridu: prostokat wysrodkowany na punkcie przejecia.

        Odstep galsow wynika ze SLADU KADRU na biezacej wysokosci i overlapu,
        wiec nie trzeba go podawac recznie. Punktami sa tylko NAROZNIKI galsow —
        po dwa na gals, na przemian, zeby dron nie wracal pusty.
        """
        slad = self.img_w * self.altitude / self.focal_px
        spacing = max(5.0, slad * (1.0 - self.search_overlap))
        n = max(1, int(self.search_w / spacing) + 1)
        self.get_logger().info(
            f"GRID {self.search_w:.0f}x{self.search_h:.0f} m | slad {slad:.0f} m "
            f"| odstep {spacing:.0f} m | {n} galsow")
        lat0, lon0 = self.home
        pts = []
        for i in range(n):
            de = -self.search_w / 2 + i * spacing
            for dn in ((self.search_h / 2, -self.search_h / 2) if i % 2 == 0
                       else (-self.search_h / 2, self.search_h / 2)):
                pts.append(self._offset_gps(lat0, lon0, dn, de))
        return pts

    def _watch_new_candidate(self, class_id) -> bool:
        """Czy w targets.json pojawil sie NOWY cel tej klasy.

        Wolane z petli akcji goto (co ~0.2 s), wiec plik czytamy najwyzej raz
        na watch_interval — reszta wywolan konczy sie na porownaniu zegara.

        Nowy znaczy: id, ktorego jeszcze nie widzielismy. Kandydaci znani przy
        przejeciu lotu i ci juz odwiedzeni siedza w _known_ids, wiec nie
        wyrwa drona z gridu po raz drugi.

        Pierwszenstwo ma zrodlo 'operator': jesli ktos wlasnie kliknal cel na
        podgladzie, to jest najswiezsza i najlepsza informacja, jaka mamy —
        lepsza niz klaster automatu, bo czlowiek widzial obiekt z pulapu misji.
        """
        now = time.time()
        if now - self._last_watch < self.watch_interval:
            return self._interrupt_cand is not None
        self._last_watch = now

        lst = self.read_targets(verbose=False).get(class_id) or []
        nowe = [c for c in lst if c[5] not in self._known_ids]
        if self.watch_sources == 'operator':
            # Kandydaci z automatu NIE przerywaja gridu, ale zapamietujemy ich
            # jako znanych — inaczej po powrocie do gridu ten sam klaster
            # zglaszalby sie co 2 s przez cala reszte przeszukiwania.
            for c in nowe:
                if c[2] != 'operator':
                    self._known_ids.add(c[5])
            nowe = [c for c in nowe if c[2] == 'operator']
        if not nowe:
            return False
        # operator przed automatem, a w obrebie zrodla - najwyzszy score
        nowe.sort(key=lambda c: (c[2] != 'operator', -c[4]))
        self._interrupt_cand = nowe[0]
        name = TARGETS[class_id][0]
        self.get_logger().info(
            f"{name}: NOWY CEL w trakcie przeszukiwania "
            f"(zrodlo={nowe[0][2]}, obs={nowe[0][3]}) — przerywam grid")
        return True

    def _best_auto_candidate(self, class_id):
        """Najlepszy kandydat tej klasy, ktorego jeszcze nie probowalismy.

        Wolane DOPIERO po pelnym przelocie gridu. W tym momencie klastry sa
        zbudowane z obserwacji z calego przeszukiwania, a nie z jednego mignicia
        w kadrze, wiec score (obs * conf) faktycznie cos znaczy i drzewo widziane
        raz przegrywa z celem widzianym na kilku galsach.

        Bierzemy najwyzszy score, tak samo jak pula w run() — nie najblizszego.
        """
        lst = self.read_targets(verbose=False).get(class_id) or []
        nowe = [c for c in lst if c[5] not in self._known_ids]
        if not nowe:
            return None
        return max(nowe, key=lambda c: c[4])

    def _watch_grid(self, class_id) -> bool:
        """Czy jest powod, zeby przerwac trwajacy dolot do punktu gridu.

        Dwa niezalezne powody, sprawdzane w tej kolejnosci:
          1. DETEKTOR domknal okno M z N — cel jest w kadrze TERAZ. Sprawdzenie
             jest darmowe (licznik w pamieci), wiec idzie pierwsze.
          2. W targets.json pojawil sie nowy cel (klik operatora albo automat,
             zaleznie od watch_sources) — to wymaga odczytu pliku, wiec jest
             ograniczone do raz na watch_interval.
        """
        if (self.grid_det_interrupt
                and time.time() > self._det_suppress_until
                and self._det_hits >= self.det_confirm_frames):
            self._det_interrupt = True
            self.get_logger().info(
                f"DETEKTOR widzi cel w locie ({self._det_hits}/"
                f"{len(self._det_window)} klatek, ID={self._cand_id}) "
                f"— przerywam dolot")
            return True
        if self.watch_sources == 'off':
            return False
        return self._watch_new_candidate(class_id)

    def _hold_and_reconfirm(self) -> bool:
        """Zatrzymaj sie tu, gdzie jestes, i sprawdz cel na stojaco.

        Konieczne, bo przerwanie akcji nie zatrzymuje drona: cel zostaje otwarty
        po stronie serwera, a ArduPilot w GUIDED trzyma ostatni zadany punkt.
        Bez tego dron lecialby dalej przez cale confirm_timeout (15 s), czyli
        przy 5 m/s uciekloby mu 75 m od celu, zanim ktokolwiek zaczalby
        centrowanie.

        Ponowne sprawdzenie na stojaco jest tez filtrem: cel potwierdzony
        w locie, ktory nie potwierdza sie po zatrzymaniu, jest podejrzany —
        rozmycie ruchem robi z krzakow dziwne rzeczy. Hamowanie przenosi nas
        o kilkanascie metrow za cel, ale gimbal patrzy w pion, a slad kadru
        na 50 m ma 63 m, wiec cel zostaje w kadrze.
        """
        # Na czas hamowania zdejmujemy haczyk — inaczej goto na wlasna pozycje
        # przerwaloby sie natychmiast, bo okno detekcji jest dalej pelne.
        hook, self.action_interrupt = self.action_interrupt, None
        try:
            self.get_logger().info("zatrzymuje sie i sprawdzam cel na stojaco")
            self.send_goto_global(self.global_lat, self.global_lon,
                                  self.target_alt)
            self._set_gimbal(self.pitch_min)
            self._spin(self.brake_settle_time)
            if self.wait_acquire(timeout=self.grid_acquire_timeout):
                return True
            self._det_suppress_until = time.time() + self.det_interrupt_cooldown
            self.get_logger().warn(
                f"na stojaco cel sie nie potwierdzil — wracam na kurs, "
                f"detektora nie slucham przez {self.det_interrupt_cooldown:.0f}s")
            return False
        finally:
            self.action_interrupt = hook

    def grid_search(self, watch_class=None) -> bool:
        """Scenariusz C: przelot po punktach przeszukiwania.

        Punkty biora sie albo z pliku (search_waypoints), albo z liczonego
        gridu. Reszta jest wspolna: lecimy po kolei i po kazdym dolocie
        sprawdzamy okno detekcji.

        watch_class: gdy podana, w trakcie lotu obserwujemy targets.json i
        przerywamy grid, gdy pojawi sie nowy cel tej klasy (patrz
        _watch_new_candidate). Wynik laduje w self._interrupt_cand, bo wartosc
        zwracana ma tu inne znaczenie: True = cel zobaczyl DETEKTOR.
        """
        if self.search_waypoints:
            pts = self._load_waypoints(self.search_waypoints)
            if not pts:
                return False
            self.get_logger().info(
                f"PRZESZUKIWANIE: {len(pts)} punktow z {self.search_waypoints}")
        elif self.search_offsets:
            o = self.search_offsets
            pts = [self._offset_gps(self.home[0], self.home[1], o[i + 1], o[i])
                   for i in range(0, len(o) - 1, 2)]
            opis = "  ".join(f"({o[i]:+.0f}E,{o[i+1]:+.0f}N)"
                             for i in range(0, len(o) - 1, 2))
            self.get_logger().info(
                f"PRZESZUKIWANIE: {len(pts)} punktow z search_offsets — {opis}")
        else:
            pts = self._grid_waypoints()

        self._interrupt_cand = None
        self._det_interrupt = False
        self._det_suppress_until = 0.0
        if watch_class is not None:
            self._last_watch = 0.0
            self.action_interrupt = lambda: self._watch_grid(watch_class)
        try:
            end = time.time() + self.search_timeout
            for i, (wlat, wlon) in enumerate(pts, 1):
                if time.time() > end or self._abort:
                    self.get_logger().warn(
                        f"przeszukiwanie: koniec czasu na punkcie {i}/{len(pts)}")
                    return False
                if self._interrupt_cand is not None:
                    return False
                self.get_logger().info(f"punkt {i}/{len(pts)}")

                # Dolot moze byc przerwany w polowie — wtedy wracamy na kurs do
                # TEGO SAMEGO punktu, zeby nie zgubic kawalka galsu.
                while not self._abort and time.time() < end:
                    self.goto(wlat, wlon)
                    if self._interrupt_cand is not None:
                        return False
                    if not self._det_interrupt:
                        break
                    self._det_interrupt = False
                    if self._hold_and_reconfirm():
                        self.get_logger().info(
                            f"cel znaleziony w locie na galsie {i}/{len(pts)}")
                        return True

                if self.wait_acquire(timeout=self.grid_acquire_timeout):
                    self.get_logger().info(f"cel znaleziony na punkcie {i}/{len(pts)}")
                    return True
        finally:
            self.action_interrupt = None
        self.get_logger().warn("wszystkie punkty przeleciane, nic nie znaleziono")
        return False

    # ═══════════════════════════════════════════════════════════
    #  Obsluga jednego kandydata
    # ═══════════════════════════════════════════════════════════

    def visit_candidate(self, class_id, cand) -> str:
        """Lec do jednego kandydata i rozstrzygnij, co z nim zrobic.

        Zwraca:
          'dropped'  - ladunek poszedl, klasa zalatwiona
          'rejected' - to nie ten cel; kandydat wypada z puli, lecimy do
                       nastepnego, a gdy pula pusta - w grid

        SPACJA znaczy zawsze to samo: "tak, zrzucamy tutaj". Jesli cel jest
        widoczny, dron najpierw sie wycentruje; jesli nie - zrzuci na
        wspolrzedne. Milczenie operatora, gdy on PATRZY, znaczy "to nie ten
        cel" i wysyla nas do kolejnego kandydata.

        Dlaczego pytamy takze przy waypoincie z automatu: ten powstal z pulapu
        80 m, gdzie namiot ma 30 px, a operator ocenia go z 50 m, gdzie ma
        49 px, na ostrym obrazie. Jego dane sa wiec LEPSZE niz te, z ktorych
        powstal waypoint. Do tego klastrowanie nie chroni przed atrapa:
        setki zbieznych obserwacji odsiewaja szum losowy, ale obiekt
        systematycznie brany za namiot zbuduje rownie pewny klaster.

        Numer ladunku = numer klasy (namiot -> 0, czlowiek -> 1).
        """
        name, _key, topic = TARGETS[class_id]
        lat, lon, src, n_obs, _score, cand_id = cand
        self._known_ids.add(cand_id)
        self.set_detection_topic(topic)
        self.get_logger().info(
            f"╔══ {name} ══ kandydat: zrodlo={src} obs={n_obs}, "
            f"{self._dist_to(lat, lon):.0f} m stad")
        self.goto(lat, lon)

        widoczny = self.wait_acquire()
        if not widoczny:
            # ── Dolecielismy, ale detektor nic nie potwierdza ──
            if self.sweep_enabled:
                widoczny = self.sweep_for_target()
            if not widoczny and self.spiral_enabled:
                widoczny = self.spiral_search(lat, lon)

        if self.operator_watching():
            if widoczny:
                monit = ("Wykryty w kadrze.\n"
                         "[SPACJA] = wycentruj i zrzuc     "
                         "[nic] = to nie ten cel, szukam dalej")
            else:
                monit = ("NIE widze celu w kadrze.\n"
                         "[SPACJA] = zrzuc na wspolrzedne mimo to     "
                         "[nic] = szukam dalej")
            approved = self.wait_confirm(
                f"=== {name} (zrodlo: {src}, obs={n_obs}) ===\n"
                f"{monit}   ({self.confirm_timeout:.0f}s)")
        else:
            # Nikt nie patrzy — pytanie poszloby w prozni. Konczymy zrzutem,
            # zeby bez nadzoru nie krecic sie w kolko po kandydatach.
            approved = True
            self.get_logger().warn(
                f"{name}: operatora nie ma (brak pulsu z GUI) — nie pytam, "
                f"zrzucam na tym kandydacie")

        if not approved:
            self.get_logger().warn(
                f"{name}: operator NIE potwierdzil — kandydat odrzucony")
            return 'rejected'

        if widoczny and self.center_over_target():
            self.get_logger().info(f"{name}: ZRZUT NAD WYCENTROWANYM CELEM")
        else:
            self.get_logger().warn(f"{name}: ZRZUT NA WSPOLRZEDNE")
            self.goto(lat, lon)
        self.drop(class_id)
        return 'dropped'

    def search_and_drop(self, class_id) -> bool:
        """Grid, gdy dla klasy nie ma juz zadnego kandydata.

        Kreci sie BEZ KONCA, dopoki operator patrzy — konczy go spacja albo
        Ctrl+C (ktory robi abort i RTL). To jest bezpieczne wlasnie dlatego,
        ze warunkiem petli jest obecnosc operatora: zawsze jest ktos, kto moze
        ja przerwac. Gdy operatora nie ma, grid idzie RAZ, bo bez nadzoru
        nikt by drona nie zatrzymal.

        Trzy rzeczy moga wyrwac drona z gridu, kazda konczy sie ta sama
        sciezka (dolot -> akwizycja -> potwierdzenie -> centrowanie -> zrzut):
          * DETEKTOR domknal okno M z N w locie      (grid_det_interrupt)
          * w targets.json pojawil sie nowy cel      (watch_sources)
          * grid przelecial calosc i nic nie znalazl (auto_after_grid)
        """
        name, _key, topic = TARGETS[class_id]
        self.set_detection_topic(topic)
        while not self._abort:
            self.get_logger().info(
                f"{name}: brak kandydatow — przeszukuje teren gridem "
                f"(obserwuje targets.json co {self.watch_interval:.0f}s)")
            znalazl = self.grid_search(watch_class=class_id)

            # Cel pojawil sie w trakcie lotu — z klikniecia operatora albo
            # z automatu, ktory wlasnie przekroczyl min_obs. Wychodzimy z gridu
            # i lecimy nad niego normalna sciezka, dokladnie tak jakbysmy mieli
            # ten waypoint od poczatku: dolot, okno akwizycji, potwierdzenie,
            # centrowanie, zrzut.
            if self._interrupt_cand is not None:
                cand = self._interrupt_cand
                self._interrupt_cand = None
                if self.visit_candidate(class_id, cand) == 'dropped':
                    return True
                continue                     # odrzucony — wracamy do gridu

            if not znalazl:
                self.get_logger().error(f"{name}: grid nic nie znalazl")

                # Grid przeleciał CALOSC i nic nie zobaczyl. Dopiero teraz
                # dopuszczamy kandydata z automatu: jego klaster zbieral sie
                # przez caly przelot, wiec score odroznia falszywke widziana raz
                # od celu widzianego na kilku galsach. W trakcie gridu ten sam
                # kandydat byl swiadomie ignorowany (watch_sources).
                if self.auto_after_grid:
                    cand = self._best_auto_candidate(class_id)
                    if cand is not None:
                        self.get_logger().info(
                            f"{name}: po pelnym gridzie biore najlepszego "
                            f"kandydata z automatu (zrodlo={cand[2]}, "
                            f"obs={cand[3]}, score={cand[4]:.0f})")
                        if self.visit_candidate(class_id, cand) == 'dropped':
                            return True
                        # Odrzucony — siedzi juz w _known_ids, wiec kolejny
                        # obieg wezmie nastepnego albo wroci do gridu.
                        continue

                if not self.operator_watching():
                    return False
                continue                     # operator patrzy — szukamy dalej

            if self.operator_watching():
                approved = self.wait_confirm(
                    f"=== {name} (znaleziony gridem) ===\n"
                    f"[SPACJA] = wycentruj i zrzuc     "
                    f"[nic] = to nie ten cel, szukam dalej   "
                    f"({self.confirm_timeout:.0f}s)")
            else:
                approved = True

            if approved:
                if self.center_over_target():
                    self.get_logger().info(
                        f"{name}: ZRZUT NAD WYCENTROWANYM CELEM")
                    self.drop(class_id)
                    return True
                # Centrowanie sie nie udalo, a w gridzie NIE MA waypointu, na
                # ktory moglibysmy zrzucic w ciemno. Cel, ktory znika w trakcie
                # centrowania, to prawie na pewno falszywka — 2026-09-02 grid
                # potwierdzil w ten sposob drzewo. Zrzut byl wtedy najgorsza
                # z mozliwych decyzji: ladunek poszedl obok drzewa.
                self.get_logger().warn(
                    f"{name}: cel zniknal w trakcie centrowania — falszywka, "
                    f"NIE zrzucam")
                if not self.operator_watching():
                    # Bez operatora NIE wracamy do gridu: drugi przelot
                    # znalazlby te sama falszywke, centrowanie znow by padlo
                    # i petla nie mialaby jak sie skonczyc, bo nie ma nikogo,
                    # kto by ja przerwal. Zamiast tego jedno podejscie do
                    # najlepszego kandydata z automatu — jego klaster zbieral
                    # sie przez caly grid, wiec jest lepsza przeslanka niz
                    # cel, ktory wlasnie zniknal. Bez operatora to podejscie
                    # zawsze konczy sie zrzutem, wiec petla ma gwarantowany
                    # koniec, a ladunek nie wraca na pokladzie.
                    cand = (self._best_auto_candidate(class_id)
                            if self.auto_after_grid else None)
                    if cand is None:
                        self.get_logger().error(
                            f"{name}: bez operatora i bez kandydata z automatu "
                            f"— ladunek zostaje na pokladzie")
                        return False
                    self.get_logger().warn(
                        f"{name}: bez operatora — zamiast powtarzac grid biore "
                        f"kandydata z automatu (obs={cand[3]}, "
                        f"score={cand[4]:.0f})")
                    return self.visit_candidate(class_id, cand) == 'dropped'
                continue                     # operator patrzy — szukamy dalej

            if not self.operator_watching():
                return False
        return False

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
        # Od tej chwili gimbal jest nasz — geolokator ma przestac trzymac pion.
        self._nadir_pub.publish(Bool(data=False))
        self.descend(self.target_alt)

        targets = self.read_targets()
        # Wszystko, co bylo w pliku PRZED przejeciem lotu, jest juz "znane" —
        # inaczej pierwszy poll w trakcie gridu uznalby te cele za nowe i
        # wyrwal drona z przeszukiwania do punktu, ktory wlasnie odrzucil.
        for _lst in targets.values():
            for _c in _lst:
                self._known_ids.add(_c[5])

        # PULA KANDYDATOW OBU KLAS, ZAWSZE OD NAJLEPSZEGO. Kolejnosc wyznacza
        # score geolokatora (obs * conf), nie odleglosc: kandydat z 300
        # zbieznych obserwacji jest pewniejszy niz plandeka z 15, nawet jesli
        # plandeka lezy blizej. To samo, gdy operatora nie ma — wtedy zrzut
        # idzie na najlepszego, a nie na najblizszego.
        #
        # Ladunek jest przypisany do klasy (namiot -> 0, czlowiek -> 1), wiec po
        # zrzucie na klase jej pozostali kandydaci wypadaja z gry.
        pool = [(cid, c) for cid, lst in targets.items() for c in lst]
        done = set()

        while not self._abort and len(done) < len(TARGETS):
            avail = [p for p in pool if p[0] not in done]
            if not avail:
                break
            cid, cand = max(avail, key=lambda p: p[1][4])   # najwyzszy score
            if self.visit_candidate(cid, cand) == 'dropped':
                done.add(cid)
            else:
                pool.remove((cid, cand))

        # Klasy bez zrzutu, ktorym skonczyli sie kandydaci — zostaje grid.
        for cid in TARGETS:
            if self._abort or cid in done:
                continue
            self.search_and_drop(cid)

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
