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
        # Okno M z N nad waypointem. 5, a nie 10: jesli cel jest pod dronem,
        # bramka 4 z 8 domyka sie w ulamku sekundy. Dziesiec sekund milczenia
        # detektora nie znaczy "zaraz go zlapie", tylko "nie ma go w kadrze" —
        # a wtedy i tak konczy sie okno poprawki albo zrzut na wspolrzedne.
        p('wp_acquire', 5.0)
        # Gdy nad waypointem pod dronem nic nie widac: skan Z TEGO MIEJSCA,
        # tymi samymi katami co skan bez geolokatora. Adres bywa przesuniety
        # (slaba geolokalizacja, cel ruszyl sie miedzy ortofoto a misja),
        # a cel oddalony o kilkadziesiat metrow jest DALEJ W ZASIEGU — tylko
        # nie w nadirze. Bez tego jedyna reakcja na "nie widze" byl zrzut
        # w ciemno na wspolrzedne.
        p('wp_scan_on_miss', True)
        p('wp_scan_arc_deg', 360.0)
        p('wp_scan_timeout', 120.0)

        # ── OKNO NA POPRAWKE ADRESU ─────────────────────────────────
        # Otwiera sie dokladnie tam, gdzie misja i tak zrzucilaby w ciemno:
        # nad waypointem, bez zrzutu nad wycentrowanym celem. Dron wisi,
        # gimbal jest w pionie, wiec operator widzi w GUI to samo co widzial
        # geolokator — to sa najlepsze warunki do klikniecia w calym locie.
        #
        # Klik w GUI -> geolokator przepisuje targets.json (do 5 s) -> misja
        # widzi nowy adres i LECI TAM zamiast zrzucac. Cisza konczy okno
        # zrzutem: brak reakcji znaczy "adres jest dobry".
        #
        # 0 = wylaczone, czyli stare zachowanie (zrzut od razu).
        p('wp_fix_window', 10.0)
        # Ile adres musi sie przesunac, zeby uznac to za POPRAWKE. Klik w ten
        # sam klaster przesuwa jego srodek o ulamek metra i nie ma po co po to
        # przelatywac; ponizej center_tol_m (3 m) to i tak szum centrowania.
        p('wp_fix_tol', 5.0)
        # Ile razy z rzedu operator moze poprawiac. Kazda poprawka to nowy
        # dolot i nowe okno, wiec bez limitu dalo by sie oprowadzac drona po
        # polu az do wyczerpania baterii.
        p('wp_fix_max', 2)

        # ── CENTROWANIE NAD CELEM Z ADRESU (sciezka B) ──────────────
        # false = mimo ze detektor widzi cel pod dronem, misja NIE centruje sie
        # na nim, tylko zrzuca na wspolrzedne z adresu. Namiot: bramka 4 z 8
        # z ciagloscia track_id — jak detektor mowi "widze", to widzi. Czlowiek:
        # bramka 2 z 8 BEZ ciaglosci ID, wiec "widzialem" moze znaczyc dwie
        # klatki cienia, a adres to klaster setek obserwacji albo klik operatora.
        p('center_on_tent', True)
        p('center_on_person', False)

        # ── POTWIERDZANIE SPACJA ────────────────────────────────────
        # false = misja NIGDY nie pyta i nie czeka na SPACJE; zachowuje sie tak,
        # jakby suas_marker_web nie byl podlaczony. Kazde miejsce, ktore pyta,
        # ma juz galaz "bez operatora" i to ona jest brana.
        p('operator_confirm', False)

        # ── ZAWIS (wspolny ogon obu sciezek) ────────────────────────
        # Te cztery zyja w suas_full_mission, a nie w kontrolerze bazowym —
        # bez wlasnej deklaracji byloby AttributeError w approach_and_center,
        # czyli w ostatnim kroku przed zrzutem.
        p('approach_timeout', 90.0)      # gorny limit PODLOTU + centrowania
        # Limit SAMEGO ZAWISU (sciezka B: dron stoi juz nad celem). Osobny od
        # approach_timeout, bo tam w limicie miesci sie jeszcze przelot
        # kilkudziesieciu metrow. Gdy zawis nie domknie sie w tym czasie,
        # regulator oscyluje wokol center_tol_m i czekanie nic nie da; deliver()
        # schodzi wtedy na zrzut po wspolrzednych, wiec ladunek pada zawsze.
        p('hover_timeout', 10.0)
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

        # ── NIZSZE BRAMKI DLA CZLOWIEKA ─────────────────────────────
        # Namiot ma na 50 m 49 px i wykrywa sie w kazdej klatce. Czlowiek
        # LEZACY ma 30x22 px, a STOJACY 8 px — detektor lapie go z przerwami,
        # wiec te same progi co dla namiotu bywaja NIEPRZECHODZALNE i dron
        # nigdy nie wchodzi w podlot, mimo ze cel jest w kadrze.
        #
        # Ryzyko falszywki rosnie, ale konsekwencja jest ograniczona: po tej
        # bramce ida jeszcze SPRAWDZ na stojaco, CELUJ, SPACJA i UDANE
        # centrowanie — a track_id < 0 (podpis migoczacej falszywki) jest
        # odrzucany niezaleznie od progow.
        #
        # Ustaw rowne wartosciom wyzej, zeby wylaczyc nadpisanie.
        p('person_scan_confirm_frames', 2)
        p('person_scan_window_frames', 8)
        p('person_transit_confirm_frames', 3)
        p('person_transit_window_frames', 8)
        p('person_det_confirm_frames', 2)
        p('person_det_window_frames', 8)
        # NAJWAZNIEJSZE dla czlowieka. require_same_track robi dwie rzeczy:
        # odrzuca track_id < 0 (podpis migoczacej falszywki) ORAZ czysci okno
        # przy kazdej zmianie ID. Przy celu widzianym w ~28% klatek tracker
        # gubi sciezke i nadaje nowe ID przy kazdym powrocie, wiec okno nigdy
        # sie nie domyka — a trafienie z track_id < 0 nie odswieza nawet
        # last_det_time, przez co CELUJ od razu melduje "cel zniknal".
        #
        # UWAGA: to zdejmuje zabezpieczenie, ktore 2026-09-02 zlapalo drzewo
        # brane za czlowieka (4/6 klatek, ID=-1). Chroni nas juz tylko reszta
        # lancucha: SPRAWDZ na stojaco, CELUJ, SPACJA i UDANE centrowanie —
        # cel skaczacy po kadrze nie wycentruje sie, wiec zrzut nie padnie.
        p('person_require_same_track', False)
        # Przerwy w detekcji czlowieka sadaja 2 s i wiecej. Przy 3.0 kontroler
        # uznaje cel za zgubiony w srodku podlotu i wraca do SEARCH.
        p('person_lost_timeout', 5.0)
        p('scan_return_to_first', True)  # po ostatnim punkcie wroc na pierwszy
        p('scan_timeout', 600.0)
        # Po zrzucie na namiot: szukaj czlowieka tym samym skanem, zamiast
        # wracac z ladunkiem. W trakcie skanu na namiot NIKT nie obserwowal
        # klasy CZLOWIEK (watcher sluchal /tent_detections), wiec teren jest
        # pod tym katem nieogladany — mimo ze dron nad nim przelecial.
        p('search_person_after_tent', True)
        p('person_scan_timeout', 300.0)
        # ── PATROL ZA CZLOWIEKIEM ───────────────────────────────────
        # Czlowiek bez adresu NIE jest szukany skanem, tylko patrolem: dron
        # lata w kolko po wlasnej trasie z kamera w dol i czeka, az operator
        # kliknie go w GUI.
        #
        # Dlaczego nie skan: na 50 m stojacy czlowiek ma ok. 8 px, a YOLO ma
        # stride 8 — nie ma czego wykrywac. Lamanie trasy i obroty yaw pod
        # detektor nie kupuja wiec nic; jedyne, co dziala, to oko operatora
        # na podgladzie. Zadaniem drona jest WOZIC KAMERE nad terenem.
        p('person_waypoints',
          '~/Dron_symulacja/src/drone_bringup/config/czlowiek.waypoints')
        # 0 = BEZ LIMITU: patrol chodzi, az operator kliknie albo sam przejmie
        # lot (RTL z aparatury / Ctrl+C). Limit ma sens tylko wtedy, gdy nikt
        # nie patrzy na podglad — a wtedy patrol i tak nie ma po co latac.
        p('person_patrol_timeout', 0.0)
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
        self.wp_scan_on_miss = g('wp_scan_on_miss').value
        self.wp_scan_arc_deg = g('wp_scan_arc_deg').value
        self.wp_scan_timeout = g('wp_scan_timeout').value
        self.wp_fix_window = g('wp_fix_window').value
        self.wp_fix_tol = g('wp_fix_tol').value
        self.wp_fix_max = max(0, int(g('wp_fix_max').value))
        self.operator_confirm = g('operator_confirm').value
        self.center_on_target = {TENT: g('center_on_tent').value,
                                 PERSON: g('center_on_person').value}
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
        self.p_scan_m = g('person_scan_confirm_frames').value
        self.p_scan_n = g('person_scan_window_frames').value
        self.p_transit_m = g('person_transit_confirm_frames').value
        self.p_transit_n = g('person_transit_window_frames').value
        self.p_det_m = g('person_det_confirm_frames').value
        self.p_det_n = g('person_det_window_frames').value
        self.p_same_track = g('person_require_same_track').value
        self.p_lost_timeout = g('person_lost_timeout').value
        # Bramka kontrolera jest MUTOWANA per klasa (_apply_class), wiec
        # wartosci wyjsciowe trzeba zapamietac — inaczej po obsluzeniu
        # czlowieka namiot dostalby jego, luzniejsze progi.
        self.tent_det_m = self.det_confirm_frames
        self.tent_det_n = self.det_window_frames
        self.tent_same_track = self.require_same_track
        self.tent_lost_timeout = self.lost_timeout
        self.scan_return_to_first = g('scan_return_to_first').value
        self.scan_timeout = g('scan_timeout').value
        self.search_person_after_tent = g('search_person_after_tent').value
        self.person_scan_timeout = g('person_scan_timeout').value
        self.person_waypoints = os.path.expanduser(
            str(g('person_waypoints').value))
        self.person_patrol_timeout = g('person_patrol_timeout').value
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
        # To samo dla trasy patrolu za czlowiekiem — brak pliku wyszedlby
        # inaczej dopiero PO zrzucie na namiot, czyli po kilku minutach lotu.
        patrol = self._resolve_asset(self.person_waypoints)
        if os.path.isfile(patrol):
            self.get_logger().info(f"trasa patrolu (czlowiek): {patrol}")
        elif self.search_person_after_tent:
            self.get_logger().error(
                "BRAK PLIKU TRASY PATROLU — jesli czlowiek nie dostanie adresu "
                f"z geolokatora, dron nie ma gdzie patrolowac ({patrol})")

        self.get_logger().info(
            "centrowanie nad celem z adresu: "
            + ", ".join(f"{self.klasy[c][0]}="
                        + ("TAK" if self.center_on_target[c] else
                           "NIE, zrzut wprost na wspolrzedne")
                        for c in (TENT, PERSON)))
        self.get_logger().info(
            f"okno na poprawke adresu nad waypointem: "
            + (f"{self.wp_fix_window:.0f}s, prog {self.wp_fix_tol:.0f} m, "
               f"max {self.wp_fix_max} razy" if self.wp_fix_window > 0
               else "WYLACZONE — zrzut od razu"))
        if self.operator_confirm:
            self.get_logger().info(
                "potwierdzanie SPACJA: WLACZONE (o ile GUI zglasza sie na "
                "/operator_online)")
        else:
            self.get_logger().warn(
                "potwierdzanie SPACJA: WYLACZONE (operator_confirm=false) — "
                "misja nie pyta o nic i nie czeka; decyduje detektor")
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
        padl, ostatnie 'true' zostaloby w pamieci na zawsze.

        Przy operator_confirm=false klamiemy w dol niezaleznie od GUI — to jedno
        miejsce wylacza WSZYSTKIE pytania o spacje, bo kazde zaczyna sie od tego
        sprawdzenia.
        """
        if not self.operator_confirm:
            return False
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

    def read_targets(self, verbose=True):
        """targets.json -> {class_id: (lat, lon, zrodlo, n_obs)}.

        verbose=False dla odpytywania w petli (okno poprawki czyta plik co
        sekunde) — inaczej kazdy odczyt dopisywalby do logu wiek pliku
        i adres kazdej klasy.

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
            if verbose:
                (self.get_logger().warn if wiek > 300
                 else self.get_logger().info)(
                    f"{self.targets_json}: zapisany {wiek:.0f} s temu")
        except FileNotFoundError:
            if verbose:
                self.get_logger().warn(
                    f"brak {self.targets_json} — zadna klasa nie ma adresu, "
                    f"namiot idzie skanem")
                inne = self._resolve_asset(self.targets_json)
                if inne != os.path.expanduser(self.targets_json):
                    self.get_logger().warn(
                        f"UWAGA: plik o tej nazwie LEZY w {inne}. Jesli to on "
                        f"ma byc zrodlem adresow, popraw targets_json w yamlu "
                        f"— NIE biore go sam, bo moglby byc z innego lotu.")
            return {}
        except Exception as e:
            if verbose:
                self.get_logger().warn(
                    f"nie moge odczytac {self.targets_json}: {e}")
            return {}

        mins = data.get('min_obs') or {}
        out = {}
        for cid, (name, key, _topic) in self.klasy.items():
            sec = data.get(key) or {}
            best = sec.get('best')
            if not best:
                if verbose:
                    self.get_logger().warn(f"{name}: BRAK adresu")
                continue
            need = mins.get(key, 10)
            src = best.get('source', '?')
            n_obs = best.get('n_obs', 0)
            if src != 'operator' and n_obs < need:
                if verbose:
                    self.get_logger().warn(
                        f"{name}: adres odrzucony — {n_obs} obserwacji przy "
                        f"progu {need} (to poziom krzaka, nie celu)")
                continue
            try:
                blat, blon = float(best['lat']), float(best['lon'])
            except (KeyError, TypeError, ValueError):
                if verbose:
                    self.get_logger().error(
                        f"{name}: 'best' bez poprawnych wspolrzednych — pomijam")
                continue
            out[cid] = (blat, blon, src, n_obs)
            if verbose:
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
        # Limit inny dla kazdej sciezki: w A miesci sie jeszcze przelot
        # kilkudziesieciu metrow, w B tylko korekta w metrach.
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

    def _skan_z_waypointu(self, cid) -> bool:
        """Skan Z WAYPOINTU, gdy pod dronem nic nie widac. True = zrzucone.

        Po co: waypoint z geolokatora bywa przesuniety o kilkadziesiat metrow
        (slaba geolokalizacja, cel ruszyl sie miedzy przelotem ortofoto
        a misja). Dotad jedyna reakcja na "nie widze" byl zrzut w ciemno na
        wspolrzedne — a cel oddalony o 40 m jest DALEJ W ZASIEGU, tylko nie
        w nadirze. Te same katy co skan bez geolokatora siegaja ok. 78 m
        wokol waypointu, wiec obejmuja caly realny blad adresu.

        Cel znaleziony skanem jest OBOK, nie pod dronem, wiec dalsza obsluga
        idzie sciezka A (CELUJ, potwierdzenie, PODLOT, ZAWIS) — a nie samym
        zawisem, ktory zaklada cel w nadirze.
        """
        name, _key, topic = self.klasy[cid]
        self.get_logger().info(
            f"{name}: pod waypointem nic nie widac — skanuje "
            f"{self.wp_scan_arc_deg:.0f} st. z tego miejsca")

        # Watcher ZAWSZE swiezy: deliver woluje sie raz na klase, a stary
        # sluchalby topicu poprzedniej (namiot vs czlowiek).
        if self.watcher is not None:
            try:
                self.destroy_subscription(self.watcher.sub)
            except Exception:
                pass
        self._apply_class(cid)
        self.watcher = ScanWatcher(self, topic, *self._gate(cid, 'scan'))
        self._deadline = time.time() + self.wp_scan_timeout

        # Luk pelny, wiec kurs nie ma znaczenia — cel moze byc w dowolna strone
        # od waypointu.
        cells = self._build_scan_cells(self.wp_scan_arc_deg, 0.0)
        od = 0
        while od < len(cells) and not self._abort:
            trafil = self.scan_at_point(cells, od)
            if trafil is None:
                return False
            if self.handle_scan_target(class_id=cid,
                                       kontekst='skanem nad waypointem'):
                return True
            # Falszywka: wznawiamy od NASTEPNEJ komorki zamiast porzucac
            # reszte obwodu.
            od = trafil + 1
        return False

    def _zrzut_na_wspolrzedne(self, cid, lat, lon) -> bool:
        """Sciezka domyslna. Wracamy nad sam waypoint, bo skan albo centrowanie
        mogly nas z niego zsunac, a adres jest tym, w co naprawde wierzymy."""
        name = self.klasy[cid][0]
        self.get_logger().warn(f"{name}: ZRZUT NA WSPOLRZEDNE")
        self.action_interrupt = None
        self._gimbal(self.pitch_min)
        self.send_goto_global(lat, lon, self.target_alt)
        self._spin(1.0)
        self.drop(cid)
        return True

    def _okno_poprawki(self, cid, lat, lon):
        """Okno na POPRAWKE adresu klikiem w GUI.

        Zwraca (lat, lon, zrodlo, n_obs) nowego adresu albo None, gdy operator
        nie zareagowal.

        Otwiera sie tam, gdzie misja i tak zrzucilaby w ciemno: dron wisi nad
        waypointem, gimbal jest w pionie, wiec operator widzi w GUI to samo, co
        widzial geolokator — najlepsze warunki do klikniecia w calym locie.

        Cisza konczy okno zrzutem. Brak reakcji ma znaczyc "adres jest dobry",
        a nie "operator zasnal": jak nikogo nie ma, zrzut i tak jest tym, co
        misja zrobilaby bez tej funkcji.
        """
        if self.wp_fix_window <= 0:
            return None
        name = self.klasy[cid][0]
        self.get_logger().warn(
            f"{name}: masz {self.wp_fix_window:.0f} s na POPRAWKE — kliknij "
            f"w GUI, jesli cel jest gdzie indziej. Cisza = zrzut na ten punkt")

        koniec = time.time() + self.wp_fix_window
        nastepny = 0.0
        while rclpy.ok() and not self._abort and time.time() < koniec:
            rclpy.spin_once(self, timeout_sec=0.1)
            now = time.time()
            if now < nastepny:
                continue
            # Czytamy raz na sekunde, nie w kazdej iteracji: geolokator
            # przepisuje plik co report_period (5 s), wiec czestsze zagladanie
            # to samo obciazenie dysku.
            nastepny = now + 1.0
            cel = self.read_targets(verbose=False).get(cid)
            if cel is None:
                continue
            nlat, nlon, nsrc, nobs = cel
            d = math.hypot((nlat - lat) * M_LAT,
                           (nlon - lon) * _m_per_deg_lon(lat))
            if d < self.wp_fix_tol:
                continue
            self.get_logger().warn(
                f"{name}: POPRAWKA — adres przesuniety o {d:.0f} m "
                f"(zrodlo={nsrc}, obs={nobs}), lece na nowy punkt")
            return nlat, nlon, nsrc, nobs
        return None

    def deliver(self, cid, lat, lon, src, n_obs) -> bool:
        """Dolot na waypoint i zrzut. Zawsze konczy sie zrzutem.

        Detektor MILCZY przez cala droge (action_interrupt=None). Adres to
        klaster z setek obserwacji albo klik operatora; okno M z N zlapane
        w locie jest przy tym slaba przeslanka, a zatrzymanie sie dla krzaka
        po drodze oznaczaloby zrzut ladunku TEJ klasy obok niego i porzucenie
        adresu, ktory ktos juz zweryfikowal.

        Dolot jest PETLA, nie jednym przelotem: za kazdym razem, gdy nad
        waypointem nie doszlo do zrzutu nad wycentrowanym celem, operator
        dostaje wp_fix_window sekund na poprawienie adresu klikiem. Poprawka
        oznacza nowy dolot i nowe okno — do wp_fix_max razy.
        """
        name, _key, topic = self.klasy[cid]
        self._apply_class(cid)
        self.set_detection_topic(topic)
        self.action_interrupt = None
        self.get_logger().info(
            f"╔══ {name} ══ DOWIEZ: {self._dist_to(lat, lon):.0f} m stad "
            f"(zrodlo={src}, obs={n_obs}), detektor milczy w drodze")

        self._gimbal(self.pitch_min)

        widoczny = False
        for runda in range(self.wp_fix_max + 1):
            self.send_goto_global(lat, lon, self.target_alt)

            # Nad waypointem cel ma byc POD nami — gimbal w pion, inaczej okno
            # akwizycji nie mialoby szans.
            self._gimbal(self.pitch_min)
            self._spin(1.0)
            blad = self._dist_to(lat, lon)
            self.get_logger().info(
                f"nad waypointem (blad {blad:.1f} m), gimbal w pionie")

            # ── Budzet 20 s: od tej chwili do decyzji ────────────────
            # Liczony od NOWA przy kazdej rundzie: poprawka to osobny dolot,
            # a nie dalszy ciag poprzedniego.
            t0 = time.time()
            zostalo = lambda: max(0.0, self.wp_budget - (time.time() - t0))

            # Akwizycja tylko wtedy, gdy jej wynik cokolwiek zmienia. Przy
            # center_on_* = false i wp_scan_on_miss = false nikt go nie czyta,
            # a kosztuje do wp_acquire sekund zawisu (klasyczny przypadek:
            # czlowiek, ktory i tak leci na wspolrzedne).
            if self.center_on_target[cid] or self.wp_scan_on_miss:
                widoczny = self.wait_acquire(
                    timeout=min(self.wp_acquire, zostalo()))
            else:
                widoczny = False
                self.get_logger().info(
                    f"{name}: bez centrowania i bez skanu — pomijam akwizycje")

            # Centrowanie mozna WYLACZYC per klasa. Rozdzielone od `widoczny`
            # celowo: gdy jest wylaczone, NIE chcemy wpasc w galaz
            # wp_scan_on_miss ponizej, bo cel przeciez widac.
            centruj = widoczny
            if widoczny and not self.center_on_target[cid]:
                centruj = False
                self.get_logger().info(
                    f"{name}: cel w kadrze, ale centrowanie dla tej klasy jest "
                    f"wylaczone — zrzut na wspolrzedne z adresu")

            if centruj:
                if self._centruj_i_zrzuc(cid, src, n_obs, zostalo):
                    return True

            # Tu jestesmy tylko wtedy, gdy do zrzutu nad wycentrowanym celem
            # NIE doszlo. Zanim zejdziemy na wspolrzedne — daj operatorowi
            # szanse wskazac wlasciwy punkt.
            if runda == self.wp_fix_max:
                # Ostatni dolot: adres jest juz poprawiony wp_fix_max razy,
                # wiec zamiast otwierac kolejne okno — zrzucamy.
                if self.wp_fix_max > 0:
                    self.get_logger().warn(
                        f"{name}: limit poprawek ({self.wp_fix_max}) "
                        f"wyczerpany — zrzut na ten adres")
                break
            nowy = self._okno_poprawki(cid, lat, lon)
            if nowy is None:
                break
            lat, lon, src, n_obs = nowy

        # Nie ma go POD dronem. Zanim zrzucimy w ciemno — sprawdzmy, czy nie
        # stoi OBOK (patrz _skan_z_waypointu). Skan konczy sie albo zrzutem
        # nad wycentrowanym celem, albo niczym; w obu razach dalej juz nie
        # pytamy, bo handle_scan_target zadal swoje pytanie.
        if not widoczny and self.wp_scan_on_miss:
            if self._skan_z_waypointu(cid):
                return True

        return self._zrzut_na_wspolrzedne(cid, lat, lon)

    def _centruj_i_zrzuc(self, cid, src, n_obs, zostalo) -> bool:
        """Cel widoczny pod dronem: (opcjonalne pytanie) -> zawis -> ZRZUT.

        True = ladunek poszedl. False = centrowanie sie nie udalo albo operator
        nie potwierdzil; decyzje co dalej podejmuje deliver().
        """
        name = self.klasy[cid][0]
        # Pytanie zadawane RAZ. Nawet jesli cel zniknie i wroci, nie pytamy
        # drugi raz — to jest zabezpieczenie przed petla nad jednym punktem.
        if not self.operator_watching():
            # Bez operatora nie pytamy i nie czekamy: cel JEST w kadrze,
            # a nieudane centrowanie i tak schodzi nizej na wspolrzedne.
            approved = True
            self.get_logger().info(
                f"{name}: bez pytania o spacje — centruje na widocznym celu")
        else:
            czas = min(self.confirm_timeout, zostalo())
            approved = False
            if czas > 0.5:
                approved = self.wait_confirm(
                    f"=== {name} (zrodlo: {src}, obs={n_obs}) ===\n"
                    f"Widze cel w kadrze.\n"
                    f"[SPACJA] = wycentruj na nim i zrzuc     "
                    f"[nic] = zrzut na wspolrzedne waypointu   "
                    f"({czas:.0f}s)", timeout=czas)
            else:
                self.get_logger().warn(
                    f"{name}: budzet {self.wp_budget:.0f}s wyczerpany "
                    f"przed pytaniem")

        if not approved:
            return False
        if self.approach_and_center(over_target=True):
            self.get_logger().info(f"{name}: ZRZUT NAD WYCENTROWANYM CELEM")
            self.drop(cid)
            return True
        self.get_logger().warn(
            f"{name}: centrowanie sie nie udalo — schodze na wspolrzedne")
        return False

    # ═══════════════════════════════════════════════════════════
    #  SCIEZKA A — skan
    # ═══════════════════════════════════════════════════════════

    def _gate(self, cid, faza):
        """Progi M z N dla danej KLASY i FAZY. Czlowiek ma wlasne, nizsze."""
        if cid == PERSON:
            return {'scan': (self.p_scan_m, self.p_scan_n),
                    'transit': (self.p_transit_m, self.p_transit_n),
                    'det': (self.p_det_m, self.p_det_n)}[faza]
        return {'scan': (self.scan_m, self.scan_n),
                'transit': (self.transit_m, self.transit_n),
                'det': (self.tent_det_m, self.tent_det_n)}[faza]

    def _apply_class(self, cid):
        """Przestaw KONTROLER pod dana klase: bramka M z N, wymog ciaglosci
        ID trackera i czas, po ktorym cel uznajemy za zgubiony.

        Wszystkie trzy sa w klasie bazowej jako pojedyncze wartosci, wiec
        misja je MUTUJE przy przejsciu miedzy klasami. Wartosci wyjsciowe
        (namiotowe) siedza w tent_*, zeby dalo sie wrocic.

        Okno jest kroczacym buforem o stalej dlugosci, wiec zmiana N wymaga
        zbudowania go od nowa — sama podmiana liczby nic by nie dala.
        """
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
        """Cisza dla obserwatora skanu — bezpieczna, gdy watchera nie ma."""
        if self.watcher is not None:
            self.watcher.mute(seconds)

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

    def handle_scan_target(self, w_locie=False, class_id=TENT,
                           kontekst='skanem') -> bool:
        """Cel zobaczony w skanie albo w przelocie — sciezka A z sekcji 2.

        STOP jest konieczny, bo przerwanie akcji NIE zatrzymuje drona:
        ArduPilot w GUIDED trzyma ostatni zadany punkt. W samym skanie dron
        juz stoi i to wyjdzie na to samo, ale w przelocie nie.

        SPRAWDZ na stojaco idzie PELNYM oknem kontrolera (4 z 8), a nie
        czulszym oknem skanu — skan ma nie przegapic, sprawdzenie ma sie nie
        dac oszukac.
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

        # SPACJA bramkuje PODLOT. Gdy operatora NIE MA, pytanie poszloby
        # w prozni — wtedy decyduje sam detektor, ale zrzut idzie WYLACZNIE
        # po udanym wycentrowaniu. Nie ma tu adresu, wiec nie ma czego zrzucic
        # w ciemno, a cel znikajacy w centrowaniu to prawie na pewno falszywka.
        if self.operator_watching():
            approved = self.wait_confirm(
                f"=== {name} (znaleziony {kontekst}) ===\n"
                f"[SPACJA] = podlec i zrzuc     "
                f"[nic] = to nie ten cel, skanuje dalej   "
                f"({self.confirm_timeout:.0f}s)")
            if not approved:
                self._mute(cisza)
                self.get_logger().warn(
                    f"{name}: operator NIE potwierdzil — wracam do skanu")
                return False
        else:
            self.get_logger().warn(
                f"{name}: bez potwierdzania — decyduje detektor, zrzut tylko "
                f"po udanym wycentrowaniu"
                + ("" if self.operator_confirm else " (operator_confirm=false)"))

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
        # Cisza z PRZELOTU nie obowiazuje w skanie. Inaczej falszywka zlapana
        # w drodze oslepia poczatek skanu na nowym punkcie — 2026-09-07
        # wyciela 4 z 6 pozycji yaw, a namiot byl na 5/6.
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
            # Falszywka: wznawiamy od NASTEPNEJ komorki, zamiast porzucac
            # reszte punktu.
            od = trafil + 1
        return False

    def _od_najblizszego(self, trasa):
        """Kolejnosc "najblizszy sasiad", liczona od BIEZACEJ pozycji drona.

        Z miejsca, w ktorym stoimy, do najblizszego punktu, potem za kazdym
        razem do najblizszego z pozostalych. Cykliczne przesuniecie listy
        (WP3 -> WP1 -> WP2) byloby gorsze, bo trasa jest LINIA, a nie petla:
        przy trzech punktach dawalo 425 m przelotu zamiast 300 m.

        Wejscie i wyjscie: [(indeks_w_pliku, lat, lon), ...].
        """
        zostalo, out = list(trasa), []
        poz_lat, poz_lon = self.global_lat, self.global_lon
        while zostalo:
            j = min(range(len(zostalo)),
                    key=lambda z: math.hypot(
                        (zostalo[z][1] - poz_lat) * M_LAT,
                        (zostalo[z][2] - poz_lon) * _m_per_deg_lon(poz_lat)))
            out.append(zostalo[j])
            poz_lat, poz_lon = zostalo[j][1], zostalo[j][2]
            zostalo.pop(j)
        return out

    def _adres_operatora(self, cid):
        """Adres klasy z targets.json, ale TYLKO gdy pochodzi od operatora.

        Warunek jest twardy i niezalezny od operator_only w geolokatorze:
        patrol czeka na klik, nie na automat. Klaster automatu na czlowieku to
        najczesciej cien albo krzak — na 50 m czlowiek ma 8 px, czyli ponizej
        stride'u YOLO, wiec cokolwiek model tam "widzi", nie jest czlowiekiem.
        """
        cel = self.read_targets(verbose=False).get(cid)
        if cel is None or cel[2] != 'operator':
            return None
        return cel

    def patrol_operatora(self, czekam):
        """Patrol z kamera w dol, w oczekiwaniu na klik operatora.

        Zwraca (class_id, lat, lon, zrodlo, n_obs) wskazane przez operatora albo None,
        gdy patrol przerwano (Ctrl+C, limit czasu, brak trasy).

        Zastepuje skan: dron lata w kolko po wlasnej trasie na target_alt
        z gimbalem w pionie i nie probuje niczego wykrywac. Detektor jest tu
        bez znaczenia — liczy sie tylko to, co operator zobaczy na podgladzie.

        `czekam` to zbior klas, ktorych ladunek NADAL jest na pokladzie.
        Patrol pilnuje ich WSZYSTKICH, nie tylko czlowieka: po nieudanym skanie
        namiot tez zostaje nieoddany, a operator, ktory widzi go na podgladzie,
        musi miec jak go wskazac. Bez tego dron krazylby z ladunkiem namiotu
        nad polem, patrzac wylacznie, czy nie pojawil sie czlowiek.

        Gdy w jednym odczycie sa adresy obu klas, decyduje kolejnosc z run():
        najpierw namiot. Ten sam powod co tam — namiot jest pewniejszy.

        Klik jest sprawdzany takze W TRAKCIE dolotu (action_interrupt), wiec
        dron reaguje w ciagu sekundy zamiast dopiero nad kolejnym punktem —
        przy galsie 250 m to roznica minuty.
        """
        czekam = [c for c in (TENT, PERSON) if c in czekam]
        if not czekam:
            return None
        opis = "/".join(self.klasy[c][0] for c in czekam)
        sciezka = self._resolve_asset(self.person_waypoints)
        bazowe = self._load_waypoints(sciezka)
        if not bazowe:
            self.get_logger().error(
                f"brak trasy patrolu ({sciezka}) — nie mam gdzie latac")
            return None

        trasa = self._od_najblizszego(
            [(i, la, lo) for i, (la, lo) in enumerate(bazowe)])

        self.action_interrupt = None
        self._gimbal(self.pitch_min, force=True)

        self._patrol_cel = None
        self._patrol_next = 0.0

        def _sprawdz():
            """Pierwsza klasa z adresem od operatora, w kolejnosci `czekam`."""
            for c in czekam:
                cel = self._adres_operatora(c)
                if cel is not None:
                    return (c,) + tuple(cel)
            return None

        def _klik():
            """Haczyk dla _send_action: True = przerwij dolot, mamy adres."""
            now = time.time()
            if now < self._patrol_next:
                return False
            self._patrol_next = now + 1.0
            self._patrol_cel = _sprawdz()
            return self._patrol_cel is not None

        limit = self.person_patrol_timeout
        t0 = time.time()
        self.get_logger().info(
            f"╔══ PATROL ({opis}) ══ {len(trasa)} punktow, "
            + " -> ".join(f"WP{i + 1}" for i, _la, _lo in trasa)
            + f", {self.target_alt:.0f} m, gimbal w pionie | "
            + ("bez limitu czasu — czekam na klik operatora"
               if limit <= 0 else f"limit {limit:.0f}s"))

        okrazenie = 0
        bledy = 0
        try:
            while rclpy.ok() and not self._abort:
                okrazenie += 1
                if okrazenie > 1:
                    self.get_logger().info(f"patrol: okrazenie {okrazenie}")
                for _idx, wlat, wlon in trasa:
                    if self._abort:
                        break
                    if limit > 0 and time.time() - t0 > limit:
                        self.get_logger().warn(
                            f"patrol: limit {limit:.0f}s — koncze")
                        return None
                    self._patrol_cel = _sprawdz()
                    if self._patrol_cel is not None:
                        break
                    # Serwo jest bez sprzezenia, wiec kat dopisujemy przy
                    # kazdym odcinku — to jedna komenda, a gwarantuje, ze
                    # kamera patrzy w dol takze po restarcie czegokolwiek.
                    self._gimbal(self.pitch_min)
                    self.action_interrupt = _klik
                    try:
                        doszedl = self.send_goto_global(
                            wlat, wlon, self.target_alt)
                    finally:
                        self.action_interrupt = None
                    if self._patrol_cel is not None:
                        break
                    # Nieudany dolot bez adresu to zwykle znak, ze dron nie
                    # jest juz nasz (operator przejal lot, RTL z aparatury).
                    # Dobijanie sie kolejnymi komendami walczyloby z nim.
                    bledy = 0 if doszedl else bledy + 1
                    if bledy >= 3:
                        self.get_logger().error(
                            "patrol: trzy doloty z rzedu nieudane — przerywam "
                            "(dron w innym trybie?)")
                        return None
                if self._patrol_cel is not None:
                    pcid, plat, plon, _psrc, _pobs = self._patrol_cel
                    self.get_logger().info(
                        f"ZNACZNIK OPERATORA [{self.klasy[pcid][0]}] "
                        f"{plat:.7f} {plon:.7f} — przerywam patrol, "
                        f"lece dowiezc")
                    return self._patrol_cel
        finally:
            self.action_interrupt = None
        return None

    def search_class(self, cid, skan_w_miejscu=False, budzet=None) -> bool:
        """Skan trasy dla JEDNEJ klasy. True = ladunek poszedl.

        Detektor patrzy rowniez w przelocie miedzy punktami, bo tu nie ma
        lepszego adresu niz to, co widac.

        skan_w_miejscu=True: zacznij od pelnego obrotu TU, GDZIE JESTESMY,
        a trase zacznij od NAJBLIZSZEGO punktu i idz dalej cyklicznie.
        Sluzy do szukania czlowieka po zrzucie na namiot: dron stoi wtedy nad
        namiotem, czyli gdzies posrodku pola, a nie na WP1. Powrot na WP1 tylko
        po to, zeby zaczac "od poczatku", bylby czystym przelotem bez zysku —
        a obrot w miejscu jest darmowy, bo dron juz tam wisi.
        """
        name, _key, topic = self.klasy[cid]
        bazowe = self._load_waypoints(self._resolve_asset(self.scan_waypoints))
        if not bazowe:
            return False

        # Kazdy punkt niesie SWOJ indeks w konfiguracji, bo kolejnosc moze sie
        # zmienic (start od najblizszego), a luk i kurs sa przypisane do
        # PUNKTU, nie do miejsca w kolejce.
        trasa = [(i, la, lo) for i, (la, lo) in enumerate(bazowe)]
        if skan_w_miejscu:
            trasa = self._od_najblizszego(trasa)
        elif self.scan_return_to_first and len(bazowe) > 1:
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
                # wracamy na kurs do TEGO SAMEGO punktu, zeby nie zgubic
                # kawalka trasy.
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
                if self.search_class(TENT):
                    done.add(cid)
            elif cid == PERSON and not self.search_person_after_tent:
                self.get_logger().warn(
                    f"{name}: brak adresu, a search_person_after_tent=false "
                    f"— nie szukam, ladunek wraca.")

        # ── PATROL: wszystko, co zostalo na pokladzie, czeka na operatora ──
        # Czlowiek trafia tu zawsze, gdy nie mial adresu (detektor nie ma
        # szans: 8 px przy stride 8). Namiot trafia tu po NIEUDANYM skanie —
        # inaczej dron krazylby z jego ladunkiem, a operator widzialby go na
        # podgladzie bez zadnego sposobu, zeby to zglosic.
        if self.search_person_after_tent:
            while not self._abort:
                czekam = [c for c in (TENT, PERSON) if c not in done]
                if not czekam:
                    break
                self.get_logger().info(
                    "na pokladzie zostalo: "
                    + ", ".join(self.klasy[c][0] for c in czekam)
                    + " — patroluje i czekam na klik operatora")
                cel = self.patrol_operatora(czekam)
                if cel is None:
                    break
                pcid, plat, plon, psrc, pobs = cel
                if not self.deliver(pcid, plat, plon, psrc, pobs):
                    # deliver() konczy sie zrzutem na kazdej sciezce, wiec to
                    # nie powinno sie zdarzyc. Gdyby jednak — NIE wracamy do
                    # patrolu: ten sam adres wciaz lezy w targets.json, wiec
                    # patrol oddalby go natychmiast i petla kręciłaby sie
                    # w miejscu, nawet nie latajac.
                    self.get_logger().error(
                        f"{self.klasy[pcid][0]}: dowiezienie nieudane — "
                        f"koncze patrol, ladunek zostaje")
                    break
                done.add(pcid)

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
