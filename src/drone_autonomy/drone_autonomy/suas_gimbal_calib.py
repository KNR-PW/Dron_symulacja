#!/usr/bin/env python3
"""
suas_gimbal_calib — znajdz PWM, przy ktorym kamera patrzy DOKLADNIE w nadir.

NARZEDZIE POLOWE, nie czesc misji. Nie zmienia niczego na stale: liczy liczbe,
ktora sam wpisujesz do drone_handler.gimbal_pitch_callback. Zaden inny wezel
nie wie o jego istnieniu.

PO CO TO JEST
-------------
Cala dokladnosc geolokatora stoi na jednym zalozeniu: ze przy komendzie -90 st.
kamera patrzy pionowo w dol. Ale kat serwa zalezy od mechaniki — orczyka,
wydruku, luzu — a nie od tego, co jest wpisane w kodzie. Na 80 m KAZDY stopien
bledu montazu to 1.4 m przesuniecia celu w terenie, zawsze w te sama strone
(blad systematyczny, wiec usrednianie po wielu obserwacjach go NIE skasuje).
Obecna kalibracja w drone_handler (-90 = 1100 us) to wartosc z biurka, nigdy
nie zmierzona w locie.

ZASADA POMIARU
--------------
Zostaw znacznik na ziemi w miejscu startu, wznies sie PIONOWO na 80 m i wisz.
Punkt zostaje dokladnie pod dronem, wiec idealna kamera pokazuje go w srodku
kadru. Klikasz go w marker_web (:5000), a ten wezel z piksela + telemetrii
liczy wstecz, pod jakim katem kamera NAPRAWDE jest, i podaje PWM do poprawki.

Dlaczego nie wystarczy "patrz, czy punkt jest na srodku": gimbal nie jest
stabilizowany, wiec kamera pochyla sie razem z rama. Przy zawisie kadlub trzyma
1-2 st., a to juz 14-28 px przesuniecia — tyle samo, ile chcemy zmierzyc.
Dlatego _project_pixel dostaje roll/pitch z chwili KLATKI i odejmuje je
rachunkiem. Sprawdzone numerycznie: przy kamerze -83.5 st. i dronie
roll +0.8 / pitch +1.5 solver odzyskuje -83.50 st., blad 0.000 st.

Skala: 1 st. = 14.2 px = 1.40 m na 80 m; 1 us = 0.090 st. = 1.28 px.
Czyli metoda rozroznia pojedyncze mikrosekundy i nie ma sensu szukac
dokladniej niz ~5 us.

CZEGO WEZEL NIE POPRAWI
-----------------------
Gimbal ma JEDNA os (pitch), wiec umie skasowac tylko blad wzdluz osi drona.
Skladowa BOCZNA (kamera wkrecona krzywo w uchwyt albo staly offset rolla)
zostaje i jest raportowana osobno jako "reszta boczna". Jesli wychodzi duza,
to nie jest robota dla PWM — to albo mechanika, albo parametr
cam_yaw_offset_deg geolokatora.

URUCHOMIENIE
------------
Terminal 1 — normalny stack (handler + kamera + detektor + geolokator):
    ros2 launch drone_bringup suas_bringup.launch.py

Terminal 2 — ten wezel:
    python3 ~/Dron_symulacja/src/drone_autonomy/drone_autonomy/suas_gimbal_calib.py

  (albo `ros2 run drone_autonomy suas_gimbal_calib`, jesli dopiszesz wpis
   w setup.py — patrz docs/gimbal_kalibracja_nadir.md)

Terminal 3 — zmiana PWM w locie, gdy chcesz dojechac recznie:
    ros2 param set /suas_gimbal_calib pwm 1075
    ros2 topic pub --once /gimbal_calib/nudge std_msgs/msg/Int32 "{data: -10}"
    ros2 topic pub --once /gimbal_calib/reset std_msgs/msg/Bool "{data: true}"

UWAGA — dwie rzeczy, ktore cicho zabijaja kalibracje:
  * SERVO7_FUNCTION musi byc 0. Przy wartosci 7 (mount) ArduPilot nadpisuje
    DO_SET_SERVO wlasna funkcja wyjscia i PWM z tego wezla NIE DOCHODZI do
    serwa — a w logach wszystko wyglada normalnie.
  * SERVO7_MIN utnie PWM od dolu. Przy SERVO7_MIN=1100 nie zejdziesz ponizej
    1100 us, choc wezel bedzie o to prosil. Wezel ostrzega, gdy sugestia
    wpada pod ten prog (parametr fc_servo_min).

Nie uruchamiaj rownolegle suas_gimbal_controller — bilby sie o gimbal.
"""

import bisect
import math
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Bool, Int32

from drone_autonomy.geometry import _project_pixel
from drone_interfaces.msg import Telemetry, OperatorMark
from drone_interfaces.srv import SetServo


# Zakres i krok przeszukiwania kata montazu. -140..-40 z zapasem obejmuje
# kazdy realny blad orczyka; 0.25 st. zgrubnie, potem 0.01 st. dokladnie
# (0.01 st. = 0.14 px, czyli grubo ponizej szumu klikniecia).
SCAN_LO, SCAN_HI = -140.0, -40.0
SCAN_COARSE, SCAN_FINE = 0.25, 0.01


class SuasGimbalCalib(Node):

    def __init__(self):
        super().__init__('suas_gimbal_calib')

        # ── PWM, ktore trzymamy na serwie ───────────────────────────
        self.declare_parameter('pwm', 1100)
        self.declare_parameter('servo_id', 7)
        # Krance BEZPIECZENSTWA tego wezla — szersze niz SERVO7_MIN/MAX, bo
        # caly sens kalibracji to sprawdzic, czy prawda lezy poza obecnym
        # zakresem. Fizyczny kraniec i tak pilnuje ArduPilot.
        self.declare_parameter('pwm_min', 1000)
        self.declare_parameter('pwm_max', 2000)
        # SERVO7_MIN z ArduPilota — tylko po to, zeby ostrzec, ze FC utnie
        # sugerowana wartosc. Wezel tego nie ustawia i nie czyta z FC.
        self.declare_parameter('fc_servo_min', 1100)

        # ── Kalibracja, ktora chcemy poprawic ───────────────────────
        # Te same liczby co w drone_handler.gimbal_pitch_callback. Trzymane
        # jako parametry, zeby dalo sie zmierzyc takze inny kat niz -90.
        self.declare_parameter('target_angle_deg', -90.0)   # a1: kat kalibrowany
        self.declare_parameter('ref_angle_deg', -45.0)      # a2: drugi punkt
        self.declare_parameter('ref_pwm', 1600)             # p2
        # Skala serwa. Pomiar jednopunktowy NIE mierzy skali, tylko offset,
        # wiec skale bierzemy z obecnej kalibracji: (1600-1100)/45 = 11.111.
        self.declare_parameter('us_per_deg', 500.0 / 45.0)

        # ── Geometria kamery — MUSI byc taka sama jak w geolokatorze ─
        self.declare_parameter('focal_px', 813.0)
        self.declare_parameter('img_w', 1024)
        self.declare_parameter('img_h', 1024)
        self.declare_parameter('cam_yaw_offset_deg', 0.0)
        self.declare_parameter('gimbal_stabilized', False)

        # ── Zachowanie ──────────────────────────────────────────────
        # Geolokator z lock_nadir=true wysyla -90 co 2 s, co kazaloby handlerowi
        # nadpisac nasze surowe PWM wartoscia 1100. Zwalniamy blokade — od tej
        # chwili gimbal jest nasz, a geolokator nadal rzutuje zakladajac -90
        # (jego mount_pitch zostaje przy domyslnej wartosci, bo nikt juz nie
        # publikuje na gimbal_pitch). To jest dokladnie ten uklad, ktorego
        # chcemy: on wierzy w -90, my sprawdzamy, czy to prawda.
        self.declare_parameter('release_nadir', True)
        self.declare_parameter('hold_rate', 1.0)
        self.declare_parameter('min_alt', 20.0)
        self.declare_parameter('stamp_max_skew', 60.0)
        self.declare_parameter('telemetry_samples', 300)
        self.declare_parameter('telemetry_max_age', 0.5)

        p = self.get_parameter
        self.pwm = int(p('pwm').value)
        self.servo_id = int(p('servo_id').value)
        self.pwm_min = int(p('pwm_min').value)
        self.pwm_max = int(p('pwm_max').value)
        self.fc_servo_min = int(p('fc_servo_min').value)
        self.target_angle = float(p('target_angle_deg').value)
        self.ref_angle = float(p('ref_angle_deg').value)
        self.ref_pwm = int(p('ref_pwm').value)
        self.us_per_deg = float(p('us_per_deg').value)
        self.focal_px = float(p('focal_px').value)
        self.cx = p('img_w').value / 2.0
        self.cy = p('img_h').value / 2.0
        self.cam_yaw_offset = math.radians(p('cam_yaw_offset_deg').value)
        self.gimbal_stabilized = bool(p('gimbal_stabilized').value)
        self.release_nadir = bool(p('release_nadir').value)
        self.min_alt = float(p('min_alt').value)
        self.stamp_max_skew = float(p('stamp_max_skew').value)
        self.telemetry_max_age = float(p('telemetry_max_age').value)
        n_telem = max(2, int(p('telemetry_samples').value))
        hold_rate = float(p('hold_rate').value)

        # ── STAN ────────────────────────────────────────────────────
        self._telem = deque(maxlen=n_telem)
        self._telem_t = deque(maxlen=n_telem)
        # Kazdy pomiar to slownik; kluczowe pole to 'pwm_target' — PWM
        # sugerowane dla kata docelowego. Jest NIEZALEZNE od tego, przy jakim
        # PWM klikales, wiec pomiary z roznych ustawien wolno usredniac
        # i srednia zbiega sie w miare klikania.
        self._meas = []
        self._pending = None      # jedno wywolanie serwisu w locie naraz
        self._warned_fn = False

        # ── PUB / SUB / SRV ─────────────────────────────────────────
        self._lock_pub = self.create_publisher(Bool, '/geolocator/lock_nadir', 10)
        self.create_subscription(OperatorMark, '/operator_mark', self._mark_cb, 10)
        self.create_subscription(Telemetry, 'knr_hardware/telemetry',
                                 self._telem_cb, 10)
        self.create_subscription(Int32, '/gimbal_calib/pwm', self._pwm_topic_cb, 10)
        self.create_subscription(Int32, '/gimbal_calib/nudge', self._nudge_cb, 10)
        self.create_subscription(Bool, '/gimbal_calib/reset', self._reset_cb, 10)
        self._servo_cli = self.create_client(SetServo, 'knr_hardware/set_servo')
        self.add_on_set_parameters_callback(self._on_param)

        self.create_timer(1.0 / max(0.1, hold_rate), self._hold)

        self._banner()

    # ────────────────────── Start ──────────────────────

    def _banner(self):
        self.get_logger().info(
            "\n"
            "══════════ KALIBRACJA NADIRU GIMBALA ══════════\n"
            f"  serwo {self.servo_id}, PWM startowe {self.pwm} us\n"
            f"  kalibruje kat {self.target_angle:+.0f} st.  "
            f"(skala {self.us_per_deg:.3f} us/st.)\n"
            f"  kamera: focal {self.focal_px:.0f} px, srodek "
            f"({self.cx:.0f}, {self.cy:.0f})\n"
            "───────────────────────────────────────────────\n"
            "  1. zostaw znacznik na ziemi, wznies sie PIONOWO na 80 m\n"
            "  2. zawis, poczekaj az dron sie uspokoi\n"
            "  3. w marker_web (:5000) zamroz klatke i kliknij ten znacznik\n"
            "  4. czytaj SUGESTIE ponizej; klikaj kilka razy\n"
            "───────────────────────────────────────────────\n"
            "  zmiana PWM w locie:\n"
            "    ros2 param set /suas_gimbal_calib pwm 1075\n"
            "    ros2 topic pub --once /gimbal_calib/nudge "
            "std_msgs/msg/Int32 \"{data: -10}\"\n"
            "  1 us = 0.090 st. = 1.28 px = 0.13 m na 80 m\n"
            "═══════════════════════════════════════════════")
        if self.release_nadir:
            self.get_logger().warn(
                "zwalniam blokade nadiru geolokatora — gimbal jest teraz MOJ. "
                "Geolokator dalej rzutuje zakladajac -90 st. i o to chodzi. "
                "NIE uruchamiaj suas_gimbal_controller.")

    # ────────────────────── Trzymanie PWM ──────────────────────

    def _hold(self):
        """Powtarzamy komende co takt: serwo nie ma sprzezenia, a przy
        restarcie czegokolwiek po drugiej stronie wartosc musi wrocic sama."""
        if self.release_nadir:
            self._lock_pub.publish(Bool(data=False))

        if not self._servo_cli.service_is_ready():
            if not self._warned_fn:
                self._warned_fn = True
                self.get_logger().warn(
                    "brak serwisu knr_hardware/set_servo — drone_handler nie "
                    "chodzi albo nie zlapal FC")
            return
        # Jedno wywolanie naraz: inaczej przy zerwanym laczu futures rosloby
        # w nieskonczonosc.
        if self._pending is not None and not self._pending.done():
            return
        req = SetServo.Request()
        req.servo_id = self.servo_id
        req.pwm = self.pwm
        self._pending = self._servo_cli.call_async(req)

    def _set_pwm(self, value, zrodlo):
        value = int(value)
        if not (self.pwm_min <= value <= self.pwm_max):
            self.get_logger().warn(
                f"PWM {value} poza zakresem wezla {self.pwm_min}..{self.pwm_max} "
                f"— ignoruje ({zrodlo})")
            return False
        old, self.pwm = self.pwm, value
        d_deg = (value - old) / self.us_per_deg
        self.get_logger().info(
            f"PWM {old} -> {value} us  ({value - old:+d} us = {d_deg:+.2f} st.) "
            f"[{zrodlo}]")
        if value < self.fc_servo_min:
            self.get_logger().warn(
                f"{value} < SERVO7_MIN ({self.fc_servo_min}) — ArduPilot UTNIE "
                f"to do {self.fc_servo_min}. Obniz SERVO7_MIN albo nie zejdziesz nizej")
        self._hold()
        return True

    def _on_param(self, params):
        for prm in params:
            if prm.name == 'pwm':
                if not self._set_pwm(prm.value, 'param'):
                    return SetParametersResult(successful=False,
                                               reason='PWM poza zakresem')
        return SetParametersResult(successful=True)

    def _pwm_topic_cb(self, msg: Int32):
        self._set_pwm(msg.data, 'topic')

    def _nudge_cb(self, msg: Int32):
        self._set_pwm(self.pwm + int(msg.data), f'nudge {int(msg.data):+d}')

    def _reset_cb(self, msg: Bool):
        if msg.data:
            n = len(self._meas)
            self._meas.clear()
            self.get_logger().info(f"skasowano {n} pomiarow — licze od nowa")

    # ────────────────────── Telemetria ──────────────────────

    def _telem_cb(self, msg: Telemetry):
        t = time.time()
        self._telem.append((t, msg.alt, msg.roll, msg.pitch, msg.yaw))
        self._telem_t.append(t)

    def _telem_at(self, t):
        """Telemetria w chwili t, interpolowana. Ta sama logika co w
        geolokatorze — klik dotyczy klatki sprzed kilku sekund, wiec
        'teraz' byloby innym miejscem w powietrzu."""
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
        out = [a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f]
        for k in range(2, 5):
            out.append(_lerp_angle(a[k], b[k], f))
        return tuple(out)

    # ────────────────────── Pomiar ──────────────────────

    def _solve_angle(self, u, v, alt, roll, pitch, yaw):
        """Jaki kat montazu sprawia, ze piksel (u,v) rzutuje sie DOKLADNIE
        pod drona.

        Szukamy minimum hypot(d_north, d_east) po kacie. Gimbal ma jedna os,
        wiec zwykle nie da sie zejsc do zera — to, co zostaje w minimum, jest
        skladowa BOCZNA, ktorej pitch nie rusza (krzywo wkrecona kamera albo
        staly offset rolla). Zwracamy ja osobno, bo to inna usterka i inna
        naprawa.

        Skan zamiast wzoru: _project_pixel jest tania, przedzial waski,
        a szukanie po siatce nie ma jak sie rozjechac przy dziwnej telemetrii.
        """
        def dist(m):
            pr = _project_pixel(u, v, alt, roll, pitch, yaw, m,
                                self.cam_yaw_offset, self.focal_px,
                                self.cx, self.cy, self.gimbal_stabilized)
            return None if pr is None else math.hypot(pr[0], pr[1])

        best_d, best_m = None, None
        steps = int((SCAN_HI - SCAN_LO) / SCAN_COARSE) + 1
        for i in range(steps):
            m = SCAN_LO + i * SCAN_COARSE
            d = dist(m)
            if d is not None and (best_d is None or d < best_d):
                best_d, best_m = d, m
        if best_m is None:
            return None

        lo, hi = best_m - SCAN_COARSE, best_m + SCAN_COARSE
        steps = int((hi - lo) / SCAN_FINE) + 1
        for i in range(steps):
            m = lo + i * SCAN_FINE
            d = dist(m)
            if d is not None and d < best_d:
                best_d, best_m = d, m
        return best_m, best_d

    def _mark_cb(self, msg: OperatorMark):
        now = time.time()
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if not (ts > 0.0 and abs(ts - now) < self.stamp_max_skew):
            self.get_logger().warn(
                "znacznik bez wiarygodnego stempla klatki — odrzucam. Bez "
                "stempla nie wiem, w ktorym miejscu dron zrobil to zdjecie, "
                "a caly pomiar jest o polozenie. Zamroz i kliknij jeszcze raz")
            return

        tel = self._telem_at(ts)
        if tel is None:
            self.get_logger().warn(
                f"brak telemetrii dla klatki sprzed {now - ts:.1f} s "
                f"(bufor: {len(self._telem)} probek)")
            return
        _, alt, roll, pitch, yaw = tel

        if alt < self.min_alt:
            self.get_logger().warn(
                f"wysokosc {alt:.1f} m < min_alt {self.min_alt:.0f} m — nisko "
                f"kazdy blad piksela wazy wiecej w stopniach. Wznies sie wyzej")
            return

        sol = self._solve_angle(msg.u, msg.v, alt, roll, pitch, yaw)
        if sol is None:
            self.get_logger().warn("promien nie trafia w ziemie — sprawdz telemetrie")
            return
        m_true, resid = sol

        # Blad, ktory geolokator popelnia TERAZ, wierzac w kat docelowy.
        pr = _project_pixel(msg.u, msg.v, alt, roll, pitch, yaw,
                            self.target_angle, self.cam_yaw_offset,
                            self.focal_px, self.cx, self.cy,
                            self.gimbal_stabilized)
        err_m = math.hypot(pr[0], pr[1]) if pr is not None else float('nan')

        # Poprawka. Skale bierzemy z obecnej kalibracji (pomiar jednopunktowy
        # mierzy offset, nie skale), wiec to jest przesuniecie calej prostej.
        d_deg = self.target_angle - m_true
        pwm_target = self.pwm + d_deg * self.us_per_deg

        self._meas.append({
            'pwm_at': self.pwm,
            'm_true': m_true,
            'pwm_target': pwm_target,
            'resid': resid,
            'alt': alt,
        })
        self._report(msg, alt, roll, pitch, yaw, m_true, resid, err_m,
                     d_deg, pwm_target)

    def _report(self, msg, alt, roll, pitch, yaw, m_true, resid, err_m,
                d_deg, pwm_target):
        strona = "W PRZOD" if m_true > self.target_angle else "W TYL"
        lines = [
            "",
            f"────── POMIAR #{len(self._meas)} ──────────────────────────────",
            f"  piksel ({msg.u:.0f}, {msg.v:.0f})   odchylka od srodka "
            f"({msg.u - self.cx:+.0f}, {msg.v - self.cy:+.0f}) px",
            f"  alt {alt:.1f} m   roll {math.degrees(roll):+.1f}  "
            f"pitch {math.degrees(pitch):+.1f}  yaw {math.degrees(yaw):.0f}",
            f"  przy PWM {self.pwm} us punkt pod dronem rzutuje sie "
            f"{err_m:.1f} m od drona",
            f"     = o tyle myli sie geolokator, wierzac w {self.target_angle:+.0f} st.",
            f"  kamera jest FIZYCZNIE na {m_true:+.2f} st.  "
            f"(o {abs(d_deg):.2f} st. za bardzo {strona})",
            f"  reszta boczna {resid:.2f} m  (jedna os — pitch tego nie poprawi)",
            f"  >>> PWM dla {self.target_angle:+.0f} st. = {pwm_target:.0f} us "
            f"({pwm_target - self.pwm:+.0f} od obecnego)",
        ]

        if len(self._meas) >= 2:
            vals = [m['pwm_target'] for m in self._meas]
            mean = sum(vals) / len(vals)
            spread = max(vals) - min(vals)
            lines += [
                "",
                f"  srednia z {len(vals)} pomiarow: {mean:.0f} us   "
                f"(rozrzut {spread:.0f} us = {spread / self.us_per_deg:.2f} st.)",
            ]
            if spread > 30:
                lines.append(
                    "  rozrzut duzy — dron sie bujal albo klikales niedokladnie. "
                    "Poczekaj na spokojny zawis i powtorz")

        if len(self._meas) >= 3:
            lines += self._copy_paste()

        self.get_logger().info("\n".join(lines))

    def _copy_paste(self):
        """Gotowy blok do wklejenia. Przesuwamy OBA punkty kalibracyjne o to
        samo: pomiar jednopunktowy mierzy offset montazu, nie skale serwa,
        wiec prosta ma zostac rownolegla."""
        vals = [m['pwm_target'] for m in self._meas]
        mean = sum(vals) / len(vals)
        delta = mean - self._nominal_pwm(self.target_angle)
        p1 = int(round(mean))
        p2 = int(round(self.ref_pwm + delta))

        out = [
            "",
            "──── DO WPISANIA w drone_hardware/drone_handler.py ────",
            "     (funkcja gimbal_pitch_callback)",
            "",
            f"        a1, p1 = {self.target_angle:.1f}, {p1}",
            f"        a2, p2 = {self.ref_angle:.1f}, {p2}",
            "",
            f"  oba punkty przesuniete o {delta:+.0f} us — poprawiam offset "
            f"montazu, skale serwa zostawiam",
        ]
        if p1 < self.fc_servo_min:
            out.append(
                f"  UWAGA: {p1} < SERVO7_MIN ({self.fc_servo_min}) w ArduPilocie. "
                f"Bez obnizenia SERVO7_MIN do <= {p1} FC utnie te wartosc "
                f"i kalibracja nic nie da")
        out.append("───────────────────────────────────────────────────────")
        return out

    def _nominal_pwm(self, angle):
        """PWM, ktore OBECNY kod wyslalby dla tego kata."""
        return self.ref_pwm + (angle - self.ref_angle) * self.us_per_deg

    # ────────────────────── Podsumowanie ──────────────────────

    def summary(self):
        if not self._meas:
            self.get_logger().info("koniec — zero pomiarow, nic nie policzylem")
            return
        vals = [m['pwm_target'] for m in self._meas]
        mean = sum(vals) / len(vals)
        angles = [m['m_true'] for m in self._meas]
        self.get_logger().info("\n".join([
            "",
            "══════════ PODSUMOWANIE ══════════",
            f"  pomiarow: {len(vals)}",
            f"  kat kamery przy PWM z pomiarow: srednio {sum(angles)/len(angles):+.2f} st.",
            f"  PWM dla {self.target_angle:+.0f} st.: {mean:.0f} us "
            f"(rozrzut {max(vals) - min(vals):.0f} us)",
        ] + self._copy_paste()))


def _lerp_angle(a, b, f):
    """Interpolacja katow z owinieciem przez +-pi (yaw skacze na polnocy)."""
    d = (b - a + math.pi) % (2 * math.pi) - math.pi
    return a + d * f


def main(args=None):
    rclpy.init(args=args)
    node = SuasGimbalCalib()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.summary()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
