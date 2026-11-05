#!/usr/bin/env python3
"""Porownuje targets.json geolokatora z prawda odniesienia ze swiata Gazebo.

WAZNE - skad brac punkt odniesienia:
SITL NIE uzywa <spherical_coordinates> ze swiata Gazebo. Ma wlasny home
(domyslnie CMAC pod Canberra), a wtyczka ArduPilota podaje mu pozycje wzgledem
punktu, w ktorym model sie zrodzil. Dlatego wspolrzedne GPS obiektow trzeba
liczyc od POZYCJI STARTOWEJ DRONA, a nie od srodka swiata.

Pozycje startowa odczytasz PRZED startem:
    ros2 topic echo /knr_hardware/telemetry --once | grep global_

Uzycie (UWAGA: ze znakiem "=", bo ujemna szerokosc zaczyna sie od minusa
i argparse wzialby ja za nazwe opcji):

    # pelne porownanie obu celow wzgledem punktu startu drona
    python3 check_targets.py --home=-35.363261,149.165230

    # szybki test bez znajomosci home: wisisz nad celem, podajesz swoja
    # biezaca pozycje i sprawdzasz, czy kandydat wypada w tym samym miejscu
    python3 check_targets.py --over tent --at=-35.363261,149.165450
"""
import argparse
import json
import math
import os
import sys

# Pozycje obiektow w swiecie suas_field.sdf [m], uklad ENU (X=wschod, Y=polnoc)
WORLD_XY = {
    'tent':   (30.0, 0.0),
    'people': (15.0, 12.0),
}
SPAWN_XY = (10.0, 0.0)          # tam rodzi sie iris_with_gimbal
PROG_M = {'tent': 3.0, 'people': 5.0}


def m_per_deg(lat):
    return 111_320.0, 111_320.0 * math.cos(math.radians(lat))


def offset_gps(lat0, lon0, d_east, d_north):
    mlat, mlon = m_per_deg(lat0)
    return lat0 + d_north / mlat, lon0 + d_east / mlon


def dist_m(lat_a, lon_a, lat_b, lon_b):
    mlat, mlon = m_per_deg(lat_a)
    return math.hypot((lat_a - lat_b) * mlat, (lon_a - lon_b) * mlon)


def parse_ll(text):
    lat, lon = text.replace(' ', '').split(',')
    return float(lat), float(lon)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--home', help='lat,lon punktu STARTU drona (spawn)')
    ap.add_argument('--over', choices=sorted(WORLD_XY),
                    help='nad ktorym celem wisisz (tryb szybki)')
    ap.add_argument('--at', help='lat,lon biezacej pozycji drona (tryb szybki)')
    ap.add_argument('json', nargs='?',
                    default=os.path.expanduser('~/suas_targets/targets.json'))
    a = ap.parse_args()

    if not os.path.exists(a.json):
        print(f"BRAK PLIKU: {a.json}")
        print("Geolokator zapisuje go dopiero, gdy ma choc jednego kandydata.")
        return 1
    with open(a.json) as f:
        data = json.load(f)
    print(f"plik: {a.json}\nzapisany: {data.get('created')}\n")

    # --- prawda odniesienia ---
    if a.over and a.at:
        lat, lon = parse_ll(a.at)
        truth = {a.over: (lat, lon)}
        print(f"tryb szybki: wisisz nad '{a.over}', pozycja drona "
              f"{lat:.7f} {lon:.7f}")
        print("(kamera w pionie, wiec cel powinien wypasc tam, gdzie dron)\n")
    elif a.home:
        hlat, hlon = parse_ll(a.home)
        truth = {}
        for k, (x, y) in WORLD_XY.items():
            truth[k] = offset_gps(hlat, hlon,
                                  x - SPAWN_XY[0], y - SPAWN_XY[1])
        print(f"home (spawn drona): {hlat:.7f} {hlon:.7f}")
        for k, (tl, tn) in truth.items():
            dx = WORLD_XY[k][0] - SPAWN_XY[0]
            dy = WORLD_XY[k][1] - SPAWN_XY[1]
            print(f"  {k:7s} = home + ({dx:+.0f} m E, {dy:+.0f} m N) "
                  f"= {tl:.7f} {tn:.7f}")
        print()
    else:
        ap.error("podaj --home lat,lon ALBO --over <cel> --at lat,lon")

    ok_all = True
    for key, (t_lat, t_lon) in truth.items():
        sec = data.get(key) or {}
        best = sec.get('best')
        cands = sec.get('candidates') or []
        print(f"=== {key.upper()} (prawda: {t_lat:.7f} {t_lon:.7f}) ===")
        if not cands:
            print("  brak kandydatow\n")
            ok_all = False
            continue
        for c in cands[:5]:
            d = dist_m(c['lat'], c['lon'], t_lat, t_lon)
            star = " <-- BEST" if best and c['id'] == best['id'] else ""
            print(f"  #{c['id']:<3d} {c['lat']:.7f} {c['lon']:.7f}  "
                  f"blad={d:6.2f} m  obs={c['n_obs']:4d} "
                  f"conf={c['mean_conf']:.2f} "
                  f"rozrzut={c['point_spread_m']:.1f} m "
                  f"zrodlo={c['source']}{star}")
        if best is None:
            print("  BEST: brak (zaden klaster nie zebral min_obs)\n")
            ok_all = False
            continue
        d = dist_m(best['lat'], best['lon'], t_lat, t_lon)
        prog = PROG_M[key]
        print(f"  BEST blad = {d:.2f} m  (prog {prog:.0f} m)  -> "
              f"{'OK' if d <= prog else 'PONIZEJ KRYTERIUM'}\n")
        ok_all = ok_all and d <= prog

    print("WYNIK:", "zaliczone" if ok_all else "cos nie gra - patrz wyzej")
    return 0 if ok_all else 2


if __name__ == '__main__':
    sys.exit(main())
