#!/usr/bin/env python3
"""Generuje targets.json pod konkretny scenariusz testowy misji.

Wspolrzedne liczymy od POZYCJI STARTOWEJ DRONA, bo SITL ma wlasny home i nie
uzywa <spherical_coordinates> ze swiata Gazebo. Home odczytaj PRZED startem:

    ros2 topic echo --once /knr_hardware/telemetry | grep global_

Uzycie (znak "=" jest konieczny, bo ujemna szerokosc zaczyna sie od minusa
i argparse wzialby ja za nazwe opcji):

    python3 make_test_targets.py --home=-35.363261,149.165348 --scenariusz pelny
    python3 make_test_targets.py --home=-35.363261,149.165348 --scenariusz atrapa

Scenariusze:
  pelny     namiot (auto) + lezacy czlowiek (operator). Oba istnieja w swiecie
            suas_field.sdf, wiec detektor je potwierdzi.
  atrapa    dwa kandydaty namiotu: #1 to prawdziwy namiot, #2 punkt 40 m dalej,
            gdzie nic nie ma. Odrzuc #1 spacja i sprawdz, czy dron leci do #2,
            a nie od razu w grid.
  smiec     kandydaci ponizej progu min_obs. Maja zostac ODFILTROWANI - misja
            ma zglosic brak kandydatow i przejsc do gridu.
  bez_czlowieka   tylko namiot; czlowiek ma pojsc w grid.
"""
import argparse
import json
import math
import os

M_LAT = 111_320.0

# Offsety obiektow wzgledem SPAWNU drona w swiecie suas_field.sdf.
# Dron rodzi sie na (10, 0); world X = wschod, Y = polnoc.
#   namiot          (30, 0)   -> +20 E,   0 N
#   czlowiek lezacy (45, 0)   -> +35 E,   0 N
#   czlowiek stojacy(15, 12)  ->  +5 E, +12 N
TENT_E, TENT_N = 20.0, 0.0
LYING_E, LYING_N = 35.0, 0.0


def gps(lat0, lon0, d_east, d_north):
    return (lat0 + d_north / M_LAT,
            lon0 + d_east / (M_LAT * math.cos(math.radians(lat0))))


def cand(cid, lat, lon, source, n_obs, conf=0.95):
    return {
        'id': cid, 'class_id': 0, 'source': source,
        'lat': round(lat, 7), 'lon': round(lon, 7),
        'n_obs': n_obs, 'mean_conf': conf, 'n_passes': 2,
        'score': round(n_obs * conf, 2),
        'point_spread_m': 1.5, 'drone_spread_m': 30.0,
        'first_seen': 0.0, 'last_seen': 1.0,
    }


def sekcja(lista):
    return {'best': lista[0] if lista else None, 'candidates': lista}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--home', required=True, help='lat,lon spawnu drona')
    ap.add_argument('--scenariusz', required=True,
                    choices=['pelny', 'atrapa', 'smiec', 'bez_czlowieka'])
    ap.add_argument('--out', default=os.path.expanduser('~/suas_targets/targets.json'))
    a = ap.parse_args()

    lat0, lon0 = (float(x) for x in a.home.replace(' ', '').split(','))
    tent = gps(lat0, lon0, TENT_E, TENT_N)
    lying = gps(lat0, lon0, LYING_E, LYING_N)
    # Punkt 40 m na polnoc od namiotu - w swiecie NIC tam nie stoi.
    pusty = gps(lat0, lon0, TENT_E, TENT_N + 40.0)

    if a.scenariusz == 'pelny':
        tents = [cand(1, *tent, 'auto', 300)]
        people = [cand(10, *lying, 'operator', 1)]
    elif a.scenariusz == 'atrapa':
        # #1 prawdziwy namiot (odrzucasz go spacja), #2 pusty punkt.
        tents = [cand(1, *tent, 'auto', 300),
                 cand(2, *pusty, 'auto', 120)]
        people = [cand(10, *lying, 'operator', 1)]
    elif a.scenariusz == 'smiec':
        # ponizej min_obs (10 namiot / 5 czlowiek) -> filtr ma je wyciac
        tents = [cand(1, *tent, 'auto', 4)]
        people = [cand(10, *lying, 'auto', 2)]
    else:                                    # bez_czlowieka
        tents = [cand(1, *tent, 'auto', 300)]
        people = []

    for c in people:
        c['class_id'] = 1

    data = {
        'created': 'TEST',
        'min_obs': {'tent': 10, 'people': 5},
        'tent': sekcja(tents),
        'people': sekcja(people),
    }
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    with open(a.out, 'w') as fh:
        json.dump(data, fh, indent=2)

    print(f"zapisano {a.out}  (scenariusz: {a.scenariusz})")
    print(f"home {lat0:.7f} {lon0:.7f}")
    for nazwa, lst in (('NAMIOT', tents), ('CZLOWIEK', people)):
        if not lst:
            print(f"  {nazwa:9s} brak kandydatow -> spodziewaj sie GRIDU")
        for i, c in enumerate(lst, 1):
            de = (c['lon'] - lon0) * M_LAT * math.cos(math.radians(lat0))
            dn = (c['lat'] - lat0) * M_LAT
            prog = data['min_obs']['tent' if c['class_id'] == 0 else 'people']
            ok = c['source'] == 'operator' or c['n_obs'] >= prog
            print(f"  {nazwa:9s} #{i} {c['source']:8s} obs={c['n_obs']:3d} "
                  f"({de:+5.0f} E, {dn:+5.0f} N)  "
                  f"{'przejdzie filtr' if ok else 'ODFILTROWANY (obs < %d)' % prog}")


if __name__ == '__main__':
    main()
