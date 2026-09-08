#!/usr/bin/env python3
"""Generator testowego targets.json — CELOWO BLEDNY adres z geolokatora.

Po co: sprawdzic sciezke wp_scan_on_miss w suas_mission. Dron leci pod adres,
patrzy w dol, nic tam nie ma — i zamiast zrzucic w ciemno robi skan 360 st.
z tego miejsca. Zeby to przetestowac, adres musi byc bledny o tyle, zeby cel
NIE byl w nadirze, ale DALEJ w zasiegu skanu.

Uruchamiaj NA HOSCIE — wynik laduje w src/, ktory jest zamontowany
w kontenerze, wiec plik jest ten sam po obu stronach.

    python3 scripts/make_targets_test.py            # domyslnie 45 m bledu
    python3 scripts/make_targets_test.py 100        # blad 100 m
    python3 scripts/make_targets_test.py 45 --person  # dodaj tez adres czlowieka

ILE BLEDU MA SENS (namiot 3 m, pulap 50 m, focal 813 px):
    blad   slant   piksele   werdykt
     30 m   58 m    42 px    latwy
     45 m   67 m    36 px    pewny  <- domyslny
     60 m   78 m    31 px    prog pewnego wykrycia
     80 m   94 m    26 px    marginalny
    100 m  112 m    22 px    ponizej progu, skan raczej NIE znajdzie
Skan siega kadrem do ~78 m, wiec przy bledzie 100 m test pokaze sciezke
"skan tez nic nie dal -> zrzut na wspolrzedne", a nie znalezienie celu.
"""
import json
import math
import os
import sys

# Pozycja domowa = spawn drona w swiecie suas_field (world 10, 0).
# Odczytana z telemetrii SITL: pierwszy "Moving global" po starcie.
HOME_LAT, HOME_LON = -35.3632623, 149.1653479
# Namiot w swiecie: world (-40, 220), spawn drona world (10, 0)
# -> wzgledem HOME: 50 m na ZACHOD, 220 m na POLNOC.
TENT_E, TENT_N = -50.0, 220.0
M_LAT = 111_320.0


def gps(de, dn):
    lat = HOME_LAT + dn / M_LAT
    lon = HOME_LON + de / (M_LAT * math.cos(math.radians(HOME_LAT)))
    return round(lat, 7), round(lon, 7)


def kandydat(idx, class_id, lat, lon, source, n_obs, score):
    return {
        "id": idx, "class_id": class_id, "source": source,
        "lat": lat, "lon": lon, "n_obs": n_obs,
        "mean_conf": 0.95, "n_passes": 2, "score": score,
        "point_spread_m": 1.5, "drone_spread_m": 30.0,
        "first_seen": 0.0, "last_seen": 1.0,
    }


def main():
    blad = float(sys.argv[1]) if len(sys.argv) > 1 and sys.argv[1][0].isdigit() else 45.0
    z_czlowiekiem = '--person' in sys.argv

    # Blad kierujemy na POLUDNIOWY WSCHOD od namiotu, czyli w glab pola.
    # Na polnoc albo na zachod bledny punkt wyszedlby poza obszar 150x250
    # i dron skanowalby teren, ktorego w zadaniu nie ma.
    k = blad / math.sqrt(2.0)
    zly_e, zly_n = TENT_E + k, TENT_N - k
    zly_lat, zly_lon = gps(zly_e, zly_n)
    prawdziwy_lat, prawdziwy_lon = gps(TENT_E, TENT_N)

    data = {
        "created": f"TEST wp_scan_on_miss: adres bledny o {blad:.0f} m",
        "min_obs": {"tent": 10, "people": 5},
        "tent": {
            "best": kandydat(1, 0, zly_lat, zly_lon, "auto", 300, 285.0),
            "candidates": [kandydat(1, 0, zly_lat, zly_lon, "auto", 300, 285.0)],
        },
        "people": {"best": None, "candidates": []},
    }
    if z_czlowiekiem:
        # Czlowiek lezy w swiecie na (35, 130) -> wzgledem HOME (25, 130).
        cz_lat, cz_lon = gps(25.0, 130.0)
        data["people"] = {
            "best": kandydat(10, 1, cz_lat, cz_lon, "operator", 1, 0.95),
            "candidates": [kandydat(10, 1, cz_lat, cz_lon, "operator", 1, 0.95)],
        }

    sciezka = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           '..', 'src', 'drone_bringup', 'config', 'targets.json')
    sciezka = os.path.normpath(sciezka)
    with open(sciezka, 'w') as fh:
        json.dump(data, fh, indent=2, ensure_ascii=False)

    slant = math.hypot(50.0, blad)
    px = 813.0 * 3.0 / slant
    print(f"zapisano {sciezka}")
    print(f"  NAMIOT prawdziwy : {prawdziwy_lat:.7f} {prawdziwy_lon:.7f}"
          f"   (world -40, 220)")
    print(f"  NAMIOT w pliku   : {zly_lat:.7f} {zly_lon:.7f}"
          f"   BLAD {blad:.0f} m na poludniowy wschod")
    print(f"  z blednego punktu namiot ma {px:.0f} px (slant {slant:.0f} m) — "
          + ("skan powinien go znalezc" if px >= 30 else
             "PONIZEJ progu 30 px, skan raczej nie znajdzie"))
    print(f"  CZLOWIEK: {'adres podany' if z_czlowiekiem else 'BRAK adresu'}")


if __name__ == '__main__':
    main()
