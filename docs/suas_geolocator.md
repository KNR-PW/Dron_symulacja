# Zapis wspolrzednych namiotu (suas_geolocator)

Node slucha detekcji YOLO i telemetrii, rzutuje kazda detekcje z piksela na punkt
na ziemi (wysokosc + ogniskowa kamery) i grupuje punkty przestrzennie. Prawdziwy
namiot zbiera setki trafien w jednym miejscu, false positive kilka rozrzuconych.
Wynik ladnie w `~/suas_targets/tent_target.json`.

Node **nie steruje dronem** — ustawia tylko gimbal na -90 st. (prosto w dol).

## Uruchomienie

```bash
# terminal 1 — telemetria + gimbal
ros2 run drone_hardware drone_handler --ros-args -p fc_ip:=/dev/ttyACM0

# terminal 2 — kamera + detekcja TYLKO namiotow (classes:=0)
ros2 launch drone_bringup suas_detect_jetson.launch.py \
    model_path:=/home/jetsonknr/Dron_symulacja/src/drone_detector/models/MODEL5.engine \
    classes:=0

# terminal 3 — zapis wspolrzednych
ros2 launch drone_bringup suas_geolocator.launch.py
```

`classes:=0` jest **konieczne**: model jest dwuklasowy (`{0: 'namiot', 1: 'people'}`),
a detektor publikuje jeden najpewniejszy box na klatke.

**Nie uruchamiaj rownolegle `suas_gimbal_controller`** — bilby sie o kat gimbala.

Ctrl+C konczy i domyka pliki.

## Co widac w terminalu

```
NOWY KANDYDAT #2  52.123901 21.124882  conf=0.44
--- KANDYDACI (namiot) -------------------------------
 #1  52.123456  21.123456   obs= 142  conf=0.71  przeloty=2  rozrzut=2.4m   <-- ZAPISANY
 #2  52.123901  21.124882   obs=   9  conf=0.44  przeloty=1  rozrzut=6.1m
```

`obs` = liczba trafien, `przeloty` = na ilu osobnych przelotach cel byl widziany,
`rozrzut` = jak blisko siebie padly rzutowane punkty (realna dokladnosc).
Do `best` trafia klaster z najwyzszym `obs * conf`, ktory ma co najmniej
`min_obs` (10) trafien.

## Wyniki

```
~/suas_targets/
  tent_target.json          <- najlepszy punkt + ranking; to czyta misja zrzutu
  2026-08-29_14-03/
    target.json             <- kopia z tego lotu
    observations.csv        <- kazda przyjeta detekcja, dane surowe
    kandydat_01.jpg ...     <- klatka z ramka, 1 na kandydata
```

Po locie obejrzyj `kandydat_NN.jpg` — od razu widac, czy kandydat #2 byl
prawdziwym namiotem, bez przewijania wideo.

## Parametry

```bash
ros2 launch drone_bringup suas_geolocator.launch.py cluster_radius:=6.0
```

| parametr | domyslnie | co robi |
|---|---|---|
| `cluster_radius` | 10.0 | promien laczenia detekcji w jeden cel [m]; ~0,12 * wysokosc |
| `min_obs` | 10 | ile trafien musi zebrac klaster, zeby trafic do `best` |
| `det_latency` | 0.20 | szacowane opoznienie detekcji [s] — o tyle cofamy telemetrie |
| `mount_pitch_deg` | -90.0 | kat gimbala; rzutowanie zaklada pion |
| `lock_nadir` | true | node sam trzyma gimbal w nadirze (wysyla co 2 s) |
| `snapshots` | true | zapis klatki przy nowym kandydacie |
| `save_dir` | `~/suas_targets` | katalog wynikow |

Reszta (`focal_px`, `max_tilt`, `center_frac`, `tent_size_m`, `min_alt`...) przez
`--ros-args -p`, patrz sekcja PARAMETRY w `suas_geolocator.py`.

## Test bez latania

Kamera + detektor + namiot podstawiony pod obiektyw, a telemetria udawana:

```bash
ros2 topic pub -r 10 /knr_hardware/telemetry drone_interfaces/msg/Telemetry \
  "{global_lat: 52.2000000, global_lon: 21.0000000, alt: 80.0, roll: 0.0, pitch: 0.0, yaw: 0.0}"
```

- namiot w **srodku kadru** -> zapisany punkt = 52.200000 / 21.000000
- namiot przy **gornej krawedzi** -> punkt ok. 50 m na polnoc (`80*tan(32.2)`)
- `yaw: 1.5708` (90 st.) i namiot u gory -> punkt przesuniety na **wschod**

Uwaga: `tent_size_m` (3 m) i `alt` (80 m) daja oczekiwany box 30 px, a namiot pod
obiektywem wypelnia pol kadru — do testu na biurku podaj mala `alt` albo
`-p size_tol_hi:=100.0`, inaczej bramka rozmiaru odrzuci wszystko (widac
w logu jako `odrzucone: rozmiar=...`).

## Sciagawka do CSV

`observations.csv` trzyma dane **surowe** (`u, v, alt, roll, pitch, yaw, lat, lon`),
wiec cale rzutowanie da sie przeliczyc offline — z innym `det_latency` albo
z dolozona korekta przechylow — bez powtarzania lotu.
