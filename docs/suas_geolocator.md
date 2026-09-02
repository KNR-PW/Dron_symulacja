# Zapis wspolrzednych celow (suas_geolocator)

Node slucha detekcji YOLO i telemetrii, rzutuje kazda detekcje z piksela na punkt
na ziemi i grupuje punkty przestrzennie. Prawdziwy cel zbiera setki trafien
w jednym miejscu, false positive kilka rozrzuconych. Obsluguje **obie klasy
naraz** — namiot i czlowieka — trzymajac je w osobnych klastrach.
Wynik w `~/suas_targets/targets.json`.

Node **nie steruje dronem** — ustawia tylko gimbal na -90 st. (prosto w dol).

## Rzutowanie uwzglednia przechyl ramy

Gimbal ma jedna os i **nie jest stabilizowany** (`MNT1_TYPE=0`,
`SERVO7_FUNCTION=0`), wiec kamera pochyla sie razem z dronem. Rzutowanie buduje
kierunek promienia z kata montazu ORAZ roll/pitch/yaw z telemetrii i przecina go
z ziemia. Bez tego lot 8 m/s na 80 m dawalby 14 m systematycznego bledu, bo
kopter trzyma wtedy staly pitch ok. 10 st.

Parametr `gimbal_stabilized` (domyslnie `false`) wylacza te kompensacje — ustaw
`true` TYLKO jesli kiedys wejdzie stabilizowany mount, inaczej korekta policzy
sie podwojnie.

## Uruchomienie

```bash
# terminal 1 — telemetria + gimbal
ros2 run drone_hardware drone_handler --ros-args -p fc_ip:=/dev/ttyACM0

# terminal 2 — kamera + detekcja OBU klas
ros2 launch drone_bringup suas_detect_jetson.launch.py \
    model_path:=/home/jetsonknr/Dron_symulacja/src/drone_detector/models/MODEL5.engine

# terminal 3 — zapis wspolrzednych
ros2 launch drone_bringup suas_geolocator.launch.py
```

`classes:=0` **nie jest juz potrzebne**. Detektor publikuje `/detections`
z kompletem boxow z klatki, a geolokator sam rozdziela je na klasy i klastruje
kazda osobno. Filtr `classes` zostaje wylacznie jako awaryjny.

**Nie uruchamiaj rownolegle `suas_gimbal_controller`** — bilby sie o kat gimbala.

Ctrl+C konczy i domyka pliki.

## Co widac w terminalu

```
NOWY KANDYDAT #1 [namiot]  -35.363258 149.165570  conf=0.94  zrodlo=auto
--- KANDYDACI (NAMIOT) ---------------------------
 #1  -35.363258  149.165570   obs= 171  conf=0.94  przeloty=2  rozrzut=1.9m   <-- ZAPISANY
--- KANDYDACI (CZLOWIEK) ---------------------------
 #3  -35.363301  149.165612   obs=   7  conf=0.51  przeloty=1  rozrzut=3.2m  [OPERATOR]  <-- ZAPISANY
```

`obs` = liczba trafien, `przeloty` = na ilu osobnych przelotach cel byl widziany,
`rozrzut` = jak blisko siebie padly rzutowane punkty (realna dokladnosc).
Do `best` trafia klaster z najwyzszym `obs * conf`, ktory ma co najmniej
`min_obs` trafien (10 dla namiotu, 5 dla czlowieka — jest mniejszy i gorzej
wykrywany).

## Automat nie zapisuje czlowieka z wysoka

Powyzej `person_max_alt` (domyslnie **50 m**) detekcje klasy `czlowiek` sa
odrzucane — w logu jako `czlowiek_wysoko`. 50 m to wysokosc zrzutu, czyli
najnizszy pulap misji: automat dostaje szanse dokladnie tam, gdzie ma jej
najwiecej, a caly przelot ortofoto jest odciety.

Powod jest fizyczny: czlowiek ma z nadiru ok. 0,5 m, czyli 8 px z 50 m i 5 px
z 80 m, a YOLO ma stride 8. Ponizej ~10 px nie ma czego wykrywac, wiec kazda
automatyczna detekcja czlowieka z pulapu ortofoto jest z definicji podejrzana.

To nie jest ostroznosc na wyrost. W symulacji model bral znacznik ArUco za
czlowieka **przez 166 klatek przy conf 0,53** — taki klaster spokojnie
przekraczal `min_obs_person` i awansowal na `best`, czyli misja poleciałaby
zrzucic ladunek na znacznik.

**Bramka dotyczy WYLACZNIE automatu.** Klikniecie operatora dziala na kazdej
wysokosci i to jest caly sens recznego oznaczania: tam, gdzie model jest slepy,
czlowiek przy ekranie widzi wiecej.

**Znacznik operatora ma pierwszenstwo.** Klaster oznaczony recznie (`[OPERATOR]`)
zostaje `best` niezaleznie od liczby obserwacji. Na 80 m czlowiek ma w kadrze
ok. 11 px, czyli ponizej progu YOLO — tam czlowiek przy ekranie widzi wiecej niz
model, wiec to jego wskazanie ma byc adresem zrzutu.

Reczny znacznik da sie wyslac takze z konsoli, bez GUI (piksel w ORYGINALNEJ
klatce 1024x1024; srodek kadru = punkt dokladnie pod dronem):

```bash
ros2 topic pub --once /operator_mark drone_interfaces/msg/OperatorMark \
  "{header: {stamp: {sec: 0, nanosec: 0}}, class_id: 1, u: 512.0, v: 512.0}"
```

## Jak to testowac

**Zawis NIE wystarczy.** Cel jest wtedy pod dronem, przesuniecie bliskie zeru
i ewentualny blad skali rzutowania nie ma na czym zadzialac — wynik wyglada
idealnie, nawet gdy geometria jest zla. Testem jest **przelot obok celu**:

- jeden obiekt ma dac **jeden** klaster, nie dwa,
- `rozrzut` w okolicach metra,
- kandydat nie przesuwa sie razem z dronem.

Zmierzone w symulacji 2026-09-01: 171 obserwacji, rozrzut 1,9 m, blad wzgledem
prawdziwej pozycji modelu w Gazebo **0,45 m**.

Prawde odniesienia w symulacji dostaniesz tak:

```bash
gz model -m iris_with_gimbal -p                 # pozycja XYZ drona w Gazebo
ros2 service call /knr_hardware/get_location_relative \
    drone_interfaces/srv/GetLocationRelative     # ta sama pozycja w metrach od home
```

## Wyniki

```
~/suas_targets/
  targets.json              <- sekcje 'tent' i 'people', kazda z 'best' i rankingiem
  tent_target.json          <- stary format, tylko namiot; zostaje dla zgodnosci
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
