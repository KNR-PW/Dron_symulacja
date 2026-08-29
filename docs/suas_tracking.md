# Misja namiotowa — co jest czym i jak uruchomić

Wariant ArduPilot + Gazebo (branch `ardupilot_gz`).

## Baza (symulacja)

```bash
docker exec -it knr_drone bash
source ~/ros_ws/install/setup.bash 
# Panel 1 — Gazebo + drone_handler (swiat: aruco_plain, dron NA ZIEMI 20 m przed namiotem)
ros2 launch drone_bringup gazeboo_ap_sim.launch.py

# Panel 2 — ArduPilot SITL (z hosta)
cd scripts && ./run_ap_gazeboo_sitl.sh
```



## suas_detect_gazebo — detekcja namiotu

Most kamery Gazebo→ROS + YOLO (`MODEL4.pt`) + podgląd rqt.
Publikuje wykrycia na `/tent_detections`.

```bash
source ~/ros_ws/install/setup.bash 
ros2 launch drone_bringup suas_detect_gazebo.launch.py
# opcje: conf:=0.25  input_size:=768  model_path:=...
```

W rqt widać obraz z ramką na namiocie; w logu `FPS ... Detekcje namiotu: X/Y`.

## suas_detect_jetson — detekcja namiotu (Hardware / Jetson)

Pobiera obraz z prawdziwej kamery (np. OAK-D na `/oak/rgb/image_raw`), wykorzystuje akcelerację TensorRT (model `.engine`) na GPU Jetsona i dodatkowo kompresuje obraz wynikowy (do `/tent_detections/image_compressed`), aby nie obciążać łącza Wi-Fi ze stacją naziemną. **Używaj tego zamiast wersji z Gazebo podczas lotów fizycznym dronem.**

```bash
source ~/ros_ws/install/setup.bash 
ros2 launch drone_bringup suas_detect_jetson.launch.py
# opcje: conf:=0.35  camera_topic:=/oak/rgb/image_raw  model_path:=...
```

Wymaga działającego węzła obsługującego kamerę fizyczną.

## suas_gimbal_controller — test samego gimbala

Śledzi namiot **tylko pitchem gimbala** (dron się nie rusza). Kąt liczony z błędu
na obrazie i FOV kamery, wysyłany przez MAVLink (`knr_hardware/gimbal_pitch`).
Wymaga działającego `suas_detect_gazebo` lub `suas_detect_jetson`.

```bash
ros2 launch drone_bringup suas_gimbal_controller.launch.py
# opcje: damping:=0.6  deadzone:=0.1
```

Logi: `[SLEDZE]` — dosuwa gimbal, `[CENTR]` — namiot wycentrowany, `[SZUKAM]` — brak namiotu.

## gui_panel — panel sterowania dronem

Okno PyQt z telemetrią (bateria, tryb, wysokość, prędkość), przyciskami
arm/takeoff/land, ręcznym sterowaniem (klawiatura/przyciski) i strojeniem
regulatora P (Follow ArUco). Wymaga działającego `drone_handler`
(startuje z `gazeboo_ap_sim.launch.py`) oraz SITL.

```bash
# W kontenerze (okno pojawi sie na X hosta)
ros2 run drone_gui gui_panel
```

W misji namiotowej służy tylko do uzbrojenia i startu drona — potem zamknij
panel, żeby nie publikował prędkości równolegle z `suas_flight_controller`.

## suas_flight_controller — pełna misja (lot + gimbal)

Maszyna stanów: **SEARCH** (czeka na detekcję, nie steruje) → **APPROACH**
(leci w stronę namiotu, gimbal domyka się w dół) → **HOVER** (zawis nad namiotem).
W tle trzyma wysokość `target_alt` (liczoną od miejsca uzbrojenia!).

Kolejność: `suas_detect_gazebo` (lub `_jetson`) działa → uzbrój drona w `gui_panel` → **zamknij gui_panel** →

```bash
ros2 launch drone_bringup suas_flight_controller.launch.py
# opcje: target_alt:=8.0  kp_vx:=4.0  max_vel:=4.0 ...
```

Log CSV: `~/suas_flight_controller_log.csv` (w kontenerze; nazwa pliku bierze
się z nazwy węzła).

## suas_simple_mission — cała misja jednym poleceniem

To samo sterowanie co `suas_flight_controller`, ale z automatycznym startem
i powrotem — bez `gui_panel`:

1. **ARM** (GUIDED) → 2. **TAKEOFF** na `target_alt` → 3. zawis `settle_time` s →
4. start maszyny stanów **SEARCH → APPROACH → HOVER** → 5. czekanie, aż dron
naprawdę zawiśnie nad namiotem (stan HOVER **i** namiot w środku kadru,
`|ex|,|ey| <= center_tol`, przez `hover_hold_time` s) → 6. **RTL**.

Sam stan HOVER to za mało: kontroler przełącza się w niego na progu kąta
gimbala (`pitch_hover_thr`), a dosuwanie się nad cel mikrokorektami trwa
jeszcze kilkanaście sekund. Bez tego warunku dron zaczynał RTL z boku namiotu.

Wymaga tylko działającego `drone_handler` i detektora (`suas_detect_gazebo`
lub `suas_detect_jetson`).

```bash
# Real — domyslne wartosci, bez nadpisywania
ros2 launch drone_bringup suas_simple_mission.launch.py
# Gazebo — te same katy i FOV, tylko luzniejsze okno detekcji (GZ ~2-4 FPS, real 7-14)
ros2 launch drone_bringup suas_simple_mission.launch.py det_confirm_gap:=1.5
# opcje misji: target_alt:=8.0  hover_hold_time:=5.0  search_timeout:=120.0
#              settle_time:=3.0  center_tol:=0.12  finish_action:=rtl|land|none
```

**Uwaga:** w SEARCH dron tylko wisi i czeka na detekcję — misja nie przeszukuje
terenu, więc namiot musi trafić w kadr po starcie. Jeśli w `search_timeout`
sekund nie dojdzie do zawisu nad namiotem, dron i tak wraca (`finish_action`).

Log CSV: `~/suas_simple_mission_log.csv` (ten sam format co niżej).

## Kopiowanie logów z kontenera

```bash
# Z hosta: CSV misji do biezacego katalogu
docker cp knr_drone:/root/suas_flight_controller_log.csv .
docker cp knr_drone:/root/suas_simple_mission_log.csv .

# Z hosta: logi ros2 launch — najpierw znajdz katalog (najnowsze na gorze),
# potem skopiuj caly
docker exec knr_drone ls -t /root/.ros/log | head
docker cp knr_drone:/root/.ros/log/<KATALOG> ./logi_launch
```

Alternatywa bez kopiowania: katalog `src/` repo jest zamontowany w kontenerze
jako `/root/ros_ws/src`, więc plik zapisany tam pojawia się od razu na hoście:

```bash
# W kontenerze — wyjscie launcha na ekran i do pliku src/flight_log.txt
ros2 launch drone_bringup suas_flight_controller.launch.py 2>&1 | tee /root/ros_ws/src/flight_log.txt
```

## Gimbal w symulacji — ta sama ścieżka co na realu

`drone_handler` steruje gimbalem surowym PWM: `DO_SET_SERVO(7)` na **SERVO7**
(kalibracja `1100 µs = -90°` prosto w dół, `1600 µs = -45°`). Żeby Gazebo robiło
dokładnie to samo:

- [`gazebo/models/iris_with_gimbal/model.sdf`](../src/drone_bringup/gazebo/models/iris_with_gimbal/model.sdf) —
  pitch gimbala siedzi na **kanale 6** `ArduPilotPlugin` (= SERVO7), z mapowaniem
  odwzorowującym tę kalibrację (`multiplier -1.2566`, `offset -1.25`).
- [`SITL_param/gazebo_iris.parm`](../src/drone_bringup/SITL_param/gazebo_iris.parm) —
  `SERVO7_FUNCTION 0` (bez tego `DO_SET_SERVO` jest ignorowane) oraz
  `MNT1_TYPE 0` + `SERVO10_FUNCTION 0`, żeby mount nie walczył o ten sam przegub.

Wcześniej pitch szedł przez SERVO10 i mount ArduPilota — kod dawno przeszedł na
surowe PWM, więc kamera w symulacji po prostu stała w miejscu i patrzyła w przód,
a `gimbal=...` w logach było tylko zmienną w Pythonie.

**Dzięki temu w Gazebo i na realu obowiązują te same parametry** — żadnych
`pitch_min:=` / `pitch_max:=` / `vfov_deg:=` do nadpisywania.

Test bez lotu (Gazebo + SITL + `drone_handler` w tle):

```bash
ros2 topic pub --once /knr_hardware/gimbal_pitch std_msgs/msg/Float32 "{data: -90.0}"   # prosto w dół
ros2 topic pub --once /knr_hardware/gimbal_pitch std_msgs/msg/Float32 "{data: -45.0}"   # skos
ros2 topic pub --once /knr_hardware/gimbal_pitch std_msgs/msg/Float32 "{data: -18.0}"   # górny limit
```
W panelu SITL `status SERVO_OUTPUT_RAW` — `servo7_raw` idzie 1100 / 1600 / 1900.

Uwaga przy diagnostyce: ręczne `gz topic -t /gimbal/cmd_pitch -p ...` **nic nie
da przy podpiętym SITL** — `ArduPilotPlugin` republikuje ten topic w każdej
iteracji, więc natychmiast nadpisuje ręczną wartość.

## Gimbal na prawdziwym dronie

Ta sama ścieżka ROS→MAVLink→serwo. Podłączenie i parametry ArduPilota (Orange
Cube, MAIN 7): `docs/gimbal_setup_mamba.md`.
