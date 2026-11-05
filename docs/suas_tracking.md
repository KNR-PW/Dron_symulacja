# Misja namiotowa — co jest czym i jak uruchomić

Wariant ArduPilot + Gazebo (branch `ardupilot_gz`).

wazne : zaloguj si ena ubuntu on Xorg na panelu logowania i przed wpisaniem hhasla ikona zebatki 

## Baza (symulacja)

```bash
docker exec -it knr_drone bash
source ~/ros_ws/install/setup.bash 
# Panel 1 — Gazebo + drone_handler (swiat: aruco_plain, dron na platformie 30 m przed namiotem)
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

Log CSV: `~/suas_flight_controller_log.csv` (w kontenerze).

## Kopiowanie logów z kontenera

```bash
# Z hosta: CSV misji do biezacego katalogu
docker cp knr_drone:/root/suas_flight_controller_log.csv .

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

## Gimbal na prawdziwym dronie

Ta sama ścieżka ROS→MAVLink→serwo. Podłączenie i parametry ArduPilota (Orange
Cube, MAIN 7): `docs/gimbal_setup_mamba.md`.
