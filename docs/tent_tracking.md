# Misja namiotowa — co jest czym i jak uruchomić

Wariant ArduPilot + Gazebo (branch `ardupilot_gz`).

## Baza (symulacja)

```bash
# Panel 1 — Gazebo + drone_handler (swiat: aruco_plain, dron na platformie 30 m przed namiotem)
ros2 launch drone_bringup gazeboo_ap_sim.launch.py

# Panel 2 — ArduPilot SITL (z hosta)
cd scripts && ./run_ap_gazeboo_sitl.sh
```

> Po starcie SITL wpisz raz `reboot` w jego konsoli — bez tego gimbal (mount) nie reaguje.

## tent_detect — detekcja namiotu

Most kamery Gazebo→ROS + YOLO (`MODEL4.pt`) + podgląd rqt.
Publikuje wykrycia na `/tent_detections`.

```bash
ros2 launch drone_bringup tent_detect.launch.py
# opcje: conf:=0.25  input_size:=768  model_path:=...
```

W rqt widać obraz z ramką na namiocie; w logu `FPS ... Detekcje namiotu: X/Y`.

## gimbal_tracker — test samego gimbala

Śledzi namiot **tylko pitchem gimbala** (dron się nie rusza). Kąt liczony z błędu
na obrazie i FOV kamery, wysyłany przez MAVLink (`knr_hardware/gimbal_pitch`).
Wymaga działającego `tent_detect`.

```bash
ros2 launch drone_bringup gimbal_tracker.launch.py
# opcje: damping:=0.6  deadzone:=0.1
```

Logi: `[SLEDZE]` — dosuwa gimbal, `[CENTR]` — namiot wycentrowany, `[SZUKAM]` — brak namiotu.

## tent_tracker — pełna misja (lot + gimbal)

Maszyna stanów: **SEARCH** (czeka na detekcję, nie steruje) → **APPROACH**
(leci w stronę namiotu, gimbal domyka się w dół) → **HOVER** (zawis nad namiotem).
W tle trzyma wysokość `target_alt` (liczoną od miejsca uzbrojenia!).

Kolejność: `tent_detect` działa → uzbrój drona w `gui_panel` → **zamknij gui_panel** →

```bash
ros2 launch drone_bringup tent_tracker.launch.py
# opcje: target_alt:=8.0  kp_vx:=4.0  max_vel:=4.0 ...
```

Log CSV: `~/tent_tracker_log.csv` (w kontenerze).

