# YOLO Following - analiza i migracja na lidar_GZ

## 1. Cel dokumentu

Ten dokument opisuje:
- jak dziala pipeline "wykryj obiekt YOLO i podazaj dronem" z brancha `feature-following`,
- co jest wymagane, aby to dzialalo,
- jak przeniesc to na branch oparty o PX4/Gazebo (`lidar_GZ`) albo inny nowy branch.

Dokument jest przygotowany pod praktyczne wdrozenie, bez zmiany aktualnego brancha roboczego.

---

## 2. Co jest na branchu feature-following

Na `feature-following` jest gotowy tor:

1. Kamera (`/gimbal_camera`) -> detekcja (`/detections`)
2. Misja `follow_detections` subskrybuje detekcje i telemetrie
3. Node sterowania wysyla wektory predkosci `knr_hardware/velocity_vectors`
4. Handler lotu wykonuje ruch drona

Glowne pliki:
- `src/drone_autonomy/drone_autonomy/follow_detections.py`
- `src/drone_autonomy/drone_autonomy/kalman_filter.py`
- `src/drone_detector/drone_detector/hybrid_tracker_node.py`
- `src/drone_detector/drone_detector/yolo_detector.py`
- `src/ros2_yolo/ros2_yolo/yolo_node.py`
- `src/drone_bringup/launch/drone_simulation.launch.py`
- `docker/run_yolo_mission.sh`

---

## 3. Jak to dziala krok po kroku

## 3.1 Wejscie obrazu i detekcja

Sa dwa tryby:

- `detector:=yolo`
  - uruchamia `ros2_yolo/yolo_node.py`
  - YOLO publikuje `vision_msgs/Detection2DArray` na `/detections`

- `detector:=hybrid`
  - uruchamia `drone_detector/hybrid_tracker_node.py`
  - laczy YOLO + tracker OpenCV (KCF/CSRT/itp.)
  - tracker jest domyslnie wylaczony i wlaczany serwisem `enable_tracker`
  - publikuje:
    - `/detections`
    - `/detections/annotated`

W `hybrid_tracker_node`:
- co klatke probuje tracker,
- co `detection_interval` klatek robi korekte YOLO,
- przy utracie celu przelacza na "searching/detecting".

## 3.2 Misja follow_detections

Node `follow_detections.py`:
- dziedziczy po `DroneController`,
- subskrybuje `/detections` i telemetrie `knr_hardware/telemetry`,
- z najlepszej detekcji liczy blad obrazu (`ex_px`, `ey_px`),
- estymuje pozycje celu wzgledem drona (z uwzglednieniem:
  - wysokosci,
  - pitch/yaw,
  - kata gimbala),
- aktualizuje filtr Kalmana (`EKF_CTRV`),
- w petli sterowania liczy:
  - `vx` (do przodu/tylu),
  - `yaw_rate` (obrot do celu),
- wysyla komendy:
  - `send_vectors(vx, 0.0, vz, yaw_rate)`.

Zabezpieczenia:
- `lost_timeout`: po utracie detekcji dron sie zatrzymuje,
- clamp predkosci i katow.

## 3.3 Start misji

`docker/run_yolo_mission.sh` uruchamia:
- `ros2 run drone_autonomy follow_detections ...`

W `main()` tej misji:
- arm + takeoff,
- wlaczenie velocity control,
- wlaczenie tracker node,
- start petli podazania.

Uwaga: w pliku sa tez fragmenty testowe zwiazane z ruchem "car" (`/cmd_vel`) do benchmarku w Webots.

---

## 4. Wymagane interfejsy i kontrakty

Aby ten pipeline dzialal, potrzebne sa:

1. Topic detekcji:
- `/detections` (`vision_msgs/Detection2DArray`)

2. Telemetria z orientacja i predkosciami:
- `drone_interfaces/msg/Telemetry.msg` z polami:
  - `roll`, `pitch`, `yaw`
  - `roll_speed`, `pitch_speed`, `yaw_speed`
  - `vx`, `vy`, `vz`

3. Sterowanie predkosciami:
- `knr_hardware/velocity_vectors` (`VelocityVectors`)
- `knr_hardware/toggle_v_control` (`ToggleVelocityControl`)

4. Gimbal:
- serwis `SetGimbalAngle` (`set_gimbal_angle`)
- (opcjonalnie) status kata gimbala `/gimbal/current_angle`

---

## 5. Co obecnie blokuje 1:1 przeniesienie na lidar_GZ

Na `lidar_GZ`:
- nie ma plikow `follow_detections.py`, `hybrid_tracker_node.py`, `yolo_node.py`,
- nie ma pakietu `src/ros2_yolo`,
- `Telemetry.msg` jest krotszy (brakuje roll/pitch/yaw + rates + vx/vy/vz),
- nie ma `SetGimbalAngle.srv`,
- `drone_handler_px4.py` publikuje ograniczony zestaw telemetrii,
- kamera w sciezce PX4/GZ jest zwykle `/rgb_camera/image` (a nie `/gimbal_camera`).

Wniosek: potrzebna jest kontrolowana migracja warstwami.

---

## 6. Plan migracji na nowy branch (zalecany)

Zakladamy nowy branch roboczy, np. `port/yolo-following-px4`.

## 6.1 Etap A - przeniesienie kodu detekcji i misji

Przeniesc z `feature-following`:
- `src/drone_autonomy/drone_autonomy/follow_detections.py`
- `src/drone_autonomy/drone_autonomy/kalman_filter.py`
- `src/drone_autonomy/drone_autonomy/benchmark_tracker.py`
- aktualizacje `src/drone_autonomy/setup.py` (console script `follow_detections`)
- `src/drone_detector/drone_detector/hybrid_tracker_node.py`
- `src/drone_detector/drone_detector/opencv_tracker_wrapper.py`
- `src/drone_detector/drone_detector/yolo_detector.py`
- aktualizacje `src/drone_detector/setup.py`
- caly pakiet `src/ros2_yolo`

## 6.2 Etap B - interfejsy

Dodac/rozszerzyc:
- `src/drone_interfaces/srv/SetGimbalAngle.srv`
- `src/drone_interfaces/msg/Telemetry.msg` o pola orientacji i predkosci katowych/liniowych.

Po zmianach interfejsow:
- przebudowac workspace (colcon), bo wygenerowane typy msg/srv musza byc zgodne.

## 6.3 Etap C - adapter PX4 (kluczowe)

W `src/drone_hardware/drone_hardware/drone_handler_px4.py`:
- uzupelnic publikacje telemetrii o nowe pola:
  - roll/pitch/yaw,
  - roll_speed/pitch_speed/yaw_speed,
  - vx/vy/vz.
- dodac serwis `set_gimbal_angle`:
  - wariant minimum: stub zwracajacy `success=True` i zapamietujacy setpoint,
  - wariant docelowy: mapowanie na komende PX4 gimbala (jezeli konfiguracja to wspiera).

Bez Etapu C `follow_detections` nie bedzie mialo poprawnych danych do stabilnego sterowania.

## 6.4 Etap D - launch i topici

W launchu PX4 (`sim_px4.launch.py` lub osobnym launchu):
- podniesc node detektora (`hybrid` lub `yolo`),
- zapewnic zgodnosc topicu kamery:
  - albo remap `/rgb_camera/image` -> `/gimbal_camera`,
  - albo parametr/zmiana subskrypcji w detektorze na `/rgb_camera/image`.

## 6.5 Etap E - testy integracyjne

Kolejnosc testow:
1. Czy publikuje sie `/detections`?
2. Czy `follow_detections` dostaje telemetrie z nowymi polami?
3. Czy `toggle_v_control` dziala?
4. Czy `send_vectors` rusza dronem zgodnie z celem?
5. Czy po utracie detekcji dron sie zatrzymuje (`lost_timeout`)?

---

## 7. Szybka sciezka MVP (najmniejsze ryzyko)

Jezeli celem jest szybkie "dziala podazanie":

1. Uruchomic najpierw `detector:=yolo` (bez hybrydowego trackera).
2. W `follow_detections` tymczasowo:
- pominac twarda zaleznosc od serwisu gimbala,
- przyjac staly kat kamery.
3. Dopiero potem dolaczyc hybryde i pelna obsluge gimbala.

To minimalizuje liczbe ruchomych elementow i przyspiesza pierwsze uruchomienie.

---

## 8. Checklista "gotowe do PR"

Migracje mozna uznac za gotowa, gdy:
- [ ] node detekcji publikuje `/detections` stabilnie,
- [ ] `follow_detections` startuje bez brakujacych interfejsow,
- [ ] `Telemetry.msg` ma wszystkie pola uzywane przez logike sterowania,
- [ ] `drone_handler_px4` publikuje poprawne dane orientacji i predkosci,
- [ ] dron podaza za celem i bezpiecznie zatrzymuje sie po utracie detekcji,
- [ ] launch uruchamia komplet procesu jednym poleceniem.

---

## 9. Najwazniejsze ryzyka

1. Niezgodnosc klas YOLO:
- w jednym miejscu target class id to `2`, w innym `0`.

2. Roznica backendu lotu:
- `feature-following` opiera sie o `drone_handler` (Webots/ArduPilot path),
- `lidar_GZ` opiera sie o `drone_handler_px4`.

3. Roznica topicow kamery:
- `/gimbal_camera` vs `/rgb_camera/image`.

4. Brakujace pola telemetrii:
- bez nich regulator yaw/pozycji bedzie niestabilny albo slepy.

---

## 10. Podsumowanie

Na `feature-following` istnieje kompletny prototyp "YOLO + tracking + follow".
Na `lidar_GZ` da sie to przeniesc, ale wymaga:
- przeniesienia pakietow detekcji/misji,
- rozszerzenia interfejsow,
- adaptera w `drone_handler_px4`,
- spięcia launchy i topicow kamery.

Najbezpieczniej wdrazac etapami: najpierw detekcja + proste follow (MVP), potem pelna integracja gimbala i hybrydowego trackera.
