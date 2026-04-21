# Baza Wiedzy Projektu (PX4 + Gazebo Sim)

## 1. Zakres dokumentu

Ten dokument opisuje **tylko** sciezke projektu oparta o:
- **PX4 Autopilot**
- **Gazebo Sim (gz)**
- **ROS 2**

Pomija celowo workflow i komponenty zwiazane z ArduPilot.

## 2. Srodowisko i wersje (potwierdzone z repo i kontenera)

- **Host OS:** Ubuntu 24.x (uzytkownik deklaruje Ubuntu 24)
- **Docker base image (PX4):** `ubuntu:24.04` (z `docker/docker_px4/Dockerfile`)
- **ROS 2 distro:** `jazzy` (sourcing `/opt/ros/jazzy/setup.bash`)
- **ROS 2 pakiet bazowy:** `ros-jazzy-desktop 0.11.0-1noble.20251108.005426` (w kontenerze)
- **Gazebo:** `Gazebo Sim 8.10.0` (`gz sim --version` w kontenerze `knr_drone_px4`)
- **Typ SITL:** `PX4 SITL + Gazebo Sim` (targety `make px4_sitl gz_*`)
- **PX4 Autopilot fork/submodule (KNR):**
  - repo: `KNR_Drone_PX4_Autopilot`
  - upstream/fork owner: `github.com/KNR-PW/KNR_Drone_PX4_Autopilot`
  - branch: `KNR_stable_v1.16.1`
  - describe: `v1.16.1-knr-0.1-2-g9414ab51fb`
  - commit: `9414ab51fb`
- **MicroXRCE-DDS-Agent submodule:**
  - describe: `v2.4.3`
  - commit: `7362281`
- **px4_msgs submodule commit:** `49a0f6c`

### 2.1 Polityka forkow w tym repo (istotne)

- W praktyce **uzywamy forkow tylko** w:
  - `KNR_Drone_PX4_Autopilot`
  - `KNR_Drone_PX4_Autopilot/Tools/simulation/gz` (wlasne swiaty/konfiguracja symulacji gz)
- `src/Micro-XRCE-DDS-Agent` i `src/px4_msgs` sa traktowane jako standardowe zaleznosci/submoduly, nie jako aktywnie utrzymywane forki KNR.

## 3. Glowny cel projektu (PX4)

Budowa i rozwijanie warstwy autonomii drona VTOL w symulacji, gdzie:
- PX4 odpowiada za low-level sterowanie i model lotu,
- ROS 2 odpowiada za logike misji, integracje sensorow i uslugi wysokiego poziomu,
- Gazebo Sim dostarcza srodowisko symulacyjne,
- QGroundControl sluzy do monitoringu/uzbrajania i wsparcia operatorskiego.

W praktyce "proof of operation" to poprawne przejscie sekwencji:
1) start SITL,
2) bridge DDS,
3) uruchomienie `drone_handler_px4`,
4) wykonanie misji (np. `test_mission.py`) bez utraty kontroli i z poprawnym telemetria.

## 4. Architektura wysokiego poziomu

```
[Gazebo Sim world + model]
          ^
          | (PX4 gz bridge, make px4_sitl gz_*)
          v
[PX4 SITL] <-> [MicroXRCEAgent udp4:8888] <-> [ROS 2 nodes]
                                             |
                                             +-> drone_hardware/drone_handler_px4
                                             +-> drone_autonomy/* (misje)
                                             +-> drone_camera/*, detector/gui/web (opcjonalnie)
```

### Co uruchamia skrypt orkiestrujacy

Skrypt `scripts/start_sim/start_px4_sim.sh` automatyzuje caly start w Terminatorze i tworzy panele:
- **Panel 1:** PX4 SITL (`./run_px4_vtol_sitl.sh <world>`)
- **Panel 2:** `MicroXRCEAgent udp4 -p 8888` (w kontenerze)
- **Panel 3:** `ros2 run drone_hardware drone_handler_px4` (w kontenerze)
- **Panel 4:** shell w kontenerze
- dodatkowo uruchamia **QGroundControl** (jesli nie dziala)

## 5. Glowne pakiety w workspace (skrotowo)

Najwazniejsze dla sciezki PX4:
- `drone_hardware` - most ROS 2 <-> PX4 (`drone_handler_px4`), uslugi i akcje sterowania.
- `drone_autonomy` - misje wysokopoziomowe i klient sterowania (`DroneController`).
- `drone_interfaces` - custom `msg/srv/action` uzywane miedzy node'ami.
- `drone_bringup` - launch files (`sim_px4.launch.py`, `vtol_preflight.launch.py` itd.).
- `px4_msgs` - definicje wiadomosci PX4 dla ROS 2.
- `drone_camera` - rejestracja obrazu i uslugi foto.
- `ros2_aruco` - detekcja markerow ArUco (uzywana np. w preflight).

Pakiety obecne, ale niekluczowe dla bazowego PX4 SITL (w tej sciezce): m.in. `drone_detector`, `drone_gui`, `drone_web`, `mqtt_telemetry`, `droniada_inspekcja`, `webots_simulation`.

## 6. Kluczowe node'y i interfejsy ROS 2

## 6.1 Najwazniejsze node'y

- `drone_handler_px4` (`drone_hardware`) - centralny node sterowania lotem po stronie ROS 2.
- `drone_controller` (`drone_autonomy/.../drone_controller.py`) - klient akcji/uslug uzywany przez misje.
- `gz_bridge` (`ros_gz_bridge/parameter_bridge`) - most obrazu Gazebo -> ROS (`/rgb_camera/image`).
- `images_recorder` (`drone_camera`) - zapisywanie obrazu/misji foto.
- proces `MicroXRCEAgent` - kluczowy middleware miedzy PX4 a ROS 2.

## 6.2 Topici kluczowe (z `drone_handler_px4.py`)

Subskrypcje z PX4 (`px4_msgs`):
- `/fmu/out/vehicle_status_v1` (`VehicleStatus`)
- `/fmu/out/vehicle_global_position` (`VehicleGlobalPosition`)
- `/fmu/out/vehicle_local_position_v1` (`VehicleLocalPosition`)
- `/fmu/out/vehicle_attitude` (`VehicleAttitude`)
- `/fmu/out/battery_status_v1` (`BatteryStatus`)

Publikacje do PX4:
- `/fmu/in/offboard_control_mode` (`OffboardControlMode`)
- `/fmu/in/vehicle_command` (`VehicleCommand`)
- `/fmu/in/trajectory_setpoint` (`TrajectorySetpoint`)
- `/fmu/in/actuator_servos` (`ActuatorServos`)

Topici aplikacyjne ROS:
- `knr_hardware/velocity_vectors` (`drone_interfaces/VelocityVectors`) - wejscie sterowania wektorowego.
- `knr_hardware/telemetry` (`drone_interfaces/Telemetry`) - telemetria publikowana przez `drone_handler_px4`.
- `/rgb_camera/image` (`sensor_msgs/Image`) - z `ros_gz_bridge` w `sim_px4.launch.py`.

## 6.3 Uslugi i akcje (namespace `knr_hardware/`)

Uslugi:
- `set_mode` (`SetMode`)
- `toggle_v_control` (`ToggleVelocityControl`)
- `set_servo` (`SetServo`)
- `calib_servo` (`VtolServoCalib`)

Akcje:
- `Arm`
- `takeoff`
- `goto_global`
- `goto_relative`
- `Set_yaw`

## 7. Jak uruchamiac symulacje (PX4)

## 7.1 Przygotowanie jednorazowe

1. Pobierz obraz:
   - `docker image pull dierust/knr_px4_sim:latest`
2. Zainicjalizuj submoduly:
   - `git submodule update --init --recursive`
   - `cd KNR_Drone_PX4_Autopilot && git submodule update --init --recursive`
3. Utworz kontener:
   - `cd docker/docker_px4`
   - `./setup_container_gpu.sh dierust/knr_px4_sim:latest`
   - (alternatywnie CPU: `./setup_container_cpu.sh ...`)

## 7.2 Standardowy start (zalecany)

Najprostsza sciezka:
- `cd scripts/start_sim`
- `./start_px4_sim.sh` (lub `./start_px4_sim.sh <world>`, domyslnie `aruco`)

Skrypt:
- czyści stare procesy `gz-sim/px4/ruby`,
- startuje kontener `knr_drone_px4`,
- uruchamia SITL + MicroXRCEAgent + `drone_handler_px4`,
- odpala QGroundControl (jesli nie byl uruchomiony).

## 7.3 Start misji testowej

- `scripts/run_test_mission.sh`
- skrypt uruchamia `test_mission.py` w aktywnym kontenerze (`knr_drone_px4` albo `knr_drone`).

`test_mission.py` wykonuje sekwencje:
- arm -> takeoff 20m -> przejscie do `MULTICOPTER`,
- petla waypointow globalnych (kwadrat ok. 25m),
- test zmian wysokosci,
- ladowanie.

## 8. Obecny stan projektu (na podstawie plikow)

- Dzialajacy, automatyczny pipeline startu PX4 SITL jest gotowy (`start_px4_sim.sh` + `run_px4_vtol_sitl.sh`).
- Integracja PX4 <-> ROS 2 jest oparta o `MicroXRCEAgent` (UDP 8888).
- Warstwa sterowania i telemetrii w ROS 2 jest zaimplementowana w `drone_handler_px4`.
- Istnieje gotowa misja testowa do szybkiej walidacji (`test_mission.py`).
- W dokumentacji repo sa nadal pliki/historyczne kroki ArduPilot; dla PX4 nalezy trzymac sie komend z sekcji 7 i plikow PX4.

## 9. Ograniczenia i uwagi operacyjne

- Nazwy kontenerow sa "na sztywno": glownie `knr_drone_px4`.
- `start_px4_sim.sh` wymaga:
  - `terminator`
  - pliku `QGroundControl-x86_64.AppImage` w `scripts/start_sim/`
- Start worlda jest parametryzowany (`aruco` domyslnie), a skrypt SITL dotyka `CMakeLists.txt`, gdy target `gz_*_<world>` nie istnieje.
- W `drone_handler_px4` promien akceptacji `goto_global` zalezy od trybu:
  - multicopter: `2.0 m`
  - fixed-wing: `80.0 m`

## 10. Najwazniejsze pliki do szybkiego przegladu

Pod onboarding nowego czatu AI:
- `KNR_Drone_PX4_Autopilot/Tools/simulation/gz/`
- `KNR_Drone_PX4_Autopilot/Tools/simulation/gz/worlds/terrain.sdf`
- `README.md`
- `docs/Start_Sim.md`
- `docs/px4_sim_open.md`
- `docs/co_do_czego.md`
- `scripts/start_sim/start_px4_sim.sh`
- `scripts/run_px4_vtol_sitl.sh`
- `scripts/run_px4_quad_sitl.sh`
- `scripts/run_microxrce.sh`
- `scripts/run_test_mission.sh`
- `src/drone_hardware/drone_hardware/drone_handler_px4.py`
- `src/drone_autonomy/drone_autonomy/drone_comunication/drone_controller.py`
- `src/drone_autonomy/drone_autonomy/test_mission.py`
- `src/drone_bringup/launch/sim_px4.launch.py`

## 11. Gotowy prompt startowy do nowego czatu AI

Skopiuj do nowego czatu:

> Pracujemy tylko na sciezce PX4 + Gazebo Sim (bez ArduPilot).  
> Bazuj na pliku `projekt_kontekst.md` i traktuj go jako zrodlo prawdy o architekturze, uruchamianiu i interfejsach ROS 2.  
> Priorytet: utrzymanie workflow `scripts/start_sim/start_px4_sim.sh` oraz zgodnosc z `drone_handler_px4.py` i `drone_controller.py`.

