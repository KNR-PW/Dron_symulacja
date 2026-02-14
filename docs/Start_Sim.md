# Skrypt startowy symulacji PX4 (start_px4_sim.sh)

Skrypt ten automatyzuje uruchamianie pełnego środowiska symulacyjnego w jednym oknie terminala (Terminator), podzielonym na panele.
## Spis treści

- [I. Wymagania](#i-wymagania)
- [II. Jak uruchomić symulację](#ii-jak-uruchomić-symulację)
- [III. Opis okna konsoli](#iii-opis-okna-konsoli)
- [IV. Opis procesu](#iv-opis-procesu)
## I. Wymagania

### Terminator
```bash
sudo apt install terminator
```

### QGroundControl 
musi być w tym folderze start_sim pod nazwą `QGroundControl-x86_64.AppImage`.

## II. Uruchomienie
wejdź do folderu i uruchom skrypt:
```bash
cd ~/Dron_symulacja/docker/start_sim
./start_px4_sim.sh
```

## III. Opis okna konsoli
### Panel 1: PX4 SITL (na hoście)
- Czyści stare procesy.
- Uruchamia właściwą symulację (fizyka, Gazebo).
```bash
cd scripts

pkill -9 px4; pkill -9 gz; pkill -9 ruby

./run_px4_vtol_sitl.sh
```


### Panel 2: MicroXRCEAgent (Docker)
Mostek tłumaczący MAVLink na ROS2.
1. Wchodzi do kontenera:
```bash
docker exec -it knr_drone_px4 bash
```
2. Wewnątrz kontenera uruchamia:
```bash
source /opt/ros/jazzy/setup.bash && source ~/ros_ws/install/setup.bash

pkill -f MicroXRCEAgent

MicroXRCEAgent udp4 -p 8888
```

### Panel 3: Drone_handler_px4 (Docker)
Główny węzeł sterujący dronem
1. Wchodzi do kontenera:
2. Wewnątrz kontenera uruchamia:
```bash
source /opt/ros/jazzy/setup.bash && source ~/ros_ws/install/setup.bash

pkill -f drone_handler_px4

ros2 run drone_hardware drone_handler_px4
```

### Panel 4: Docker
Otwiera  konsolę  wewnątrz kontenera .





