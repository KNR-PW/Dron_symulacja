# KNR Drone Simulation

Symulacja drona KNR: **ROS 2 Humble + ArduPilot SITL  + Gazebo Harmonic**, wszystko w kontenerze Docker.
Na hoście potrzebujesz tylko Dockera i repozytorium. Kod edytujesz na hoście, a kompilujesz i uruchamiasz w kontenerze.

- **Gazebo** – świat i fizyka drona, kamera, gimbal. Z SITL łączy się przez wtyczkę ArduPilota.
- **ArduCopter SITL** – symulowany flight controller, ten sam firmware co na prawdziwym dronie.
- **drone_handler** – most między SITL a ROS 2. Rozmawia z SITL przez MAVLink i udostępnia drona jako akcje, serwisy i topici.
- **Node'y ROS 2** – misje, detekcja, kamera i reszta logiki projektu. Z dronem komunikują się przez `drone_handler`.
- **Stacja naziemna** – QGroundControl albo Mission Planner na hoście, łączy się bezpośrednio z SITL.

---

## 1. Wymagania

- Linux: Ubuntu 22.04 lub 24.04 z sesją **Xorg** (patrz krok 2.4). Windows i macOS nie są wspierane.
- Docker Engine.
- Opcjonalnie GPU: NVIDIA (wymaga NVIDIA Container Toolkit) albo AMD (działa od razu). Bez GPU też działa, tylko wolniej.
- Na słabym laptopie symulacja może się zacinać albo w ogóle nie zadziałać.

---

## 2. Instalacja (jednorazowo)

### 2.1 Instalacja Dockera

Zainstaluj **Docker Engine** . **Nie instaluj Docker Desktop**, 

Komendy z [oficjalnej instrukcji](https://docs.docker.com/engine/install/ubuntu/):

```bash
# [host]
sudo apt update
sudo apt install -y ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt update
sudo apt install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```


### 2.2 Docker bez `sudo`

Dodaj swojego użytkownika do grupy `docker`. Dzięki temu komendy `docker` i skrypty z repo (`docker/docker_ap/*.sh`, `scripts/*.sh`) uruchamiasz bez `sudo`.

```bash
# [host]
sudo usermod -aG docker $USER
# wyloguj się i zaloguj ponownie (albo uruchom: newgrp docker)
docker run --rm hello-world      # test
```

### 2.3 NVIDIA Container Toolkit (tylko z kartą NVIDIA)

Szczegóły są w [oficjalnej instrukcji](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html).

```bash
# [host]
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt update && sudo apt install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

docker run --rm --gpus all ubuntu nvidia-smi    # test: powinna się pokazać tabela z GPU
```

### 2.4 Dostęp kontenera do ekranu (okno Gazebo)

Ubuntu domyślnie loguje do sesji Wayland, na której okno Gazebo z kontenera często się nie otwiera albo jest czarne. Dotyczy to zwłaszcza kart NVIDIA. Na ekranie logowania kliknij swojego użytkownika, potem ikonę zębatki w prawym dolnym rogu, i wybierz **Ubuntu on Xorg**.

```bash
# [host]
echo $XDG_SESSION_TYPE      # musi wypisać: x11

echo "xhost +local:docker > /dev/null" >> ~/.bashrc
source ~/.bashrc
```

### 2.5 Repozytorium i obraz

```bash
# [host]
git clone https://github.com/KNR-PW/Dron_symulacja.git
cd Dron_symulacja
docker pull dierust/knr_ap_sim:latest
```

Submodułów (`git submodule update`) nie pobieraj. Są potrzebne tylko do PX4, a ArduPilot z nich nie korzysta.

### 2.6 Utworzenie kontenera `knr_drone`

Robisz to **raz**. Później kontener tylko uruchamiasz (patrz [rozdział 5](#5-praca-z-kontenerem)). Skrypt uruchom z katalogu `docker/docker_ap` i wybierz wariant pasujący do Twojej karty:

```bash
# [host]
cd docker/docker_ap
./setup_container_gpu.sh dierust/knr_ap_sim:latest   # karta NVIDIA
./setup_container_amd.sh dierust/knr_ap_sim:latest   # karta AMD
./setup_container_cpu.sh dierust/knr_ap_sim:latest   # bez GPU
```

Po wykonaniu skryptu jesteś w powłoce kontenera. Sprawdź, czy katalog `src` się zamontował:

```bash
# [kontener]
ls ~/ros_ws/src      # muszą być widoczne pakiety drone_bringup, drone_hardware, ...
```

### 2.7 Pierwszy build

```bash
# [kontener]
cd ~/ros_ws
colcon build --packages-skip microxrcedds_agent px4_msgs px4_ros_com
source install/setup.bash

# Jednorazowo: każdy nowy terminal w kontenerze będzie miał od razu załadowany workspace
echo "source ~/ros_ws/install/setup.bash" >> ~/.bashrc
```

---

## 3. Uruchomienie symulacji: Gazebo + SITL + drone_handler


### 3.1 Start kontenera

```bash
# [host]
docker start knr_drone
```

### 3.2 Terminal 1: Gazebo + drone_handler

Ze skryptu:

```bash
# [host]
cd scripts
./run_ap_gazeboo_ros.sh                  # domyślny świat: aruco_plain.sdf
./run_ap_gazeboo_ros.sh suas_field.sdf   # wybrany świat
```

Alternatywnie bez skryptu, bezpośrednio w kontenerze:

```bash
# [host] otwórz nowy terminal w kontenerze
docker exec -it knr_drone bash

# [kontener]
cd ~/ros_ws
colcon build --packages-skip microxrcedds_agent px4_msgs px4_ros_com
source install/setup.bash
ros2 launch drone_bringup gazeboo_ap_sim.launch.py world:=aruco_plain.sdf
```

### 3.3 Terminal 2: ArduCopter SITL

```bash
# [host]
cd scripts
./run_ap_gazeboo_sitl.sh
```

> Pierwsze uruchomienie SITL kompiluje ArduCopter, co trwa kilka minut. Kolejne starty są szybkie, dopóki nie usuniesz kontenera.

### 3.4 Jak sprawdzić, że wszystko działa

1. Jest okno Gazebo, a w nim dron na ziemi.
2. W konsoli SITL pojawia się `EKF3 IMU0 is using GPS`.
3. W terminalu 1 widać zielony komunikat `Copter connected, ready to arm`. Może to potrwać do około minuty, bo handler czeka na GPS i EKF.

Szybki test lotu wpisany w konsoli SITL (MAVProxy):

```
mode guided
arm throttle
takeoff 5
```

### 3.5 Światy Gazebo

Światy są w [src/drone_bringup/gazebo/worlds/](src/drone_bringup/gazebo/worlds/):

- `aruco_plain.sdf`
- `suas_field.sdf`
- `terrain_tent_sunset.sdf`

### 3.6 Restart i sprzątanie procesów z symulacji

Zatrzymaj symulację przez `Ctrl+C` w obu terminalach. Jeśli coś zostało w tle, zabij pozostałe procesy:

```bash
# [host]
./scripts/kill_sim.sh
```

---

## 4. Uruchamianie misji 

Gazebo i SITL muszą już działać, a handler musi wypisać `ready to arm`.

### 4.1  misja testowa

```bash
# [kontener]
python3 ~/ros_ws/src/drone_autonomy/drone_autonomy/test_mission.py
```

Lub ze skryptu:

```bash
# [host]
cd scripts
./run_test_mission.sh
```

Jeśli dron się nie uzbraja, wpisz w konsoli SITL `arm throttle` i zobacz, jaki komunikat `PreArm:` się pojawi.

### 4.2 Detekcja YOLO w Gazebo

```bash
# [kontener]
ros2 launch drone_bringup suas_detect_gazebo.launch.py
```

---

## 5. Komendy docker 


```bash
# [host]
docker start knr_drone            # start (np. po restarcie komputera)
docker exec -it knr_drone bash    # nowy terminal
docker attach knr_drone           # główna powłoka; odłączenie: Ctrl+P, Ctrl+Q
docker stop knr_drone             # stop
docker ps -a                      # stan
docker rm knr_drone               # usunięcie (potem znowu krok 2.6)
```


---

Opis wszystkich pakietów: [docs/co_do_czego.md](docs/co_do_czego.md). 

Praca z gitem: [docs/git_komendy.md](docs/git_komendy.md).

---
