# Setup MATLAB R2026a + ROS2 do sterowania dronem PX4 (KNR-PW ERC)

> 🇬🇧 English version: [below](#english-version)

Kompletna instrukcja postawienia MATLABa od zera do działającego sterowania dronem przez ROS2 (`drone_interfaces` actions/services -> `drone_handler_px4` -> PX4 SITL).

Kroki oznaczone **[WSL]** są specyficzne dla setupu na WSL2 i mogą być zbędne na natywnym Linuksie. Reszta jest uniwersalna.

---

## 0. Wymagania wstępne

- MATLAB R2026a z toolboxami: **ROS Toolbox**, Simulink, Stateflow
- Działający `drone_handler_px4` w kontenerze (wystawia `/knr_hardware/*`).
- **QGroundControl** odpalony na hoście (sekcja 1, bez niego PX4 nie uzbroi drona).
- Repo `Dron_symulacja` **z zainicjalizowanymi submodułami** (sekcja 1), z pakietami `px4_msgs` i `drone_interfaces` w `src/`.

---

## 1. Repo i submoduły (PX4 + światy Gazebo)

Struktura:

- `Dron_symulacja` -> submoduł `KNR_Drone_PX4_Autopilot` (fork PX4 KNR, branch **`KNR_stable_v1.16.1`** — zawsze działa; `main` to branch roboczy)
- `KNR_Drone_PX4_Autopilot` -> submoduł `Tools/simulation/gz` = `KNR-PW/KNR-PX4-gazebo-models` (światy i modele Gazebo, m.in. `ERC_test`)

Świeży klon:

```bash
git clone --recursive https://github.com/KNR-PW/Dron_symulacja.git
```

Istniejący klon / po każdym pullu, który zmienia wskaźnik submodułu:

```bash
cd ~/Dron_symulacja
docker pull dierust/knr_px4_sim:latest
git pull
git submodule sync --recursive
git submodule update --init --recursive
```

Weryfikacja:

```bash
ls ~/Dron_symulacja/KNR_Drone_PX4_Autopilot/Tools/simulation/gz/worlds/ERC_test.sdf
git -C ~/Dron_symulacja/KNR_Drone_PX4_Autopilot remote -v                       # KNR_Drone_PX4_Autopilot
git -C ~/Dron_symulacja/KNR_Drone_PX4_Autopilot/Tools/simulation/gz remote -v   # KNR-PX4-gazebo-models
```

### Pułapki

- **Pusty `Tools/simulation/gz/`** -> submoduł niezainicjalizowany -> `git submodule update --init --recursive`.
- **`fatal: 'origin' does not appear to be a git repository`** w submodule -> brak remote'a. Dodaj ręcznie URL z `.gitmodules`:
  ```bash
  git -C ~/Dron_symulacja/KNR_Drone_PX4_Autopilot remote add origin https://github.com/KNR-PW/KNR_Drone_PX4_Autopilot.git
  ```
- **Po zmianie wersji PX4** (bump submodułu) usuń stary cache CMake przed buildem: `rm -rf ~/Dron_symulacja/KNR_Drone_PX4_Autopilot/build`.
- **Submoduły po `update` są w detached HEAD.** Zanim zaczniesz commitować: w PX4 `git switch KNR_stable_v1.16.1`, w `Tools/simulation/gz` `git switch main`.
- **`fatal: not a git repository: .../.git/modules/...` w kontenerze** (build PX4 pada na `px_update_git_header.py`) -> repo submodułów leży poza katalogiem zamontowanym w kontenerze. Fix: usuń zagnieżdżone submoduły i sklonuj je od nowa wewnątrz PX4:
  ```bash
  cd ~/Dron_symulacja/KNR_Drone_PX4_Autopilot && git config -f .gitmodules --get-regexp '\.path$' | awk '{print $2}' | xargs -r sudo rm -rf && sudo rm -rf .git/modules && git submodule update --init --recursive
  ```
- **`dubious ownership` / `rev-parse` exit 128 w kontenerze** -> `docker exec knr_drone_px4 git config --global --add safe.directory '*'`.
- **Pull requesty w forkach** (PX4, gazebo-models): GitHub domyślnie ustawia *base repository* na upstream (`PX4/...`). Zawsze sprawdź, że base to `KNR-PW/...`.
- **Kolejność przy zmianach w submodule:** commit + push w submodule -> dopiero potem `git add <submoduł>` + commit + push w repo nadrzędnym (bump wskaźnika). Odwrotnie = wskaźnik na commit, którego nie ma na remote -> `--recursive` u innych się wysypie.
- **Nowe światy/modele Gazebo dla PX4** commitujesz w `KNR-PX4-gazebo-models`, nie w PX4 — w PX4 `Tools/simulation/gz` to tylko wskaźnik.

### Gdzie trzymamy światy Gazebo

| Autopilot | Światy / modele | Dron w świecie | Nowy świat |
|---|---|---|---|
| **PX4** | `KNR_Drone_PX4_Autopilot/Tools/simulation/gz/` → repo `KNR-PX4-gazebo-models` | **nie** — PX4 spawnuje go sam (`TARGET`) | commit + push w gazebo-models → bump `gz` w PX4 (`KNR_stable_v1.16.1`) → bump PX4 w `Dron_symulacja` |
| **ArduPilot** | `src/drone_bringup/gazebo/{worlds,models}` | **tak** — model z pluginem ArduPilota wpisany w `.sdf` | zwykły commit w `Dron_symulacja` |

Nie mieszać: świat ArduPilota z dronem w środku nie zadziała pod PX4 (dwa drony / brak gz_bridge), a świat PX4 nie ma drona dla ArduPilota.

Nowy świat PX4 krok po kroku:

```bash
# 1. gazebo-models
cd ~/Dron_symulacja/KNR_Drone_PX4_Autopilot/Tools/simulation/gz && git switch main && git pull
#    wrzuc worlds/<nazwa>.sdf (+ models/<model>/), <world name="..."> = nazwa pliku
git add worlds/<nazwa>.sdf models/<model> && git commit -m "Add <nazwa> world" && git push origin main
# 2. PX4 stable — najpierw przetestuj świat w SITL (stable ma zawsze działać)
cd ~/Dron_symulacja/KNR_Drone_PX4_Autopilot && git switch KNR_stable_v1.16.1 && git pull
git add Tools/simulation/gz && git commit -m "Bump gz: <nazwa> world" && git push origin KNR_stable_v1.16.1
# 3. Dron_symulacja
cd ~/Dron_symulacja && git add KNR_Drone_PX4_Autopilot && git commit -m "Bump PX4: <nazwa> world" && git push
```

### Odpalenie symulacji ERC

```bash
~/Dron_symulacja/scripts/WSL_scripts/run_px4_sim_erc.sh              # świat ERC_test, gz_x500_mono_cam_down
~/Dron_symulacja/scripts/WSL_scripts/run_px4_sim_erc.sh --headless   # bez GUI Gazebo
```

Skrypt odpala PX4 SITL + Gazebo w kontenerze `knr_drone_px4` (z zamontowanym `KNR_Drone_PX4_Autopilot`). Jeśli kontenera nie ma, wypisze gotową komendę `docker run`. Przed startem sprawdza, czy istnieje świat, czy `<world name>` w SDF zgadza się z nazwą pliku i czy jest airframe dla targetu.

### QGroundControl (WYMAGANY — bez niego `arm` nic nie robi)

PX4 SITL odrzuca uzbrojenie, dopóki nie widzi stacji naziemnej (heartbeat MAVLink). Objaw: `d.arm()` zwraca `status=1 result=0`, handler loguje `Arm:` na czerwono, dron stoi. QGC musi **działać przez cały czas** pracy z symulacją.

Instalacja (jednorazowo, Ubuntu 24.04 / WSL2 z WSLg):

```bash
sudo apt install -y libfuse2 libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor0
cd ~/Dron_symulacja/scripts/start_sim
wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl-x86_64.AppImage
chmod +x QGroundControl-x86_64.AppImage
```

AppImage leży w `scripts/start_sim/` (tam szuka go `start_px4_sim.sh`; `*.AppImage` jest w `.gitignore`, więc nie wpadnie do commita).

Uruchomienie — **przed** `d.arm()`, jako zwykły user (nie `sudo`):

```bash
~/Dron_symulacja/scripts/start_sim/QGroundControl-x86_64.AppImage &
```

Kontener chodzi na `--network host`, więc QGC łapie SITL sam (UDP 14550). Weryfikacja: w lewym górnym rogu QGC status zamiast `Disconnected`, dron widoczny na mapie.

Opcjonalnie (tylko jeśli podpinasz prawdziwy FC po USB, w SITL zbędne):

```bash
sudo usermod -aG dialout "$(id -un)"          # wyloguj się i zaloguj ponownie
sudo systemctl mask --now ModemManager.service # na WSL bez systemd może nie istnieć — wtedy pomiń
```

Pułapki:

- **`dlopen(): error loading libfuse.so.2`** -> brak `libfuse2`.
- **`qt.qpa.plugin: Could not load the Qt platform plugin "xcb"`** -> brak `libxcb-cursor0` / `libxcb-xinerama0`.
- **Okno się nie pokazuje na WSL** -> brak WSLg (wymaga Windows 11 / aktualnego WSL: `wsl --update` w PowerShellu).
- **QGC nie startuje jako root** -> odpalaj bez `sudo`.

---

## 2. Instalacja MATLAB

Instalacja przez **GUI installer** (nie MPM — MPM nie ma aktywatora, kończy się `Licensing shutdown`).

---

## 3. Zależności systemowe (build typów ROS2)

`ros2genmsg` buduje typy natywnie i ma **twarde wymogi wersji** — Ubuntu 24.04 ma za nowe Pythona i gcc, trzeba dołożyć starsze.

### Python 3.10 (ROS Toolbox wymaga 3.9–3.10; Ubuntu 24.04 ma 3.12 — za nowe)

```bash
sudo add-apt-repository ppa:deadsnakes/ppa
sudo apt update
sudo apt install python3.10 python3.10-dev python3.10-venv
```

### gcc-12 (ros2genmsg wymaga gcc <13.1; Ubuntu 24.04 ma 13.3 — za nowe)

```bash
sudo apt install gcc-12 g++-12
```

### Pozostałe zależności

```bash
sudo apt install python3-dev libpython3-dev python3-numpy
```

---

## 4. Przygotowanie pakietów interfejsów

Skopiuj `px4_msgs` i `drone_interfaces` z repo do osobnych folderów roboczych (ros2genmsg buduje w miejscu i zaśmieca folder — nie buduj wprost w `src/`).

```bash
mkdir -p ~/matlab_msgs ~/matlab_interfaces
cp -r ~/Dron_symulacja/src/px4_msgs ~/matlab_msgs/
cp -r ~/Dron_symulacja/src/drone_interfaces ~/matlab_interfaces/
```

### WAŻNE: usuń polskie znaki z definicji interfejsów

Parser `rosidl` (Python 3.9 wewnątrz MATLAB) **dławi się na nie-ASCII** w komentarzach `.msg`/`.srv`/`.action` — build pada z błędem typu `'t' is an invalid message name` (parser bierze fragment "kąt" za nazwę pola).

```bash
cd ~/matlab_interfaces/drone_interfaces
for f in msg/*.msg srv/*.srv action/*.action; do
    iconv -f UTF-8 -t ASCII//TRANSLIT "$f" -o "$f.tmp" 2>/dev/null && mv "$f.tmp" "$f"
done
# weryfikacja — nie moze nic wypisac:
grep -rlP '[^\x00-\x7F]' msg/ srv/ action/
```

---

## 5. Profil UDP dla Fast DDS

**Tylko na WSL.** Discovery MATLAB<->kontener działa (multicast UDP), ale **transport danych** domyślnie idzie przez Fast DDS **shared memory (SHM)**, który NIE przekracza granicy kontenera. Objaw: `ros2 topic list` widzi topiki, ale `receive`/subscriber nie dostaje danych (`LatestMessage: []`).

Fix: wymuś transport UDP profilem XML (tylko po stronie MATLAB — mostu microXRCE NIE trzeba ruszać).

```bash
cat > ~/fastdds_udp.xml << 'EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<dds xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <profiles>
        <transport_descriptors>
            <transport_descriptor>
                <transport_id>udp_transport</transport_id>
                <type>UDPv4</type>
            </transport_descriptor>
        </transport_descriptors>
        <participant profile_name="udp_participant" is_default_profile="true">
            <rtps>
                <userTransports>
                    <transport_id>udp_transport</transport_id>
                </userTransports>
                <useBuiltinTransports>false</useBuiltinTransports>
            </rtps>
        </participant>
    </profiles>
</dds>
EOF
```

Zmienna `FASTRTPS_DEFAULT_PROFILES_FILE` wskazująca ten plik ustawiana jest w `startup.m` (sekcja 7).

---

## 6. Build typów ROS2 (JEDNORAZOWO)

Odpal MATLAB **z ustawionym gcc-12** (inaczej ros2genmsg użyje domyślnego 13.3 i padnie):

```bash
export CC=/usr/bin/gcc-12
export CXX=/usr/bin/g++-12
matlab
```

W MATLAB (jednorazowo — patrz `matlab_ros2_setup_ONCE.m`). Uwaga: MATLAB **nie rozwija `$USER`** jak bash — użyj `getenv('HOME')`:

```matlab
home = getenv('HOME');
pyenv('Version', '/usr/bin/python3.10');              % ROS Toolbox build wymaga 3.10
ros2genmsg(fullfile(home, 'matlab_msgs'));             % build px4_msgs (~kilka min)
ros2genmsg(fullfile(home, 'matlab_interfaces'));       % build drone_interfaces (~kilka min)
```

Czekaj na `Build succeeded` przy każdym. Potem:

```matlab
addpath(fullfile(home, 'matlab_msgs/matlab_msg_gen_R2026a/glnxa64/install/m'));
addpath(fullfile(home, 'matlab_interfaces/matlab_msg_gen_R2026a/glnxa64/install/m'));
clear classes
rehash toolboxcache
```

### Jeśli build pada

- Sprawdź log: `!cat ~/matlab_interfaces/matlab_msg_gen_R2026a/glnxa64/log/build_*/drone_interfaces/stderr.log`
- `'X' is an invalid message name` -> nie-ASCII w plikach, wróć do sekcji 4 (iconv).
- `Could not find package geometry_msgs/std_msgs` -> zależność zewnętrzna; standardowe typy MATLAB ma wbudowane, ale sprawdź czy build je widzi.

---

## 7. startup.m (PER SESJA — automatyczny przy każdym starcie)

Umieść w userpath (sprawdź: `userpath` w MATLAB, zwykle `~/Documents/MATLAB/`). Ładuje się sam przy każdym starcie MATLAB — rejestruje typy i wymusza UDP.

```matlab
% ~/Documents/MATLAB/startup.m
home = getenv('HOME');
addpath(fullfile(home, 'matlab_msgs/matlab_msg_gen_R2026a/glnxa64/install/m'));
addpath(fullfile(home, 'matlab_interfaces/matlab_msg_gen_R2026a/glnxa64/install/m'));
setenv('FASTRTPS_DEFAULT_PROFILES_FILE', fullfile(home, 'fastdds_udp.xml'));  % [WSL] UDP
setenv('ROS_DOMAIN_ID', '0');
```

> `savepath` gołe wywala warning (brak prawa zapisu do root-owned instalacji MATLAB). Jeśli chcesz zapisać path na trwałe: `savepath(fullfile(userpath,'pathdef.m'))`. Ale `startup.m` z `addpath` załatwia to per sesja bez potrzeby savepath.

---

## 8. Weryfikacja

Po starcie MATLAB (ze świeżym `startup.m`):

```matlab
% typy widoczne?
ros2 msg list      % szukaj: drone_interfaces/Telemetry, px4_msgs/VehicleOdometry

% [WSL] czy UDP profil ustawiony?
getenv('FASTRTPS_DEFAULT_PROFILES_FILE')   % musi zwrocic sciezke, nie puste

% test transportu — subskrypcja telemetrii PX4 (wymaga zywej symulacji + mostu)
node = ros2node("/matlab_test");
sub = ros2subscriber(node, "/fmu/out/vehicle_odometry", "px4_msgs/VehicleOdometry", ...
      @(m) fprintf('pos: %.2f %.2f %.2f\n', m.position(1), m.position(2), m.position(3)), ...
      "Reliability","besteffort");
% powinno zaczac drukowac pozycje; jak cisza -> UDP profil nie zadzialal
```

---

## 9. Sterowanie dronem — `DroneControllerMATLAB`

Klasa opakowująca ROS2 actions/services handlera. Wymaga **działającego `drone_handler_px4`** (wystawia `/knr_hardware/*`) + zbudowanych typów.

### Kluczowe ustalenia o handlerze

- Actions/services są pod namespace **`/knr_hardware/`**.
- Handler mapuje **stringi ArduPilot** na komendy PX4:
  - `'GUIDED'` -> PX4 **offboard** (`DO_SET_MODE param2=6`)
  - `'LAND'` -> land
  - `'RTL'` -> return to launch
  - `'Offboard'/'Land'/'Return'` **NIE działają** (brak `if` w handlerze — no-op!).
- **Sekwencja uzbrojenia:** `set_mode('GUIDED')` (offboard) -> pauza (~2s, PX4 potrzebuje czasu wejść w offboard, nav_state=14) -> akcja `Arm`. Sam arm bez offboard = dron uzbraja się w loiter i auto-disarmuje.
- `set_mode` rzuca `ros:mlros2:serviceclient:CallError` mimo że **działa** — bo `SetMode.srv` ma pusty response, na którym `call()` się wykłada. Komenda i tak dochodzi do PX4. **Nie bać się tego błędu.** (Uwaga: obsługa tego błędu nie jest ograna dla wszystkich trybów — jak coś nie działa, sprawdź czy klasa łyka CallError dla danego trybu.)

### Użycie

```matlab
d = DroneControllerMATLAB();   % konstruktor lapie serwery /knr_hardware/*
d.arm();                        % GUIDED(offboard) -> pauza -> Arm
d.takeoff(10);
d.goto_global(47.398, 8.549, 20);
d.goto_relative(5, 0, -2);      % NED: down dodatni=w dol, ujemny=w gore
d.land();
d.stop();
```

## Szybki start (gdy wszystko już postawione)

1. Zaktualizuj repo + submoduły: `git pull && git submodule update --init --recursive` (sekcja 1).
2. Odpal symulację ERC: `~/Dron_symulacja/scripts/WSL_scripts/run_px4_sim_erc.sh`, potem most microXRCE + `drone_handler_px4`.
3. Odpal QGroundControl: `~/Dron_symulacja/scripts/start_sim/QGroundControl-x86_64.AppImage &`.
4. Odpal MATLAB (startup.m załaduje typy + UDP automatycznie).
5. `d = DroneControllerMATLAB(); d.arm(); d.takeoff(...);`

## Kolejność debugowania gdy nie działa

- `BLAD: brak ... ERC_test.sdf` / pusty `Tools/simulation/gz/` -> submoduły niezainicjalizowane (sekcja 1).
- `BLAD: w ERC_test.sdf jest <world name=...>` -> nazwa świata w SDF musi być równa nazwie pliku.
- Cisza na subscriberze -> [WSL] UDP profil (`getenv FASTRTPS...`).
- `Unknown package drone_interfaces` w kontenerze -> `source install/setup.bash`.
- `d.arm()` zwraca `status=1 result=0`, handler `Arm:` na czerwono -> **nie działa QGroundControl** (sekcja 1).
- set_mode `CallError` -> to normalne (pusty response), komenda działa.
- Dron nie lata mimo arm -> sprawdź `nav_state` (`ros2 topic echo /fmu/out/vehicle_status_v1`): musi być 14 (offboard) ORAZ arming_state 2.
- Klasa nie przeładowuje zmian -> `clear classes; rehash`; jak nie pomaga, **restart MATLAB**.

<br>

---
---

<br>

# English version

# MATLAB R2026a + ROS2 setup for PX4 drone control (KNR-PW ERC)

Complete guide from a fresh MATLAB install to working drone control over ROS2 (`drone_interfaces` actions/services -> `drone_handler_px4` -> PX4 SITL).

Steps marked **[WSL]** are specific to the WSL2 setup and may be unnecessary on native Linux. Everything else is universal.

---

## 0. Prerequisites

- MATLAB R2026a with toolboxes: **ROS Toolbox**, Simulink, Stateflow
- Running `drone_handler_px4` in the container (exposes `/knr_hardware/*`).
- **QGroundControl** running on the host (section 1, without it PX4 won't arm).
- `Dron_symulacja` repo **with initialized submodules** (section 1), with the `px4_msgs` and `drone_interfaces` packages in `src/`.

---

## 1. Repo and submodules (PX4 + Gazebo worlds)

Structure:

- `Dron_symulacja` -> submodule `KNR_Drone_PX4_Autopilot` (KNR PX4 fork, branch **`KNR_stable_v1.16.1`** — always working; `main` is the working branch)
- `KNR_Drone_PX4_Autopilot` -> submodule `Tools/simulation/gz` = `KNR-PW/KNR-PX4-gazebo-models` (Gazebo worlds and models, incl. `ERC_test`)

Fresh clone:

```bash
git clone --recursive https://github.com/KNR-PW/Dron_symulacja.git
```

Existing clone / after every pull that changes a submodule pointer:

```bash
cd ~/Dron_symulacja
git pull
git submodule sync --recursive
git submodule update --init --recursive
```

Verification:

```bash
ls ~/Dron_symulacja/KNR_Drone_PX4_Autopilot/Tools/simulation/gz/worlds/ERC_test.sdf
git -C ~/Dron_symulacja/KNR_Drone_PX4_Autopilot remote -v                       # KNR_Drone_PX4_Autopilot
git -C ~/Dron_symulacja/KNR_Drone_PX4_Autopilot/Tools/simulation/gz remote -v   # KNR-PX4-gazebo-models
```

### Pitfalls

- **Empty `Tools/simulation/gz/`** -> submodule not initialized -> `git submodule update --init --recursive`.
- **`fatal: 'origin' does not appear to be a git repository`** in a submodule -> remote missing. Add the URL from `.gitmodules` manually:
  ```bash
  git -C ~/Dron_symulacja/KNR_Drone_PX4_Autopilot remote add origin https://github.com/KNR-PW/KNR_Drone_PX4_Autopilot.git
  ```
- **After a PX4 version change** (submodule bump) delete the old CMake cache before building: `rm -rf ~/Dron_symulacja/KNR_Drone_PX4_Autopilot/build`.
- **Submodules are in detached HEAD after `update`.** Before committing: in PX4 `git switch KNR_stable_v1.16.1`, in `Tools/simulation/gz` `git switch main`.
- **`fatal: not a git repository: .../.git/modules/...` in the container** (PX4 build fails in `px_update_git_header.py`) -> the submodule repos live outside the directory mounted in the container. Fix: delete the nested submodules and re-clone them inside PX4:
  ```bash
  cd ~/Dron_symulacja/KNR_Drone_PX4_Autopilot && git config -f .gitmodules --get-regexp '\.path$' | awk '{print $2}' | xargs -r sudo rm -rf && sudo rm -rf .git/modules && git submodule update --init --recursive
  ```
- **`dubious ownership` / `rev-parse` exit 128 in the container** -> `docker exec knr_drone_px4 git config --global --add safe.directory '*'`.
- **Pull requests in forks** (PX4, gazebo-models): GitHub defaults the *base repository* to upstream (`PX4/...`). Always check the base is `KNR-PW/...`.
- **Order when changing a submodule:** commit + push inside the submodule -> only then `git add <submodule>` + commit + push in the parent repo (pointer bump). The other way round = pointer to a commit that doesn't exist on the remote -> `--recursive` breaks for everyone else.
- **New Gazebo worlds/models for PX4** are committed to `KNR-PX4-gazebo-models`, not to PX4 — in PX4, `Tools/simulation/gz` is only a pointer.

### Where Gazebo worlds live

| Autopilot | Worlds / models | Drone in the world | New world |
|---|---|---|---|
| **PX4** | `KNR_Drone_PX4_Autopilot/Tools/simulation/gz/` → `KNR-PX4-gazebo-models` repo | **no** — PX4 spawns it (`TARGET`) | commit + push in gazebo-models → bump `gz` in PX4 (`KNR_stable_v1.16.1`) → bump PX4 in `Dron_symulacja` |
| **ArduPilot** | `src/drone_bringup/gazebo/{worlds,models}` | **yes** — model with the ArduPilot plugin is in the `.sdf` | plain commit in `Dron_symulacja` |

Don't mix them: an ArduPilot world with a drone inside won't work under PX4 (two drones / no gz_bridge), and a PX4 world has no drone for ArduPilot.

New PX4 world step by step:

```bash
# 1. gazebo-models
cd ~/Dron_symulacja/KNR_Drone_PX4_Autopilot/Tools/simulation/gz && git switch main && git pull
#    add worlds/<name>.sdf (+ models/<model>/), <world name="..."> = file name
git add worlds/<name>.sdf models/<model> && git commit -m "Add <name> world" && git push origin main
# 2. PX4 stable — test the world in SITL first (stable must always work)
cd ~/Dron_symulacja/KNR_Drone_PX4_Autopilot && git switch KNR_stable_v1.16.1 && git pull
git add Tools/simulation/gz && git commit -m "Bump gz: <name> world" && git push origin KNR_stable_v1.16.1
# 3. Dron_symulacja
cd ~/Dron_symulacja && git add KNR_Drone_PX4_Autopilot && git commit -m "Bump PX4: <name> world" && git push
```

### Running the ERC simulation

```bash
~/Dron_symulacja/scripts/WSL_scripts/run_px4_sim_erc.sh              # ERC_test world, gz_x500_mono_cam_down
~/Dron_symulacja/scripts/WSL_scripts/run_px4_sim_erc.sh --headless   # no Gazebo GUI
```

The script starts PX4 SITL + Gazebo in the `knr_drone_px4` container (with `KNR_Drone_PX4_Autopilot` mounted). If the container doesn't exist, it prints a ready `docker run` command. Before starting it checks that the world exists, that `<world name>` in the SDF matches the file name, and that an airframe exists for the target.

### QGroundControl (REQUIRED — without it `arm` does nothing)

PX4 SITL rejects arming until it sees a ground station (MAVLink heartbeat). Symptom: `d.arm()` returns `status=1 result=0`, the handler logs `Arm:` in red, the drone doesn't move. QGC must **stay running** the whole time you work with the simulation.

Install (one-time, Ubuntu 24.04 / WSL2 with WSLg):

```bash
sudo apt install -y libfuse2 libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor0
cd ~/Dron_symulacja/scripts/start_sim
wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl-x86_64.AppImage
chmod +x QGroundControl-x86_64.AppImage
```

The AppImage lives in `scripts/start_sim/` (that's where `start_px4_sim.sh` looks for it; `*.AppImage` is in `.gitignore`, so it won't get committed).

Run it **before** `d.arm()`, as a regular user (not `sudo`):

```bash
~/Dron_symulacja/scripts/start_sim/QGroundControl-x86_64.AppImage &
```

The container runs with `--network host`, so QGC picks up SITL automatically (UDP 14550). Check: the top-left status in QGC is no longer `Disconnected` and the drone shows up on the map.

Optional (only for a real FC over USB, not needed for SITL):

```bash
sudo usermod -aG dialout "$(id -un)"          # log out and back in
sudo systemctl mask --now ModemManager.service # may not exist on WSL without systemd — skip then
```

Pitfalls:

- **`dlopen(): error loading libfuse.so.2`** -> missing `libfuse2`.
- **`qt.qpa.plugin: Could not load the Qt platform plugin "xcb"`** -> missing `libxcb-cursor0` / `libxcb-xinerama0`.
- **No window on WSL** -> no WSLg (needs Windows 11 / up-to-date WSL: `wsl --update` in PowerShell).
- **QGC won't start as root** -> run it without `sudo`.

---

## 2. MATLAB installation

Install via the **GUI installer** (not MPM — MPM has no activator and ends with `Licensing shutdown`).

---

## 3. System dependencies (building ROS2 types)

`ros2genmsg` builds types natively and has **hard version requirements** — Ubuntu 24.04 ships Python and gcc that are too new, so older ones must be added.

### Python 3.10 (ROS Toolbox requires 3.9–3.10; Ubuntu 24.04 has 3.12 — too new)

```bash
sudo add-apt-repository ppa:deadsnakes/ppa
sudo apt update
sudo apt install python3.10 python3.10-dev python3.10-venv
```

### gcc-12 (ros2genmsg requires gcc <13.1; Ubuntu 24.04 has 13.3 — too new)

```bash
sudo apt install gcc-12 g++-12
```

### Other dependencies

```bash
sudo apt install python3-dev libpython3-dev python3-numpy
```

---

## 4. Preparing interface packages

Copy `px4_msgs` and `drone_interfaces` from the repo into separate working folders (ros2genmsg builds in place and pollutes the folder — don't build directly in `src/`).

```bash
mkdir -p ~/matlab_msgs ~/matlab_interfaces
cp -r ~/Dron_symulacja/src/px4_msgs ~/matlab_msgs/
cp -r ~/Dron_symulacja/src/drone_interfaces ~/matlab_interfaces/
```

### IMPORTANT: remove Polish characters from interface definitions

The `rosidl` parser (Python 3.9 inside MATLAB) **chokes on non-ASCII** in `.msg`/`.srv`/`.action` comments — the build fails with an error like `'t' is an invalid message name` (the parser takes a fragment of "kąt" as a field name).

```bash
cd ~/matlab_interfaces/drone_interfaces
for f in msg/*.msg srv/*.srv action/*.action; do
    iconv -f UTF-8 -t ASCII//TRANSLIT "$f" -o "$f.tmp" 2>/dev/null && mv "$f.tmp" "$f"
done
# verification — must print nothing:
grep -rlP '[^\x00-\x7F]' msg/ srv/ action/
```

---

## 5. UDP profile for Fast DDS

**WSL only.** MATLAB<->container discovery works (UDP multicast), but **data transport** goes through Fast DDS **shared memory (SHM)** by default, which does NOT cross the container boundary. Symptom: `ros2 topic list` sees the topics, but `receive`/the subscriber gets no data (`LatestMessage: []`).

Fix: force UDP transport with an XML profile (MATLAB side only — the microXRCE bridge does NOT need to be touched).

```bash
cat > ~/fastdds_udp.xml << 'EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<dds xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <profiles>
        <transport_descriptors>
            <transport_descriptor>
                <transport_id>udp_transport</transport_id>
                <type>UDPv4</type>
            </transport_descriptor>
        </transport_descriptors>
        <participant profile_name="udp_participant" is_default_profile="true">
            <rtps>
                <userTransports>
                    <transport_id>udp_transport</transport_id>
                </userTransports>
                <useBuiltinTransports>false</useBuiltinTransports>
            </rtps>
        </participant>
    </profiles>
</dds>
EOF
```

The `FASTRTPS_DEFAULT_PROFILES_FILE` variable pointing to this file is set in `startup.m` (section 7).

---

## 6. Building ROS2 types (ONE-TIME)

Start MATLAB **with gcc-12 set** (otherwise ros2genmsg uses the default 13.3 and fails):

```bash
export CC=/usr/bin/gcc-12
export CXX=/usr/bin/g++-12
matlab
```

In MATLAB (one-time — see `matlab_ros2_setup_ONCE.m`). Note: MATLAB **does not expand `$USER`** like bash — use `getenv('HOME')`:

```matlab
home = getenv('HOME');
pyenv('Version', '/usr/bin/python3.10');              % ROS Toolbox build requires 3.10
ros2genmsg(fullfile(home, 'matlab_msgs'));             % build px4_msgs (~a few min)
ros2genmsg(fullfile(home, 'matlab_interfaces'));       % build drone_interfaces (~a few min)
```

Wait for `Build succeeded` for each. Then:

```matlab
addpath(fullfile(home, 'matlab_msgs/matlab_msg_gen_R2026a/glnxa64/install/m'));
addpath(fullfile(home, 'matlab_interfaces/matlab_msg_gen_R2026a/glnxa64/install/m'));
clear classes
rehash toolboxcache
```

### If the build fails

- Check the log: `!cat ~/matlab_interfaces/matlab_msg_gen_R2026a/glnxa64/log/build_*/drone_interfaces/stderr.log`
- `'X' is an invalid message name` -> non-ASCII in the files, go back to section 4 (iconv).
- `Could not find package geometry_msgs/std_msgs` -> external dependency; MATLAB has the standard types built in, but check that the build can see them.

---

## 7. startup.m (PER SESSION — runs automatically on every start)

Put it in userpath (check with `userpath` in MATLAB, usually `~/Documents/MATLAB/`). It loads automatically on every MATLAB start — registers the types and forces UDP.

```matlab
% ~/Documents/MATLAB/startup.m
home = getenv('HOME');
addpath(fullfile(home, 'matlab_msgs/matlab_msg_gen_R2026a/glnxa64/install/m'));
addpath(fullfile(home, 'matlab_interfaces/matlab_msg_gen_R2026a/glnxa64/install/m'));
setenv('FASTRTPS_DEFAULT_PROFILES_FILE', fullfile(home, 'fastdds_udp.xml'));  % [WSL] UDP
setenv('ROS_DOMAIN_ID', '0');
```

> Bare `savepath` throws a warning (no write access to the root-owned MATLAB install). To save the path permanently: `savepath(fullfile(userpath,'pathdef.m'))`. But `startup.m` with `addpath` handles it per session without needing savepath.

---

## 8. Verification

After starting MATLAB (with a fresh `startup.m`):

```matlab
% types visible?
ros2 msg list      % look for: drone_interfaces/Telemetry, px4_msgs/VehicleOdometry

% [WSL] is the UDP profile set?
getenv('FASTRTPS_DEFAULT_PROFILES_FILE')   % must return a path, not empty

% transport test — subscribe to PX4 telemetry (requires a live simulation + bridge)
node = ros2node("/matlab_test");
sub = ros2subscriber(node, "/fmu/out/vehicle_odometry", "px4_msgs/VehicleOdometry", ...
      @(m) fprintf('pos: %.2f %.2f %.2f\n', m.position(1), m.position(2), m.position(3)), ...
      "Reliability","besteffort");
% should start printing positions; silence -> UDP profile didn't work
```

---

## 9. Drone control — `DroneControllerMATLAB`

A class wrapping the handler's ROS2 actions/services. Requires a **running `drone_handler_px4`** (exposes `/knr_hardware/*`) + built types.

### Key findings about the handler

- Actions/services live under the **`/knr_hardware/`** namespace.
- The handler maps **ArduPilot strings** to PX4 commands:
  - `'GUIDED'` -> PX4 **offboard** (`DO_SET_MODE param2=6`)
  - `'LAND'` -> land
  - `'RTL'` -> return to launch
  - `'Offboard'/'Land'/'Return'` **DO NOT work** (no `if` in the handler — no-op!).
- **Arming sequence:** `set_mode('GUIDED')` (offboard) -> pause (~2 s, PX4 needs time to enter offboard, nav_state=14) -> `Arm` action. Arming without offboard = the drone arms in loiter and auto-disarms.
- `set_mode` throws `ros:mlros2:serviceclient:CallError` even though it **works** — because `SetMode.srv` has an empty response that `call()` trips over. The command still reaches PX4. **Don't worry about this error.** (Note: handling of this error isn't covered for all modes — if something doesn't work, check whether the class swallows CallError for that mode.)

### Usage

```matlab
d = DroneControllerMATLAB();   % constructor connects to /knr_hardware/* servers
d.arm();                        % GUIDED(offboard) -> pause -> Arm
d.takeoff(10);
d.goto_global(47.398, 8.549, 20);
d.goto_relative(5, 0, -2);      % NED: down positive = down, negative = up
d.land();
d.stop();
```

## Quick start (once everything is set up)

1. Update the repo + submodules: `git pull && git submodule update --init --recursive` (section 1).
2. Start the ERC simulation: `~/Dron_symulacja/scripts/WSL_scripts/run_px4_sim_erc.sh`, then the microXRCE bridge + `drone_handler_px4`.
3. Start QGroundControl: `~/Dron_symulacja/scripts/start_sim/QGroundControl-x86_64.AppImage &`.
4. Start MATLAB (startup.m loads the types + UDP automatically).
5. `d = DroneControllerMATLAB(); d.arm(); d.takeoff(...);`

## Debugging order when something doesn't work

- `BLAD: brak ... ERC_test.sdf` / empty `Tools/simulation/gz/` -> submodules not initialized (section 1).
- `BLAD: w ERC_test.sdf jest <world name=...>` -> the world name in the SDF must match the file name.
- Silence on the subscriber -> [WSL] UDP profile (`getenv FASTRTPS...`).
- `Unknown package drone_interfaces` in the container -> `source install/setup.bash`.
- `d.arm()` returns `status=1 result=0`, handler logs `Arm:` in red -> **QGroundControl isn't running** (section 1).
- set_mode `CallError` -> normal (empty response), the command works.
- Drone doesn't fly despite arming -> check `nav_state` (`ros2 topic echo /fmu/out/vehicle_status_v1`): must be 14 (offboard) AND arming_state 2.
- Class doesn't reload changes -> `clear classes; rehash`; if that doesn't help, **restart MATLAB**.
