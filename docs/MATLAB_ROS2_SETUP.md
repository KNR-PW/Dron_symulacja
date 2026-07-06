# Setup MATLAB R2026a + ROS2 do sterowania dronem PX4 (KNR-PW ERC)

Kompletna instrukcja postawienia MATLABa od zera do działającego sterowania dronem
przez ROS2 (`drone_interfaces` actions/services -> `drone_handler_px4` -> PX4 SITL).

Kroki oznaczone **[WSL]** są specyficzne dla setupu na WSL2 i mogą być zbędne
na natywnym Linuksie. Reszta jest uniwersalna.

---

## 0. Wymagania wstępne

- MATLAB R2026a z toolboxami: **ROS Toolbox**, Simulink, Stateflow
- Działający `drone_handler_px4` w kontenerze (wystawia `/knr_hardware/*`).
- Repo `Dron_symulacja` z pakietami `px4_msgs` i `drone_interfaces` w `src/`.

---

## 1. Instalacja MATLAB

Instalacja przez **GUI installer** (nie MPM — MPM nie ma aktywatora, kończy się
`Licensing shutdown`).

---

## 2. Zależności systemowe (build typów ROS2)

`ros2genmsg` buduje typy natywnie i ma **twarde wymogi wersji** — Ubuntu 24.04
ma za nowe Pythona i gcc, trzeba dołożyć starsze.

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

## 3. Przygotowanie pakietów interfejsów

Skopiuj `px4_msgs` i `drone_interfaces` z repo do osobnych folderów roboczych
(ros2genmsg buduje w miejscu i zaśmieca folder — nie buduj wprost w `src/`).

```bash
mkdir -p ~/matlab_msgs ~/matlab_interfaces
cp -r ~/Dron_symulacja/src/px4_msgs ~/matlab_msgs/
cp -r ~/Dron_symulacja/src/drone_interfaces ~/matlab_interfaces/
```

### WAŻNE: usuń polskie znaki z definicji interfejsów
Parser `rosidl` (Python 3.9 wewnątrz MATLAB) **dławi się na nie-ASCII**
w komentarzach `.msg`/`.srv`/`.action` — build pada z błędem typu
`'t' is an invalid message name` (parser bierze fragment "kąt" za nazwę pola).

```bash
cd ~/matlab_interfaces/drone_interfaces
for f in msg/*.msg srv/*.srv action/*.action; do
    iconv -f UTF-8 -t ASCII//TRANSLIT "$f" -o "$f.tmp" 2>/dev/null && mv "$f.tmp" "$f"
done
# weryfikacja — nie moze nic wypisac:
grep -rlP '[^\x00-\x7F]' msg/ srv/ action/
```

---

## 4. Profil UDP dla Fast DDS

**Tylko na WSL.** Discovery MATLAB<->kontener działa (multicast UDP), ale
**transport danych** domyślnie idzie przez Fast DDS **shared memory (SHM)**,
który NIE przekracza granicy kontenera. Objaw: `ros2 topic list` widzi topiki,
ale `receive`/subscriber nie dostaje danych (`LatestMessage: []`).

Fix: wymuś transport UDP profilem XML (tylko po stronie MATLAB — mostu
microXRCE NIE trzeba ruszać).

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

Zmienna `FASTRTPS_DEFAULT_PROFILES_FILE` wskazująca ten plik ustawiana jest
w `startup.m` (sekcja 6).

---

## 5. Build typów ROS2 (JEDNORAZOWO)

Odpal MATLAB **z ustawionym gcc-12** (inaczej ros2genmsg użyje domyślnego 13.3
i padnie):
```bash
export CC=/usr/bin/gcc-12
export CXX=/usr/bin/g++-12
matlab
```

W MATLAB (jednorazowo — patrz `matlab_ros2_setup_ONCE.m`).
Uwaga: MATLAB **nie rozwija `$USER`** jak bash — użyj `getenv('HOME')`:
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
- `'X' is an invalid message name` -> nie-ASCII w plikach, wróć do sekcji 3 (iconv).
- `Could not find package geometry_msgs/std_msgs` -> zależność zewnętrzna;
  standardowe typy MATLAB ma wbudowane, ale sprawdź czy build je widzi.

---

## 6. startup.m (PER SESJA — automatyczny przy każdym starcie)

Umieść w userpath (sprawdź: `userpath` w MATLAB, zwykle `~/Documents/MATLAB/`).
Ładuje się sam przy każdym starcie MATLAB — rejestruje typy i wymusza UDP.

```matlab
% ~/Documents/MATLAB/startup.m
home = getenv('HOME');
addpath(fullfile(home, 'matlab_msgs/matlab_msg_gen_R2026a/glnxa64/install/m'));
addpath(fullfile(home, 'matlab_interfaces/matlab_msg_gen_R2026a/glnxa64/install/m'));
setenv('FASTRTPS_DEFAULT_PROFILES_FILE', fullfile(home, 'fastdds_udp.xml'));  % [WSL] UDP
setenv('ROS_DOMAIN_ID', '0');
```

> `savepath` gołe wywala warning (brak prawa zapisu do root-owned instalacji MATLAB).
> Jeśli chcesz zapisać path na trwałe: `savepath(fullfile(userpath,'pathdef.m'))`.
> Ale `startup.m` z `addpath` załatwia to per sesja bez potrzeby savepath.

---

## 7. Weryfikacja

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

## 8. Sterowanie dronem — `DroneControllerMATLAB`

Klasa opakowująca ROS2 actions/services handlera. Wymaga **działającego
`drone_handler_px4`** (wystawia `/knr_hardware/*`) + zbudowanych typów.

### Kluczowe ustalenia o handlerze
- Actions/services są pod namespace **`/knr_hardware/`**.
- Handler mapuje **stringi ArduPilot** na komendy PX4:
  - `'GUIDED'` -> PX4 **offboard** (`DO_SET_MODE param2=6`)
  - `'LAND'` -> land
  - `'RTL'` -> return to launch
  - `'Offboard'/'Land'/'Return'` **NIE działają** (brak `if` w handlerze — no-op!).
- **Sekwencja uzbrojenia:** `set_mode('GUIDED')` (offboard) -> pauza (~2s, PX4
  potrzebuje czasu wejść w offboard, nav_state=14) -> akcja `Arm`.
  Sam arm bez offboard = dron uzbraja się w loiter i auto-disarmuje.
- `set_mode` rzuca `ros:mlros2:serviceclient:CallError` mimo że **działa** —
  bo `SetMode.srv` ma pusty response, na którym `call()` się wykłada. Komenda
  i tak dochodzi do PX4. **Nie bać się tego błędu.** (Uwaga: obsługa tego błędu
  nie jest ograna dla wszystkich trybów — jak coś nie działa, sprawdź czy klasa
  łyka CallError dla danego trybu.)

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

1. Odpal symulację + most microXRCE + `drone_handler_px4`.
2. Odpal MATLAB (startup.m załaduje typy + UDP automatycznie).
3. `d = DroneControllerMATLAB(); d.arm(); d.takeoff(...);`

## Kolejność debugowania gdy nie działa
- Cisza na subscriberze -> [WSL] UDP profil (`getenv FASTRTPS...`).
- `Unknown package drone_interfaces` w kontenerze -> `source install/setup.bash`.
- set_mode `CallError` -> to normalne (pusty response), komenda działa.
- Dron nie lata mimo arm -> sprawdź `nav_state` (`ros2 topic echo
  /fmu/out/vehicle_status_v1`): musi być 14 (offboard) ORAZ arming_state 2.
- Klasa nie przeładowuje zmian -> `clear classes; rehash`; jak nie pomaga,
  **restart MATLAB**.
