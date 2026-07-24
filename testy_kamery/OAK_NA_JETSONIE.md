# OAK (USB + PoE) na Jetson Orin — Notatki techniczne

Data: 2026-05-11
Platforma: Jetson Orin, Ubuntu 22.04, ROS2 Humble, kernel 5.15.185-tegra

## TL;DR

Na Jetsonie ROS-owy `depthai_ros_driver` (2.x i 3.x z apt) **NIE łączy się** z naszą
kamerą OAK-D-PRO-W-POE mimo że low-level `depthai` w Pythonie łączy bez problemu.
Driver milcząco rzuca wyjątek w pętli retry i nigdy nie publikuje topików.

Rozwiązanie: napisany własny lekki node Python — `drone_camera/oak_publisher.py` —
który używa działającej ścieżki `dai.Device(deviceInfo)` z `state=ANY`
i publikuje topiki kompatybilne z `depthai_ros_driver`
(`/oak/rgb/image_raw`, `/oak/rgb/camera_info`).

USB OAK: działa po dodaniu reguł udev i wymuszeniu USB2 (`i_usb_speed: HIGH`)
przez param drivera v3 (na Jetsonie Orin USB3 ma re-enumerację po boocie firmware).
PoE OAK: nie działa z driverami ROS, używamy `oak_publisher`.

## Stan końcowy — uruchamianie

```bash
source /opt/ros/humble/setup.bash
source ~/Dron_symulacja/install/setup.bash
ros2 launch drone_bringup drone_mamba_POE.launch.py
```

`drone_mamba.launch.py` startuje:
- `oak_publisher` (PoE 169.254.1.222) → `/oak/rgb/image_raw` @ 15 Hz, 1280x720, bgr8
- `drone_hardware/drone_handler`
- `drone_camera/images_recorder`
- `ros2_aruco/aruco_node`

Sam podgląd kamery (debug):
```bash
ros2 run drone_camera oak_publisher --ros-args -p ip:=169.254.1.222
# w drugim terminalu:
ros2 run rqt_image_view rqt_image_view /oak/rgb/image_raw
ros2 topic hz /oak/rgb/image_raw   # powinno być ~15 Hz
```

## Co zostało zrobione (kolejność rzeczywista)

### 1. Instalacja stosu DepthAI ROS

Próbowaliśmy najpierw `ros-humble-depthai-ros-driver-v3` (3.1.1, depthai-core 3.2.2).
Potem `ros-humble-depthai-ros-driver` (2.12.2, depthai-core 2.31.1).
**Oba pakiety mają identyczny bug na Jetsonie z PoE** — patrz niżej.

Stan końcowy paczek (`dpkg -l | grep depthai`):
```
ros-humble-depthai                2.31.1
ros-humble-depthai-bridge         2.12.2
ros-humble-depthai-descriptions   2.12.2
ros-humble-depthai-examples       2.12.2
ros-humble-depthai-filters        2.12.2
ros-humble-depthai-ros            2.12.2
ros-humble-depthai-ros-driver     2.12.2
ros-humble-depthai-ros-msgs       2.12.2
```

Pakiety v3 zostały usunięte (`sudo dpkg --purge --force-depends ros-humble-depthai-*-v3`
— wymagało bypass apt bo deps się przeplatały, dpkg po kolei usunął wszystkie -v3).

### 2. USB OAK na Jetsonie — problem `X_LINK_DEVICE_NOT_FOUND` po boocie firmware

OAK USB enumeruje się jako `03e7:2485` (unbooted), driver wgrywa firmware,
urządzenie re-enumeruje się jako `03e7:f63b` i znika dla drivera. Powód:
brak reguł udev + Jetson Orin ma problemy z re-enumeracją USB3 (zasilanie/sync).

Fix:
```bash
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' \
  | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
# odepnij i wepnij OAK USB
```

Drugie kluczowe: wymuszenie **USB2 HIGH** w YAML konfigu drivera (USB3 nie działa
stabilnie na tym Jetsonie z tą kamerą):
```yaml
/oak:
  ros__parameters:
    driver:
      i_usb_speed: "HIGH"
```

Po tej konfiguracji `ros2 launch depthai_ros_driver_v3 driver.launch.py`
podłączył USB OAK-D-PRO-W i wystawił pełen RGBD pipeline (testowany tylko z v3,
ale v2 powinien zachować się analogicznie).

### 3. PoE OAK — diagnoza



#### Co NIE działa

`ros2 launch depthai_ros_driver rgbd_pcl.launch.py params_file:=...` zawisa w pętli:
```
[oak]: No devices detected by autodiscovery, trying to connect to camera via IP: 169.254.1.222
[oak]: Connecting to the camera using ip: 169.254.1.222
```
Każda iteracja ~1s, brak komunikatu błędu — driver łapie wyjątek z depthai-core
i restartuje connect, w nieskończoność.

#### Przyczyna

Kamera jest w stanie **BOOTED** (firmware standalone działa, port TCP **11490** open,
**11491** — bootloader — refused):
```
nc -zv 169.254.1.222 11490  → succeeded
nc -zv 169.254.1.222 11491  → refused
```

Test stanów (python `dai.DeviceInfo` + ręcznie ustawiony `info.state`):

| state                | rezultat                                       |
|----------------------|------------------------------------------------|
| `X_LINK_ANY_STATE`   | **CONNECTED** — działa                         |
| `X_LINK_UNBOOTED`    | FAIL: Cannot find any device with given deviceInfo |
| `X_LINK_BOOTED`      | FAIL (mimo że port 11490 nasłuchuje)           |
| `X_LINK_BOOTLOADER`  | FAIL                                           |

ROS-owy `depthai_ros_driver` przy konstruowaniu `DeviceInfo` wstawia konkretny
state (UNBOOTED), bo zakłada że kamera startuje od bootloadera na 11491.
Nasza kamera startuje od razu w BOOTED na 11490 → driver nigdy nie wpasuje się
w żaden ze stanów które probuje.

Sprawdzenie czy w flash kamery jest standalone application (`clear_standalone.py`):
```
hasApplication: False
firmwareVersion: ""
applicationName:  ""
```
Tj. NIC nie jest sflashowane — kamera po prostu domyślnie startuje
firmware z RAM uploadowanego z poprzedniej sesji depthai-pythona,
a `flashClear` niczego nie zmienia. Po power-cycle też wraca do BOOTED.

Wniosek: na tym Jetson + bootloader 0.0.28 ROS-owy driver nie umie
zbudować poprawnego `DeviceInfo` dla naszej kamery PoE. Naprawa wymagałaby
rebuildu drivera ze źródeł z patchem na `state=ANY`.

### 4. Rozwiązanie — `drone_camera/oak_publisher.py`

Lekki node w Pythonie który omija zepsutego drivera i wystawia te same topiki.
Plik: `src/drone_camera/drone_camera/oak_publisher.py`

Co robi:
1. Konstruuje pipeline depthai: `ColorCamera` 1080P → resize `setVideoSize(1280, 720)`
   → `XLinkOut` o nazwie strumienia `rgb`
2. Łączy się przez `dai.Device(pipeline, dai.DeviceInfo("169.254.1.222"))`
   — `DeviceInfo` BEZ jawnego state → użyte `ANY` → działa
3. W osobnym wątku wykonuje `outputQueue.tryGet()` i publikuje co klatkę:
   - `/oak/rgb/image_raw` (`sensor_msgs/Image`, bgr8)
   - `/oak/rgb/camera_info` (`sensor_msgs/CameraInfo`)

Parametry ROS:
| param      | typ    | default                              |
|------------|--------|--------------------------------------|
| `ip`       | str    | `""` (puste → USB autodetect)        |
| `fps`      | int    | `60`                                 |
| `width`    | int    | `1920`                               |
| `height`   | int    | `1080`                                |
| `frame_id` | str    | `oak_rgb_camera_optical_frame`       |



Wpięte do `setup.py` drone_camera (entry_point `oak_publisher`).

### 5. Nowy `drone_mamba_POE.launch.py`

Plik: `src/drone_bringup/launch/drone_mamba_POE.launch.py`

Wariant `drone_mamba.launch.py` dla PoE — zamiast
`IncludeLaunchDescription(depthai_ros_driver/rgbd_pcl.launch.py)` startuje
`drone_camera/oak_publisher` z IP `169.254.1.222` w parameter.
Reszta launchu (`drone_handler`, `images_recorder`, `aruco_node`) bez zmian.

Oryginalny `drone_mamba.launch.py` **NIE jest modyfikowany** — zostaje z
referencją do `depthai_ros_driver/rgbd_pcl.launch.py` (działa wszędzie tam
gdzie ROS driver łapie kamerę, np. na malinach).

Wybór wariantu:
| sprzęt                          | launch                          |
|---------------------------------|---------------------------------|
| Raspberry Pi + OAK              | `drone_mamba.launch.py`         |
| Jetson Orin + OAK-D-PRO-W-POE   | `drone_mamba_POE.launch.py`     |


## Co należałoby jeszcze zrobić

1. **Depth + stereo w oak_publisher** — obecnie publikujemy tylko RGB.
   Jeśli ArUco wystarcza obraz mono, OK. Jeśli pipeline drona potrzebuje depth/PCL,
   trzeba dodać `MonoCamera × 2 + StereoDepth` do pipeline'u i publikować
   `/oak/stereo/image_raw`, `/oak/depth/image_raw`, opcjonalnie `/oak/points`.
2. **TF publisher** — `depthai_ros_driver` publikował też static TF
   z URDF kamery. Jeśli używamy TF do aruco/pcl, trzeba dodać `robot_state_publisher`
   z URDF z `depthai_descriptions` (paczka jest zainstalowana).
3. **Reconnect on error** — obecnie jeśli kamera zniknie z sieci,
   node po prostu się wywali. Dodać auto-retry w `_publish_loop`.
4. **CameraInfo z kalibracji** — teraz `CameraInfo` jest pusty (tylko width/height).
   Pobrać kalibrację z urządzenia (`device.readCalibration()`)
   i wypełnić `k`, `d`, `r`, `p`. ArUco używa kalibracji do detekcji.

## Wersje (snapshot)

- depthai-core (ROS): `2.31.1` (`ros-humble-depthai`)
- depthai-python (po `pip install --user depthai==2.31.1`): `2.31.1.0`
- depthai-ros-driver: `2.12.2`
- ROS2: Humble
- Kamera bootloader: `0.0.28` (na flashu — odczyt przez `DeviceBootloader.getVersion()`)
- Kamera firmware: `OAK-D-PRO-W-POE`, MxID `19443010417E842F00`, IP `169.254.1.222`
