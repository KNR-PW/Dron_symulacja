# Plan testow SUAS — real (Jetson + Orange Cube + OAK-D)

Kazdy terminal zaczyna od:
```bash
cd ~/Dron_symulacja && source install/setup.bash
```

Build (raz, po zmianach w kodzie):
```bash
colcon build --symlink-install --packages-select drone_hardware drone_detector drone_autonomy drone_bringup drone_camera
```

---

## 0. Przed startem

```bash
sudo nvpmodel -m 1 && sudo jetson_clocks && sudo nvpmodel -q   # 25W
ls /dev/ttyACM*                                                # port Cube'a
ls -lh src/drone_detector/models/MODEL4.engine                 # model TensorRT istnieje?
```

Bez smigiel. Cube po reboocie (po zapisie parametrow gimbala — patrz `docs/gimbal setup.md`).

---

## 1. drone_handler  [T1]

```bash
ros2 run drone_hardware drone_handler --ros-args -p fc_ip:=/dev/ttyACM0
```

Sprawdzenie (T2):
```bash
ros2 topic echo knr_hardware/telemetry --once
ros2 topic pub --once knr_hardware/gimbal_pitch std_msgs/msg/Float32 "{data: 0.0}"    # poziomo
ros2 topic pub --once knr_hardware/gimbal_pitch std_msgs/msg/Float32 "{data: -45.0}"  # pod katem w dol/przod = SEARCH
ros2 topic pub --once knr_hardware/gimbal_pitch std_msgs/msg/Float32 "{data: -90.0}"  # prosto w dol
```
**PASS:** serwo staje na zadanym kacie. Nie rusza → BEC / reboot Cube'a / `MNT1_DEFLT_MODE=2`. ok 
1.ok
---

## 2. suas_detect_jetson  [T2]

```bash
ros2 launch drone_bringup suas_detect_jetson.launch.py
# opcje: conf:=0.35  camera_topic:=/oak/rgb/image_raw  model_path:=/pelna/sciezka/MODEL4.engine
#        debug_every_n:=2 (rzadszy podglad)  debug_jpeg_quality:=20
```

Sprawdzenie (T3):
```bash
ros2 topic hz /oak/rgb/image_raw
ros2 topic echo /oak/rgb/image_raw --field height --once   # ma byc 760 (config 12MP + ISP 1/4)
ros2 topic hz /tent_detections
ros2 topic echo /tent_detections --once
```
**PASS:** w logu `FPS ... Detekcje namiotu: X/Y`, `detected: true` na namiocie.

Zmierzone (29.07): `image_raw` ~12 Hz (nie 15, jitter 0.065–0.20 s), `height=760`.

`The message type 'drone_interfaces/msg/TentDetection' is invalid` = terminal bez
`source install/setup.bash`. Topic jest publikowany, tylko `ros2 topic echo/hz` nie zna typu.

Podglad na stacji naziemnej (ta sama siec ROS_DOMAIN_ID):
```bash
ros2 run rqt_image_view rqt_image_view /tent_detections/image/compressed
```

---

## 3. suas_gimbal_controller  [T3]

Wymaga: T1 + T2 dzialaja.

```bash
ros2 launch drone_bringup suas_gimbal_controller.launch.py img_h:=760 vfov_deg:=64.4
# opcje: damping:=0.4  deadzone:=0.06  control_rate:=10.0
#        pitch_min:=-90.0  pitch_max:=-45.0  pitch_search:=-45.0
```

Zakres pitcha (domyslny, konwencja realu wg `docs/gimbal setup.md`):
`0` = poziomo, `-45` = pod katem w dol/przod (SEARCH), `-90` = prosto w dol.
Gimbal pracuje w `-90..-45`, czyli nigdy nie patrzy nad horyzont.
W Gazebo kalibracja jest inna (`-45` = prosto w dol) — tam podaj
`pitch_min:=-45.0 pitch_max:=45.0 pitch_search:=30.0`.

Domyslne `img_h:=1024 vfov_deg:=114.6` sa dla Gazebo — nie uzywaj na realu.
`64.4` = pomiar z 29.07 (patrz 3a). Powtorz pomiar po zmianie configu kamery.

**Nie wpisuj 720.** `/tent_detections/image` ma 960x720, bo podglad debug jest
skalowany do 960 px szerokosci. Wspolrzedne w `/tent_detections` sa w pikselach
oryginalnego obrazu, czyli `img_h` = 760 (to co zwraca `image_raw --field height`).

### 3a. Pomiar VFOV — raz, przy dzialajacym T2

Wklej w terminal, przepisz `VFOV` do `vfov_deg:=`:

```bash
python3 - <<'EOF'
import math, rclpy
from sensor_msgs.msg import CameraInfo
rclpy.init(); n = rclpy.create_node('fov'); box = []
n.create_subscription(CameraInfo, '/oak/rgb/camera_info', box.append, 10)
while not box: rclpy.spin_once(n)
m = box[0]
print(f"img_h  = {m.height}")
print(f"VFOV   = {math.degrees(2*math.atan(m.height/(2*m.k[4]))):.1f}")
EOF
```

Powtorz po kazdej zmianie `i_resolution` / `i_isp_den` w `config/oak_rgb.yaml`.

Logi: `[SLEDZE]` dosuwa, `[CENTR]` wycentrowany, `[SZUKAM]` brak namiotu.

**Test:** namiot ponizej srodka kadru → gimbal jedzie w dol → namiot wraca na srodek.
Oscyluje → `damping:=0.25`. Reaguje za wolno → `damping:=0.6`.
Jedzie w zla strone → zly znak/`SERVO7_REVERSED`, nie zmieniaj kodu.

---

## 4. Nagrywanie podgladu z detekcja  [T4]

Najpierw zmierz realna czestotliwosc nagrywanego topicu — to jest `fps` do zapisu:
```bash
ros2 topic hz /tent_detections/image
```
Uwaga: to nie jest rate kamery (~12 Hz zmierzone). Podglad leci z kazdej klatki
detektora — jesli chcesz rzadziej (mniej CPU / mniej danych po Wi-Fi), odpal
detektor z `debug_every_n:=2`. Do `.avi` wpisz to co pokazal `hz`, nie 15.

**Bag (zalecane — fps nieistotny, bag trzyma znaczniki czasu):**
```bash
mkdir -p ~/loty && ros2 bag record -o ~/loty/test_$(date +%m%d_%H%M) \
  /tent_detections/image/compressed /tent_detections /knr_hardware/telemetry
```
Stop: `Ctrl+C`. Odtworzenie pozniej:
```bash
ros2 bag play ~/loty/test_XXXX
ros2 run rqt_image_view rqt_image_view /tent_detections/image/compressed
```

**Albo od razu plik .avi — podaj `fps` z pomiaru wyzej:**
```bash
ros2 run drone_camera video_recorder --ros-args \
  -p camera_topic:=/tent_detections/image -p fps:=15.0 \
  -p save_directory_base:=/home/jetsonknr/saved_video
```
Start / stop nagrywania (T5):
```bash
ros2 service call knr_video/turn_on_video  drone_interfaces/srv/TurnOnVideo  "{}"
ros2 service call knr_video/turn_off_video drone_interfaces/srv/TurnOffVideo "{}"
```
Plik: `<save_directory_base>/<N>/videoN.avi` (MJPG). Zly `fps` = wideo za szybkie/wolne.

Miejsce na dysku przed lotem:
```bash
df -h ~
```

---

## Kolejnosc uruchamiania (podsumowanie)

| Term | Co | Kiedy |
|---|---|---|
| T1 | `drone_handler` | pierwszy, czekaj na polaczenie z FC |
| T2 | `suas_detect_jetson` | gdy kamera widoczna, czekaj na `ready` + FPS |
| T3 | `suas_gimbal_controller` | dopiero gdy `/tent_detections` publikuje |
| T4 | `ros2 bag record` | tuz przed testem |

Zamykanie: odwrotnie (T4 → T3 → T2 → T1), `Ctrl+C` w kazdym.

---

## Szybka diagnostyka

```bash
ros2 node list
ros2 topic list | grep -E "tent|oak|knr_hardware"
ros2 topic hz /tent_detections/image/compressed
ros2 param get /suas_gimbal_controller vfov_deg
tegrastats                                        # obciazenie GPU/CPU Jetsona
```

| Objaw | Sprawdz |
|---|---|
| Brak `/oak/rgb/image_raw` | kabel USB3, `i_pipeline_type: RGB` w `config/oak_rgb.yaml` |
| Detektor sie nie laduje | sciezka `.engine`, engine zbudowany dla `imgsz=1024` |
| `/tent_detections` pusty | `conf:=0.25`, namiot w kadrze, oswietlenie |
| `message type ... is invalid` | brak `source install/setup.bash` w tym terminalu |
| `/tent_detections/image` cisza | podglad idzie tylko gdy ktos subskrybuje — odpal `rqt_image_view` albo `hz` |
| Gimbal stoi mimo `[SLEDZE]` | T1 zyje? `ros2 topic echo knr_hardware/gimbal_pitch` |
| Laguje obraz na GCS | nagrywaj lokalnie, na GCS tylko `image/compressed` |
