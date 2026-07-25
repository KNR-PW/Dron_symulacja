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
ros2 topic pub --once knr_hardware/gimbal_pitch std_msgs/msg/Float32 "{data: -45.0}"  # w dol
ros2 topic pub --once knr_hardware/gimbal_pitch std_msgs/msg/Float32 "{data: 30.0}"   # pozycja SEARCH
```
**PASS:** serwo staje na zadanym kacie. Nie rusza → BEC / reboot Cube'a / `MNT1_DEFLT_MODE=2`.

---

## 2. suas_detect_jetson  [T2]

```bash
ros2 launch drone_bringup suas_detect_jetson.launch.py
# opcje: conf:=0.35  camera_topic:=/oak/rgb/image_raw  model_path:=/pelna/sciezka/MODEL4.engine
```

Sprawdzenie (T3):
```bash
ros2 topic hz /oak/rgb/image_raw
ros2 topic echo /oak/rgb/image_raw --field height --once   # ma byc 760 (config 12MP + ISP 1/4)
ros2 topic hz /tent_detections
ros2 topic echo /tent_detections --once
```
**PASS:** w logu `FPS ... Detekcje namiotu: X/Y`, `detected: true` na namiocie.

Podglad na stacji naziemnej (ta sama siec ROS_DOMAIN_ID):
```bash
ros2 run rqt_image_view rqt_image_view /tent_detections/image/compressed
```

---

## 3. suas_gimbal_controller  [T3]

Wymaga: T1 + T2 dzialaja.

```bash
ros2 launch drone_bringup suas_gimbal_controller.launch.py img_h:=760 vfov_deg:=<zmierzone>
# opcje: damping:=0.4  deadzone:=0.06  control_rate:=10.0
```

Domyslne `img_h:=1024 vfov_deg:=114.6` sa dla Gazebo — nie uzywaj na realu.

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
Uwaga: to rate YOLO, nie kamery. Kamera daje 15, detektor zwykle mniej.

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
| Gimbal stoi mimo `[SLEDZE]` | T1 zyje? `ros2 topic echo knr_hardware/gimbal_pitch` |
| Laguje obraz na GCS | nagrywaj lokalnie, na GCS tylko `image/compressed` |
