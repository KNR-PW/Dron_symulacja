KAMERA OAK POE - IP I YOLO ENGINE (JETSON)

## 1) Przypisanie IP (juz jest przypisane na stałe) i test polaczenia

Szybki test po restarcie (na porcie enP8p1s0):

```bash
sudo ip addr add 169.254.1.10/24 dev enP8p1s0
ping -c 3 169.254.1.222
```

Jesli `ping` odpowiada, kamera jest osiagalna pod `169.254.1.222`.

## 2) Konwersja modelu YOLO z .pt do .engine (TensorRT)

W katalogu z modelem uruchom:

```bash
cd /home/jetsonknr/Dron_symulacja/testy_kamery
yolo export model=yolo26s.pt format=engine imgsz=640 device=0 half=True
```

Po eksporcie powinien powstac plik `yolo26s.engine`.

Alternatywnie (Python):

```bash
python3 -c "from ultralytics import YOLO; YOLO('yolo26s.pt').export(format='engine', imgsz=640, device=0, half=True)"
```

## 3) Uruchomienie streamu z plikiem .engine

```bash
cd /home/jetsonknr/Dron_symulacja/testy_kamery
python3 run_jetson_yolo.py --ip 169.254.1.222 --model /home/jetsonknr/Dron_symulacja/testy_kamery/yolo26s.engine --device cuda:0
```

## 4) YOLO NA ROSIE NA JETSONIE

Node `yolo_detector` (paczka `drone_autonomy`) subskrybuje `/oak/rgb/image_raw`
i publikuje annotowany obraz na `/yolo/image_raw`. Domyslnie laduje
`yolo26s.engine` na `cuda:0`.

Build (raz po dodaniu node'a):

```bash
cd ~/Dron_symulacja
colcon build --packages-select drone_autonomy
source install/setup.bash
```

Trzy terminale do uruchomienia:

```bash
# Terminal A — kamera (osobno, zeby restart YOLO nie ruszal OAK)
ros2 run drone_camera oak_publisher --ros-args -p ip:=169.254.1.222

# Terminal B — detektor YOLO (z Twoim .engine domyslnie)
ros2 run drone_autonomy yolo_detector

# Terminal C — podglad annotowanego obrazu
ros2 run rqt_image_view rqt_image_view /yolo/image_raw
```

### Parametry runtime (bez rebuilda)

```bash
ros2 run drone_autonomy yolo_detector --ros-args \
    -p model_path:=/sciezka/do/innego.pt \
    -p conf:=0.4 \
    -p device:=cpu \
    -p image_topic:=/jakas/inna/kamera \
    -p output_topic:=/yolo/debug
```

| param          | typ    | default                                                          |
|----------------|--------|------------------------------------------------------------------|
| `model_path`   | str    | `/home/jetsonknr/Dron_symulacja/testy_kamery/yolo26s.engine`     |
| `conf`         | double | `0.25`                                                           |
| `imgsz`        | int    | `640`                                                            |
| `device`       | str    | `cuda:0` (alternatywa: `cpu`)                                    |
| `image_topic`  | str    | `/oak/rgb/image_raw`                                             |
| `output_topic` | str    | `/yolo/image_raw`                                                |

## 5) Wazne uwagi (Jetson)

- `.engine` najlepiej budowac na tym samym Jetsonie i tej samej wersji JetPack/TensorRT, na ktorej bedzie uruchamiany.
- Jesli zmieniasz `imgsz` przy eksporcie, uruchamiaj inference z takim samym `--imgsz`.
- Dla FP16 zostaw `half=True` (najczesciej szybsze na Jetsonie).

## 6) Niskopoziomowy test kamery (bez ROSa)

```bash
cd /home/jetsonknr/depthai-core/examples/python/Camera
python3 camera_output.py --ip 169.254.1.222
```