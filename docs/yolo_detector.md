# YOLO Detector — detekcja namiotow

## Co to robi

Node ROS 2 `yolo_detector` wykrywa namioty na obrazie z kamery drona
za pomoca modelu YOLOv8n w formacie ONNX (inferencja przez `onnxruntime` na CPU).

## Pliki

| Plik | Opis |
|------|------|
| `src/drone_detector/drone_detector/yolo_detector.py` | Node ROS 2 |
| `src/drone_detector/launch/yolo_detector.launch.py` | Launch: most GZ + node + rqt |
| `src/drone_interfaces/msg/TentDetection.msg` | Wiadomosc detekcji |
| `yolo/best.pt` | Model YOLOv8n (Ultralytics, 1 klasa: "namiot") |
| `yolo/best.onnx` | Model ONNX (wygenerowany z best.pt) |
| `yolo/convert_to_onnx.py` | Skrypt konwersji .pt -> .onnx |

## Topici

| Topic | Typ | Kierunek |
|-------|-----|----------|
| `/rgb_camera/image` | `sensor_msgs/Image` | subskrypcja (kamera Gazebo) |
| `/tent_detections` | `drone_interfaces/TentDetection` | publikacja |
| `/tent_detections/image` | `sensor_msgs/Image` | publikacja (podglad z BB) |

## TentDetection.msg

```
bool detected          # true = namiot wykryty
float32[4] bounding_box  # [x, y, w, h] piksele (lewy gorny rog + rozmiar)
float32 confidence     # pewnosc 0.0-1.0
```

## Parametry node'a

| Parametr | Domyslna wartosc | Opis |
|----------|-----------------|------|
| `camera_topic` | `/rgb_camera/image` | Topic z obrazem z kamery |
| `model_path` | `yolo/best.onnx` | Sciezka do modelu ONNX |
| `confidence_threshold` | `0.5` | Prog pewnosci detekcji |
| `nms_threshold` | `0.45` | Prog NMS |
| `input_size` | `1024` | Rozmiar wejscia sieci (model trenowany na 1024) |
| `debug_width` | `480` | Szerokosc obrazu debug w pikselach |

## Wymagania (w kontenerze)

```bash
pip install --break-system-packages onnxruntime
```

## Konwersja modelu (jednorazowo, lokalnie)

```bash
cd ~/Dron_symulacja
source .venv/bin/activate
python3 yolo/convert_to_onnx.py
```

Tworzy `yolo/best.onnx` (imgsz=1024). Docker widzi go automatycznie przez mount.

## Build (w kontenerze)

```bash
cd ~/Dron_symulacja
colcon build --packages-select drone_interfaces drone_detector
source install/setup.bash
```

## Uruchomienie

### Sposob 1: Launch (most + node + rqt — wszystko razem)

Uzywac gdy symulacja Gazebo dziala, ale **bez** `sim_px4.launch.py`
(bo launch sam uruchamia most kamery):

```bash
source /opt/ros/jazzy/setup.bash
source ~/Dron_symulacja/install/setup.bash
ros2 launch drone_detector yolo_detector.launch.py
```

### Sposob 2: Sam node (gdy most juz dziala, np. z sim_px4.launch.py)

```bash
source /opt/ros/jazzy/setup.bash
source ~/Dron_symulacja/install/setup.bash
ros2 run drone_detector yolo_detector --ros-args \
    -p model_path:=/root/Dron_symulacja/yolo/best.onnx
```

### Podglad detekcji w terminalu

```bash
ros2 topic echo /tent_detections
```

### Podglad obrazu z BB

```bash
ros2 run rqt_image_view rqt_image_view /tent_detections/image
```

## Wydajnosc (CPU, Ryzen 5 1600X)

- Input 1024x1024: ~1-2 fps (inferencja ~500-800 ms/klatka)
- Input 640x640: ~3-5 fps (~200-300 ms/klatka)

## Sciezka Jetson Nano (TensorRT) — TODO

Na Jetsonie po wygenerowaniu ONNX:

```bash
trtexec --onnx=yolo/best.onnx --saveEngine=yolo/best.engine --fp16
```

Osobny node `yolo_detector_jetson` z TensorRT Python API zamiast onnxruntime.
Interfejs ROS (topici, wiadomosci, parametry) pozostaje identyczny.
