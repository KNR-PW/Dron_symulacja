# OAK: kadr i rozdzielczosc detekcji

**Cel misji: wykrywanie LUDZI z ~50 m AGL.** Nie namiotow z 5 m — reszta docs
(`plan_testow_suas.md`, `suas_tracking.md`) opisuje jeszcze stary scenariusz.

Sprzet: OAK-D PRO W (IMX378, 4056x3040), model TensorRT `MODEL4.engine`
ze **sztywnym wejsciem 1024x1024**. Branch `suas-oak-wide-plan-testow`, baza `d3dedf1`.

---

## Liczby

Ogniskowa pelnego sensora, z pomiaru VFOV 64,4 st. dla klatki 4:3:

```
f_sensor = 3040 / (2 * tan(64.4/2)) = 2414 px
GSD      = wysokosc / f_model          px_celu = rozmiar_celu / GSD
FOV      = 2 * atan(1024 / (2*f_model))    slad = 2 * wysokosc * tan(FOV/2)
```

Czlowiek z gory (nadir) = ok. 0,5 m. Z 50 m:

| wariant | kadr | f_model | px na czlowieku | FOV | slad @50 m |
|---|---|---|---|---|---|
| przed 28.08 | 1014x760 | 609 | 6 px | 80x64 st. | 84x63 m |
| **1 — obecny** | 1024x1024, ISP/2 + crop bokow | 813 | **8 px** | 64x64 st. | 63x63 m |
| **2 — plan B** | 1024x1024 natywny crop z 12 MP | 2414 | **24 px** | 24x24 st. | 21x21 m |

YOLO ma stride 8 — ponizej ~10 px obiekt jest niewykrywalny, sensowne minimum to ~20 px.
Iloczyn (px na celu) x (slad) jest staly: to czysty handel polem widzenia za rozdzielczosc.

---

## Wariant 1 — stan obecny: sprawdzenie

Zmiana z 28.08: klatka 4:3 zamieniona na kwadrat 1024x1024, zeby wypelnic caly
tensor modelu (przy 4:3 ultralytics dokladal 257 px szarych pasow = 25% tensora
w nic). Crop tnie **boki**, nie gore/dol, wiec `vfov_deg = 64.4` zostalo bez zmian.

Zmienione: `config/oak_rgb.yaml` (ISP/2, preview 1024 kwadrat), `img_w`/`img_h`
1014x760 -> 1024x1024 w `suas_flight_controller` i `suas_gimbal_controller`
(launche + node'y), `drone_mamba_detect_jetson.launch.py`, `plan_testow_suas.md`.

**Niezweryfikowane:** nazwy `i_enable_preview` / `i_preview_size` /
`i_keep_preview_aspect_ratio` wziete z dokumentacji depthai-ros, sterownika nie
bylo na maszynie gdzie pisano zmiane.

### 1. Czy sterownik przyjal parametry

```bash
ros2 param list /oak | grep -E "preview|isp|output"
ros2 topic echo /oak/rgb/image_raw --field width  --once   # ma dac 1024
ros2 topic echo /oak/rgb/image_raw --field height --once   # ma dac 1024
```

Brak `i_preview_size` -> sprobuj `i_preview_width` / `i_preview_height`.
Wymiary nadal 2028x1520 -> `i_output_isp: false` nie zadzialalo.

### 2. Crop czy rozciagniecie (najwazniejszy test)

Wyceluj w cos **okraglego** (talerz, kolo) i obejrzyj `http://<ip-jetsona>:8080/`.

- okrag okragly -> crop, OK
- owal rozciagniety w pionie -> sterownik **scisnal** 4:3 do kwadratu zamiast
  przyciac. Znieksztalcone proporcje = detekcja do kosza. Sprobuj
  `i_keep_preview_aspect_ratio: false`, a jak dalej zle — wariant 2.

### 3. FOV

```bash
python3 - <<'EOF'
import math, rclpy
from sensor_msgs.msg import CameraInfo
rclpy.init(); n = rclpy.create_node('fov'); box = []
n.create_subscription(CameraInfo, '/oak/rgb/preview/camera_info', box.append, 10)
while not box: rclpy.spin_once(n)
m = box[0]; fx, fy = m.k[0], m.k[4]
print(f"rozmiar = {m.width}x{m.height}")
print(f"fx = {fx:.1f}  fy = {fy:.1f}   (oczekiwane ~813)")
print(f"HFOV = {math.degrees(2*math.atan(m.width /(2*fx))):.1f}")
print(f"VFOV = {math.degrees(2*math.atan(m.height/(2*fy))):.1f}   (oczekiwane 64.4)")
EOF
```

`fx` musi rownac sie `fy`, HFOV musi rownac sie VFOV. Jesli sie roznia —
sterownik nie uwzglednil cropa w kalibracji i ta liczba jest falszywa;
wtedy wierz testowi z okregiem, nie temu skryptowi.

### 4. Kiedy przejsc do wariantu 2

Ktorekolwiek z:
- czlowiek z 50 m ma mniej niz ~15 px (zmierz na zapisanej klatce, nie na oko)
- detekcja lapie cel dopiero ponizej ~35 m wysokosci
- test z okregiem pokazal rozciagniecie i przelaczenie flagi nie pomoglo

Jesli problemem jest **rozmycie**, a nie rozmiar celu — patrz ostrzezenie
o ekspozycji nizej, wieksza rozdzielczosc tego nie naprawi.

---

## Wariant 2 — natywny crop 1024x1024 z 12 MP

3x wieksza rozdzielczosc katowa (24 px zamiast 8 px na czlowieku z 50 m).
Koszt: FOV 64 -> 24 st., slad 63 -> 21 m.

### 1. Kamera

**Nie przez `depthai_ros_driver`** — nie ma tam pewnego sposobu na natywny crop.
Uzyj `src/drone_camera/drone_camera/oak_publisher.py`, ktory juz istnieje.
W depthai `setVideoSize()` mniejsze niz wyjscie ISP robi **centralny crop bez
skalowania**. Teraz jest bezuzyteczne, bo przy `THE_1080_P` crop 1920x1080
z 1920x1080 jest zerowy.

W `_build_pipeline()` (`oak_publisher.py:56-69`):

```python
cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_12_MP)  # ISP 4056x3040
cam.setVideoSize(1024, 1024)   # centralny crop NATYWNYCH pikseli, bez skalowania
cam.initialControl.setManualExposure(3000, 400)   # us, ISO — patrz ostrzezenie nizej
```

Sprawdz, czy parametry ROS `width`/`height` (`oak_publisher.py:35-36`) nie
nadpisuja tego z powrotem na 1920x1080.

### 2. `vfov_deg` 64.4 -> 24.0 — obowiazkowo

`2*atan(1024/(2*2414))` = 24,0 st. Zostawienie 64.4 da korekte pitcha
**2,7x za duza** i gimbal wpadnie w oscylacje.

Do poprawienia w czterech miejscach:
`suas_flight_controller.launch.py`, `suas_gimbal_controller.launch.py`
oraz domyslne w `suas_flight_controller.py` i `suas_gimbal_controller.py`.

`img_w`/`img_h` zostaja 1024/1024. Rozwaz zmniejszenie `damping` (0.4 gimbal /
0.6 flight controller) — przy waskim FOV ten sam blad wzgledny to mniejszy blad
katowy, wiec regulator jest efektywnie ostrzejszy.

### 3. CameraInfo

`oak_publisher.py:100-105` publikuje tylko `width`/`height`, macierz `K` jest
pusta — skrypt FOV wyzej podzieli przez zero. Wpisz `fx = fy = 2414`, `cx = cy = 512`
albo wypelnij z `device.readCalibration().getCameraIntrinsics(...)`.

### 4. Misja

Slad 21 m zamiast 63 m -> odstep galsow ~16 m zamiast ~47 m. Na 100 m szerokosci
to ~7 przelotow zamiast 2. Bez zageszczenia trajektorii przeleci sie obok celu.

---

## Trzy ostrzezenia

**Rozmycie ruchem** — najwieksze ryzyko wariantu 2. GSD 2 cm/px, przy 10 m/s:
1/60 s = 8 px smugi (czlowiek ma 24 px, rozmazany calkowicie), 1/200 s = 2,4 px,
1/500 s = 1 px. Auto-exposure pod chmurami zejdzie do 1/60. **Wymus manualna
ekspozycje**, inaczej caly zysk rozdzielczosci przepada.

**Model** — `MODEL4.engine` byl trenowany na namioty. Sprawdz, czy w ogole lapie
ludzi, **zanim** zaczniesz stroic kamere. Zaden zabieg optyczny tego nie zastapi.

**FPS** — tryb 12 MP zwykle nie wyrabia 15 fps, spodziewaj sie ~10.
Zmierz `ros2 topic hz /oak/rgb/image_raw` przed strojeniem `control_rate`.

Po kazdej zmianie: `colcon build` + `source install/setup.bash` (configi ida
przez `share/`, sam edit w `src/` nie wystarczy).
