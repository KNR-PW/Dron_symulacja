# START — jak to uruchomic

Sciagawka. Kazdy krok = osobny terminal, chyba ze napisano inaczej.
Kolejnosc ma znaczenie: najpierw 1-3, potem to, czego akurat potrzebujesz.

---

## 1. Polaczenie z Jetsonem

```bash
ssh jetsonknr@100.84.102.43
```

---

## 2. Source — w KAZDYM nowym terminalu

```bash
source ~/Dron_symulacja/install/setup.bash
```

---

## 3. Build 

```bash
cd ~/Dron_symulacja
git pull
colcon build --packages-select drone_interfaces drone_camera drone_detector drone_bringup drone_autonomy
source install/setup.bash
```

---

## 4. Drone handler 

```bash
ros2 run drone_hardware drone_handler
```

Czekaj na `Copter connected, ready to arm`.

Testy na biurku, bez GPS i pre-arm checkow:

```bash
ros2 run drone_hardware drone_handler --ros-args -p dev:=true
ros2 run drone_hardware drone_handler --ros-args -p dev:="'true'"

```

---

## 5. suas_detect_jetson — kamera + YOLO

Uruchamia kamere OAK, detekcje i serwer podgladu.

- kamera OAK
- detekcja YOLO
- podglad www

```bash
ros2 launch drone_bringup suas_detect_jetson.launch.py debug_every_n:=2
```

Detektor jest **dwuklasowy** i przy jednej inferencji publikuje trzy rzeczy:

| topic | co niesie |
|---|---|
| `/detections` | komplet boxow z klatki, obie klasy, z `header.stamp` klatki |
| `/tent_detections` | najpewniejszy **namiot**, KAZDA klatke (takze pusty) |
| `/people_detections` | najpewniejszy **czlowiek**, KAZDA klatke (takze pusty) |

Najpewniejszy box wybierany jest **w obrebie klasy**, wiec namiot nie zaslania juz
czlowieka. `classes:=0` / `classes:=1` zostaje jako filtr awaryjny (gdyby ktoras
klasa sypala smieciami) — do normalnej pracy nie jest potrzebny.

Szybkie sprawdzenie:

```bash
ros2 topic echo /detections --once
ros2 topic hz /tent_detections /people_detections
```



## 6. suas_gimbal_controller — sam gimbal, bez lotu

Test sledzenia celu samym pitchem gimbala. Dron stoi.

- sledzi cel
- rusza gimbalem

```bash
ros2 launch drone_bringup suas_gimbal_controller.launch.py
```

Wymaga: drone_handler (4) + detekcji (5).

Reczne ustawienie gimbala, bez zadnego kontrolera (stopnie, -90 = prosto w dol,
zakres do ok. -18):

```bash
ros2 topic pub --once /knr_hardware/gimbal_pitch std_msgs/msg/Float32 "{data: -90.0}"
```
reczne ustawienie dropera 
rzut namiotu (1602):
  ros2 service call /knr_hardware/set_servo drone_interfaces/srv/SetServo "{servo_id: 13,
  pwm: 1602}"

  Zrzut człowieka (988):
  ros2 service call /knr_hardware/set_servo drone_interfaces/srv/SetServo "{servo_id: 13,
  pwm: 988}"

  Neutral / spoczynek (1327):
  ros2 service call /knr_hardware/set_servo drone_interfaces/srv/SetServo "{servo_id: 13,
  pwm: 1327}"

---


## 7. suas_simple_mission — pelna uproszczona misja

- arm i wzlot
- podlot nad cel
- powrot RTL

```bash
ros2 launch drone_bringup suas_simple_mission.launch.py
```

Wymaga: drone_handler (4) + detekcji (5).

W symulacji zamiast kroku 5 idzie
`ros2 launch drone_bringup suas_detect_gazebo.launch.py` — ma ten sam zestaw
argumentow (`conf`, `classes`, `camera_topic`, `debug_every_n`,
`debug_jpeg_quality`), wiec komendy sa przenoszalne miedzy symulacja a realem.

W stanie SEARCH dron NIE przeszukuje terenu — wisi i czeka, az detektor
zobaczy cel. Namiot musi byc w kadrze.

Log CSV: `~/suas_simple_mission_log.csv`
---

## 8. Podglad w przegladarce

Dziala z dowolnego urzadzenia w Tailscale, bez ROS-a po stronie klienta.
Serwer startuje razem z krokiem 5.

**Detekcja z ramkami:**

http://100.84.102.43:8080/stream_viewer?topic=/tent_detections/image&type=ros_compressed

**Surowy obraz z kamery:**

http://100.84.102.43:8080/stream_viewer?topic=/oak/rgb/preview/image_raw&type=ros_compressed

**Lista wszystkich topicow z obrazem:**

http://100.84.102.43:8080/

---

## 9. Zapis materialu

Kamera musi juz chodzic (krok 5). Szczegoly w osobnych plikach:

**Zdjecia** — [oak_zdjecia.md](oak_zdjecia.md)

```bash
ros2 launch drone_bringup oak_photos.launch.py
```
Zapis do `~/oak_photos/1`, `2`, `3`... Nowy podkatalog przy kazdym starcie.

**Wideo** — [oak_wideo.md](oak_wideo.md)

```bash
ros2 launch drone_bringup oak_video.launch.py
```
Nagrywa od razu, `Ctrl+C` konczy i domyka plik `.mp4` w `~/oak_video/1`, `2`...

---

## 10. suas_bringup — caly stack oprocz misji, jeden terminal

handler + kamera + YOLO + geolokator + marker naraz (zastepuje kroki 4 i 5).

```bash
ros2 launch drone_bringup suas_bringup.launch.py
```

Przydatne parametry (`nazwa:=wartosc`):

| parametr | domyslnie | co robi |
|---|---|---|
| `debug_every_n` | `1` | podglad co N-ta klatka (`2` lzej na LTE) |
| `preview_max_fps` | `4.0` | limit FPS podgladu markera (:5000); wyzej = plynniej |
| `conf` | `0.35` | prog pewnosci detekcji |
| `fc_ip` | `/dev/ttyACM0` | port Orange Cube |
| `lock_nadir` | `true` | geolokator trzyma gimbal w pionie |

GUI oznaczania celow: http://100.84.102.43:5000/

```bash
ros2 launch drone_bringup suas_bringup.launch.py debug_every_n:=2
```

### Jakie pliki odpala i gdzie ich szukac

`suas_bringup` sam nic nie liczy — wlacza inne launche i node'y. Z lewej co sie
uruchamia, z prawej gdzie lezy plik:

```
suas_bringup.launch.py               src/drone_bringup/launch/
├─ drone_handler                     src/drone_hardware/drone_hardware/drone_handler.py
├─ suas_detect_jetson.launch.py      src/drone_bringup/launch/
│  ├─ kamera OAK (depthai) + config  src/drone_bringup/config/oak_rgb.yaml
│  ├─ yolo_detector_jetson           src/drone_detector/drone_detector/yolo_detector_jetson.py
│  │     (wspolna baza)              src/drone_detector/drone_detector/yolo_detector_base.py
│  ├─ web_video_server (:8080)       pakiet zewnetrzny (ROS)
│  └─ suas_marker_web (:5000)        src/drone_autonomy/drone_autonomy/suas_marker_web.py
└─ suas_geolocator.launch.py         src/drone_bringup/launch/
   └─ suas_geolocator                src/drone_autonomy/drone_autonomy/suas_geolocator.py
```

Parametry:
- dopisz `nazwa:=wartosc` do komendy (doraznie)
- albo zmien `default_value` w danym `.launch.py` 
Config kamery (rozdzielczosc, FPS, FOV) — `oak_rgb.yaml`.

**Ostrzejszy / plynniejszy podglad**, gdy lacze nie laguje:

```bash
ros2 launch drone_bringup suas_bringup.launch.py debug_jpeg_quality:=60 debug_every_n:=1 preview_max_fps:=10
```
- `debug_jpeg_quality` 20 -> 50-80 = ostrzejszy obraz
- `debug_every_n:=1` = kazda klatka detektora (a nie co 2-3)
- `preview_max_fps` 4 -> 10 (albo `0` = bez limitu) = plynniejszy podglad markera

Sama detekcja jest ograniczona modelem (~12 FPS na Jetsonie), tego nie podniesiesz
parametrem. FPS kamery ustawia sie w `config/oak_rgb.yaml` (`i_fps`).

---

## 11. suas_full_mission — pelna misja (dwa cele, dwa zrzuty)

Wymaga stacku z kroku 10.


```bash
ros2 run drone_autonomy suas_full_mission --ros-args \
  --params-file ~/Dron_symulacja/src/drone_bringup/config/suas_mission.yaml
```

**Test bez trasy AUTO** (sam uzbraja i wznosi sie na target_alt):

```bash
ros2 run drone_autonomy suas_full_mission --ros-args \
  --params-file ~/Dron_symulacja/src/drone_bringup/config/suas_mission.yaml \
  -p auto_takeoff:=true
```

Przydatne parametry (`-p nazwa:=wartosc`):

| parametr | domyslnie | co robi |
|---|---|---|
| `auto_takeoff` | `false` | `true` = sam start zamiast czekania na GUIDED (test) |
| `target_alt` | `50.0` | wysokosc lotu/zrzutu [m] |
| `finish_action` | `rtl` | `rtl` / `land` na koniec |
| `targets_json` | `~/suas_targets/targets.json` | plik z celami |

