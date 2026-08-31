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
```

---

## 5. suas_detect_jetson — kamera + YOLO

Uruchamia kamere OAK, detekcje i serwer podgladu.

- kamera OAK
- detekcja YOLO
- podglad www

```bash
ros2 launch drone_bringup suas_detect_jetson.launch.py
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

