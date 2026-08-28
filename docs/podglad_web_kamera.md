# Podglad kamery przez przegladarke (web_video_server + Tailscale)

Obraz z dowolnego topicu ROS ogladany na telefonie/laptopie przez przegladarke,

link:
```
http://100.84.102.43:8080/stream_viewer?topic=/tent_detections/image&type=ros_compressed
```
> detektor publikuje podglad **tylko gdy ktos go oglada**
> Zamkniecie karty w przegladarce = zerowy narzut. Nie musisz nic wylaczac.
---

## 1. Instalacja (raz, na Jetsonie)

```bash
sudo apt update
sudo apt install ros-humble-web-video-server
```

---

## 2. Uruchomienie

Na Jetsonie uruchamiane juz w `ros2 launch drone_bringup suas_detect_jetson.launch.py` 

lub możliwe uruchomienie w osobnym terminalu:
```bash
source /opt/ros/humble/setup.bash
source ~/Dron_symulacja/install/setup.bash
ros2 run web_video_server web_video_server --ros-args \
  -p port:=8080 \
  -p address:=0.0.0.0
```

Zostaw terminal otwarty. `Ctrl+C` konczy serwer.

---

## 3. Podglad 

Wklej w przegladarke na dowolnym urzadzeniu w tailnecie.

**Detekcja  :**
```
http://100.84.102.43:8080/stream_viewer?topic=/tent_detections/image&type=ros_compressed
```

**Kamera OAK (surowy obraz):**
```
http://100.84.102.43:8080/stream_viewer?topic=/oak/rgb/image_raw&type=ros_compressed
```

**Lista wszystkich topicow**

```
http://100.84.102.43:8080/
```
- wybierz `stream_viewer` , `ros_compressed`

---





## 4. Gdy podglad muli (Tailscale, brak WiFi)

W terenie ruch idzie przez Tailscale po LTE, czesto przez przekaznik DERP —
opoznienia sa nie do uniknięcia. Jedyne, co realnie pomaga, to **mniej bajtow
na sekunde**. Ponizsze kroki psuja TYLKO podglad; obraz, ktory dostaje detektor,
i parametry kamery zostaja nietkniete.

### a) Zbij jakosc JPEG podgladu z OAK (najwiekszy zysk, zero kosztu CPU)

Kompresja i tak zachodzi, domyslnie z jakoscia 80. Zejscie na 25 tnie rozmiar
klatki kilkukrotnie:

```bash
ros2 param list /oak | grep -i jpeg      # znajdz dokladna nazwe parametru
ros2 param set /oak <nazwa> 25
```

Dziala od reki, bez restartu. Nie zmienia obrazu dla YOLO — detektor czyta
surowy topic, nie skompresowany.

### b) Ogranicz liczbe klatek podgladu

Przepuszczaj co N-ta klatke do osobnego topicu i ogladaj wlasnie ten:

```bash
sudo apt install ros-humble-topic-tools     # raz

ros2 run topic_tools throttle messages \
  /oak/rgb/image_raw/compressed 5.0 /oak/preview/compressed
```

Potem w przegladarce (topic bazowy `/oak/preview`, bez `/compressed`):
```
http://100.84.102.43:8080/stream_viewer?topic=/oak/preview&type=ros_compressed
```

`5.0` = 5 klatek na sekunde. Mniej = plynniej przy slabym laczu.

### c) Ostatecznosc: przeskaluj w locie

```
http://100.84.102.43:8080/stream_viewer?topic=/oak/rgb/image_raw&type=mjpeg&quality=15&width=480
```

Dziala tylko z `mjpeg`, wiec Jetson przekodowuje kazda klatke — CPU zabrane
detekcji. Uzywaj, gdy a) i b) nie wystarczyly.

### Uwagi

- Kazda otwarta karta w przegladarce to osobny strumien. Zamykaj niepotrzebne.
- Do pilotowania i tak lepszy jest `/tent_detections/image` — jest juz zmniejszony
  do 960 px i skompresowany z jakoscia 20, a dodatkowo ma ramki detekcji.
- Sprawdzenie, czy Tailscale idzie bezposrednio czy przez przekaznik:
  `tailscale ping jetsonknr-desktop` — linia z `via DERP` oznacza dluzsza droge.
