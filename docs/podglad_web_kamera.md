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





