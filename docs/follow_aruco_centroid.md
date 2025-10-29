

---

````markdown
#  follow_aruco_centroid

Node odpowiada za **podążanie drona za markerem ArUco**.  
Na podstawie środka markera (`/aruco_markers`) oblicza wektory prędkości i publikuje je na `/velocity_vectors`.  
Używany w **symulacji Webots** oraz **w locie rzeczywistym** z kamerą.

---

##  Parametry

| Parametr        | Opis |
|-----------------|------|
| `image_width`, `image_height` | Rozdzielczość obrazu (px) |
| `aruco_topic`   | Topic z detekcji markera |
| `target_alt`    | Docelowa wysokość lotu (m) |
| `kp`            | wzmocnienie P; prędkość [m/s], im większe tym bardziej drona rwie do celu  |
| `deadband_px`   | Martwa strefa (px) |
| `max_vel`       | Maks. prędkość (m/s) |
| `lowpass`       | Wygładzanie sygnału (0–1) |
| `lost_timeout`  | Czas bez markera zanim dron się zatrzyma (s) |

---

 Uruchomienie (przykładowe, pod webots)

Najpierw odpal symulację 

Następnie w nowym terminalu uruchom node:

```bash
ros2 run drone_autonomy follow_aruco_centroid \
  --ros-args \
  -p image_width:=640 -p image_height:=480 \
  -p aruco_topic:=/aruco_markers \
  -p target_alt:=2.0 \
  -p kp:=0.1 \
  -p deadband_px:=20 \
  -p max_vel:=1.0 \
  -p lowpass:=0.3 \
  -p lost_timeout:=0.8
```



---

node subskrybuje /aruco_markers a publikuje /knr.hardware.velocity_vectors





