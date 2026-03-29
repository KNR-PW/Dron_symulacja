# tent_follower — sterowanie dronem nad namiot

## Co robi

Node ROS 2 `tent_follower` steruje dronem wektorowo (velocity control) tak, aby
na podstawie detekcji YOLO namiotu podleciał nad niego i zawisł.

Kamera patrzy prosto w dół (90°). Node oblicza błąd pozycji namiotu
w kadrze i generuje wektory prędkości (vx, vy) + obrót yaw.

## Pliki

| Plik | Opis |
|------|------|
| `src/drone_autonomy/drone_autonomy/tent_follower.py` | Node ROS 2 |
| `src/drone_autonomy/setup.py` | Entry point `tent_follower` |

## Zależności

- **yolo_detector** — musi działać i publikować `/tent_detections`
- **drone_handler_px4** — musi działać (telemetria + velocity control)
- Kamera w modelu SDF musi patrzeć w dół (pitch=1.5708 rad)

## Topici

| Topic | Typ | Kierunek |
|-------|-----|----------|
| `/tent_detections` | `drone_interfaces/TentDetection` | subskrypcja |
| `knr_hardware/telemetry` | `drone_interfaces/Telemetry` | subskrypcja |
| `knr_hardware/velocity_vectors` | `drone_interfaces/VelocityVectors` | publikacja |

## Parametry

| Parametr | Domyślna | Opis |
|----------|----------|------|
| `target_alt` | 20.0 | Wysokość wzlotu [m] |
| `kp_xy` | 1.5 | Wzmocnienie regulatora XY |
| `kp_yaw` | 0.8 | Wzmocnienie regulatora yaw |
| `max_vel` | 2.0 | Maks. prędkość [m/s] |
| `max_yaw_rate` | 0.5 | Maks. prędkość yaw [rad/s] |
| `deadzone` | 0.05 | Strefa martwa (normalizowana) |
| `lost_timeout` | 3.0 | Czas utraty detekcji → hover [s] |
| `img_w` / `img_h` | 1920 / 1080 | Rozdzielczość obrazu kamery |
| `control_rate` | 10.0 | Częstotliwość pętli sterowania [Hz] |

## Build (w kontenerze)

```bash
cd ~/Dron_symulacja
colcon build --packages-select drone_interfaces drone_hardware drone_autonomy
source install/setup.bash
```

## Uruchomienie

### Sposób 1: Jeden launch (zalecany)

Wymaga: PX4 SITL + MicroXRCEAgent już uruchomione (`./scripts/start_sim/start_px4_sim.sh terrain` — bez drone_handler, bo launch go podniesie).

```bash
source /opt/ros/jazzy/setup.bash
source ~/Dron_symulacja/install/setup.bash
ros2 launch drone_bringup tent_follower.launch.py
```

Z parametrami:
```bash
ros2 launch drone_bringup tent_follower.launch.py \
    target_alt:=50.0 kp_xy:=2.0 max_vel:=3.0
```

Launch uruchamia: `gz_bridge` (kamera) + `drone_handler_px4` + `yolo_detector` + `tent_follower`.

### Sposób 2: Osobne terminale

#### 1. Symulacja + drone_handler_px4

```bash
./scripts/start_sim/start_px4_sim.sh terrain
```

#### 2. YOLO detektor (w kontenerze, osobny terminal)

```bash
source /opt/ros/jazzy/setup.bash
source ~/Dron_symulacja/install/setup.bash
ros2 run drone_detector yolo_detector --ros-args \
    -p model_path:=/root/Dron_symulacja/yolo/best.onnx
```

#### 3. tent_follower (w kontenerze, kolejny terminal)

```bash
source /opt/ros/jazzy/setup.bash
source ~/Dron_symulacja/install/setup.bash
ros2 run drone_autonomy tent_follower
```

## Algorytm

1. **Arm → Takeoff → Velocity control**
2. Pętla 10 Hz:
   - Centroid namiotu w pikselach → błąd znormalizowany `[-1, 1]`
   - Regulator P: `vx = kp * ey_norm * max_vel`, `vy = kp * ex_norm * max_vel`
   - Yaw: `yaw_rate = kp_yaw * ex_norm * max_yaw_rate`
   - Deadzone → hover gdy wycentrowany
   - Lost timeout → hover gdy brak detekcji
3. **Ctrl+C** — powrót do position control

## Zmiany w projekcie

- **model.sdf** (knr_tiltrotor): kamera pitch 45° → 90° (prosto w dół)
- **Telemetry.msg**: dodane pola `roll`, `pitch`, `yaw`
- **drone_handler_px4.py**: publikacja roll/pitch/yaw z quaterniona
