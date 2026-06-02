# Web Dashboard

Dashboard webowy -- telemetria (WebSocket) + obraz z kamery (WebRTC) na `http://localhost:8080`.

## Uruchamianie

```bash
# Instalacja zaleznosci (raz, w kontenerze knr_drone)
pip install aiohttp aiortc av

# Build
cd ~/ros_ws && colcon build --packages-select drone_interfaces drone_web drone_bringup && source install/setup.bash
```

```bash
# Hardware (OAK-D)
ros2 launch drone_bringup dashboard.launch.py

# Symulacja Webots
ros2 launch drone_bringup dashboard.launch.py camera_topic:=/camera

# Test bez drona (fake telemetria)
ros2 run drone_web ros_test_publisher &
ros2 run drone_web webcam_dashboard
```

## Parametry

| Parametr | Domyslnie | Opis |
|---|---|---|
| `camera_topic` | `/oak/rgb/image_raw` | Topic kamery. Webots: `/camera` |
| `port` | `8080` | Port HTTP |

## Telemetria

Subskrybuje `knr_hardware/telemetry` (`drone_interfaces/Telemetry`) -- ten sam topic co `drone_handler`.

## Pliki

```
src/drone_web/drone_web/webcam_dashboard.py    -- serwer
src/drone_web/drone_web/ros_test_publisher.py  -- fake telemetria
src/drone_web/drone_web/dashboard/             -- HTML/CSS/JS
src/drone_bringup/launch/dashboard.launch.py   -- launch file
```

## Docker

Kontener musi miec `-p 8080:8080` zeby dashboard byl widoczny z hosta.
