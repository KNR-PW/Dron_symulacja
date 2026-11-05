```bash
ssh jet
```

## 1. Terminal 1 — caly stack

```bash
ros2 launch drone_bringup suas_bringup.launch.py

```

## 2. Terminal 2 — misja

```bash
ros2 run drone_autonomy suas_mission --ros-args \
  --params-file ~/Dron_symulacja/src/drone_bringup/config/misja1.yaml
```

Podglady:
 http://100.84.102.43:5000/



Nowe okno: `Ctrl+B` `c`. Odlaczenie: `Ctrl+B` `d`. Powrot: `tmux attach -t suas`.

```bash
colcon build --symlink-install --packages-select \
  drone_interfaces drone_camera drone_detector drone_hardware \
  drone_bringup drone_autonomy
source install/setup.bash
```

```bash

rm -rf build/drone_bringup install/drone_bringup
```

ros2 launch drone_bringup suas_bringup.launch.py debug_jpeg_quality:=30 preview_max_fps:=10