
## 0. 

```bash
ssh jetsonknr@100.84.102.43
tmux new -s suas
```

## 1. Terminal 1 — caly stack

```bash
source ~/Dron_symulacja/install/setup.bash
ros2 launch drone_bringup suas_bringup.launch.py
ros2 launch drone_bringup suas_bringup.launch.py debug_jpeg_quality:=30 preview_max_fps:=10

```

Podglady:
- obraz z ramkami — http://100.84.102.43:8080/
- **GUI oznaczania — http://100.84.102.43:5000/** 

---

## 2. Terminal 2 — misja

```bash
source ~/Dron_symulacja/install/setup.bash
ros2 run drone_autonomy suas_mission --ros-args \
  --params-file ~/Dron_symulacja/src/drone_bringup/config/misja.yaml
```



Nowe okno: `Ctrl+B` `c`. Odlaczenie: `Ctrl+B` `d`. Powrot: `tmux attach -t suas`.

```bash
cd ~/Dron_symulacja
git pull
colcon build --symlink-install --packages-select \
  drone_interfaces drone_camera drone_detector drone_hardware \
  drone_bringup drone_autonomy
source install/setup.bash
```

```bash

rm -rf build/drone_bringup install/drone_bringup
```

colcon build --symlink-install --packages-select drone_hardware