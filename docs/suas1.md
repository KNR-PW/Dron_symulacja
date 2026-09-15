```bash
ssh jet
```

## 1. Terminal 1 — caly stack

```bash
ros2 launch drone_bringup suas_bringup.launch.py alt_offset:=6.0

```

## 2. Terminal 2 — misja

```bash
ros2 run drone_autonomy suas_mission --ros-args \
  --params-file ~/Dron_symulacja/src/drone_bringup/config/misja1.yaml
```

Podglad
http://100.84.102.43:5000/

