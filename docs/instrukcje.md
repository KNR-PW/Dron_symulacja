
####  1. Run Webots Simulation and ROS2 Packages

```bash
docker start knr_drone
docker attach knr_drone
```

Inside the container:
```bash
source ~/ros_ws/install/setup.bash 

ros2 launch drone_bringup drone_simulation.launch.py 

ros2 launch drone_bringup drone_simulation.launch.py world:=aruco_tests_simple.wbt
``` 
#### 2. Run Ardupilot SITL

```bash
cd ~/Dron_symulacja/docker
./run_ardupilot_sitl.sh
```


#### 3. Run Autonomic Mission

  ```bash
  cd ~/Dron_symulacja/docker
  ./run_test_mission.sh
  ```

#### 4. Build and Run

```bash
cd ~/Dron_symulacja/docker
./build_and_run.sh
```

### Reszta

#### Wejdź do kontenera
```bash
docker exec -it knr_drone bash
```
#### Stop the container:
```bash
0027951936
docker stop knr_drone
```

#### ROS2 Workspace and Build Process

```bash
cd ~/ros_ws
colcon build
source install/setup.bash
```

```bash
cd /home/pawel/Dron_symulacja/docker
sudo ./dockerise_cmd.sh "source ~/ros_ws/install/setup.bash && python3 ~/ros_ws/src/drone_autonomy/drone_autonomy/run_test_mission.py"

sudo ./dockerise_cmd.sh "source ~/ros_ws/install/setup.bash && python3 ~/ros_ws/src/drone_autonomy/drone_autonomy/follow_aruco_centroid_copy.py"


```

lub
```bash
docker exec -it knr_drone bash
cd ~/ros_ws
source ~/ros_ws/install/setup.bash &&
python3 ~/ros_ws/src/drone_autonomy/drone_autonomy/mission.py
```
