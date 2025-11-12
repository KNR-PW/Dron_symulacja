### 1.Reattach to a running container (no need to create new every time):
```bash
docker start knr_drone

cd ~/Dron_symulacja/docker
./run_ardupilot_sitl.sh
```


### 2. builduje cały projekt, oraz uruchamia ona webootsa i **drone_simulation.launch.py**)
```bash
cd ~/Dron_symulacja/docker
./build_and_run.sh
```

### 3. Uruchomienie Autonomicznej Misji

  ```bash
  cd ~/Dron_symulacja/docker
  ./run_test_mission.sh
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
####  Run Webots Simulation and ROS2 Packages
Inside the container:
```bash
docker attach knr_drone

source ~/ros_ws/install/setup.bash 
ros2 launch drone_bringup drone_simulation.launch.py
```
#### ROS2 Workspace and Build Process

```bash
cd ~/ros_ws
colcon build
source install/setup.bash
```

```bash
cd /home/pawel/Dron_symulacja/docker
sudo ./dockerise_cmd.sh "source ~/ros_ws/install/setup.bash && python3 ~/ros_ws/src/drone_autonomy/drone_autonomy/mission.py"
```
lub
```bash
docker exec -it knr_drone bash

source ~/ros_ws/install/setup.bash &&
python3 ~/ros_ws/src/drone_autonomy/drone_autonomy/mission.py
```
