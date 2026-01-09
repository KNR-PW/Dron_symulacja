# KNR Drone Simulation - README

## Installation Instructions

### 1. Install Docker
Ensure you have Docker installed on your system. You can follow the official installation guide:  
[Docker Installation Guide](https://docs.docker.com/get-docker/)

### 2. Install NVIDIA Container Toolkit
For systems with NVIDIA GPUs, install the NVIDIA Container Toolkit ([official installation guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)):
```bash
distribution=$(. /etc/os-release;echo $ID$VERSION_ID) curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg

curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

sudo apt update
sudo apt install -y nvidia-container-toolkit
sudo systemctl restart docker
```

### 3. Pull the KNR Drone Simulation Docker Image
Download the prebuilt Docker image from Docker Hub:
```bash
docker image pull dierust/knr_ap_sim:latest
```

### 4. Start the Docker Container
First you should source your graphic session to docker:
```bash
echo "xhost +local:docker" >> ~/.bashrc
```

Navigate to the `docker/docker_ap` directory and execute the setup script:  
If your host has nvidia gpu:
```bash
cd docker/docker_ap
./setup_container_gpu.sh dierust/knr_ap_sim:latest
```
If your host doesnt have nvidia gpu:
```bash
cd docker/docker_ap
./setup_container_cpu.sh dierust/knr_ap_sim:latest
```
Once executed, you will be attached to the newly created Docker container.

> **Note:**  
> In some cases, the provided setup scripts may not work correctly and the container might not mount the `src` directory as expected (for example, you see an empty container after running `ls`).  
> If this happens, you can start the container manually with the following command (replace `/absolute/path/to/Dron_symulacja/src` with the full path to your `src` directory on your machine):
> ```bash
> docker run --gpus all -it   --name knr_drone   -v /absolute/path/to/Dron_symulacja/src:/root/ros_ws/src   -e DISPLAY=$DISPLAY   -v /tmp/.X11-unix:/tmp/.X11-unix   --device /dev/dri   dierust/knr_ap_sim:latest

> ```
> For systems without NVIDIA GPU, remove the `--gpus all` flag:
> ```bash
> docker run -it   --name knr_drone   -v /absolute/path/to/Dron_symulacja/src:/root/ros_ws/src   -e DISPLAY=$DISPLAY   -v /tmp/.X11-unix:/tmp/.X11-unix   --device /dev/dri   dierust/knr_ap_sim:latest
> ```

### 5. ROS2 Workspace and Build Process
Inside the container:
```bash
cd ~/ros_ws
colcon build
source install/setup.bash
```

### 6. Run Webots Simulation and ROS2 Packages
To launch the Webots simulator along with required ROS2 packages:
```bash
ros2 launch drone_bringup drone_simulation.launch.py
```

### 7. Run ArduPilot SITL (Software-in-the-Loop)
To start the ArduPilot SITL environment, open new terminal and run the following command **outside** the Docker container:
```bash
./run_ardupilot_sitl.sh
```
First run can take a while since ardupilot will build required components.

### 8. Drone control

You have two options to control the drone:

- **Option A – custom mission**  
  Write your own `test_mission` node/script and run it using:
  ```bash
  ./run_test_mission.sh

### 9. Managing the Docker Container
#### Detach from the container (keep it running):
Press `Ctrl + P` then `Ctrl + Q`

#### Reattach to a running container (no need to create new every time):
```bash
docker attach knr_drone
```

#### Start a stopped container:
```bash
docker start knr_drone
```

#### Stop the container:
```bash
docker stop knr_drone
```
### 10. Important Notes
- The `src` directory is shared between the host and the Docker container. In the container, it is located at `~/ros_ws/src`.
- Your development code should be placed inside this directory to be visible in the container.
- After stopping the container, its memory persists. However, if the container is removed, all data outside the shared `src` directory will be lost.
- **Keep everything in the `src` directory** to prevent data loss.
- **If you encounter any issues, contact: piotrek2995 / UwURetardOwO / DieRust on Discord.**

