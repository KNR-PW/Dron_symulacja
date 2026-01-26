# KNR Drone Simulation - AI Coding Instructions

## Project Overview
This is a ROS 2 (Humble) based drone simulation and control framework using ArduPilot SITL and Webots. The entire environment is containerized using Docker.

## Architecture
- **Core Abstraction**: `DroneController` (`src/drone_autonomy/drone_autonomy/drone_comunication/drone_controller.py`) is the base class for all missions. It handles services, actions, and topics for drone control.
- **Packages**:
  - `drone_autonomy`: High-level mission logic (Python nodes).
  - `drone_bringup`: Launch files (`drone_simulation.launch.py`, `drone_dev.launch.py`).
  - `drone_hardware`: MavLink communication with Flight Controller (`drone_handler.py`).
  - `drone_interfaces`: Custom ROS 2 messages, services, and actions.
  - `webots_simulation`: Webots worlds and SITL parameters.

## Development Workflow
**CRITICAL**: All ROS 2 commands must run inside the `knr_drone` Docker container.

### 1. Environment Management
- **Host Scripts** (in `docker/`):
  - `./run_ardupilot_sitl.sh`: Starts ArduPilot SITL (runs `sim_vehicle.py` inside container).
  - `./build_and_run.sh`: Builds workspace and launches simulation (`drone_simulation.launch.py`).
  - `./run_test_mission.sh`: Runs a specific mission node.
  - `./dockerise_cmd.sh "<command>"`: Helper to execute commands inside the container.

### 2. Build Process
Inside the container (`~/ros_ws`):
```bash
colcon build --symlink-install
source install/setup.bash
```
*Note: `build_and_run.sh` often skips `microxrcedds_agent`, `px4_msgs`, `px4_ros_com` to save time.*

### 3. Creating Missions
- Create new Python files in `src/drone_autonomy/drone_autonomy/`.
- Inherit from `DroneController`.
- Example pattern:
  ```python
  from drone_autonomy.drone_comunication.drone_controller import DroneController

  class MyMission(DroneController):
      def __init__(self):
          super().__init__()
          # Your initialization
  ```

## Key Conventions
- **Path Handling**: Use absolute paths mapped to `/root/ros_ws/src` inside the container.
- **Launch Files**: Located in `src/drone_bringup/launch/`.
- **Parameters**: Use ROS 2 parameters for configuration (e.g., `image_width`, `target_alt`).
- **Communication**:
  - **MavLink**: Handled by `drone_hardware`.
  - **Vision**: `drone_camera` publishes images; `drone_detector` publishes detections.

## Debugging
- **Logs**: Check ROS 2 logs inside the container.
- **SITL Console**: The terminal running `./run_ardupilot_sitl.sh` provides the ArduPilot console (e.g., `arm throttle`, `mode guided`).
- **Webots**: Visual feedback for simulation.
