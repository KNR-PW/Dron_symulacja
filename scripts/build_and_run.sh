
#!/bin/bash

if docker ps --filter "status=running" --format '{{.Names}}' | grep -Fxq "knr_drone"; then
    ./dockerise_cmd.sh "cd ~/ros_ws && colcon build --packages-skip microxrcedds_agent px4_msgs px4_ros_com && source install/setup.bash && ros2 launch drone_bringup drone_simulation.launch.py"
elif docker ps --filter "status=running" --format '{{.Names}}' | grep -Fxq "knr_drone_px4"; then
    ./dockerise_cmd.sh "cd ~/Dron_symulacja && colcon build --packages-skip px4 && source install/setup.bash && ros2 launch drone_bringup sim_px4.launch.py"
else
    echo "you should start the docker container"
fi
