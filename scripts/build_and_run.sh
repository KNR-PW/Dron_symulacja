#!/bin/bash

export QT_ENABLE_HIGHDPI_SCALING=0
export QT_SCALE_FACTOR=2
export QT_FONT_DPI=80

if docker ps --filter "status=running" --format '{{.Names}}' | grep -Fxq "knr_drone"; then
    ./dockerise_cmd.sh "export QT_ENABLE_HIGHDPI_SCALING=0 && export QT_SCALE_FACTOR=2 && export QT_FONT_DPI=80 && cd ~/ros_ws && colcon build --packages-skip microxrcedds_agent px4_msgs px4_ros_com && source install/setup.bash && ros2 launch drone_bringup oa_sim.launch.py"
elif docker ps --filter "status=running" --format '{{.Names}}' | grep -Fxq "knr_drone_px4"; then
    ./dockerise_cmd.sh "export QT_ENABLE_HIGHDPI_SCALING=0 && export QT_SCALE_FACTOR=2 && export QT_FONT_DPI=80 && cd ~/Dron_symulacja && colcon build --packages-skip px4 && source install/setup.bash && ros2 launch drone_bringup sim_px4.launch.py"
else
    echo "you should start the docker container"
fi