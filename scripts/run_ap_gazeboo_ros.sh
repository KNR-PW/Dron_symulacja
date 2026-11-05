#!/bin/bash
# Buduje workspace i uruchamia Gazebo + drone_handler.
#
# Uzycie:
#   ./run_ap_gazeboo_ros.sh                  -> swiat domyslny (aruco_plain.sdf)
#   ./run_ap_gazeboo_ros.sh suas_field.sdf   -> swiat SUAS (namiot + czlowiek)
#
# WAZNE: uruchamiaj z katalogu scripts/ (skrypt wola ./dockerise_cmd.sh).
# Gazebo musi wstac PRZED SITL - to SITL laczy sie do wtyczki ArduPilota,
# nie odwrotnie.
WORLD="${1:-aruco_plain.sdf}"
./dockerise_cmd.sh "cd ~/ros_ws && colcon build --packages-skip microxrcedds_agent px4_msgs px4_ros_com && source install/setup.bash && ros2 launch drone_bringup gazeboo_ap_sim.launch.py world:=${WORLD}"
