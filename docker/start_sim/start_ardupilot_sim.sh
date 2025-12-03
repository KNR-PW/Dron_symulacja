#!/bin/bash

# Skrypt uruchamiający symulację ArduPilot w Webots
# Uruchamia: ROS2 Simulation, ArduPilot SITL

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WORKSPACE_DIR="$( cd "$DIR/../.." && pwd )"
CONFIG_FILE="/tmp/terminator_ardupilot_config"

echo "=== Symulacja ArduPilot ==="

cat > "$CONFIG_FILE" << EOF
[global_config]
  suppress_multiple_term_dialog = True
[keybindings]
[profiles]
  [[default]]
    exit_action = hold
    scrollback_infinite = True
[layouts]
  [[ardupilot_sim]]
    [[[child0]]]
      type = Window
      parent = ""
      order = 0
      position = 0:0
      maximised = False
      fullscreen = False
      size = 1800, 1000
    [[[child1]]]
      type = VPaned
      parent = child0
      order = 0
      position = 250
      ratio = 0.25
    [[[terminal_ros2]]]
      type = Terminal
      parent = child1
      order = 0
      profile = default
      command = bash -c "cd $WORKSPACE_DIR/docker && echo '--- Panel 1: ROS2 Simulation ---' && docker exec -it knr_drone bash -c 'source ~/ros_ws/install/setup.bash && ros2 launch drone_bringup drone_simulation.launch.py'; docker exec -it knr_drone bash"
    [[[child2]]]
      type = VPaned
      parent = child1
      order = 1
      position = 250
      ratio = 0.333
    [[[terminal_ardupilot]]]
      type = Terminal
      parent = child2
      order = 0
      profile = default
      command = bash -c "cd $WORKSPACE_DIR/docker && echo '--- Panel 2: ArduPilot SITL ---' && xhost +local:docker && docker exec -it knr_drone bash -c 'git config --global --add safe.directory /tools/ardupilot && /tools/ardupilot/Tools/autotest/sim_vehicle.py -v ArduCopter -w --model webots-python --add-param-file=/root/ros_ws/src/webots_simulation/SITL_param/iris.parm --out=udp:172.17.0.1:14550'; docker exec -it knr_drone bash"
    [[[child3]]]
      type = VPaned
      parent = child2
      order = 1
      position = 250
      ratio = 0.5
    [[[terminal_docker]]]
      type = Terminal
      parent = child3
      order = 0
      profile = default
      command = bash -c "cd $WORKSPACE_DIR/docker && echo '--- Panel 3: Docker Shell ---' && sleep 1 && docker exec -it knr_drone bash -c 'cd ~/ros_ws && source install/setup.bash && exec bash'"
    [[[terminal_shell]]]
      type = Terminal
      parent = child3
      order = 1
      profile = default
      command = bash -c "cd $WORKSPACE_DIR/docker && echo '--- Panel 4: Shell ---' && sleep 1 && docker exec -it knr_drone bash -c 'cd ~/ros_ws && source install/setup.bash && exec bash'"
EOF

echo "Uruchamianie kontenera knr_drone..."
docker start knr_drone

echo "Uruchamianie Terminatora..."
terminator -u -g "$CONFIG_FILE" -l ardupilot_sim
