#!/bin/bash

# Skrypt uruchamiający symulację PX4 VTOL w Gazebo
# Uruchamia: PX4 SITL, MicroXRCEAgent, drone_handler_px4, QGroundControl

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WORKSPACE_DIR="$( cd "$DIR/../.." && pwd )"
CONFIG_FILE="/tmp/terminator_px4_config"

echo "=== Symulacja PX4 VTOL ==="

cat > "$CONFIG_FILE" << EOF
[global_config]
  suppress_multiple_term_dialog = True
[keybindings]
[profiles]
  [[default]]
    exit_action = hold
    scrollback_infinite = True
[layouts]
  [[px4_sim]]
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
    [[[terminal_px4_sitl]]]
      type = Terminal
      parent = child1
      order = 0
      profile = default
      command = bash -c "cd $WORKSPACE_DIR/docker && echo '--- Panel 1: PX4 SITL ---' && docker exec -it knr_drone_px4 bash -c 'pkill -9 px4 2>/dev/null; pkill -9 gz 2>/dev/null; pkill -9 ruby 2>/dev/null; sleep 2; source /opt/ros/jazzy/setup.bash && cd /tools/PX4-Autopilot && make px4_sitl gz_tiltrotor_aruco'; docker exec -it knr_drone_px4 bash"
    [[[child2]]]
      type = VPaned
      parent = child1
      order = 1
      position = 250
      ratio = 0.3333
    [[[terminal_microxrce]]]
      type = Terminal
      parent = child2
      order = 0
      profile = default
      command = bash -c "cd $WORKSPACE_DIR/docker && echo '--- Panel 2: MicroXRCEAgent ---' && echo 'Czekam 15s na PX4 SITL...' && sleep 15 && docker exec -it knr_drone_px4 bash -c 'pkill -9 MicroXRCEAgent 2>/dev/null; sleep 1; source /opt/ros/jazzy/setup.bash && source ~/ros_ws/install/setup.bash && MicroXRCEAgent udp4 -p 8888'; docker exec -it knr_drone_px4 bash"
    [[[child3]]]
      type = VPaned
      parent = child2
      order = 1
      position = 250
      ratio = 0.5
    [[[terminal_ros2]]]
      type = Terminal
      parent = child3
      order = 0
      profile = default
      command = bash -c "cd $WORKSPACE_DIR/docker && echo '--- Panel 3: ROS2 drone_handler_px4 ---' && echo 'Czekam 25s na MicroXRCE...' && sleep 25 && docker exec -it knr_drone_px4 bash -c 'source /opt/ros/jazzy/setup.bash && cd ~/ros_ws && source install/setup.bash && ros2 run drone_hardware drone_handler_px4'; docker exec -it knr_drone_px4 bash"
    [[[terminal_shell]]]
      type = Terminal
      parent = child3
      order = 1
      profile = default
      command = bash -c "cd $WORKSPACE_DIR/docker && echo '--- Panel 4: Shell ---' && sleep 5 && docker exec -it knr_drone_px4 bash -c 'cd ~/ros_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash && exec bash'"
EOF

echo "Uruchamianie kontenera knr_drone_px4..."
docker start knr_drone_px4

echo "Uruchamianie QGroundControl..."
"$DIR/QGroundControl-x86_64.AppImage" &

echo "Uruchamianie Terminatora..."
terminator -u -g "$CONFIG_FILE" -l px4_sim
