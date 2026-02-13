#!/bin/bash

# Skrypt uruchamiający symulację PX4 VTOL w Gazebo
# Uruchamia: PX4 SITL, MicroXRCEAgent, drone_handler_px4, QGroundControl
# Użycie: ./start_px4_sim.sh [nazwa_świata]
# Przykład: ./start_px4_sim.sh terrain

WORLD=${1:-aruco}

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
SCRIPTS_DIR="$( cd "$DIR/.." && pwd )"
WORKSPACE_DIR="$( cd "$DIR/../.." && pwd )"
CONFIG_FILE="/tmp/terminator_px4_config"

echo "=== Symulacja PX4 VTOL ==="
echo "Świat: $WORLD"

# Wyczyść stare procesy (uważaj żeby nie zabić tego skryptu!)
pkill -9 -f gz-sim 2>/dev/null
pkill -9 -x px4 2>/dev/null
pkill -9 ruby 2>/dev/null

# Uruchom kontener przed generowaniem configu
echo "Uruchamianie kontenera knr_drone_px4..."
docker start knr_drone_px4

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
      size = 960, 1050
    [[[child1]]]
      type = VPaned
      parent = child0
      order = 0
      position = 250
      ratio = 0.25
    [[[child_top]]]
      type = HPaned
      parent = child1
      order = 0
      position = 480
      ratio = 0.5
    [[[terminal_px4_sitl]]]
      type = Terminal
      parent = child_top
      order = 0
      profile = default
      command = bash -c "cd $SCRIPTS_DIR && echo '--- Panel 1: PX4 SITL (świat: $WORLD) ---' && ./run_px4_vtol_sitl.sh $WORLD; exec bash"
    [[[terminal_scripts]]]
      type = Terminal
      parent = child_top
      order = 1
      profile = default
      command = bash -c "cd $SCRIPTS_DIR && echo '--- Panel 5: Scripts ---' && exec bash"
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
      command = bash -c "echo '--- Panel 2: MicroXRCEAgent ---' && echo 'Czekam 5s na PX4...' && sleep 5 && docker exec -it knr_drone_px4 bash -c 'pkill -x MicroXRCEAgent || true; sleep 1; source /opt/ros/jazzy/setup.bash && source ~/ros_ws/install/setup.bash && MicroXRCEAgent udp4 -p 8888'; echo 'Zakończone. Wchodzę do dockera...'; docker exec -it knr_drone_px4 bash -c 'source /opt/ros/jazzy/setup.bash && source ~/ros_ws/install/setup.bash && exec bash'"
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
      command = bash -c "echo '--- Panel 3: drone_handler_px4 ---' && echo 'Czekam 5s na MicroXRCE...' && sleep 5 && docker exec -it knr_drone_px4 bash -c 'source /opt/ros/jazzy/setup.bash && source ~/ros_ws/install/setup.bash && ros2 run drone_hardware drone_handler_px4'; echo 'Zakończone. Wchodzę do dockera...'; docker exec -it knr_drone_px4 bash -c 'source /opt/ros/jazzy/setup.bash && source ~/ros_ws/install/setup.bash && exec bash'"
    [[[terminal_shell]]]
      type = Terminal
      parent = child3
      order = 1
      profile = default
      command = bash -c "echo '--- Panel 4: Shell ---' && sleep 5 && docker exec -it knr_drone_px4 bash -c 'source /opt/ros/jazzy/setup.bash && source ~/ros_ws/install/setup.bash && exec bash'"
EOF

if pgrep -f "QGroundControl" > /dev/null; then
    echo "QGroundControl już działa - pomijam uruchamianie."
else
    echo "Uruchamianie QGroundControl..."
    "$DIR/QGroundControl-x86_64.AppImage" &
fi

echo "Uruchamianie Terminatora..."
terminator -u -g "$CONFIG_FILE" -l px4_sim
