#!/usr/bin/env bash
# show_gui.sh — dopina okno GUI Gazebo do dzialajacego serwera w kontenerze.
# Odpalasz w DRUGIM terminalu, gdy serwer (./run_px4_sim.sh) juz wstal.
#
# Uzycie:
#   ./show_gui.sh
#
# Jak Docker wymaga sudo:
#   DOCKER="sudo docker" ./show_gui.sh
set -euo pipefail

DOCKER="${DOCKER:-docker}"
CONTAINER="${CONTAINER:-knr_drone_px4}"

if ! $DOCKER ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
    echo "BLAD: kontener '$CONTAINER' nie dziala. Odpal najpierw ./run_px4_sim.sh" >&2
    exit 1
fi

# Czekaj az serwer Gazebo nasluchuje (max 30s), potem dopnij GUI.
echo ">> czekam az serwer Gazebo wstanie..."
exec $DOCKER exec -it -e DISPLAY="${DISPLAY:-:0}" "$CONTAINER" bash -c '
export GZ_CONFIG_PATH=/usr/share/gz
for i in $(seq 1 30); do
    if gz topic -l 2>/dev/null | grep -q "/world/"; then
        echo ">> serwer wykryty, startuje GUI."
        exec gz sim -g
    fi
    sleep 1
done
echo "BLAD: serwer Gazebo nie wstal w 30s. Czy ./run_px4_sim.sh dziala?" >&2
exit 1
'
