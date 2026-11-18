#!/usr/bin/env bash
# SITL ArduPilot dla Webots.
#
# Użycie:
#   ./run_ap_webots_sitl.sh          # bez wystawiania MAVLinka na hosta
#   ./run_ap_webots_sitl.sh --mp     # + MAVLink na hosta (Mission Planner)
#
# --mp dodaje --out=udp:172.17.0.1:14550. 172.17.0.1 to host widziany
# z kontenera przez docker0.

set -euo pipefail

OUT=""
case "${1:-}" in
    "")     ;;
    --mp)   OUT="--out=udp:172.17.0.1:14550" ;;
    *)
        echo "Użycie: $0 [--mp]" >&2
        exit 1
        ;;
esac

xhost +local:docker

docker exec -it knr_drone bash -c "git config --global --add safe.directory /tools/ardupilot && /tools/ardupilot/Tools/autotest/sim_vehicle.py -v ArduCopter -w --model webots-python --add-param-file=/root/ros_ws/src/webots_simulation/SITL_param/iris.parm ${OUT}"