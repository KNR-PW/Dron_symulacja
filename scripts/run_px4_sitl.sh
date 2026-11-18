#!/usr/bin/env bash
# Użycie: ./run_px4_sitl.sh <quad|vtol> [świat]
# Przykłady:
#   ./run_px4_sitl.sh quad small_city
#   ./run_px4_sitl.sh vtol            # świat domyślny: aruco

set -euo pipefail

TYP=${1:-}
WORLD=${2:-aruco}

case "$TYP" in
    quad)
        MODEL="x500"
        POSE="0,0,5"
        ;;
    vtol)
        MODEL="knr_tiltrotor"
        POSE="0,0,4"
        ;;
    *)
        echo "Użycie: $0 <quad|vtol> [świat]" >&2
        exit 1
        ;;
esac

cd "$(dirname "$0")"

if [ "$TYP" = "vtol" ]; then
    cp -r ../knr_tiltrotor ../KNR_Drone_PX4_Autopilot/Tools/simulation/gz/models
fi

cd ../KNR_Drone_PX4_Autopilot

TARGET="gz_${MODEL}_${WORLD}"

# Odśwież CMake tylko jeśli cel nie istnieje
if ! ninja -C build/px4_sitl_default -t targets 2>/dev/null | grep -q "$TARGET"; then
    echo "Nowy świat - odświeżam CMake..."
    touch src/modules/simulation/gz_bridge/CMakeLists.txt
fi

# Pozycja spawnu drona (x,y,z)
export PX4_GZ_MODEL_POSE="$POSE"

make px4_sitl "$TARGET"