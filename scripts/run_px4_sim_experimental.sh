#!/usr/bin/env bash
# run_px4_sim.sh — HOST-side wrapper. Odpala PX4 SITL + Gazebo w kontenerze
# Jak Docker wymaga sudo (nie jestes w grupie docker):
#   DOCKER="sudo docker" ./run_px4_sim.sh
set -euo pipefail

DOCKER="${DOCKER:-docker}"
CONTAINER="${CONTAINER:-knr_drone_px4}"
IMAGE="${IMAGE:-knr_px4_sim:fixed}"      # po commicie; fallback ponizej
PX4_DIR="${PX4_DIR:-/root/KNR_Drone_PX4_Autopilot}"
TARGET="${TARGET:-gz_x500_aruco}"
REPO="${REPO:-$HOME/Dron_symulacja}"

HEADLESS=0
if [[ "${1:-}" == "-h" || "${1:-}" == "--headless" || "${HEADLESS:-0}" == "1" ]]; then
    HEADLESS=1
fi

# --- 1. Czy kontener istnieje / dziala? ---
if ! $DOCKER ps -a --format '{{.Names}}' | grep -qx "$CONTAINER"; then
    echo "BLAD: kontener '$CONTAINER' nie istnieje." >&2
    echo "Odpal go najpierw (oba mounty PX4!):" >&2
    cat >&2 <<EOF
  $DOCKER run --gpus all --shm-size=1g -dit \\
    --env=NVIDIA_DRIVER_CAPABILITIES=all --env=NVIDIA_VISIBLE_DEVICES=all \\
    -e DISPLAY=\$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw \\
    --name $CONTAINER \\
    -v $REPO/src:/root/ros_ws/src:rw \\
    -v $REPO/KNR_Drone_PX4_Autopilot:$PX4_DIR:rw \\
    --network host \\
    $IMAGE
EOF
    exit 1
fi

if ! $DOCKER ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
    echo ">> kontener zatrzymany — startuje..."
    $DOCKER start "$CONTAINER" >/dev/null
fi

# --- 2. Komenda wykonywana W KONTENERZE (z zaszytymi fixami) ---
# GZ_CONFIG_PATH: ROS vendor nadpisuje go bez komendy 'sim' -> wymuszamy systemowy.
# DISPLAY: WSLg :0. exec startuje swieze env, wiec ustawiamy jawnie.
REMOTE_CMD=$(cat <<EOF
set -e
export GZ_CONFIG_PATH=/usr/share/gz
export DISPLAY=\${DISPLAY:-:0}
export PX4_GZ_MODEL_POSE=\${PX4_GZ_MODEL_POSE:-0,0,0.5}
export HEADLESS=$HEADLESS

if ! gz sim --help 2>/dev/null | grep -q 'Run and manage Gazebo'; then
    echo "BLAD: 'gz sim' niewidoczne w kontenerze (GZ_CONFIG_PATH zle)." >&2
    exit 1
fi
if [ ! -f "$PX4_DIR/Makefile" ]; then
    echo "BLAD: brak $PX4_DIR/Makefile — PX4 nie zamontowany w kontenerze." >&2
    exit 1
fi

echo ">> GZ_CONFIG_PATH=\$GZ_CONFIG_PATH DISPLAY=\$DISPLAY HEADLESS=\$HEADLESS target=$TARGET"
cd "$PX4_DIR"
exec make px4_sitl $TARGET
EOF
)

# -it: interaktywny TTY, zeby logi PX4 lecialy na zywo i Ctrl+C dzialalo.
exec $DOCKER exec -it \
    -e DISPLAY="${DISPLAY:-:0}" \
    "$CONTAINER" bash -lc "$REMOTE_CMD"
