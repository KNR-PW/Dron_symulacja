#!/usr/bin/env bash
# run_px4_sim_experimental.sh — HOST-side wrapper. Odpala PX4 SITL + Gazebo w kontenerze.
# Domyslnie: swiat inzynierki (brik_yard), x500_gimbal, spawn z ~/brik_gz/spawn_pose.txt
#
#   ./run_px4_sim_experimental.sh                 # swiat inzynierki
#   ./run_px4_sim_experimental.sh --headless      # bez GUI Gazebo
#   SPAWN_FWD=250 ./run_px4_sim_experimental.sh   # start 250 m dalej wzdluz drogi (domyslnie 150)
#   PX4_GZ_WORLD=default TARGET=gz_x500_mono_cam_down_ERC_test ./run_px4_sim_experimental.sh   # ERC
#
# Jak Docker wymaga sudo:  DOCKER="sudo docker" ./run_px4_sim_experimental.sh
set -euo pipefail

DOCKER="${DOCKER:-docker}"
CONTAINER="${CONTAINER:-knr_drone_px4}"
IMAGE="${IMAGE:-knr_px4_sim:fixed}"      # po commicie; fallback ponizej
PX4_DIR="${PX4_DIR:-/root/KNR_Drone_PX4_Autopilot}"
REPO="${REPO:-$HOME/Dron_symulacja}"

# --- swiat inzynierki jako domyslny ---
PX4_GZ_WORLD="${PX4_GZ_WORLD:-brik_yard}"
TARGET="${TARGET:-gz_x500_gimbal}"
SPAWN_FILE="${SPAWN_FILE:-$HOME/brik_gz/spawn_pose.txt}"
if [[ -z "${PX4_GZ_MODEL_POSE:-}" ]]; then
    if [[ "$PX4_GZ_WORLD" == "brik_yard" && -f "$SPAWN_FILE" ]]; then
        # droga i tory biegna wzdluz osi Y swiata (ENU), dron patrzy w +Y;
        # spawn z gen_world.py jest 25 m od poczatku placu - przesuwamy w glab
        SPAWN_FWD="${SPAWN_FWD:-150}"
        PX4_GZ_MODEL_POSE="$(tr -d '[:space:]' < "$SPAWN_FILE" |
            awk -F, -v d="$SPAWN_FWD" 'BEGIN{OFS=","}{$2=sprintf("%.1f",$2+d); print}')"
    else
        PX4_GZ_MODEL_POSE="0,0,0.5"
    fi
fi

# HEADLESS z env albo z flagi (wczesniej HEADLESS=0 nadpisywalo zmienna z env)
HEADLESS="${HEADLESS:-0}"
if [[ "${1:-}" == "-h" || "${1:-}" == "--headless" ]]; then
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
    --device /dev/dxg -v /usr/lib/wsl:/usr/lib/wsl:ro \\
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
# GPU: jesli kontener widzi /dev/dxg i libd3d12, wymuszamy Mesa d3d12 na RTX
#      (bez tego llvmpipe albo Intel iGPU).
REMOTE_CMD=$(cat <<EOF
set -e
export GZ_CONFIG_PATH=/usr/share/gz
export DISPLAY=\${DISPLAY:-:0}
[ "$HEADLESS" = "1" ] && export HEADLESS=1 || unset HEADLESS
export PX4_GZ_WORLD="$PX4_GZ_WORLD"
export PX4_GZ_MODEL_POSE="$PX4_GZ_MODEL_POSE"

if [ -e /dev/dxg ] && [ -f /usr/lib/wsl/lib/libd3d12.so ]; then
    export LD_LIBRARY_PATH=/usr/lib/wsl/lib:\${LD_LIBRARY_PATH:-}
    export GALLIUM_DRIVER=d3d12
    export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
else
    echo "UWAGA: brak /dev/dxg lub /usr/lib/wsl w kontenerze - render na CPU (llvmpipe)" >&2
fi

if ! gz sim --help 2>/dev/null | grep -q 'Run and manage Gazebo'; then
    echo "BLAD: 'gz sim' niewidoczne w kontenerze (GZ_CONFIG_PATH zle)." >&2
    exit 1
fi
if [ ! -f "$PX4_DIR/Makefile" ]; then
    echo "BLAD: brak $PX4_DIR/Makefile — PX4 nie zamontowany w kontenerze." >&2
    exit 1
fi
if [ ! -f "$PX4_DIR/Tools/simulation/gz/worlds/\$PX4_GZ_WORLD.sdf" ]; then
    echo "BLAD: brak swiata \$PX4_GZ_WORLD.sdf w $PX4_DIR/Tools/simulation/gz/worlds" >&2
    echo "      dla brik_yard: python3 ~/brik_data/gen_world.py --px4 ~/Dron_symulacja/KNR_Drone_PX4_Autopilot --install --copy" >&2
    exit 1
fi
if [ "\$PX4_GZ_WORLD" = "brik_yard" ] && [ ! -d "$PX4_DIR/Tools/simulation/gz/models/brik_env" ]; then
    echo "BLAD: brak modelu brik_env w $PX4_DIR/Tools/simulation/gz/models (gen_world.py --install --copy)" >&2
    exit 1
fi

echo ">> world=\$PX4_GZ_WORLD pose=\$PX4_GZ_MODEL_POSE target=$TARGET HEADLESS=\${HEADLESS:-0}"
echo ">> GALLIUM_DRIVER=\${GALLIUM_DRIVER:-brak}"
cd "$PX4_DIR"
exec make px4_sitl $TARGET
EOF
)

# -it: interaktywny TTY, zeby logi PX4 lecialy na zywo i Ctrl+C dzialalo.
exec $DOCKER exec -it \
    -e DISPLAY="${DISPLAY:-:0}" \
    "$CONTAINER" bash -lc "$REMOTE_CMD"
