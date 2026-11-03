#!/usr/bin/env bash
# run_px4_sim_erc.sh — HOST-side wrapper. Odpala PX4 SITL + Gazebo w kontenerze.
# Domyslnie: swiat ERC_test, x500_mono_cam_down_ERC_test, spawn 0,0,0.5
#
#   ./run_px4_sim_erc.sh                          # swiat ERC_test
#   ./run_px4_sim_erc.sh --headless               # bez GUI Gazebo
#   PX4_GZ_MODEL_POSE="2,3,0.5,0,0,1.57" ./run_px4_sim_erc.sh   # inny spawn (x,y,z,r,p,yaw)
#   TARGET=gz_x500_gimbal ./run_px4_sim_erc.sh    # inny model
#
# Jak Docker wymaga sudo:  DOCKER="sudo docker" ./run_px4_sim_erc.sh
set -euo pipefail

DOCKER="${DOCKER:-docker}"
CONTAINER="${CONTAINER:-knr_drone_px4}"
IMAGE="${IMAGE:-knr_px4_sim:fixed}"
PX4_DIR="${PX4_DIR:-/root/KNR_Drone_PX4_Autopilot}"
REPO="${REPO:-$HOME/Dron_symulacja}"

# --- swiat ERC jako domyslny ---
PX4_GZ_WORLD="${PX4_GZ_WORLD:-ERC_test}"
TARGET="${TARGET:-gz_x500_mono_cam_down}"
PX4_GZ_MODEL_POSE="${PX4_GZ_MODEL_POSE:-0,0,0.5}"

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

# --- 2. Komenda wykonywana W KONTENERZE ---
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
WORLD_SDF="$PX4_DIR/Tools/simulation/gz/worlds/\$PX4_GZ_WORLD.sdf"
if [ ! -f "\$WORLD_SDF" ]; then
    echo "BLAD: brak \$WORLD_SDF" >&2
    exit 1
fi
# gz_bridge PX4 szuka swiata po nazwie z <world name="...">, nie po nazwie pliku
WORLD_NAME=\$(grep -oP '<world\s+name="\K[^"]+' "\$WORLD_SDF" | head -1)
if [ "\$WORLD_NAME" != "\$PX4_GZ_WORLD" ]; then
    echo "BLAD: w \$PX4_GZ_WORLD.sdf jest <world name=\"\$WORLD_NAME\">, a musi byc \"\$PX4_GZ_WORLD\"." >&2
    echo "      Popraw nazwe w SDF albo odpal z PX4_GZ_WORLD=\$WORLD_NAME (jesli taki plik istnieje)." >&2
    exit 1
fi
AIRFRAME=\$(ls "$PX4_DIR"/ROMFS/px4fmu_common/init.d-posix/airframes/ 2>/dev/null | grep -E "_${TARGET#gz_}\$" || true)
if [ -z "\$AIRFRAME" ]; then
    echo "BLAD: brak airframe'u dla targetu $TARGET w ROMFS/px4fmu_common/init.d-posix/airframes" >&2
    exit 1
fi

echo ">> world=\$PX4_GZ_WORLD pose=\$PX4_GZ_MODEL_POSE target=$TARGET HEADLESS=\${HEADLESS:-0}"
echo ">> GALLIUM_DRIVER=\${GALLIUM_DRIVER:-brak}"
cd "$PX4_DIR"
exec make px4_sitl $TARGET
EOF
)

exec $DOCKER exec -it \
    -e DISPLAY="${DISPLAY:-:0}" \
    "$CONTAINER" bash -lc "$REMOTE_CMD"
