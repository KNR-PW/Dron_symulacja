#!/usr/bin/env bash
set -euo pipefail

# Wariant dla kart AMD Radeon (akceleracja przez Mesa + /dev/dri).
# Odpowiednik setup_container_gpu.sh, ale BEZ NVIDIA Container Toolkit:
# zamiast `--gpus all` przekazujemy urządzenia DRI i grupy uprawnień.
#
# Użycie: ./setup_container_amd.sh dierust/knr_ap_sim:latest

if [ -z "${1:-}" ]; then
  echo "Error: No Docker image specified."
  echo "Usage: $0 <docker-image-name>"
  exit 1
fi

IMAGE="$1"
CONTAINER_NAME="knr_drone"

echo "using docker image: $IMAGE"
echo "creating new container with name: $CONTAINER_NAME"

# Pozwól kontenerowi rysować w X11 hosta (okno Gazebo/Webots).
xhost +local:root 1>/dev/null 2>&1 || true

# Katalog źródeł: uruchamiaj ten skrypt z katalogu 'docker/docker_ap'.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_HOST="${SCRIPT_DIR}/../../src"

docker run \
  --shm-size=1g \
  -it \
  -e DISPLAY="${DISPLAY}" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --device /dev/dri:/dev/dri \
  --group-add video \
  --group-add render \
  --name "${CONTAINER_NAME}" \
  -v "${SRC_HOST}:/root/ros_ws/src:rw" \
  -p 5763:5763 \
  --add-host=host.docker.internal:host-gateway \
  "${IMAGE}"
