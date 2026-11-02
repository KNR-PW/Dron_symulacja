#!/usr/bin/env bash
set -euo pipefail

# Wariant bez GPU (renderowanie programowe).
#
# Użycie: ./setup_container_cpu.sh dierust/knr_ap_sim:latest

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

# Ścieżka do src/ liczona od położenia skryptu, więc działa z dowolnego katalogu.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_HOST="${SCRIPT_DIR}/../../src"

docker run \
  --shm-size=1g \
  -it \
  -e DISPLAY="${DISPLAY}" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --name "${CONTAINER_NAME}" \
  -v "${SRC_HOST}:/root/ros_ws/src:rw" \
  -p 5763:5763 \
  --add-host=host.docker.internal:host-gateway \
  "${IMAGE}"
