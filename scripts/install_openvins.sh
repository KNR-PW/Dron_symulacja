#!/usr/bin/env bash
# install_openvins.sh - kompiluje OpenVINS (ov_msckf) w knr_drone_px4.

set -eo pipefail

WS="${WS:-/root/ros_ws}"
OV_REPO="${OV_REPO:-https://github.com/rpng/open_vins.git}"
OV_REF="${OV_REF:-}"
JOBS="${JOBS:-4}"

SUDO=""
if [ "$(id -u)" -ne 0 ]; then SUDO="sudo"; fi

echo "==> 1/5 ROS check..."
if [ -n "${ROS_DISTRO:-}" ] && [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
else
  ROS_SETUP="$(ls /opt/ros/*/setup.bash 2>/dev/null | head -n 1 || true)"
fi
if [ -z "$ROS_SETUP" ]; then
  echo "ERROR: ROS nie zainstalowany w /opt/ros" >&2
  exit 1
fi

source "$ROS_SETUP"
echo "    ROS distro: ${ROS_DISTRO}"

echo "==> 2/5 Instalowanie wymaganych komponentów..."
export DEBIAN_FRONTEND=noninteractive
$SUDO apt-get update
$SUDO apt-get install -y --no-install-recommends \
  git build-essential cmake \
  libeigen3-dev libboost-all-dev libceres-dev libopencv-dev \
  "ros-${ROS_DISTRO}-cv-bridge" \
  "ros-${ROS_DISTRO}-image-transport" \
  "ros-${ROS_DISTRO}-tf2-ros" \
  "ros-${ROS_DISTRO}-tf2-geometry-msgs"

echo "==> 3/5 Kompilowanie OpenVINS do ${WS}/src/open_vins"
mkdir -p "${WS}/src"
if [ -d "${WS}/src/open_vins/.git" ]; then
  echo "    Już jest, pomijam"
else
  git clone "${OV_REPO}" "${WS}/src/open_vins"
fi
if [ -n "${OV_REF}" ]; then
  git -C "${WS}/src/open_vins" checkout "${OV_REF}"
fi

echo "==> 4/5 Kompilowanie ov_msckf i zależności (chwilę trwa)"
cd "${WS}"
MAKEFLAGS="-j${JOBS}" colcon build \
  --packages-up-to ov_msckf \
  --parallel-workers 1 \
  --event-handlers console_cohesion+ \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "==> 5/5 Ładowanie workspacu"
SETUP_LINE="source ${WS}/install/setup.bash"
touch ~/.bashrc
if ! grep -qxF "${SETUP_LINE}" ~/.bashrc; then
  echo "${SETUP_LINE}" >> ~/.bashrc
fi

#check czy działa
bash -ic "ros2 pkg prefix ov_msckf && ros2 launch ov_msckf subscribe.launch.py --show-args | head" < /dev/null

echo "==> Zrobione"
