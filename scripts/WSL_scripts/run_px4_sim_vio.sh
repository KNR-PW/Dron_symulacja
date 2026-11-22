#!/usr/bin/env bash
# run_px4_sim_vio.sh — test odpalenia x500_oak_d (stereo + IMU) na GPS.
# Wrapper na run_px4_sim_erc.sh: inny model + bridge kamer stereo, IMU i ground truth.
set -euo pipefail

ERC_SCRIPT="${ERC_SCRIPT:-$HOME/Dron_symulacja/scripts/WSL_scripts/run_px4_sim_erc.sh}"

export TARGET="${TARGET:-gz_x500_oak_d}"
export BRIDGE_TOPICS="${BRIDGE_TOPICS:-\
/camera/left/image@sensor_msgs/msg/Image[gz.msgs.Image \
/camera/left/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo \
/camera/right/image@sensor_msgs/msg/Image[gz.msgs.Image \
/camera/right/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo \
/imu@sensor_msgs/msg/Imu[gz.msgs.IMU \
/ground_truth/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry \
/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock}"

exec "$ERC_SCRIPT" "$@"
