#!/bin/bash
./dockerise_cmd.sh "export FASTDDS_BUILTIN_TRANSPORTS=UDPv4 && source ~/ros_ws/install/setup.bash && ros2 run drone_camera images_recorder --ros-args -p camera_topic:=/world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image -p enable_timer:=false"
