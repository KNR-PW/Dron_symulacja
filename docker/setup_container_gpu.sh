xhost +local:docker

echo creating new container with name: knr_drone

docker run --gpus all \
    --shm-size=1g \
    -it --env="NVIDIA_DRIVER_CAPABILITIES=all" \
    --env="NVIDIA_VISIBLE_DEVICES=all" \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --name knr_drone \
    -v ./../src:/ros_ws/src:rw \
    -it \
    stas/rostest:9
