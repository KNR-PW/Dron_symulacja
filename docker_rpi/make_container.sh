docker run --shm-size=1g -it --device=/dev/ttyACM0:/dev/ttyACM0 --net=host --name knr_drone -v ./../src:/root/ros_ws/src:rw -it staskolo/knr-drone-rpi
