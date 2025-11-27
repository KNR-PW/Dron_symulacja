# echo "xhost +local:docker" >> ~/.bashrc
source ~/.bashrc

# Check if a Docker image is provided
if [ -z "$1" ]; then
    echo "Error: No Docker image specified."
    echo "Usage: $0 <docker-image-name>"
    exit 1
fi

echo "using docker image: $1"
echo "creating new container with name: knr_drone_px4"

docker run \
    --shm-size=1g \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --device /dev/dri:/dev/dri \
    --name knr_drone_px4 \
    -v ./../../src:/root/ros_ws/src:rw \
    -it \
    --network host \
    $1
