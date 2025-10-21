source ~/.bashrc

# Check if a Docker image is provided
if [ -z "$1" ]; then
    echo "Error: No Docker image specified."
    echo "Usage: $0 <docker-image-name>"
    exit 1
fi

echo "using docker image: $1"
echo "creating new container with name: knr_drone"

docker run \
    --shm-size=1g \
    -it \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --name knr_drone \
    -v ./../src:/root/ros_ws/src:rw \
    -p 5763:5763 \
    --add-host=host.docker.internal:host-gateway \
    -it \
    $1
