# Check if command is provided
if [ -z "$1" ]; then
    echo "Error: No command specified."
    echo "Usage: $0 <bash-comand>"
    exit 1
fi

if [ "$(docker ps -a -q -f status=running -f name=knr_drone)" ]; then
    if [ "$(docker ps -a -q -f status=running -f name=knr_drone_px4)" ]; then
        echo "you have open 2 containers you should stop one to working with the scrpits"
        exit 1
    fi
fi

if [ "$(docker ps -a -q -f status=running -f name=knr_drone)" ]; then
    echo "running command in knr_drone container: $1"

    docker exec -it knr_drone bash -c "source /opt/ros/humble/setup.bash && $1"
elif [ "$(docker ps -a -q -f status=running -f name=knr_drone_px4)" ]; then
    echo "running command in knr_drone_px4 container: $1"

    docker exec -it knr_drone_px4 bash -c "source /opt/ros/humble/setup.bash && $1"
else
    echo "you should start the docker container"
fi


