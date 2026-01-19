# Check if command is provided
if [ -z "$1" ]; then
    echo "Error: No command specified."
    echo "Usage: $0 <bash-comand>"
    exit 1
fi

if docker ps --filter "status=running" --format '{{.Names}}' | grep -Fxq "knr_drone"; then
    if docker ps --filter "status=running" --format '{{.Names}}' | grep -Fxq "knr_drone_px4"; then
        echo "You have both 'knr_drone' and 'knr_drone_px4' running. Stop one before using the scripts."
        exit 1
    fi
fi

if docker ps --filter "status=running" --format '{{.Names}}' | grep -Fxq "knr_drone"; then
    echo "running command in knr_drone container: $1"

    docker exec -it knr_drone bash -c "source /opt/ros/humble/setup.bash && $1"
elif docker ps --filter "status=running" --format '{{.Names}}' | grep -Fxq "knr_drone_px4"; then
    echo "running command in knr_drone_px4 container: $1"

    docker exec -it knr_drone_px4 bash -c "source /opt/ros/jazzy/setup.bash && $1"
else
    echo "you should start the docker container"
fi


