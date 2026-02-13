#!/bin/sh

# Check if command is provided

EXEC_CMD="python3 examples/basic_mission/basic_mission.py"

if docker ps --filter "status=running" --format '{{.Names}}' | grep -Fxq "knr_drone"; then
    if docker ps --filter "status=running" --format '{{.Names}}' | grep -Fxq "knr_drone_px4"; then
        echo "You have both 'knr_drone' and 'knr_drone_px4' running. Stop one before using the scripts."
        exit 1
    fi
fi

if docker ps --filter "status=running" --format '{{.Names}}' | grep -Fxq "knr_drone"; then
    docker exec -it knr_drone bash -c "source /opt/ros/humble/setup.bash && cd ros_ws && source install/setup.bash \
    && cd src/drone_autonomy/drone_autonomy && export PYTHONPATH=\$PYTHONPATH:/root/ros_ws/src/drone_autonomy/drone_autonomy && $EXEC_CMD"
elif docker ps --filter "status=running" --format '{{.Names}}' | grep -Fxq "knr_drone_px4"; then
    docker exec -it knr_drone_px4 bash -c "source /opt/ros/jazzy/setup.bash && cd ros_ws && source install/setup.bash \
    && cd src/drone_autonomy/drone_autonomy && export PYTHONPATH=\$PYTHONPATH:/root/ros_ws/src/drone_autonomy/drone_autonomy && $EXEC_CMD"
else
    echo "you should start the docker container"
fi
