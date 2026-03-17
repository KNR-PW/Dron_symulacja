
if docker ps --filter "status=running" --format '{{.Names}}' | grep -Fxq "knr_drone"; then
    ./dockerise_cmd.sh "source ~/ros_ws/install/setup.bash && python3 ~/ros_ws/src/drone_autonomy/drone_autonomy/test_mission.py"
elif docker ps --filter "status=running" --format '{{.Names}}' | grep -Fxq "knr_drone_px4"; then
    ./dockerise_cmd.sh "source ~/ros_ws/install/setup.bash && python3 ~/Dron_symulacja/src/drone_autonomy/drone_autonomy/test_mission.py"
else
    echo "you should start the docker container"
fi
