if [ "$(docker ps -a -q -f status=running -f name=knr_drone)" ]; then
    if [ "$(docker ps -a -q -f status=running -f name=knr_drone_px4)" ]; then
        echo "you have open 2 containers you should stop one to working with the scrpits"
        exit 1
    fi
fi

if [ "$(docker ps -a -q -f status=running -f name=knr_drone)" ]; then
    ./dockerise_cmd.sh "cd ~/ros_ws && colcon build --packages-skip Micro-XRCE-DDS-Agent px4_msgs px4_ros_com && source install/setup.bash && ros2 launch drone_bringup drone_simulation.launch.py"
elif [ "$(docker ps -a -q -f status=running -f name=knr_drone_px4)" ]; then
    echo "darkside"
else
    echo "you should start the docker container"
fi
