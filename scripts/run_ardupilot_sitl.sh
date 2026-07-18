INSTANCE=${1:-0}
docker exec -it knr_drone bash -c "git config --global --add safe.directory /tools/ardupilot && /tools/ardupilot/Tools/autotest/sim_vehicle.py -v ArduCopter -I$INSTANCE -w --model webots-python --add-param-file=/root/ros_ws/src/webots_simulation/SITL_param/iris.parm"
