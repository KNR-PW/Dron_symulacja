# SITL ArduPilot dla Gazebo.
# --out=udp:172.17.0.1:14550 wystawia MAVLink na hosta, zeby dalo sie podpiac
# Mission Planner (wgranie trasy survey, przelaczanie AUTO/GUIDED).
# 172.17.0.1 to adres hosta widziany z kontenera przez docker0 - ten sam,
# ktorego uzywa scripts/start_sim/start_ardupilot_sim.sh.
docker exec -it knr_drone bash -c "git config --global --add safe.directory /tools/ardupilot && /tools/ardupilot/Tools/autotest/sim_vehicle.py -v ArduCopter -w -f gazebo-iris --model JSON --add-param-file=/root/ros_ws/src/drone_bringup/SITL_param/gazebo_iris.parm --out=udp:172.17.0.1:14550"
