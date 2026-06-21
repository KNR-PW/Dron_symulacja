./dockerise_cmd.sh "cd ~/ros_ws && colcon build --packages-skip microxrcedds_agent px4_msgs px4_ros_com && source install/setup.bash && ros2 launch drone_bringup gazeboo_ap_sim.launch.py"

