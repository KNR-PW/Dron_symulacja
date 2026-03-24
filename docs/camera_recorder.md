source /opt/ros/jazzy/setup.bash
source ~/Dron_symulacja/install/setup.bash
ros2 run ros_gz_bridge parameter_bridge /rgb_camera/image@sensor_msgs/msg/Image[gz.msgs.Image

ros2 run drone_camera images_recorder --ros-args \
  -p camera_topic:=/rgb_camera/image \
  -p save_directory_base:=/root/Dron_symulacja/dataset_yolo/images_raw \
  -p fps:=0.25