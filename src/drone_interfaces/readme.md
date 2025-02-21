# Drone Interfaces Package
This package contains custom ROS2 interfaces

## Install
For custom interfaces to work properly, first install these packages

```shell
pip install lark
pip install catkin_pkg
pip install empy
```

## Messages
Custom messages published on topics in drone programs
### Detection
    int64[4] bounding_box
    string color_name
    int64[2] gps_position
### DetectionsList
Msg to send array of *Detection* messages

    DetectionMsg[] detections_list

