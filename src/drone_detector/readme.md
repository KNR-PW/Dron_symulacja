# Drone Detector Package
ROS2 Detector package. Provides detections of circles (trees) in Droniada competition 

## Install
Install OpenCV 2
```shell
pip3 install opencv-python
```

#### Detector Interfaces
Detector Package requires properly installed [Drone Interfaces Package](https://github.com/KNR-PW/drone-knr/tree/detector_devel/src/drone_interfaces) with all its requirements

## Running detector
Run detector ROS2 node
```shell
ros2 run drone_detector detector
```
In order to get a view of the detections displayed in the image run
```shell
ros2 launch drone_detector detector_display.launch.py
```


The frames on which detection is performed are obtained from topic */camera*.
Detections are published on topic */detections* using the [DetectionsList Interface](https://github.com/KNR-PW/drone-knr/blob/885131e07e8d5b3f7f33693c277fb853c6018018/src/drone_interfaces/readme.md)

## Calibrating detector thresholds
Detector can be calibrated to detect objects in three colors (Brown, Golden, Beige).
Lower and upper threshold of RGB values that indicate certain color can be adjusted
with dedicated [**Detector GUI**](https://github.com/KNR-PW/drone-knr/blob/885131e07e8d5b3f7f33693c277fb853c6018018/src/drone_gui/readme.md)

