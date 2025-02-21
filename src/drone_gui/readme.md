# PyQt ROS2 Drone Detector GUI
This package is detector calibration app with pyqt5 and ROS2.

## Environment
App requires ROS2 Humble and Ubuntu 22.04 or similar

## Install
Install PyQt5

```shell
pip3 install pyqt5
```
**Make sure you have a properly installed [Detector Package](https://github.com/KNR-PW/drone-knr/tree/detector_devel/src/drone_detector) before running GUI**


## Usage
Source ROS2 and run GUI module
```shell
ros2 run drone_gui detector_gui
```

GUI has /camera topic subscriber and adjusted thresholds are published on /detector_thresholds topic.