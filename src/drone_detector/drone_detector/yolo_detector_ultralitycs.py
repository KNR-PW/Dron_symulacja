#!/usr/bin/env python3
"""YOLO Detector — wersja SYMULACJA / PC."""

import rclpy
from drone_detector.yolo_detector_base import YoloDetectorBase


class YoloDetectorSim(YoloDetectorBase):
    def __init__(self):
        super().__init__(
            node_name="yolo_detector_ultralitycs",
            default_model="yolov8n.pt",
        )


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
