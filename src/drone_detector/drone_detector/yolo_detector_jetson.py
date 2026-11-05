#!/usr/bin/env python3
"""YOLO Detector — wersja JETSON / REAL."""

import rclpy
from drone_detector.yolo_detector_base import YoloDetectorBase


class YoloDetectorJetson(YoloDetectorBase):
    def __init__(self):
        super().__init__(
            node_name="yolo_detector_jetson",
            default_model="/home/jetsonknr/Dron_symulacja/testy_kamery/yolo26s.engine",
            track_kwargs={"device": "cuda:0", "half": True}
        )


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorJetson()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
