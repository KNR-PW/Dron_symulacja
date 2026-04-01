#!/usr/bin/env python3
"""YOLO Detector — wersja JETSON NANO (TensorRT / half-precision)."""

import rclpy
from drone_detector.yolo_detector_base import YoloDetectorBase


class YoloDetectorJetson(YoloDetectorBase):
    def __init__(self):
        super().__init__(
            node_name="yolo_detector",
            default_model="yolov8n.engine",
            track_kwargs={"half": True},
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
        rclpy.shutdown()


if __name__ == "__main__":
    main()
