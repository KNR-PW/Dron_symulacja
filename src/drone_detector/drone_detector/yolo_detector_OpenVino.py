#!/usr/bin/env python3
"""YOLO Detector — wersja ONNX z wbudowanym, zoptymalizowanym rysowaniem."""

import rclpy
import os
os.environ["OMP_NUM_THREADS"] = "12"
os.environ["ONNXRUNTIME_NUM_THREADS"] = "12"
from drone_detector.yolo_detector_base import YoloDetectorBase
import cv2
import time
from drone_interfaces.msg import TentDetection

class YoloDetectorONNX(YoloDetectorBase):
    def __init__(self):
        super().__init__(
            node_name="yolo_detector_OpenVino",
            default_model="/root/Dron_symulacja/yolo/best_openvino_model",
        )
        self.get_logger().info("YOLO OpenVINO Fast Mode initialized.")
        self.input_size = 1024
        self._detected_frames = 0

    # Nadpisujemy metode do szybkiego rysowania, zwalniajac procesor
    def _on_image(self, msg):
        frame = self.br.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        results = self.model.track(
            frame,
            conf=self.conf,
            persist=True,
            verbose=False,
            imgsz=self.input_size,
            **self._track_kwargs,
        )

        det = TentDetection()
        det.detected = False
        det.bounding_box = [0.0, 0.0, 0.0, 0.0]
        det.confidence = 0.0

        if results and len(results[0].boxes) > 0:
            box = results[0].boxes[0]
            x1, y1, x2, y2 = map(float, box.xyxy[0].tolist())
            det.detected = True
            det.bounding_box = [x1, y1, x2 - x1, y2 - y1]
            det.confidence = float(box.conf[0])

        self.pub.publish(det)

        if self.img_pub.get_subscription_count() > 0:
            if det.detected:
                x, y, w, h = x1, y1, x2 - x1, y2 - y1
                cv2.rectangle(frame, (int(x), int(y)), (int(x+w), int(y+h)), (0, 255, 0), 2)
                cv2.putText(frame, f"{det.confidence:.2f}", (int(x), int(y)-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            self.img_pub.publish(self.br.cv2_to_imgmsg(frame, encoding="bgr8"))

        self._frames += 1
        if det.detected:
            self._detected_frames += 1

        dt = time.monotonic() - self._t0
        if dt >= 2.0:
            fps = self._frames / dt
            self.get_logger().info(f"FPS: {fps:.1f}  ({1000/fps:.0f} ms/frame) | Detekcje namiotu: {self._detected_frames}/{self._frames}")
            self._frames = 0
            self._detected_frames = 0
            self._t0 = time.monotonic()

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorONNX()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
