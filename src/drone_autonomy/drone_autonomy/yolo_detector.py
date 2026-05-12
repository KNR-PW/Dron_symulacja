"""YOLO inference node — sample/test wrapper around ultralytics.YOLO.

Subscribes to a sensor_msgs/Image topic, runs YOLO inference on every frame,
publishes an annotated sensor_msgs/Image with bounding boxes and per-frame
FPS / inference-FPS overlay.

Loads .pt, .onnx or .engine via ultralytics.YOLO. On Jetson the .engine path
runs through TensorRT (significantly faster than .pt on cuda:0). The .engine
file is device-specific — build it on the machine you intend to run on.

Topics:
  in  (default): /oak/rgb/image_raw    sensor_msgs/Image, bgr8
  out (default): /yolo/image_raw       sensor_msgs/Image, bgr8

Parameters:
  model_path   (str)    Path to .pt/.onnx/.engine. Default: Jetson .engine
  conf         (double) Detection confidence threshold. Default: 0.25
  imgsz        (int)    YOLO input size. Default: 640
  device       (str)    "cuda:0" or "cpu". Default: "cuda:0"
  image_topic  (str)    Input topic. Default: "/oak/rgb/image_raw"
  output_topic (str)    Output topic. Default: "/yolo/image_raw"
"""

import time

import cv2
import numpy as np
import rclpy
import torch
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from ultralytics import YOLO


def _get_color(cls_id: int) -> tuple:
    return (
        int((37 * cls_id) % 255),
        int((17 * cls_id) % 255),
        int((29 * cls_id) % 255),
    )


class YoloDetector(Node):
    def __init__(self):
        super().__init__("yolo_detector")

        self.declare_parameter(
            "model_path",
            "/home/jetsonknr/Dron_symulacja/testy_kamery/yolo26s.engine",
        )
        self.declare_parameter("conf", 0.25)
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("image_topic", "/oak/rgb/image_raw")
        self.declare_parameter("output_topic", "/yolo/image_raw")

        self.model_path = self.get_parameter("model_path").value
        self.conf = float(self.get_parameter("conf").value)
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.device = self.get_parameter("device").value
        image_topic = self.get_parameter("image_topic").value
        output_topic = self.get_parameter("output_topic").value

        if "cuda" in self.device:
            if not torch.cuda.is_available():
                raise RuntimeError("CUDA not available — pass device:=cpu to run on CPU")
            torch.backends.cudnn.benchmark = False
            torch.backends.cudnn.deterministic = True
            torch.backends.cuda.matmul.allow_tf32 = True

        self.get_logger().info(f"Loading model {self.model_path} on {self.device}")
        self.model = YOLO(self.model_path)
        self.names = self.model.names

        # Warmup so first real frame doesn't pay engine init cost
        dummy = np.zeros((self.imgsz, self.imgsz, 3), dtype=np.uint8)
        for _ in range(3):
            self.model(
                dummy, imgsz=self.imgsz, conf=self.conf,
                device=self.device, verbose=False,
            )
        if "cuda" in self.device:
            torch.cuda.synchronize()
        self.get_logger().info("Warmup done.")

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, output_topic, 10)
        self.sub = self.create_subscription(
            Image, image_topic, self._on_image, qos_profile_sensor_data,
        )

        self._last_t = time.time()
        self._fps = 0.0
        self.get_logger().info(
            f"YOLO node listening on {image_topic} → {output_topic}"
        )

    def _on_image(self, msg: Image) -> None:
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        t0 = time.time()
        results = self.model(
            frame, imgsz=self.imgsz, conf=self.conf,
            device=self.device, verbose=False,
        )
        infer_time = time.time() - t0
        infer_fps = 1.0 / infer_time if infer_time > 0 else 0.0

        annotated = frame.copy()
        r = results[0]
        if r.boxes is not None:
            boxes = r.boxes.xyxy.cpu().numpy()
            scores = r.boxes.conf.cpu().numpy()
            classes = r.boxes.cls.cpu().numpy().astype(int)
            for box, score, cls in zip(boxes, scores, classes):
                if score < self.conf:
                    continue
                x1, y1, x2, y2 = map(int, box)
                color = _get_color(cls)
                label = f"{self.names[cls]} {score:.2f}"
                cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
                (w, h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                cv2.rectangle(annotated, (x1, y1 - h - 6), (x1 + w, y1), color, -1)
                cv2.putText(
                    annotated, label, (x1, y1 - 3),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1,
                )

        now = time.time()
        dt = now - self._last_t
        if dt > 0:
            self._fps = 0.9 * self._fps + 0.1 * (1.0 / dt)
        self._last_t = now

        cv2.putText(annotated, f"PIPE FPS: {self._fps:.1f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(annotated, f"INF FPS: {infer_fps:.1f}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        cv2.putText(annotated, f"DEVICE: {self.device}", (10, 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)

        out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        out_msg.header = msg.header
        self.pub.publish(out_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
