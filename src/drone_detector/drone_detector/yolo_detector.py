"""
Node ROS 2 — detekcja namiotu YOLOv8n ONNX.

Subskrybuje: /rgb_camera/image (sensor_msgs/Image)
Publikuje:   /tent_detections (drone_interfaces/TentDetection)
             /tent_detections/image (sensor_msgs/Image) — podglad z BB
"""

import os
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from drone_interfaces.msg import TentDetection
from rclpy.node import Node
from sensor_msgs.msg import Image
import onnxruntime as ort


class YoloDetector(Node):

    def __init__(self):
        super().__init__("yolo_detector")

        self.declare_parameter("camera_topic", "/rgb_camera/image")
        self.declare_parameter("model_path", "yolo/best.onnx")
        self.declare_parameter("confidence_threshold", 0.5)
        self.declare_parameter("nms_threshold", 0.45)
        self.declare_parameter("input_size", 1024)
        self.declare_parameter("debug_width", 480)

        self._camera_topic = self.get_parameter("camera_topic").value
        model_path = self.get_parameter("model_path").value
        self._conf = self.get_parameter("confidence_threshold").value
        self._nms = self.get_parameter("nms_threshold").value
        self._size = self.get_parameter("input_size").value
        self._dw = self.get_parameter("debug_width").value

        self._br = CvBridge()
        self._sess, self._in_name = self._load(model_path)

        self._pub = self.create_publisher(TentDetection, "/tent_detections", 10)
        self._img_pub = self.create_publisher(Image, "/tent_detections/image", 1)
        self.create_subscription(Image, self._camera_topic, self._on_image, 10)

        self._frame_count = 0
        self._t_start = time.monotonic()

        self.get_logger().info(f"YOLO detector | {model_path} | input {self._size}")

    def _load(self, path):
        if not os.path.isfile(path):
            raise FileNotFoundError(f"Brak modelu: {path}")
        opts = ort.SessionOptions()
        opts.intra_op_num_threads = 6
        opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        sess = ort.InferenceSession(path, sess_options=opts,
                                    providers=["CPUExecutionProvider"])
        name = sess.get_inputs()[0].name
        return sess, name

    def _on_image(self, msg):
        frame = self._br.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        h, w = frame.shape[:2]
        det = self._detect(frame, w, h)

        out = TentDetection()
        if det:
            out.detected = True
            out.bounding_box = [float(v) for v in det[:4]]
            out.confidence = float(det[4])
        else:
            out.detected = False
            out.bounding_box = [0.0, 0.0, 0.0, 0.0]
            out.confidence = 0.0
        self._pub.publish(out)

        self._frame_count += 1
        elapsed = time.monotonic() - self._t_start
        if elapsed >= 5.0:
            fps = self._frame_count / elapsed
            self.get_logger().info(f"Detekcja: {fps:.1f} fps ({1000/fps:.0f} ms/klatka)")
            self._frame_count = 0
            self._t_start = time.monotonic()

        if self._img_pub.get_subscription_count() > 0:
            self._pub_debug(frame, det)

    def _detect(self, frame, orig_w, orig_h):
        blob = cv2.dnn.blobFromImage(
            frame, 1.0 / 255.0, (self._size, self._size),
            (0, 0, 0), swapRB=True, crop=False,
        ).astype(np.float32)

        raw = self._sess.run(None, {self._in_name: blob})[0]
        preds = raw[0].T if raw.shape[1] < raw.shape[2] else raw[0]

        scores = preds[:, 4]
        mask = scores >= self._conf
        preds = preds[mask]
        if len(preds) == 0:
            return None

        sx, sy = orig_w / self._size, orig_h / self._size
        cx = preds[:, 0] * sx
        cy = preds[:, 1] * sy
        bw = preds[:, 2] * sx
        bh = preds[:, 3] * sy
        confs = preds[:, 4]

        x = cx - bw / 2
        y = cy - bh / 2
        boxes = np.stack([x, y, bw, bh], axis=1).astype(np.float32)

        idxs = cv2.dnn.NMSBoxes(boxes.tolist(), confs.tolist(), self._conf, self._nms)
        if idxs is None or len(idxs) == 0:
            return None

        i = int(np.array(idxs).flatten()[0])
        return (int(boxes[i][0]), int(boxes[i][1]),
                int(boxes[i][2]), int(boxes[i][3]), float(confs[i]))

    def _pub_debug(self, frame, det):
        h, w = frame.shape[:2]
        s = self._dw / w
        vis = cv2.resize(frame, (self._dw, int(h * s)), interpolation=cv2.INTER_NEAREST)
        if det:
            x, y, bw, bh, conf = det
            cv2.rectangle(vis, (int(x*s), int(y*s)),
                          (int((x+bw)*s), int((y+bh)*s)), (0, 255, 0), 2)
            cv2.putText(vis, f"{conf:.2f}", (int(x*s), int(y*s)-6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        self._img_pub.publish(self._br.cv2_to_imgmsg(vis, encoding="bgr8"))


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(YoloDetector())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
