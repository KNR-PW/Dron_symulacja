import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2

from ultralytics import YOLO

class YoloDetector:
    def __init__(self, weights_path='yolov8n.pt', target_class_ids=None, conf_threshold=0.5):
        self.model = YOLO(weights_path)
        self.target_class_ids = target_class_ids
        self.conf_threshold = conf_threshold

    def detect(self, image):
        results = self.model.track(image, persist=True, verbose=False)
        detections = []
        if results and results[0].boxes is not None:
            for box in results[0].boxes:
                cls_id = int(box.cls)
                conf = float(box.conf)
                # Jeżeli ustawiono listę interesujących klas, filtruj po nich
                if self.target_class_ids and cls_id not in self.target_class_ids:
                    continue
                if conf < self.conf_threshold:
                    continue
                
                # Retrieve the human-readable name (e.g., 'person', 'car')
                class_name = self.model.names[cls_id]

                x1, y1, x2, y2 = map(float, box.xyxy[0])
                tid = int(box.id[0]) if box.id is not None else None
                detections.append({
                    'class_id': cls_id,
                    'class_name': class_name,  # <--- Add this field
                    'confidence': conf,
                    'bbox': (x1, y1, x2, y2),
                    'tracking_id': tid
                })
        return detections


