import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, BoundingBox2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from drone_detector.yolo_detector import YoloDetector  # Import ze swojej biblioteki
import cv2

class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')
        self.bridge = CvBridge()
        self.detector = YoloDetector(weights_path='yolov8n.pt', target_class_ids=[2])  # np. tylko 'car'
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.detection_pub = self.create_publisher(
            Detection2DArray, '/detections', 10)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        detections = self.detector.detect(cv_image)
        msg_out = Detection2DArray()
        msg_out.header = msg.header
        # Konwersja detekcji
        for det in detections:
            d = Detection2D()
            # Bounding box (środek, szerokość, wysokość)
            x1, y1, x2, y2 = det['bbox']
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0
            w = x2 - x1
            h = y2 - y1
            d.bbox = BoundingBox2D()
            d.bbox.center.position.x = cx
            d.bbox.center.position.y = cy
            d.bbox.size_x = w
            d.bbox.size_y = h
            # Hipoteza klasy
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = str(det['class_id'])
            hyp.hypothesis.score = det['confidence']
            d.results.append(hyp)

            # annotated_frame = results[0].plot()  # Rysuje bounding boxy i etykiety
            # cv2.imshow("YOLO Detection", annotated_frame)
            # cv2.waitKey(1)  # Potrzebne do odświeżania okna

            # Opcjonalny ID śledzenia
            if det['tracking_id']:
                d.id = str(det['tracking_id'])
            msg_out.detections.append(d)
        self.detection_pub.publish(msg_out)

def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
