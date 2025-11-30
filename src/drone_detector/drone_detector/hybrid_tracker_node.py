import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import cv2

# Import your existing YOLO detector for initialization
from drone_detector.yolo_detector import YoloDetector
# Import the new generic wrapper
from drone_detector.opencv_tracker_wrapper import OpenCVTrackerWrapper

class HybridTrackerNode(Node):
    def __init__(self):
        super().__init__('hybrid_tracker_node')
        self.bridge = CvBridge()
        
        # --- Parameters ---
        self.declare_parameter('tracker_type', 'KCF') # Options: KCF, CSRT, MIL, MOSSE, etc.
        self.declare_parameter('yolo_weights', 'yolov8n.pt')
        
        tracker_type = self.get_parameter('tracker_type').value
        weights = self.get_parameter('yolo_weights').value
        
        self.get_logger().info(f"Initializing Hybrid Tracker with YOLOv8 + {tracker_type}")

        # We use YOLO to find the object initially
        self.yolo = YoloDetector(weights_path=weights, conf_threshold=0.5)
        
        # We use OpenCV tracker to follow it
        try:
            self.tracker = OpenCVTrackerWrapper(tracker_type=tracker_type)
        except RuntimeError as e:
            self.get_logger().error(str(e))
            self.destroy_node()
            return

        self.tracking_active = False
        self.tracked_class_name = ""
        self.miss_counter = 0
        self.max_misses = 10 
        
        self.image_sub = self.create_subscription(
            Image, '/gimbal_camera', self.image_callback, 10)
        
        self.detection_pub = self.create_publisher(
            Detection2DArray, '/detections', 10)
        
        self.debug_image_pub = self.create_publisher(
            Image, '/detections/annotated', 10)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        detections_msg = Detection2DArray()
        detections_msg.header = msg.header
        
        final_bbox = None
        mode = "SEARCHING"
        tracker_name = self.tracker.tracker_type

        if self.tracking_active:
            # --- TRACKER UPDATE STEP ---
            success, bbox = self.tracker.update(frame)
            
            if success:
                mode = f"TRACKING ({tracker_name})"
                x, y, w, h = [int(v) for v in bbox]
                final_bbox = (x, y, x+w, y+h)
                self.miss_counter = 0
                
                self._add_detection_to_msg(detections_msg, x, y, w, h, self.tracked_class_name, 1.0)
            else:
                self.miss_counter += 1
                # self.get_logger().warn(f"{tracker_name} lost track! Miss count: {self.miss_counter}")
                if self.miss_counter > self.max_misses:
                    self.tracking_active = False
                    self.get_logger().info("Tracking lost. Resetting to YOLO detection.")

        if not self.tracking_active:
            # --- YOLO DETECTION STEP ---
            mode = "DETECTING (YOLO)"
            yolo_detections, _ = self.yolo.detect(frame)
            
            if yolo_detections:
                # Pick the highest confidence detection
                best_det = max(yolo_detections, key=lambda x: x['confidence'])
                
                x1, y1, x2, y2 = best_det['bbox']
                w = x2 - x1
                h = y2 - y1
                
                # Initialize OpenCV Tracker
                self.tracker.init(frame, (x1, y1, w, h))
                self.tracking_active = True
                self.tracked_class_name = best_det['class_name']
                
                final_bbox = (int(x1), int(y1), int(x2), int(y2))
                
                self._add_detection_to_msg(detections_msg, x1, y1, w, h, best_det['class_name'], best_det['confidence'])

        # --- Publish Results ---
        self.detection_pub.publish(detections_msg)

        # --- Debug Image ---
        if final_bbox:
            color = (255, 0, 0) if "TRACKING" in mode else (0, 0, 255)
            cv2.rectangle(frame, (final_bbox[0], final_bbox[1]), (final_bbox[2], final_bbox[3]), color, 2)
            cv2.putText(frame, f"{mode}: {self.tracked_class_name}", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        
        debug_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        debug_msg.header = msg.header
        self.debug_image_pub.publish(debug_msg)

    def _add_detection_to_msg(self, msg, x, y, w, h, class_name, score):
        d = Detection2D()
        d.bbox = BoundingBox2D()
        d.bbox.center.position.x = x + w / 2.0
        d.bbox.center.position.y = y + h / 2.0
        d.bbox.size_x = float(w)
        d.bbox.size_y = float(h)
        
        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = class_name
        hyp.hypothesis.score = float(score)
        d.results.append(hyp)
        msg.detections.append(d)

def main(args=None):
    rclpy.init(args=args)
    node = HybridTrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()