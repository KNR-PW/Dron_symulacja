import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D, ObjectHypothesisWithPose
from std_srvs.srv import SetBool
from cv_bridge import CvBridge
import cv2
import numpy as np
import traceback
import time

from drone_detector.yolo_detector import YoloDetector
from drone_detector.opencv_tracker_wrapper import OpenCVTrackerWrapper

# run this by: `ros2 launch drone_bringup drone_simulation.launch.py detector:=hybrid tracker_type:=KCF`

class HybridTrackerNode(Node):
    def __init__(self):
        super().__init__('hybrid_tracker_node')
        self.bridge = CvBridge()
        
        # --- Parameters ---
        self.declare_parameter('tracker_type', 'KCF') 
        self.declare_parameter('yolo_weights', 'yolov8n.pt')
        self.declare_parameter('correct_colors', True) 
        
        tracker_type = self.get_parameter('tracker_type').value
        weights = self.get_parameter('yolo_weights').value
        self.correct_colors = self.get_parameter('correct_colors').value
        
        self.get_logger().info(f"Initializing Hybrid Tracker: {tracker_type} | Weights: {weights} | Color Fix: {self.correct_colors}")

        self.yolo = None
        self.tracker = None
        self.init_error = None

        try:
            # self.yolo = YoloDetector(weights_path=weights, target_class_ids=[2], conf_threshold=0.2)
            self.yolo = YoloDetector(weights_path=weights, target_class_ids=[2], conf_threshold=0.2)
            self.tracker = OpenCVTrackerWrapper(tracker_type=tracker_type)
            self.get_logger().info("Initialization SUCCESS")
        except Exception as e:
            self.init_error = str(e)
            self.get_logger().error(f"Initialization FAILED: {e}")
            self.get_logger().error(traceback.format_exc())

        self.tracking_active = False
        self.tracked_class_name = ""
        self.miss_counter = 0
        self.max_misses = 10 

        self.detection_interval = 10 # frames
        self.frame_count = 0

        self.node_enabled = False  # Start disabled!
        self.srv = self.create_service(SetBool, 'enable_tracker', self.enable_callback)
        
        self.image_sub = self.create_subscription(
            Image, '/gimbal_camera', self.image_callback, 1) # Queue size 1 to process only latest frames
        
        self.detection_pub = self.create_publisher(
            Detection2DArray, '/detections', 10)
        
        self.debug_image_pub = self.create_publisher(
            Image, '/detections/annotated', 10)
            
        self.last_time = time.time()
        self.avg_fps = 0.0

    def enable_callback(self, request, response):
        self.node_enabled = request.data
        if self.node_enabled:
            self.get_logger().info("Tracker ENABLED via service.")
            # Reset state when enabling to ensure fresh start
            self.tracking_active = True
            self.miss_counter = 0
        else:
            self.get_logger().info("Tracker DISABLED via service.")
            self.tracking_active = False
        
        response.success = True
        response.message = f"Tracker enabled: {self.node_enabled}"
        return response

    def image_callback(self, msg):
        # --- 1. Check if enabled ---
        if not self.node_enabled:
            # Optional: Publish "DISABLED" text on debug image so you know it's alive
            try:
                frame = self.bridge.imgmsg_to_cv2(msg, "bgr8").copy()
                if self.correct_colors: frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                cv2.putText(frame, "TRACKER DISABLED (WAITING FOR TAKEOFF)", (10, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                self._publish_debug(frame, msg.header)
            except: pass
            return

        # --- STABILIZED FPS CALCULATION ---
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        instant_fps = 1.0 / dt if dt > 0 else 0.0
        
        # Exponential Moving Average (EMA)
        # alpha = 0.1 means new value has 10% weight, history has 90%
        alpha = 0.1
        if self.avg_fps == 0.0:
            self.avg_fps = instant_fps
        else:
            self.avg_fps = alpha * instant_fps + (1.0 - alpha) * self.avg_fps

        # 1. Convert Image
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8").copy()
            
            if self.correct_colors:
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                
        except Exception as e:
            self.get_logger().error(f"Frame conversion error: {e}")
            return

        # 2. Check Init
        if self.yolo is None or self.tracker is None:
            self._draw_debug_info(frame, "INIT ERROR", (0,0,255), self.avg_fps)
            self._publish_debug(frame, msg.header)
            return

        detections_msg = Detection2DArray()
        detections_msg.header = msg.header
        
        final_bbox = None
        mode = "SEARCHING"
        tracker_name = self.tracker.tracker_type

        self.frame_count += 1

        try:            
            # --- TRACKER UPDATE ---
            tracking_success = False
            if self.tracking_active:
                if frame is not None and frame.size > 0:
                    success, bbox = self.tracker.update(frame)
                    if success:
                        tracking_success = True
                        mode = f"TRACKING ({tracker_name})"
                        x, y, w, h = [int(v) for v in bbox]
                        final_bbox = (x, y, x+w, y+h)
                        self._add_detection_to_msg(detections_msg, x, y, w, h, self.tracked_class_name, 1.0)
                    else:
                        self.tracking_active = False # Lost track

            # --- YOLO DETECTION ---
            # Run if: 1. Not tracking, 2. Tracking lost, 3. Periodic check
            should_detect = (not tracking_success) or (self.frame_count % self.detection_interval == 0)
            
            if should_detect:
                if not tracking_success:
                    mode = "DETECTING (YOLO)"
                
                yolo_detections, _ = self.yolo.detect(frame)
                
                if yolo_detections:
                    best_det = max(yolo_detections, key=lambda x: x['confidence'])
                    x1, y1, x2, y2 = best_det['bbox']
                    w = x2 - x1
                    h = y2 - y1
                    
                    # Sanity check
                    if w > 0 and h > 0:
                        bbox_int = (int(x1), int(y1), int(w), int(h))
                        
                        # Re-initialize tracker (correct drift or start new track)
                        self.tracker.init(frame, bbox_int)
                        self.tracking_active = True
                        self.tracked_class_name = best_det['class_name']
                        
                        # If we weren't tracking, use this detection result
                        if not tracking_success:
                            final_bbox = (int(x1), int(y1), int(x2), int(y2))
                            self._add_detection_to_msg(detections_msg, x1, y1, w, h, best_det['class_name'], best_det['confidence'])
                    
                    # self.get_logger().info("successful detection")

            # Publish
            self.detection_pub.publish(detections_msg)

            # Debug Draw
            color = (255, 0, 0) if "TRACKING" in mode else (0, 0, 255)
            if final_bbox:
                h_img, w_img = frame.shape[:2]
                x1, y1, x2, y2 = final_bbox
                cv2.rectangle(frame, (max(0,x1), max(0,y1)), (min(w_img,x2), min(h_img,y2)), color, 2)
            
            self._draw_debug_info(frame, f"{mode}: {self.tracked_class_name}", color, self.avg_fps)
            self._publish_debug(frame, msg.header)

        except Exception as e:
            self.get_logger().error(f"Runtime Error: {e}")
            self.tracking_active = False

    def _draw_debug_info(self, frame, text, color, fps):
        cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        cv2.putText(frame, f"FPS: {fps:.1f}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    def _publish_debug(self, frame, header):
        debug_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        debug_msg.header = header
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