import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2

from ultralytics import YOLO

class YoloDetector(Node):
    def __init__(self):
        super().__init__('Yolo_node')

        ## DECLARE PARAMETERS
        self.declare_parameter('camera_topic', 'camera')
        self.declare_parameter('yolo_model', 'false')

        ## DECLARE YOLO CLASS POLES
        # model_path = self.get_parameter('model_path').get_parameter_value().string_value
        model_path = "yolo_models/yolo11n.pt"
        self.model = YOLO(model_path, task="detect")
        self.conf_threshold = 0.8

        ## DECLARE CAMERA POLES
        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.get_logger().info(f'camera_topic: {camera_topic}')
        self.subscription = self.create_subscription(
            Image,
            camera_topic,
            self.listener_callback,
            10)
        self.br = CvBridge()

    def listener_callback(self, data):
        # Convert to OpenCV BGR format to preserve expected colors in saved files.
        image = self.br.imgmsg_to_cv2(data, desired_encoding='rgb8')
        results = self.model.predict(image)

        detections = []
        if results and results[0].boxes is not None:
            for box in results[0].boxes:
                cls_id = int(box.cls)
                conf = float(box.conf)
                # if self.target_class_ids and cls_id not in self.target_class_ids:
                #     continue
                if conf < self.conf_threshold:
                    continue

                class_name = self.model.names[cls_id]

                x1, y1, x2, y2 = map(float, box.xyxy[0])
                middle_of_x = abs(x1-x2)/2
                middle_of_y = abs(y1-y2)/2
                tid = int(box.id[0]) if box.id is not None else None
                detections.append({
                    'class_id': cls_id,
                    'class_name': class_name,
                    'confidence': conf,
                    'middle_xy': (middle_of_x, middle_of_y),
                    })
                self.get_logger().info(f'class_id: {cls_id}')
                self.get_logger().info(f'class_name: {class_name}')
                self.get_logger().info(f'confidence: {conf}')
                self.get_logger().info(f'middle of x: {middle_of_x}')
                self.get_logger().info(f'middle of y: {middle_of_y}')


def main(args=None):
    rclpy.init(args=args)

    yolo_node = YoloDetector()

    rclpy.spin(yolo_node)

    yolo_node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
