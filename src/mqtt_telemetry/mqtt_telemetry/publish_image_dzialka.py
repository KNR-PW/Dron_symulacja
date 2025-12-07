import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import base64
import json
from pathlib import Path


class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher_dzialka')

        self.pub = self.create_publisher(String, '/drone/image', 10)

        img_path = Path('/root/ros_ws/src/webots_simulation/resource/worlds/dzialka_inspekcja.jpg')
        if not img_path.exists():
            self.get_logger().error(f"Image not found: {img_path}")
            rclpy.shutdown()
            return

        data = img_path.read_bytes()
        b64 = base64.b64encode(data).decode('utf-8')

        payload = {
            "filename": "dzialka_inspekcja.jpg",
            "data": b64,
        }

        msg = String()
        msg.data = json.dumps(payload)

        self.pub.publish(msg)
        self.get_logger().info(f"Published image {img_path} on /drone/image")

        # mały timer, żeby dać czas na wysłanie i zamknąć node
        self.create_timer(1.0, self._shutdown)

    def _shutdown(self):
        self.get_logger().info("Shutting down image_publisher_dzialka")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
