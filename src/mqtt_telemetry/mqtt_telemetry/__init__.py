import rclpy
from rclpy.node import Node
import yaml
import json
import paho.mqtt.client as mqtt

from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py.convert import message_to_ordered_dict


class MqttBridge(Node):
    def __init__(self):
        super().__init__('mqtt_bridge')

        # Default config file location inside your package
        default_config_path = "/root/ros_ws/src/mqtt_telemetry/mqtt_telemetry/config.yaml"

        # Declare parameter for config file
        self.declare_parameter('config_file', default_config_path)

        # Use param OR fallback to default
        config_file = self.get_parameter('config_file').value

        self.get_logger().info(f"Using config file: {config_file}")

        # Load YAML configuration
        try:
            with open(config_file, 'r') as f:
                self.config = yaml.safe_load(f)
        except Exception as e:
            raise RuntimeError(f"Failed to load config file {config_file}: {e}")

        # -----------------------------
        # MQTT SETUP
        # -----------------------------
        self.mqtt_client = mqtt.Client(
            client_id=self.config["mqtt"].get("client_id", "ros_mqtt_bridge", protocol=mqtt.MQTTv5, callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
        )
        # Setting up protected communication
        self.mqtt_client.tls_set(tls_version=mqtt.client.ssl.PROTOCOL_TLS)
        # Optional username/password
        if "username" in self.config["mqtt"] and self.config["mqtt"]["username"]:
            self.mqtt_client.username_pw_set(
                self.config["mqtt"]["username"],
                self.config["mqtt"].get("password", "")
            )

        # Connect to broker
        self.mqtt_client.connect(
            self.config["mqtt"]["host"],
            int(self.config["mqtt"]["port"]),
            self.config["mqtt"].get("keepalive", 60)
        )

        self.mqtt_client.loop_start()

        # -----------------------------
        # SUBSCRIBE TO ROS TOPICS
        # -----------------------------
        self.subscribers = []

        for entry in self.config["topics"]:
            ros_topic = entry["ros_topic"]
            mqtt_topic = entry["mqtt_topic"]

            msg_type = self.get_message_type(ros_topic)
            if msg_type is None:
                self.get_logger().warn(f"Could not determine type for topic: {ros_topic}")
                continue

            self.get_logger().info(f"Subscribing: {ros_topic} → MQTT [{mqtt_topic}]")

            sub = self.create_subscription(
                msg_type,
                ros_topic,
                lambda msg, mqtt_topic=mqtt_topic: self.forward_to_mqtt(msg, mqtt_topic),
                10
            )
            self.subscribers.append(sub)

    # ----------------------------------------
    # HELPER: DETECT MESSAGE TYPE AUTOMATICALLY
    # ----------------------------------------
    def get_message_type(self, topic_name):
        topics = self.get_topic_names_and_types()
        for name, types in topics:
            if name == topic_name:
                msg_type_str = types[0]
                return get_message(msg_type_str)
        return None

    # ----------------------------------------
    # HELPER: PUBLISH ROS MESSAGE TO MQTT
    # ----------------------------------------
    def forward_to_mqtt(self, msg, mqtt_topic):
        data_dict = message_to_ordered_dict(msg)
        payload = json.dumps(data_dict)

        self.mqtt_client.publish(mqtt_topic, payload)
        self.get_logger().debug(f"[MQTT PUB] {mqtt_topic}: {payload}")


def main(args=None):
    rclpy.init(args=args)
    node = MqttBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
