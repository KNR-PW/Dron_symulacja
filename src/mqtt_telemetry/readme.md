


# ODPALANIE:    

ros2 run mqtt_telemetry mqtt_bridge

# Przykładowy test   

ros2 topic pub /drone/status std_msgs/msg/String "{data: \"{\\\"altitude\\\": 25, \\\"speed\\\": 4.2, \\\"lat\\\": 52.23, \\\"lon\\\": 21.01, \\\"flight_mode\\\": \\\"AUTO\\\"}\"}" -r 1





--


--

--

The mqtt_telemetry package forwards selected ROS 2 topics to an MQTT broker. It reads a YAML configuration file that lists the topics to subscribe to and the MQTT topics to publish to. The package listens to the specified ROS topics, converts incoming messages to JSON, and sends them to an MQTT broker using the Paho MQTT library. This allows external systems, dashboards, and IoT services to monitor ROS data easily.

Features:

Reads topics and MQTT configuration from a YAML file.

Subscribes dynamically to any number of ROS topics listed in the config.

Automatically detects ROS message types at runtime.

Converts ROS messages to JSON format.

Publishes data to an MQTT broker using Paho MQTT.

Supports optional MQTT authentication.

Default configuration file included, but a custom path can be provided.

How it works:

At startup, the node loads the configuration file located at:
src/mqtt_telemetry/mqtt_telemetry/config.yaml
(You can override this path using the config_file ROS parameter.)

The node connects to the MQTT broker using the settings provided in the YAML file.

For each entry in the "topics" list in the configuration file, the node subscribes to the specified ROS topic.

Incoming messages are converted to JSON and published to the corresponding MQTT topic.

Example configuration:
The YAML configuration file contains two sections: "mqtt" settings and "topics" to forward. Adding more topics simply requires adding more entries to the "topics" list.

Running the node:
Default usage:
ros2 run mqtt_telemetry mqtt_bridge

Using a custom configuration file:
ros2 run mqtt_telemetry mqtt_bridge --ros-args -p config_file:=/path/to/config.yaml
Dependencies:
The package depends on paho-mqtt, PyYAML, rosidl-runtime-py, and ROS 2 Humble base libraries.
