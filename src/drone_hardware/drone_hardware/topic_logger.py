import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message

import datetime
import threading
import os

class TopicLogger(Node):
    def __init__(self, node_name: str = 'topic_logger'):
        super().__init__(node_name)
        self.topic_types = {}
        self.my_subscriptions = {}
        self.filter = IgnoredTopics()

        directory = "/root/ros_ws/src/drone_hardware/drone_hardware/topic_logs/"
        os.makedirs(directory, exist_ok=True)
        i = 1

        while(True):

            file_name = f"topic_log_{i}.csv"
            topic_file_name = os.path.join(directory, file_name)

            if os.path.isfile(topic_file_name):
                i += 1

            else:
                break
            

        self.topic_log_file = open(topic_file_name, "w")

        timestamp = datetime.datetime.now().isoformat()
        message = "Log of topics and messages, created at"

        header = f"{message} {timestamp}\n"
        self.topic_log_file.write(header)
        
        self.create_timer(2.0, self.find_and_subscribe)
    
    def find_and_subscribe(self):        
        for topic_name, topic_types in self.get_topic_names_and_types():
            if topic_name in self.my_subscriptions:
                continue
            if topic_name.startswith('/rosout'):
                continue
            if self.filter.check_if_ignored(topic_name):
                continue

            type_str = topic_types[0]

            try:
                msg_type = get_message(type_str)
                self.topic_types[topic_name] = msg_type
                sub = self.create_subscription(msg_type, 
                                               topic_name, 
                                               lambda msg, 
                                               topic = topic_name: self.callback(msg, topic),
                                               qos_profile_sensor_data,
                                               raw = True)
                self.my_subscriptions[topic_name] = sub

                #self.get_logger().info(f"Subscribed: {topic_name} [{type_str}]")

            except Exception as e:
                self.get_logger().warning(f"Failed {topic_name}: {e}")
    
    def callback(self, raw_bytes, topic_name):
        try:
            msg_type = self.topic_types[topic_name]
            msg = deserialize_message(raw_bytes, msg_type)

            timestamp = datetime.datetime.now().isoformat()
            self.topic_log_file.write(f"{timestamp} ; {topic_name} ; {msg} \n")

        except Exception as e:
            #self.get_logger().warning(f"Error during callback {topic_name}: {e}")
            msg = None
    
    def process_message(self, topic, msg):
        self.get_logger().info(f"{topic}: {msg}")

    def shutDown(self):
        self.topic_log_file.close()


class LoggerControl:
    def __init__(self):
        self.node = None
        self.executor = None
        self.thread = None

    def start(self):
        if not rclpy.ok():
            rclpy.init()
        
        self.node = TopicLogger()
        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
    
    def stop(self):
        if self.executor:
            self.executor.shutdown()
        
        if self.node:
            self.node.shutDown()
            self.node.destroy_node()
        
        if self.thread:
            self.thread.join()

class IgnoredTopics:
    def __init__(self, topics_to_ignore=None):
        self.topics_to_ignore = topics_to_ignore or {
            "/fmu/out/vehicle_global_position" :        1,
            "/fmu/out/vehicle_gps_position" :           1,
            "/fmu/out/vehicle_local_position_v1" :      1,
            "/fmu/out/vehicle_odometry" :               1,
            "/fmu/in/offboard_control_mode" :           1,
            "/fmu/out/airspeed_validated_v1" :          1,
            "/fmu/out/sensor_combined" :                1,
            "/fmu/out/vehicle_attitude" :               1,
            "/fmu/in/actuator_motors":                  0,
            "/fmu/in/actuator_servos":                  0,
            "/fmu/in/arming_check_reply_v1":            0,
            "/fmu/in/aux_global_position":              0,
            "/fmu/in/config_control_setpoints":         0,
            "/fmu/in/config_overrides_request_v1":      0,
            "/fmu/in/distance_sensor":                  0,
            "/fmu/in/fixed_wing_lateral_setpoint":      0,
            "/fmu/in/fixed_wing_longitudinal_setpoint": 0,
            "/fmu/in/goto_setpoint":                    0,
            "/fmu/in/landing_gear":                     0,
            "/fmu/in/lateral_control_configuration":    0,
            "/fmu/in/longitudinal_control_configuration": 0,
            "/fmu/in/manual_control_input":             0,
            "/fmu/in/message_format_request":           0,
            "/fmu/in/mode_completed":                   0,
            "/fmu/in/obstacle_distance":                0,
            "/fmu/in/onboard_computer_status":          0,
            "/fmu/in/register_ext_component_request_v1": 0,
            "/fmu/in/rover_attitude_setpoint":          0,
            "/fmu/in/rover_position_setpoint":          0,
            "/fmu/in/rover_rate_setpoint":              0,
            "/fmu/in/rover_speed_setpoint":             0,
            "/fmu/in/rover_steering_setpoint":          0,
            "/fmu/in/rover_throttle_setpoint":          0,
            "/fmu/in/sensor_optical_flow":              0,
            "/fmu/in/telemetry_status":                 0,
            "/fmu/in/trajectory_setpoint":              0,
            "/fmu/in/unregister_ext_component":         0,
            "/fmu/in/vehicle_attitude_setpoint_v1":     0,
            "/fmu/in/vehicle_command":                  0,
            "/fmu/in/vehicle_command_mode_executor":    0,
            "/fmu/in/vehicle_mocap_odometry":           0,
            "/fmu/in/vehicle_rates_setpoint":           0,
            "/fmu/in/vehicle_thrust_setpoint":          0,
            "/fmu/in/vehicle_torque_setpoint":          0,
            "/fmu/in/vehicle_visual_odometry":          0,
            "/fmu/out/arming_check_request_v1":         0,
            "/fmu/out/battery_status_v1":               0,
            "/fmu/out/collision_constraints":           0,
            "/fmu/out/estimator_status_flags":          0,
            "/fmu/out/failsafe_flags":                  0,
            "/fmu/out/gimbal_device_attitude_status":   0,
            "/fmu/out/home_position_v1":                0,
            "/fmu/out/manual_control_setpoint":         0,
            "/fmu/out/message_format_response":         0,
            "/fmu/out/mode_completed":                  0,
            "/fmu/out/position_setpoint_triplet":       0,
            "/fmu/out/register_ext_component_reply_v1": 0,
            "/fmu/out/timesync_status":                 0,
            "/fmu/out/transponder_report":              0,
            "/fmu/out/vehicle_command_ack":             0,
            "/fmu/out/vehicle_control_mode":            0,
            "/fmu/out/vehicle_land_detected":           0,
            "/fmu/out/vehicle_status_v1":               0,
            "/fmu/out/vtol_vehicle_status":             0,
            "/fmu/out/wind":                            0,
            "/knr_hardware/Arm/_action/feedback":       0,
            "/knr_hardware/Arm/_action/status":         0,
            "/knr_hardware/Set_yaw/_action/feedback":   0,
            "/knr_hardware/Set_yaw/_action/status":     0,
            "/knr_hardware/goto_global/_action/feedback": 0,
            "/knr_hardware/goto_global/_action/status": 0,
            "/knr_hardware/goto_relative/_action/feedback": 0,
            "/knr_hardware/goto_relative/_action/status": 0,
            "/knr_hardware/takeoff/_action/feedback":   0,
            "/knr_hardware/takeoff/_action/status":     0,
            "/knr_hardware/telemetry":                  0,
            "/knr_hardware/velocity_vectors":           0,
            "/parameter_events":                        0,
            }
    
    def check_if_ignored(self, topic_name):
        return self.topics_to_ignore.get(topic_name, 0) == 1