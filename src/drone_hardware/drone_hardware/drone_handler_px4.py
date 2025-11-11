import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer


from drone_interfaces.srv import SetMode
from drone_interfaces.action import  Arm, Takeoff

from px4_msgs.msg import VehicleStatus, VehicleCommand, OffboardControlMode


class DroneHandlerPX4(Node):
    def __init__(self):
        super().__init__('drone_handler_px4')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        NAMESPACE = 'knr_hardware/'
        #declare services
        self.mode = self.create_service(SetMode, NAMESPACE+'set_mode',self.set_mode_callback)

        #declare actions
        self.arm = ActionServer(self,Arm, NAMESPACE+'Arm',self.arm_callback)
        
        #declare subcriptions
        self.status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', self.vehicle_status_callback, qos_profile)

        # # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        #declare Main loop in timer
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("starting knr drone handler px4")

        #define the variables
        # self.current_state = "IDLE"
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arm_state = VehicleStatus.ARMING_STATE_ARMED
        self.failsafe = False # if true that means drone is in failsafe mode
        self.flightCheck = False #if true drone can be armed
        self.offboard_setpoint_counter = 0

        self.dev_mode = False

    #for some reason to work with px4 offboard (guided) mode FC must recevied with minimum 2Hz control topic
    #to know the companion computer is alive
    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    #timer loop to publish above function to drone
    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.publish_offboard_control_heartbeat_signal()

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1
        
        if self.offboard_setpoint_counter == 10:
            self.get_logger().info("Vehicle is ready to be set into offboard mode")


    #Print status of the drone
    def vehicle_status_callback(self, msg):
        if (msg.nav_state != self.nav_state):
            self.get_logger().info(f"NAV_STATUS: {msg.nav_state}")
        
        if (msg.arming_state != self.arm_state):
            self.get_logger().info(f"ARM STATUS: {msg.arming_state}")

        if (msg.failsafe != self.failsafe):
            self.get_logger().info(f"FAILSAFE: {msg.failsafe}")
        
        if (msg.pre_flight_checks_pass != self.flightCheck):
            if (msg.pre_flight_checks_pass):
                self.get_logger().info(f"Drone can be armed")
            else:
                 self.get_logger().warn(f"Drone can't be armed")

        self.nav_state = msg.nav_state
        self.arm_state = msg.arming_state
        self.failsafe = msg.failsafe
        self.flightCheck = msg.pre_flight_checks_pass


    # function to send to drone command in MAVLINK style
    def publish_vehicle_command(self, command, **params) -> None:
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)
        
    #declare services callbback
    def set_mode_callback(self, request, response):
        self.get_logger().info("zmieniam tryb")
        if (request.mode == 'GUIDED'):
            self.get_logger().info("wchodze w guided")
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        if (request.mode == 'LAND'):
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

        response = SetMode.Response()
        return response
    
    #declare actions collback
    def arm_callback(self, goal_handle):
        self.get_logger().info(f'-- Arm action registered --')
        feedback_msg = Arm.Feedback()
        
        while self.flightCheck==False:
            if self.dev_mode:
                self.get_logger().info("DEV MODE is enabled. Skipping armable check.")
                break
            feedback_msg.feedback = "Waiting for vehicle to become armable..."
            self.get_logger().info(feedback_msg.feedback)
            time.sleep(1)

        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

        while self.arm_state == 1:
            feedback_msg.feedback = "Waiting for drone to become armed..."
            self.get_logger().info(feedback_msg.feedback)
            time.sleep(1)

        feedback_msg.feedback = "Vehicle is now armed."
        self.get_logger().info(feedback_msg.feedback)

        goal_handle.succeed()
        self.state = "OK"
        result = Arm.Result()
        result.result = 1

        return result
    

def main():
    rclpy.init()
    
    drone = DroneHandlerPX4()

    # rclpy.spin(drone)
    executor = MultiThreadedExecutor()
    executor.add_node(drone)
    executor.spin()

    drone.destroy_node()

    rclpy.shutdown()


if __name__ == 'main':
    main()
