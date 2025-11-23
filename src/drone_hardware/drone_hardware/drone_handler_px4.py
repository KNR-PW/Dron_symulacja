import time
import math
import haversine as hv
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer,  CancelResponse

from drone_interfaces.msg import VelocityVectors
from drone_interfaces.srv import SetMode, ToggleVelocityControl
from drone_interfaces.action import  Arm, Takeoff, GotoGlobal, GotoRelative, SetYawAction

from px4_msgs.msg import VehicleStatus, VehicleCommand, OffboardControlMode, VehicleGlobalPosition, TrajectorySetpoint, VehicleLocalPosition, VehicleAttitude
#TODO 
#1. (DONE)makes check if we get a new topic before we send mode to offboard
#2. still rebuilding the drone handler to px4
#3. convert the flight mode in NAV state to string to be readable for a man
#4. add more flight modes minimum is RTL

class GlobalPosition():
    def __init__(self):
        self.alt = float(0.0)
        self.lat = float(0.0)
        self.lon = float(0.0)

class LocalPosition():
    def __init__(self):
        # Position in local NED frame counts from start point, coordinates of start point is (0,0) 
        self.x = float(0.0) # North position in NED earth-fixed frame, (metres)
        self.y = float(0.0) # East position in NED earth-fixed frame, (metres)
        self.z = float(0.0) # Down position (negative altitude) in NED earth-fixed frame, (metres)

        # Velocity in NED frame
        self.vx = float(0.0) # North velocity in NED earth-fixed frame, (metres/sec)
        self.vy = float(0.0) # East velocity in NED earth-fixed frame, (metres/sec)
        self.vz = float(0.0) # Down velocity in NED earth-fixed frame, (metres/sec)

        self.heading = float(0.0)

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
        self.toggle_velocity_control_srv = self.create_service(ToggleVelocityControl, NAMESPACE+'toggle_v_control', self.toggle_velocity_control)

        #declare actions
        self.arm = ActionServer(self,Arm, NAMESPACE+'Arm',self.arm_callback)
        self.takeoff = ActionServer(self, Takeoff, NAMESPACE+'takeoff',self.takeoff_callback, cancel_callback=self.cancel_callback)
        self.goto_global = ActionServer(self, GotoGlobal, NAMESPACE+'goto_global', self.goto_global_action, cancel_callback=self.cancel_callback)
        self.goto_rel = ActionServer(self, GotoRelative, NAMESPACE+'goto_relative', self.goto_relative_action, cancel_callback=self.cancel_callback)
        self.yaw = ActionServer(self, SetYawAction, NAMESPACE+'Set_yaw', self.yaw_callback, cancel_callback=self.cancel_callback)

        #declare subcriptions
        self.status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', self.vehicle_status_callback, qos_profile)
        self.global_position_sub = self.create_subscription(VehicleGlobalPosition, '/fmu/out/vehicle_global_position', self.vehicle_global_position_callback, qos_profile)
        self.local_position_sub = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self.vehicle_local_position_callback, qos_profile)
        self.attitude_sub = self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.attitude_callback, qos_profile)
        self.vector_receiver = self.create_subscription(VelocityVectors, NAMESPACE+'velocity_vectors', self.velocity_control_callback ,10)

        # # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)

        #declare Main loop in timer
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("starting knr drone handler px4")

        #define the variables
        self.px4_alive_flag = False #this flag will check if we received any data from  drone
        self.px4_watchdog = 0 #this variable will check if we still getting new topics
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arm_state = VehicleStatus.ARMING_STATE_ARMED
        self.failsafe = False # if true that means drone is in failsafe mode
        self.flightCheck = False #if true drone can be armed
        self.offboard_setpoint_counter = 0
        self.flight_mode_flag = False #False = position points trajectory mode
                                      #True = velocity vectors trajectory mode

        #data structure
        self.global_position = GlobalPosition()
        self.local_position = LocalPosition()
        self.dev_mode = False

    #for some reason to work with px4 offboard (guided) mode FC must recevied with minimum 2Hz control topic
    #to know the companion computer is alive
    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        if self.flight_mode_flag:
            msg.position = False
            msg.velocity = True
        else:
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
        if self.px4_alive_flag:
            self.publish_offboard_control_heartbeat_signal()

            if self.offboard_setpoint_counter < 11:
                self.offboard_setpoint_counter += 1
            
            if self.offboard_setpoint_counter == 10:
                self.get_logger().info("Vehicle is ready to be set into offboard mode")
        
            if self.get_clock().now().nanoseconds - self.px4_watchdog > 1e9:
                self.px4_alive_flag = False
                self.get_logger().warn("Vehicle is missing ERROR PX4 not found")
                self.offboard_setpoint_counter = 0

    #declare subscribtions
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

    def vehicle_global_position_callback(self, msg):
        self.global_position.alt = msg.alt
        self.global_position.lat = msg.lat
        self.global_position.lon = msg.lon

    def vehicle_local_position_callback(self, msg):
        self.local_position.x = msg.x
        self.local_position.y = msg.y
        self.local_position.z = msg.z

        self.local_position.vx = msg.vx
        self.local_position.vy = msg.vy
        self.local_position.vz = msg.vz

        self.local_position.heading = msg.heading

        self.px4_alive_flag = True
        self.px4_watchdog = self.get_clock().now().nanoseconds
    
    def velocity_control_callback(self, msg):
        if self.flight_mode_flag:
            self.get_logger().info(f"i receive x:{msg.vx} y:{msg.vy} z:{msg.vz}")
            real_vy = -msg.vy
            cos_yaw = np.cos(self.trueYaw)
            sin_yaw = np.sin(self.trueYaw)
            velocity_world_x = (-msg.vx * cos_yaw - real_vy * sin_yaw)
            velocity_world_y = (-msg.vx * sin_yaw + real_vy * cos_yaw)
            
            velocity_world_z = msg.vz

            self.publish_velocity_setpoint(velocity_world_x , velocity_world_y, velocity_world_z)

    def attitude_callback(self, msg):
        orientation_q = msg.q

        #trueYaw is the drones current yaw value
        self.trueYaw = -(np.arctan2(2.0*(orientation_q[3]*orientation_q[0] + orientation_q[1]*orientation_q[2]), 
                                  1.0 - 2.0*(orientation_q[0]*orientation_q[0] + orientation_q[1]*orientation_q[1])))


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
    
    def publish_position_setpoint(self, x: float, y: float, z: float, yaw = "ORIGINAL"):
        """Publish the trajectory setpoint in the NED frame"""
        if yaw == "ORIGINAL":
            yaw = self.local_position.heading
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")

    def publish_velocity_setpoint(self, x: float = 0.0, y: float = 0.0, z: float = 0.0, yaw_speed: float = 0.0):
        """Publish the trajectory in velocity vectors in the NED frame"""
        msg = TrajectorySetpoint()
        msg.velocity = [x, y, z]
        msg.position = [float('nan'), float('nan'), float('nan')]
        msg.yaw = float('nan')
        msg.yawspeed = yaw_speed
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing velocity vectors {[x, y, z]} m/s, yaw_speed {yaw_speed} deg/s")

    def change_flight_mode_flag(self):
        if self.flight_mode_flag:
            self.flight_mode_flag = False
        else:
            self.flight_mode_flag = True

    #declare services callbback
    def set_mode_callback(self, request, response):
        self.get_logger().info("zmieniam tryb")
        if (request.mode == 'GUIDED'):
            self.get_logger().info("wchodze w guided")
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        if (request.mode == 'LAND'):
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        if (request.mode == 'RTL'):
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)

        response = SetMode.Response()
        return response
    
    def toggle_velocity_control(self, request, response):
        self.change_flight_mode_flag()
        response.result = self.flight_mode_flag

        return response
    
    #declare actions callback
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
        result = Arm.Result()
        result.result = 1

        return result
    
    def takeoff_callback(self, goal_handle):
        feedback_msg = Takeoff.Feedback()

        self.publish_position_setpoint(self.local_position.x, self.local_position.y, -goal_handle.request.altitude)

        while self.global_position.alt <= goal_handle.request.altitude:
            self.publish_position_setpoint(self.local_position.x, self.local_position.y, -goal_handle.request.altitude)
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Takeoff.Result()

            feedback_msg.altitude = self.global_position.alt
            self.get_logger().info(f"Altitude: {feedback_msg.altitude}")
            time.sleep(1)

        self.get_logger().info("Reached target altitude")
        
        goal_handle.succeed()
        result = Takeoff.Result()
        result.result = 1

        return result
    
    def goto_global_action(self, goal_handle):
        self.get_logger().info(f'-- Goto global action registered. Destination in global frame: --')

        request_position = GlobalPosition()
        request_position.lat = goal_handle.request.lat
        request_position.lon = goal_handle.request.lon
        request_position.alt = goal_handle.request.alt

        self.get_logger().info(f'Latitude: {request_position.lat}')
        self.get_logger().info(f'Longitude: {request_position.lon}')
        self.get_logger().info(f'Altitude: {request_position.alt}')

        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_REPOSITION, param1 = -1.0, param2 = 1.0,
                                     param5 = request_position.lat, param6 = request_position.lon, param7 = request_position.alt)

        feedback_msg = GotoGlobal.Feedback()
        feedback_msg.distance = self.get_distance_global(self.global_position, request_position)

        self.get_logger().info(f"Distance remaining: {feedback_msg.distance} m")

        while feedback_msg.distance>2:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return GotoGlobal.Result()

            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_REPOSITION, param1 = -1.0, param2 = 1.0,
                                     param5 = request_position.lat, param6 = request_position.lon, param7 = request_position.alt)
            feedback_msg.distance = self.get_distance_global(self.global_position, request_position)
            self.get_logger().info(f"Distance remaining: {feedback_msg.distance} m")
            time.sleep(1)

        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.publish_position_setpoint(self.local_position.x, 
                                       self.local_position.y,
                                       -request_position.alt)
        goal_handle.succeed()
        result = GotoGlobal.Result()
        result.result = 1

        return result
    
    def get_distance_global(self, aLocation1: GlobalPosition, aLocation2: GlobalPosition):
        coord1 = (aLocation1.lat, aLocation1.lon)
        coord2 = (aLocation2.lat, aLocation2.lon)

        return hv.haversine(coord1, coord2)*1000 # because we want it in metres
    
    ## ACTION CALLBACKS
    def goto_relative_action(self, goal_handle):
        self.get_logger().info(f'-- Goto relative action registered. Destination in local frame: --')

        north = self.local_position.x + goal_handle.request.north
        east = self.local_position.y + goal_handle.request.east
        down = self.local_position.z + goal_handle.request.down
        #velocity = goal_handle.request.velocity
        destination = LocalPosition()
        destination.x = north
        destination.y = east
        destination.z = down

        self.get_logger().info(f'North: {destination.x}')
        self.get_logger().info(f'East: {destination.y}')
        self.get_logger().info(f'Down: {destination.z}')

        self.publish_position_setpoint(destination.x, destination.y, destination.z)

        feedback_msg = GotoRelative.Feedback()
        feedback_msg.distance = self.calculate_remaining_distance_rel(destination)
        self.get_logger().info(f"Distance remaining: {feedback_msg.distance} m")

        while feedback_msg.distance>0.5:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal goto rel canceled')
                return GotoRelative.Result()

            self.publish_position_setpoint(destination.x, destination.y, destination.z)
            feedback_msg.distance = self.calculate_remaining_distance_rel(destination)
            time.sleep(1)

        goal_handle.succeed()
        result = GotoRelative.Result()
        result.result=1

        return result
    
    def calculate_remaining_distance_rel(self, destination: LocalPosition):
        dnorth = destination.x - self.local_position.x
        deast = destination.y - self.local_position.y
        ddown = destination.z - self.local_position.z
        return math.sqrt(dnorth*dnorth + deast*deast + ddown*ddown)
    

    def yaw_callback(self, goal_handle):
        self.get_logger().info(f'-- Set yaw action registered. --')
        self.__relative = goal_handle.request.relative
        setted_yaw = goal_handle.request.yaw
        actual_yaw = self.local_position.heading
        YAW_SPEED = 0.2
        cw = YAW_SPEED

        requested_yaw = self.calc_yaw(setted_yaw, actual_yaw)

        if requested_yaw<0:
            requested_yaw+=6.283185
        if setted_yaw < 0:
            setted_yaw = -setted_yaw
            cw = -cw
        yaw_deg = requested_yaw / 3.141592 * 180
        
        self.get_logger().info(f'd')
        # self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_CONDITION_YAW, param1 = yaw_deg, param3 = float(cw), param4 = 0.0)
        
        if not self.flight_mode_flag:
            prev_flight_mode_flag = self.flight_mode_flag
            self.change_flight_mode_flag()

        self.publish_velocity_setpoint(yaw_speed=cw)

        feedback_msg = SetYawAction.Feedback()
        feedback_msg.angle = self.calc_remaning_yaw(requested_yaw, actual_yaw, cw)
        self.get_logger().info(f"Angle remainig: {feedback_msg.angle}")


        while feedback_msg.angle > 0.5:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return SetYawAction.Result()

            # self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_CONDITION_YAW, param1 = yaw_deg, param3 = float(cw), param4 = 0.0)
            self.publish_velocity_setpoint(yaw_speed=cw)
            actual_yaw = self.local_position.heading
            feedback_msg.angle = self.calc_remaning_yaw(requested_yaw, actual_yaw, cw)
            self.get_logger().info(f"Angle remainig: {feedback_msg.angle}")
            time.sleep(1)

        if not prev_flight_mode_flag:
            self.change_flight_mode_flag()

        # self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        # self.publish_position_setpoint(self.local_position.x, 
        #                                self.local_position.y,
        #                                self.local_position.z)

        # self.get_logger().info(f"Angle remainig: {feedback_msg.angle}")
        goal_handle.succeed()
        result = SetYawAction.Result()
        result.result = 1

        return result
    
    def calc_yaw(self, yaw: float, actual_yaw: float)->float:
        if not self.__relative:
            return yaw
        return yaw+actual_yaw
    
    def calc_remaning_yaw(self, yaw: float, actual_yaw: float, cw)->float:
        if cw > 0:
            if actual_yaw < 0:
                actual_yaw = 2*math.pi + actual_yaw
            return abs(yaw-actual_yaw)
        if actual_yaw < 0:
            actual_yaw = 2*math.pi - actual_yaw
        return abs(-yaw+actual_yaw)

    #special method to cancel the action
    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

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
