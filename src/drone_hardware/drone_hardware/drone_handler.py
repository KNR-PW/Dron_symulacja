import rclpy
from dronekit import connect, VehicleMode, LocationLocal, LocationGlobalRelative
from pymavlink import mavutil

import argparse
import time
import math

from rclpy.node import Node
from rclpy.action import ActionServer,  CancelResponse,  GoalResponse
from rclpy.executors import MultiThreadedExecutor


from drone_interfaces.msg import Telemetry, VelocityVectors
from drone_interfaces.srv import GetAttitude, GetLocationRelative, SetServo, SetMode, SetSpeed, GetGpsPos, ToggleVelocityControl
from drone_interfaces.action import GotoRelative, GotoGlobal, Arm, Takeoff, SetYawAction

import haversine as hv
class DroneHandler(Node):
    def __init__(self):
        super().__init__('drone_handler')
        self.vehicle = None

        ## DECLARE PARAMETERS
        #---------------------'  give here ip to flight controler   '
        self.declare_parameter('fc_ip', '/dev/ttyACM0')
        self.declare_parameter('dev', 'false')

        ## DECLARE NAMESPACE
        NAMESPACE = 'knr_hardware/'

        ## DECLARE SERVICES
        self.attitude = self.create_service(GetAttitude, NAMESPACE+'get_attitude', self.get_attitude_callback)
        self.gps = self.create_service(GetLocationRelative, NAMESPACE+'get_location_relative', self.get_location_relative_callback)
        self.gps_abs = self.create_service(GetGpsPos, NAMESPACE+'get_gps', self.get_gps_callback)
        self.servo = self.create_service(SetServo, NAMESPACE+'set_servo', self.set_servo_callback)
        self.create_service(SetSpeed, NAMESPACE+'set_speed', self.set_speed_callback)
        self.mode = self.create_service(SetMode, NAMESPACE+'set_mode',self.set_mode_callback)
        
        ## DECLARE ACTIONS
        self.goto_rel = ActionServer(self, GotoRelative, NAMESPACE+'goto_relative', self.goto_relative_action, cancel_callback=self.cancel_callback)
        self.goto_global = ActionServer(self, GotoGlobal, NAMESPACE+'goto_global', self.goto_global_action, cancel_callback=self.cancel_callback)
        self.arm = ActionServer(self,Arm, NAMESPACE+'Arm',self.arm_callback)
        self.takeoff = ActionServer(self, Takeoff, NAMESPACE+'takeoff',self.takeoff_callback, cancel_callback=self.cancel_callback)
        self.yaw = ActionServer(self, SetYawAction, NAMESPACE+'Set_yaw', self.yaw_callback, cancel_callback=self.cancel_callback)

        #DECLARE PUBLISHER
        self.telemetry_publisher = self.create_publisher(Telemetry, NAMESPACE+'telemetry',10)
        
        #DECLARE VELOCITY CONTROL
        self.vector_receiver = self.create_subscription(VelocityVectors, NAMESPACE+'velocity_vectors', self.velocity_control_callback ,10)
        self._velocity_control_flag = False
        self.toggle_velocity_control_srv = self.create_service(ToggleVelocityControl, NAMESPACE+'toggle_v_control', self.toggle_velocity_control)

        # ONLY FOR TEST IF YOU SEE HERE SOMETHING UNCOMMENTED TELL THIS TO HIS CREATOR
        #self._counter = 0
        ## DRONE MEMBER VARIABLES
        self.state = "BUSY"
        self.__relative = False
        self.dev_mode = False

        ##CONNECT TO COPTER
        # parser = argparse.ArgumentParser(description='commands')
        # parser.add_argument('--connect', default='127.0.0.1:14550')
        # args = parser.parse_args()

        # connection_string = args.connect

        # ARDUPILOT
        # connection_string = '127.0.0.1:14550'
        # connection_string = None
        
        # WEBOTS
        connection_string = self.get_parameter('fc_ip').get_parameter_value().string_value
        dev = self.get_parameter('dev').get_parameter_value().string_value
        if dev == "true":
            self.dev_mode = True
            self.get_logger().warn("DEV MODE is enabled.")
        #connection_string = 'tcp:127.0.0.1:5762'
        sitl = None


        baud_rate = 57600
        self.get_logger().info(f"Connecting with copter at {connection_string}...")
        # self.vehicle = connect(connection_string, baud=baud_rate, wait_ready=False) #doesnt work with wait_ready=True
        while self.vehicle is None:
            try:
                # RPI USB-C
                # self.vehicle = connect(connection_string, baud=115200, wait_ready=True)
                self.vehicle = connect(connection_string, baud=57600, wait_ready=False)
            except Exception as e:
                self.get_logger().error(f"Connecting failed with error: {e}")
                self.get_logger().info("Retrying to connect in {3} seconds...")
                time.sleep(3)

        try:
            self.vehicle.parameters['WP_YAW_BEHAVIOR'] = 0
            self.get_logger().info("Set WP_YAW_BEHAVIOR = 0 (Strafing enabled)")
        except Exception as e:
            self.get_logger().warn(f"Could not set WP_YAW_BEHAVIOR: {e}")
        
        if not self.dev_mode:
            self.wait_fc_ready()
        self.state = "OK"
        self.get_logger().info("\033[92mCopter connected, ready to arm")
        
        self.timer = self.create_timer(.1, self.telemetry_callback)

    def wait_fc_ready(self):
        fc_ready = False
        while not fc_ready:
            try:
                self.get_logger().info("Waiting for FC to be ready...")
                if self.vehicle.is_armable:
                    fc_ready = True
                    self.get_logger().info("\033[92mFC is ready")
                else:
                    time.sleep(1)
            except Exception as e:
                self.get_logger().info(f"Error while waiting for FC to be ready: {e}")
                time.sleep(1)

    def __del__(self):
        if self.vehicle:
            self.vehicle.mode=VehicleMode("RTL")

    ## INTERNAL HELPER METHODS
    def goto_position_target_local_ned( self, north, east, down=-1):
        if down == -1:
            down = self.vehicle.location.local_frame.down

        #type_mask = 0b0000011111000000
        type_mask = 0b0000011111111000
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
            type_mask, # type_mask (only positions enabled)
            north, east, down, # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
            0, 0, 0, # x, y, z velocity in m/s  (not used)
            0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
        # send command to vehicle
        self.vehicle.send_mavlink(msg)

    def goto_position_target_global_int(self, location, yaw_mode=1):
        if yaw_mode == 0:   
        # dont rotate
            mask = 0b1111011111111000
        else:
        # face move direction
            mask = 0b0000111111111000
            
        msg = self.vehicle.message_factory.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
        mask, # type_mask (only speeds enabled)
        int(location.lat*1e7), # lat_int - X Position in WGS84 frame in 1e7 * meters
        int(location.lon*1e7), # lon_int - Y Position in WGS84 frame in 1e7 * meters
        int(location.alt), # alt - Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
        0, # X velocity in NED frame in m/s
        0, # Y velocity in NED frame in m/s
        0, # Z velocity in NED frame in m/s
        0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

        # send command to vehicle
        self.vehicle.send_mavlink(msg)

    def get_gps_callback(self, request, response):
        self.get_logger().info(f"-- Get GPS service called --")
        try:
            if self.vehicle.location.global_frame:
                lat = self.vehicle.location.global_frame.lat
                lon = self.vehicle.location.global_frame.lon
                alt = self.vehicle.location.global_frame.alt
        except Exception as e:
            self.get_logger().error(f"Error in get_gps_callback: {e}")
            lat = 0
            lon = 0
            alt = 0
        response.lat = float(lat)
        response.lon = float(lon)
        response.alt = float(alt)
        return response

        
    def set_yaw(self, yaw, cw ,relative=False):
        
        yaw = yaw / 3.141592 * 180
        if abs(self.vehicle.attitude.yaw - yaw) > 3.141592:
            dir = 1 if self.vehicle.attitude.yaw < yaw else -1
        else:
            dir = 1 if self.vehicle.attitude.yaw > yaw else -1
        if relative:
            is_relative=1 #yaw relative to direction of travel
        else:
            is_relative=0 #yaw is an absolute angle
        # create the CONDITION_YAW command using command_long_encode()
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,        # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
            0,           #confirmation
            yaw,         # param 1, yaw in degrees
            0,           # param 2, yaw speed deg/s
            cw,           # param 3, direction -1 ccw, 1 cw
            is_relative, # param 4, relative offset 1, absolute angle 0
            0, 0, 0)     # param 5 ~ 7 not used
        # send command to vehicle
        self.vehicle.send_mavlink(msg)

    def set_servo(self, servo_id, pwm):
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,          # time_boot_ms (not used)
            0,   # target system, target component
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO, #command
            0,          #not used
            servo_id,   #number of servo instance
            pwm,        #pwm value for servo control
            0,0,0,0) #not used
        # send command to vehicle
        self.vehicle.send_mavlink(msg)

    def calculate_remaining_distance_rel(self, location):
        dnorth = location.north - self.vehicle.location.local_frame.north
        deast = location.east - self.vehicle.location.local_frame.east
        ddown = location.down - self.vehicle.location.local_frame.down
        return math.sqrt(dnorth*dnorth + deast*deast + ddown*ddown)
    
    def calculate_remaining_distance_global(self, location):
        dlat = (location.lat - self.vehicle.location.global_relative_frame.lat) * 1.113195e5 ## lat/lon to meters convert magic number
        dlon = (location.lon - self.vehicle.location.global_relative_frame.lon) * 1.113195e5 ## lat/lon to meters convert magic number
        ddown = location.alt - self.vehicle.location.global_relative_frame.alt
        return math.sqrt(dlat*dlat + dlon*dlon + ddown*ddown)

    def get_distance_global(self, aLocation1, aLocation2):
        coord1 = (aLocation1.lat, aLocation1.lon)
        coord2 = (aLocation2.lat, aLocation2.lon)

        return hv.haversine(coord1, coord2)*1000 # because we want it in metres
    ## SERVICE CALLBACKS
    def get_attitude_callback(self, request, response):
        temp = self.vehicle.attitude
        response.roll=temp.roll
        response.pitch=temp.pitch
        response.yaw=temp.yaw
        self.get_logger().info(f"-- Get attitude service called --")
        self.get_logger().info(f"Roll: {response.roll}")
        self.get_logger().info(f"Pitch: {response.pitch}")
        self.get_logger().info(f"Yaw: {response.yaw}")
        return response
    
    def get_location_relative_callback(self, request, response):
        temp = self.vehicle.location.local_frame
        response.north = temp.north or 0.0
        response.east = temp.east or 0.0
        response.down = temp.down or 0.0
        self.get_logger().info(f"-- Get location relative service called --")
        self.get_logger().info(f"North: {response.north}")
        self.get_logger().info(f"East: {response.east}")
        self.get_logger().info(f"Down: {response.down}")
        return response

    def set_servo_callback(self, request, response):
        self.set_servo(request.servo_id, request.pwm)
        response = SetServo.Response()
        return response
    
    def set_mode_callback(self, request, response):
        self.vehicle.mode = VehicleMode(request.mode)
        response = SetMode.Response()
        return response

    ## ACTION CALLBACKS
    def goto_relative_action(self, goal_handle):
        self.get_logger().info(f'-- Goto relative action registered. Destination in local frame: --')

        north = self.vehicle.location.local_frame.north + goal_handle.request.north
        east = self.vehicle.location.local_frame.east + goal_handle.request.east
        down = self.vehicle.location.local_frame.down + goal_handle.request.down
        #velocity = goal_handle.request.velocity
        destination = LocationLocal(north, east, down)

        self.get_logger().info(f'North: {destination.north}')
        self.get_logger().info(f'East: {destination.east}')
        self.get_logger().info(f'Down: {destination.down}')

        self.state = "BUSY"

        self.goto_position_target_local_ned(destination.north, destination.east, destination.down)

        feedback_msg = GotoRelative.Feedback()
        feedback_msg.distance = self.calculate_remaining_distance_rel(destination)
        self.get_logger().info(f"Distance remaining: {feedback_msg.distance} m")

        while feedback_msg.distance>0.5:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal goto rel canceled')
                return GotoRelative.Result()

            feedback_msg.distance = self.calculate_remaining_distance_rel(destination)
            # self.get_logger().info(f"Distance remaining: {feedback_msg.distance} m")
            time.sleep(1)

        goal_handle.succeed()
        self.state = "OK"
        result = GotoRelative.Result()
        result.result=1

        return result
    
    def goto_global_action(self, goal_handle):
        self.get_logger().info(f'-- Goto global action registered. Destination in global frame: --')

        # lat = self.vehicle.location.global_relative_frame.lat + goal_handle.request.lat
        # lon = self.vehicle.location.global_relative_frame.lon + goal_handle.request.lon
        # alt = self.vehicle.location.global_relative_frame.alt + goal_handle.request.alt
        lat = goal_handle.request.lat
        lon = goal_handle.request.lon
        alt = goal_handle.request.alt
        destination=LocationGlobalRelative(lat,lon,alt)

        self.get_logger().info(f'Latitude: {destination.lat}')
        self.get_logger().info(f'Longitude: {destination.lon}')
        self.get_logger().info(f'Altitude: {destination.alt}')

        self.state = "BUSY"

        # self.goto_position_target_local_ned(destination.lat, destination.lon, destination.alt)
        self.goto_position_target_global_int(destination)

        feedback_msg = GotoGlobal.Feedback()
        feedback_msg.distance = self.get_distance_global(self.vehicle.location.global_relative_frame, destination)
        self.get_logger().info(f"Distance remaining: {feedback_msg.distance} m")

        while feedback_msg.distance>2:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return GotoGlobal.Result()

            feedback_msg.distance = self.get_distance_global(self.vehicle.location.global_relative_frame, destination)
            self.get_logger().info(f"Distance remaining: {feedback_msg.distance} m")
            time.sleep(1)

        goal_handle.succeed()
        self.state = "OK"
        result = GotoGlobal.Result()
        result.result = 1

        return result
    
    def arm_callback(self, goal_handle):
        self.get_logger().info(f'-- Arm action registered --')
        self.state = "BUSY"
        feedback_msg = Arm.Feedback()
        
        while self.vehicle.is_armable==False:
            if self.dev_mode:
                self.get_logger().info("DEV MODE is enabled. Skipping armable check.")
                break
            feedback_msg.feedback = "Waiting for vehicle to become armable..."
            self.get_logger().info(feedback_msg.feedback)
            time.sleep(1)

        self.vehicle.armed=True
        while self.vehicle.armed==False:
            feedback_msg.feedback = "Waiting for drone to become armed..."
            self.get_logger().info(feedback_msg.feedback)
            time.sleep(1)

        feedback_msg.feedback = "Vehicle is now armed."
        self.get_logger().info(feedback_msg.feedback)

        self.state = "OK"
        
        goal_handle.succeed()
        self.state = "OK"
        result = Arm.Result()
        result.result = 1

        return result
    
    def takeoff_callback(self, goal_handle):
        feedback_msg = Takeoff.Feedback()

        self.state = "OK"
        self.vehicle.simple_takeoff(goal_handle.request.altitude)

        # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
        #  after Vehicle.simple_takeoff will execute immediately).
        while self.vehicle.location.global_relative_frame.alt <= goal_handle.request.altitude * 0.80:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Takeoff.Result()

            feedback_msg.altitude = self.vehicle.location.global_relative_frame.alt
            self.get_logger().info(f"Altitude: {feedback_msg.altitude}")
            time.sleep(1)

        self.get_logger().info("Reached target altitude")
        
        goal_handle.succeed()
        self.state = "OK"
        result = Takeoff.Result()
        result.result = 1

        return result

    def publish_gps_pos(self):
        gps_msg = GPSPos()
        GPSPos.gps_position = vehicle.location.local_frame
        self.gps_publisher.publish(GPSPos)

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


    def yaw_callback(self, goal_handle):
        self.get_logger().info(f'-- Set yaw action registered. --')
        self.__relative = goal_handle.request.relative
        setted_yaw = goal_handle.request.yaw
        actual_yaw = self.vehicle.attitude.yaw
        cw = 1


        requested_yaw = self.calc_yaw(setted_yaw, actual_yaw)

        if requested_yaw<0:
            requested_yaw+=6.283185

        if setted_yaw < 0:
            setted_yaw = -setted_yaw
            cw = -1

        self.state = "BUSY"
        self.set_yaw(requested_yaw, cw)

        feefback_msg = SetYawAction.Feedback()
        feefback_msg.angle = self.calc_remaning_yaw(requested_yaw, actual_yaw, cw)
        self.get_logger().info(f"Angle remainig: {feefback_msg.angle}")

        #self._counter += 1

        while feefback_msg.angle > 0.5:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return SetYawAction.Result()

            feefback_msg.angle = self.calc_remaning_yaw(requested_yaw, actual_yaw, cw)
            actual_yaw = self.vehicle.attitude.yaw
            self.get_logger().info(f"Angle remainig: {feefback_msg.angle}")
            time.sleep(1)

        self.get_logger().info(f"Angle remainig: {feefback_msg.angle}")
        goal_handle.succeed()
        self.state = "OK"
        result = SetYawAction.Result()
        result.result = 1

        return result
    
    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT
    
    def set_speed_callback(self, request, response):
        self.get_logger().info(f'-- Set speed service called --')
    
        speed = request.speed
        self.get_logger().info(f"Speed: {speed} m/s")
        
        self.vehicle.groundspeed = speed
        self.get_logger().info(f"Speed set to: {self.vehicle.groundspeed} m/s")
        response = SetSpeed.Response()
        return response

    def telemetry_callback(self):
        msg = Telemetry()
        try:
            # Battery
            msg.battery_percentage = self.vehicle.battery.level
            msg.battery_voltage = self.vehicle.battery.voltage
            msg.battery_current = self.vehicle.battery.current
            # GPS
            msg.lat = self.vehicle.location.global_relative_frame.lat
            msg.lon = self.vehicle.location.global_relative_frame.lon
            msg.alt = self.vehicle.location.global_relative_frame.alt

            # Attitude
            msg.roll = self.vehicle.attitude.roll
            msg.pitch = self.vehicle.attitude.pitch
            msg.yaw = self.vehicle.attitude.yaw

            # Flight mode
            msg.flight_mode = str(self.vehicle.mode)
            
            msg.speed = float(self.vehicle.groundspeed)  

            gf = self.vehicle.location.global_frame
            if gf:  # check that GPS is valid
                msg.global_lat = float(gf.lat)
                msg.global_lon = float(gf.lon)
                msg.global_alt = float(gf.alt)
            else:
                # If no GPS fix, default to zeros
                msg.global_lat = 0.0
                msg.global_lon = 0.0
                msg.global_alt = 0.0

            # 
            #if self._counter > 0:
            #    msg.battery_voltage = 11.5
            self.telemetry_publisher.publish(msg)
            #self._counter += 1
            #self.get_logger().info(f"battery :{self.vehicle.battery}")
        except Exception as e:
            self.get_logger().error(f"Error in telemetry callback: {e}")
            self.get_logger().info(f"ESC is not initialized yet:{self.vehicle.battery}")

    #velocty control definitions

    def send_global_velocity(self,velocity_x, velocity_y, velocity_z):
        """
        Move vehicle in direction based on specified velocity vectors.
        """
        msg = self.vehicle.message_factory.set_position_target_global_int_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED  , # frame
            0b0000111111000111, # type_mask (only speeds enabled)
            0, # lat_int - X Position in WGS84 frame in 1e7 * meters
            0, # lon_int - Y Position in WGS84 frame in 1e7 * meters
            0, # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
            # altitude above terrain if GLOBAL_TERRAIN_ALT_INT
            velocity_x, # X velocity in NED frame in m/s
            velocity_y, # Y velocity in NED frame in m/s
            velocity_z, # Z velocity in NED frame in m/s
            0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

        # send command to vehicle on 1 Hz cycle

        self.vehicle.send_mavlink(msg)


    def velocity_control_callback(self, msg):
        if self._velocity_control_flag:
            self.get_logger().info(f"i receive x:{msg.vx} y:{msg.vy} z:{msg.vz}")
            
            yaw = math.radians(self.vehicle.heading)

            v_n = msg.vx * math.cos(yaw) - msg.vy * math.sin(yaw)
            v_e = msg.vx * math.sin(yaw) + msg.vy * math.cos(yaw)
            v_d = msg.vz

            self.send_global_velocity(v_n, v_e, v_d)

    def toggle_velocity_control(self, request, response):

        self._velocity_control_flag = False if self._velocity_control_flag else True

        """if self._velocity_control_flag:
            self._velocity_control_flag = False
        else:
            self._velocity_control_flag = True
                    """
        response.result = self._velocity_control_flag

        return response

def main():
    rclpy.init()
    
    drone = DroneHandler()

    # rclpy.spin(drone)
    executor = MultiThreadedExecutor()
    executor.add_node(drone)
    executor.spin()

    drone.destroy_node()

    rclpy.shutdown()


if __name__ == 'main':
    main()