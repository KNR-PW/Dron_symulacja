import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from drone_interfaces.srv import TurnOnVideo, TurnOffVideo
from drone_interfaces.srv import (
    GetLocationRelative,
    GetAttitude,
    SetMode,
    SetSpeed,
    MakePhoto,
    ToggleVelocityControl

)
from drone_interfaces.msg import Telemetry, VelocityVectors
from drone_interfaces.action import (
    Arm,
    Takeoff,
    GotoRelative,
    GotoGlobal,
    SetYawAction
)

class DroneController(Node):

    def __init__(self, node_name: str = 'drone_controller'):
        super().__init__(node_name)
        # --- Declare KNR Namespaces ---
        NAMESPACE_HARDWARE = 'knr_hardware/'
        NAMESPACE_VIDEO = 'knr_video/'

        DEV = True

        # --- Service clients ---
        self._mode_client = self.create_client(SetMode, NAMESPACE_HARDWARE+'set_mode')
        self._gps_client  = self.create_client(GetLocationRelative, NAMESPACE_HARDWARE+'get_location_relative')
        self._atti_client = self.create_client(GetAttitude, NAMESPACE_HARDWARE+'get_attitude')
        self._speed_client = self.create_client(SetSpeed, NAMESPACE_HARDWARE+'set_speed')
        self._start_video_client = self.create_client(TurnOnVideo, NAMESPACE_VIDEO+'turn_on_video')
        self._stop_video_client = self.create_client(TurnOffVideo, NAMESPACE_VIDEO+'turn_off_video')
        self.toggle_velocity_control_cli = self.create_client(ToggleVelocityControl,NAMESPACE_HARDWARE+'toggle_v_control')
        self._set_guard_client = self.create_client(SetMode, 'set_brake_on_obstacle')
        self.velocity_publisher = self.create_publisher(VelocityVectors,NAMESPACE_HARDWARE+'velocity_vectors', 10)

        self._wait_for_service(self._mode_client, NAMESPACE_HARDWARE+'set_mode')
        if not DEV:
            self._wait_for_service(self._gps_client, NAMESPACE_HARDWARE+'get_location_relative')
            self._wait_for_service(self._atti_client, NAMESPACE_HARDWARE+'get_attitude')
            self._wait_for_service(self._speed_client, NAMESPACE_HARDWARE+'set_speed')
        self._photo_client = self.create_client(MakePhoto, '/mission_make_photo')
        # self._wait_for_service(self._start_video_client, 'turn_on_video')
        # self._wait_for_service(self._stop_video_client, 'turn_off_video')


        # --- Action clients ---
        self._arm_client    = ActionClient(self, Arm, NAMESPACE_HARDWARE+'Arm')
        self._takeoff_client = ActionClient(self, Takeoff, NAMESPACE_HARDWARE+'takeoff')
        self._goto_rel_client = ActionClient(self, GotoRelative, NAMESPACE_HARDWARE+'goto_relative')
        self._goto_glob_client = ActionClient(self, GotoGlobal, NAMESPACE_HARDWARE+'goto_global')
        self._yaw_client     = ActionClient(self, SetYawAction, NAMESPACE_HARDWARE+'Set_yaw')

        # --- Telemetry subscriber ---
        self.create_subscription(Telemetry, NAMESPACE_HARDWARE+'telemetry', self._telemetry_cb, 10)

        # --- State & fail‑safe ---
        self._busy = False
        self._alarm = False
        self._voltage_spikes = 0
        self._voltage_threshold = 12.0

        self.global_lat = 0.0
        self.global_lon = 0.0
        self.global_alt = 0.0
        self.lat = 0.0
        self.lon = 0.0
        self.alt = 0.0

        # --- MakeMissionPhoto zmienne ---
        self.photo_idx = 1

    def take_mission_photo(self, ext='jpg'):
        '''
        Takes one photo from drone and save in desire folder.
        It communicate with image_recorder_KT.py
        
        :param ext: Parameters to select in with filename extension you want to take photo.
        '''
        req = MakePhoto.Request()
        req.prefix = f'Zdj{self.photo_idx}'
        req.ext = ext
        future = self.photo_client.call_async(req)

        def _done(_):
            result = future.result()
            if result is None:
                self.get_logger().error('Brak odpowiedzi z mission_make_photo')
                return
            if result.success:
                self.get_logger().info(f'Zrobione: {req.prefix}.{ext}')
                self.photo_idx += 1
            else:
                self.get_logger().warn('Nie udało się zrobić zdjęcia')

        future.add_done_callback(_done)
        
    def _wait_for_service(self, client, name=""):
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'Waiting for {name} service...')

    def _set_mode(self, mode: str, timeout: float = 5.0) -> bool:
        req = SetMode.Request()
        req.mode = mode
        fut = self._mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        if fut.result() is None:
            self.get_logger().error(f'Failed to set mode to {mode}')
            return False
        self.get_logger().info(f'Mode set to: {mode}')
        return True

    def set_obstacle_avoidance(self, active: bool) -> bool:
        """Enables or disables the obstacle avoidance guard."""
        req = SetMode.Request()
        req.mode = 'ON' if active else 'OFF'
        
        # Check if service is ready 
        if not self._set_guard_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Obstacle Avoidance service not available')
            return False
            
        fut = self._set_guard_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
        
        if fut.result() is None:
            self.get_logger().error('Failed to set obstacle avoidance mode')
            return False
            
        self.get_logger().info(f'Obstacle Avoidance set to: {req.mode}')
        return True

    def set_speed(self, speed) -> bool:
        '''
        Function to set desire maximum velocity of the drone in meters/seconds.
        To work properly you need type this method twice:
        once with 0 speed, and second with your speed.\n
        **For now only works in Ardupilot!!!**
        
        :param speed: Parameter to set velocity in meters/seconds.
        :return: True if methods work properly, 
        False if something dont work.
        :rtype: bool
        '''
        req = SetSpeed.Request()
        req.speed = float(speed)
        fut = self._speed_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
        if fut.result() is None:
            self.get_logger().error('Failed to set speed')
            return False
        self.get_logger().info(f'Speed set to: {speed}')
        return True

    def arm(self) -> bool:
        '''
        Method to arming the drone and set mode to:
        * Guided - Ardupilot
        * Offboard - PX4

        :return: True if methods work properly, 
        False if something dont work.
        :rtype: bool
        '''
        self.get_logger().info('Seting mode to guided...')
        if not self._set_mode('GUIDED'):
            return False
        self.get_logger().info('Arming drone...')
        return self._send_action(self._arm_client, Arm.Goal())

    def land(self) -> bool:
        '''
        Method to set LAND mode in drone to safely land.
        
        :return: True if methods work properly, 
        False if something dont work.
        :rtype: bool
        '''
        if not self._set_mode('LAND'):
            return False
        self.get_logger().info('Landing drone...')
        return True

    def rtl(self) -> bool:
        '''
        Method to set Return To Land (rtl) mode.
        This mode first set selected altitude in firmware, then
        goes to the start point where it was armed and land
        in there.
        
        :return: True if methods work properly, 
        False if something dont work.
        :rtype: bool
        '''
        if not self._set_mode('RTL'):
            return False
        self.get_logger().info('RTL mode...')
        return True

    def takeoff(self, altitude: float) -> bool:
        '''
        Method to set desire altitude after armed drone.
        You should use this method only when drone isn't in the air.
        If you want set higher altitude in the air you should use
        go_to_relative method.
        
        :param altitude: Param to set how high you want put your drone in meters.
        :type altitude: float
        :return: True if methods work properly, 
        False if something dont work.
        :rtype: bool
        '''
        self.get_logger().info(f'Taking off to {altitude} m')
        goal = Takeoff.Goal()
        goal.altitude = altitude
        return self._send_action(self._takeoff_client, goal)

    def send_goto_relative(self, north, east, down) -> bool:
        '''
        Method to control your drone in NED frame, which it means
        you how far it should go in north, east or down in meters
        according to compass.
        
        :param north: Parameter to set how far it will go in **+ N north** , **- N south** direction
        :type north: float
        :param east: Parameter to set how far it will go in **+ N east**, **- N west** direction
        :type east: float
        :param down: Parameter to set how far it will go in **+ N down**, **- N up** direction
        :type down: float
        :return: True if methods work properly, 
        False if something dont work.
        :rtype: bool
        '''
        north = float(north)
        east = float(east)
        down = float(down)
        self.get_logger().info(f'Moving relative N:{north}, E:{east}, D:{down}')
        goal = GotoRelative.Goal(north=north, east=east, down=down)
        return self._send_action(self._goto_rel_client, goal)

    def send_goto_global(self, lat, lon, alt) -> bool:
        '''
        Method to set desire GPS point where drone should go in GCS system.
        
        :param lat: Parameter to set latitude.
        :type lat: float
        :param lon: Parameter to set longitude.
        :type lon: float
        :param alt: Parameter to set altitude.
        :param alt: float
        :return: True if methods work properly, 
        False if something dont work.
        :rtype: bool
        '''
        lat = float(lat)
        lon = float(lon)
        alt = float(alt)
        self.get_logger().info(f'Moving global LAT:{lat}, LON:{lon}, ALT:{alt}')
        goal = GotoGlobal.Goal(lat=lat, lon=lon, alt=alt)
        return self._send_action(self._goto_glob_client, goal)

    # def send_shoot(self, color: str) -> bool:
    #     self.get_logger().info(f'Shooting color {color}')
    #     goal = Shoot.Goal(color=color)
    #     return self._send_action(self._shoot_client, goal)

    def send_set_yaw(self, yaw: float, relative: bool = True) -> bool:
        '''
        Method to set angle of rotation in yaw axis in radians.
        Yaw in:
        * **+ N** means clockwise rotation 
        * **- N** counter clockwise rotation
        
        :param self: Description
        :param yaw: Parameter to set angle in **radians**
        :type yaw: float
        :param relative: Parameter to set if our rotation will be relative to our current position: True, or to global drone heading: False 
        :type relative: bool
        :return: True if methods work properly, 
        False if something dont work.
        :rtype: bool
        '''
        self.get_logger().info(f'Setting yaw to {yaw} rad, relative={relative}')
        goal = SetYawAction.Goal(yaw=yaw, relative=relative)
        return self._send_action(self._yaw_client, goal)

    def _send_action(self, client: ActionClient, goal_msg) -> bool:
        # Wait for action server
        while not client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn(f'Waiting for {client._action_name} server...')
        # Lock state and send
        self._busy = True
        send_future = client.send_goal_async(goal_msg)
        send_future.add_done_callback(lambda f: self._on_action_response(f, client))

        # Spin until result or emergency
        while True:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self._alarm:
                self.get_logger().error('Emergency, aborting')
                return False
            if not self._busy:
                break
        return True

    def _on_action_response(self, send_future, client: ActionClient):
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'{client.action_name} goal rejected')
            self._busy = False
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self._on_action_result(f))

    def _on_action_result(self, result_future):
        status = result_future.result().status
        print(status)
        self._busy = False

    def get_gps(self):
        '''
        Method to get current gps location from drone. 
        
        :return: (north, east, down)
        :rtype: tuple
        '''
        req = GetLocationRelative.Request()
        fut = self._gps_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
        if fut.result() is None:
            self.get_logger().error('GPS service failed')
            return None
        return (fut.result().north, fut.result().east, fut.result().down)

    def get_yaw(self) -> float:
        '''
        Method to get actual yaw of drone.
        
        :return: Yaw angle in radians
        :rtype: float
        '''
        req = GetAttitude.Request()
        fut = self._atti_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
        if fut.result() is None:
            self.get_logger().error('Attitude service failed')
            return 0.0
        return fut.result().yaw

    def _telemetry_cb(self, msg: Telemetry):
        # Unpack telemetry message and save as attributes
        self.battery_percentage = msg.battery_percentage
        self.battery_voltage = msg.battery_voltage
        self.battery_current = msg.battery_current
        self.lat = msg.lat
        self.lon = msg.lon
        self.alt = msg.alt
        self.global_lat = msg.global_lat
        self.global_lon = msg.global_lon
        self.speed = msg.speed
        self.flight_mode = msg.flight_mode
        if msg.battery_voltage < self._voltage_threshold:
            self._voltage_spikes += 1
        else:
            self._voltage_spikes = 0
        if self._voltage_spikes >= 5 and not self._alarm:
            self.get_logger().warn('Low battery detected, emergency return')
            self._alarm = True

    def destroy_node(self):
        super().destroy_node()

    def start_video(self):
        '''
        Method to start video that is recording by video_recorder.py
        '''
        req = TurnOnVideo.Request()
        fut = self._start_video_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
        if fut.result() is None:
            self.get_logger().error('Failed to start video')
            return False
        self.get_logger().info(f'Start video')
        return True
    
    def stop_video(self):
        '''
        Method to stop video.
        
        '''
        req = TurnOffVideo.Request()
        fut = self._stop_video_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
        if fut.result() is None:
            self.get_logger().error('Failed to stop video')
            return False
        self.get_logger().info(f'Stop video')
        return True
    
# functions to fly by vectors
    def send_vectors(self, vx, vy, vz, yaw=0.0):
        '''
        Method to send velocity vector in FRD frame to drone_handler.
        WARNING Before you want use this method you should first use toggle_control method and
        after you done using velocity vector.
        
        :param vx: Vector in meters/seconds which is in **front** of drone direction
        :param vy: Vector in meters/seconds which is in **right** of drone direction
        :param vz: Vector in meters/seconds which is in **down** of drone direction
        :param yaw: Angle in radians which is in clockwise of drone
        '''
        vectors = VelocityVectors()
        vectors.vx = float(vx)
        vectors.vy = float(vy)
        vectors.vz = float(vz)
        vectors.yaw = float(yaw)
        self.velocity_publisher.publish(vectors)
        
    def toggle_control(self):
        '''
        Method to toggle control method in drone.
        By the default drone is control by GPS high level method for example:
        go_to_relative and go_to_global methods.
        If you use this control toggle now drone will listen for yours velocity vector and
        only for this control will react. 
        
        '''
        req = ToggleVelocityControl.Request()
        future = self.toggle_velocity_control_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10)
        if future.result().result:
            self.get_logger().info("turn into velocity control mode")
        else:
            self.get_logger().info("turn off velocity control mode")
        return future.result()
