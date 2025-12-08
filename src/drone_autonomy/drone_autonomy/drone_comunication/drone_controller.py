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
    def __init__(self):
        super().__init__('drone_controller')
        # --- Declare KNR Namespaces ---
        NAMESPACE_HARDWARE = 'knr_hardware/'
        NAMESPACE_VIDEO = 'knr_video/'

        # --- Service clients ---
        self._mode_client = self.create_client(SetMode, NAMESPACE_HARDWARE+'set_mode')
        self._gps_client  = self.create_client(GetLocationRelative, NAMESPACE_HARDWARE+'get_location_relative')
        self._atti_client = self.create_client(GetAttitude, NAMESPACE_HARDWARE+'get_attitude')
        self._speed_client = self.create_client(SetSpeed, NAMESPACE_HARDWARE+'set_speed')
        self._start_video_client = self.create_client(TurnOnVideo, NAMESPACE_VIDEO+'turn_on_video')
        self._stop_video_client = self.create_client(TurnOffVideo, NAMESPACE_VIDEO+'turn_off_video')
        self.toggle_velocity_control_cli = self.create_client(ToggleVelocityControl,NAMESPACE_HARDWARE+'toggle_v_control')
        self.velocity_publisher = self.create_publisher(VelocityVectors,NAMESPACE_HARDWARE+'velocity_vectors', 10)

        self._wait_for_service(self._mode_client, NAMESPACE_HARDWARE+'set_mode')
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

    def set_speed(self, speed) -> bool:
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
        self.get_logger().info('Seting mode to guided...')
        if not self._set_mode('GUIDED'):
            return False
        self.get_logger().info('Arming drone...')
        return self._send_action(self._arm_client, Arm.Goal())

    def land(self) -> bool:
        if not self._set_mode('LAND'):
            return False
        self.get_logger().info('Landing drone...')
        return True

    def rtl(self) -> bool:
        if not self._set_mode('RTL'):
            return False
        self.get_logger().info('RTL mode...')
        return True

    def takeoff(self, altitude: float) -> bool:
        self.get_logger().info(f'Taking off to {altitude} m')
        goal = Takeoff.Goal()
        goal.altitude = altitude
        return self._send_action(self._takeoff_client, goal)

    def send_goto_relative(self, north, east, down) -> bool:
        north = float(north)
        east = float(east)
        down = float(down)
        self.get_logger().info(f'Moving relative N:{north}, E:{east}, D:{down}')
        goal = GotoRelative.Goal(north=north, east=east, down=down)
        return self._send_action(self._goto_rel_client, goal)

    def send_goto_global(self, lat, lon, alt) -> bool:
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
        req = GetLocationRelative.Request()
        fut = self._gps_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
        if fut.result() is None:
            self.get_logger().error('GPS service failed')
            return None
        return (fut.result().north, fut.result().east, fut.result().down)

    def get_yaw(self) -> float:
        req = GetAttitude.Request()
        fut = self._atti_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
        if fut.result() is None:
            self.get_logger().error('Attitude service failed')
            return 0.0
        return fut.result().yaw
    
    def get_pitch(self) -> float:
        req = GetAttitude.Request()
        fut = self._atti_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
        if fut.result() is None:
            self.get_logger().error('Attitude service failed')
            return 0.0
        return fut.result().pitch

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
        req = TurnOnVideo.Request()
        fut = self._start_video_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
        if fut.result() is None:
            self.get_logger().error('Failed to start video')
            return False
        self.get_logger().info(f'Start video')
        return True
    
    def stop_video(self):
        req = TurnOffVideo.Request()
        fut = self._stop_video_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
        if fut.result() is None:
            self.get_logger().error('Failed to stop video')
            return False
        self.get_logger().info(f'Stop video')
        return True
    
# functions to fly by vectors
    def send_vectors(self, vx, vy, vz, yaw_rate=0.0):
        vectors = VelocityVectors()
        vectors.vx = float(vx)
        vectors.vy = float(vy)
        vectors.vz = float(vz)
        vectors.yaw = float(yaw_rate) # New field
        self.velocity_publisher.publish(vectors)
        
    def toggle_control(self):
        req = ToggleVelocityControl.Request()
        future = self.toggle_velocity_control_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10)
        if future.result().result:
            self.get_logger().info("true")
        else:
            self.get_logger().info("false")
        return future.result()
