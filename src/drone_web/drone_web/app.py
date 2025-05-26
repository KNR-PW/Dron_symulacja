import threading
import time
from flask import Flask, request, jsonify
from rclpy.executors import MultiThreadedExecutor

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from drone_interfaces.srv import TurnOnVideo, TurnOffVideo
from drone_interfaces.srv import (
    GetLocationRelative,
    GetAttitude,
    SetMode,
    SetSpeed,
)
from drone_interfaces.msg import Telemetry
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

        # --- Service clients ---
        self._mode_client = self.create_client(SetMode, 'set_mode')
        self._gps_client  = self.create_client(GetLocationRelative, 'get_location_relative')
        self._atti_client = self.create_client(GetAttitude, 'get_attitude')
        self._speed_client = self.create_client(SetSpeed, 'set_speed')
        self._start_video_client = self.create_client(TurnOnVideo, 'turn_on_video')
        self._stop_video_client = self.create_client(TurnOffVideo, 'turn_off_video')
        # self._wait_for_service(self._mode_client, 'set_mode')
        # self._wait_for_service(self._gps_client, 'get_location_relative')
        # self._wait_for_service(self._atti_client, 'get_attitude')
        # self._wait_for_service(self._speed_client, 'set_speed')
        # self._wait_for_service(self._start_video_client, 'turn_on_video')
        # self._wait_for_service(self._stop_video_client, 'turn_off_video')


        # --- Action clients ---
        self._arm_client    = ActionClient(self, Arm, 'Arm')
        self._takeoff_client = ActionClient(self, Takeoff, 'takeoff')
        self._goto_rel_client = ActionClient(self, GotoRelative, 'goto_relative')
        self._goto_glob_client = ActionClient(self, GotoGlobal, 'goto_global')
        self._yaw_client     = ActionClient(self, SetYawAction, 'Set_yaw')

        # --- Telemetry subscriber ---
        self.create_subscription(Telemetry, 'telemetry', self._telemetry_cb, 10)

        # --- State & fail‑safe ---
        self._busy = False
        self._alarm = False
        self._voltage_spikes = 0
        self._voltage_threshold = 12.0

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

    def takeoff(self, altitude: float) -> bool:
        self.get_logger().info(f'Taking off to {altitude} m')
        goal = Takeoff.Goal()
        goal.altitude = altitude
        return self._send_action(self._takeoff_client, goal)

    def send_goto_relative(self, north: float, east: float, down: float) -> bool:
        self.get_logger().info(f'Moving relative N:{north}, E:{east}, D:{down}')
        goal = GotoRelative.Goal(north=north, east=east, down=down)
        return self._send_action(self._goto_rel_client, goal)

    def send_goto_global(self, lat: float, lon: float, alt: float) -> bool:
        self.get_logger().info(f'Moving global LAT:{lat}, LON:{lon}, ALT:{alt}')
        goal = GotoGlobal.Goal(lat=lat, lon=lon, alt=alt)
        return self._send_action(self._goto_glob_client, goal)

    def send_shoot(self, color: str) -> bool:
        self.get_logger().info(f'Shooting color {color}')
        goal = Shoot.Goal(color=color)
        return self._send_action(self._shoot_client, goal)

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

    def _telemetry_cb(self, msg: Telemetry):
        # Unpack telemetry message and save as attributes
        self.battery_percentage = msg.battery_percentage
        self.battery_voltage = msg.battery_voltage
        self.battery_current = msg.battery_current
        self.lat = msg.lat
        self.lon = msg.lon
        self.alt = msg.alt
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



app = Flask(__name__)
drone = None

@app.route('/takeoff', methods=['POST'])
def takeoff():
    try:
        data = request.get_json()
        altitude = float(data.get('altitude', 2.0))  # default to 2m if not provided

        success = drone.arm() and drone.takeoff(altitude)
        return jsonify({'success': success}), (200 if success else 500)
    except Exception as e:
        return jsonify({'error': str(e)}), 500

@app.route('/goto_relative', methods=['POST'])
def goto_relative():
    data = request.get_json()
    north = float(data.get('north', 0.0))
    east = float(data.get('east', 0.0))
    down = float(data.get('down', 0.0))

    success = drone.send_goto_relative(north, east, down)
    return jsonify({'success': success}), (200 if success else 500)

@app.route('/telemetry', methods=['GET'])
def telemetry():
    gps = drone.get_gps()
    yaw = drone.get_yaw()
    if gps is None:
        return jsonify({'error': 'Failed to get telemetry'}), 500

    # Get additional telemetry values from the drone object
    battery_percentage = getattr(drone, 'battery_percentage', None)
    battery_voltage = getattr(drone, 'battery_voltage', None)
    battery_current = getattr(drone, 'battery_current', None)
    lat = getattr(drone, 'lat', None)
    lon = getattr(drone, 'lon', None)
    alt = getattr(drone, 'alt', None)
    flight_mode = getattr(drone, 'flight_mode', None)

    return jsonify({
        'north': gps[0],
        'east': gps[1],
        'down': gps[2],
        'yaw': yaw,
        'battery_percentage': battery_percentage,
        'battery_voltage': battery_voltage,
        'battery_current': battery_current,
        'lat': lat,
        'lon': lon,
        'alt': alt,
        'flight_mode': flight_mode
    })

def ros_spin_thread():
    rclpy.init()
    global drone
    drone = DroneController()
    executor = MultiThreadedExecutor()
    executor.add_node(drone)
    executor.spin()
    drone.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    thread = threading.Thread(target=ros_spin_thread, daemon=True)
    thread.start()
    time.sleep(2)  # Give time to initialize ROS 2 node
    app.run(host='0.0.0.0', port=5002)
