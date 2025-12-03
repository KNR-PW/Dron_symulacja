#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import rclpy
from rclpy.node import Node
import math

from vision_msgs.msg import Detection2DArray
from drone_interfaces.msg import VelocityVectors, Telemetry
from drone_interfaces.srv import ToggleVelocityControl, SetGimbalAngle
from drone_autonomy.drone_comunication.drone_controller import DroneController

# TODO: fix aruco node (numpy)
# TODO: fix wrong world opening

def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class FollowDetections(DroneController):
    def __init__(self):
        super().__init__()

        # ---- Parametry ----
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('detections_topic', '/detections')
        self.declare_parameter('target_alt', 2.0)
        self.declare_parameter('kp', 0.05) # declared as input parameter, currently 0.6
        self.declare_parameter('deadband_px', 0)
        self.declare_parameter('max_vel', 1.0)
        self.declare_parameter('lowpass', 0.3)
        self.declare_parameter('lost_timeout', 0.8)  # s

        # Gimbal tracking params
        self.declare_parameter('gimbal_kp_deg', 0.7)     # degrees per normalized image unit
        self.declare_parameter('gimbal_min_deg', -15.0)
        self.declare_parameter('gimbal_max_deg', 90.0)
        self.declare_parameter('gimbal_rate_hz', 50.0)    # command rate

        self.img_w = int(self.get_parameter('image_width').value)
        self.img_h = int(self.get_parameter('image_height').value)
        self.cx = self.img_w / 2.0
        self.cy = self.img_h / 2.0
        self.kp = float(self.get_parameter('kp').value)
        self.deadband_px = int(self.get_parameter('deadband_px').value)
        self.max_vel = float(self.get_parameter('max_vel').value)
        self.lowpass = float(self.get_parameter('lowpass').value)
        self.lost_timeout = float(self.get_parameter('lost_timeout').value)
        self.detections_topic = str(self.get_parameter('detections_topic').value)
        self.target_alt = float(self.get_parameter('target_alt').value)

        # gimbal state
        self.gimbal_kp_deg = float(self.get_parameter('gimbal_kp_deg').value)
        self.gimbal_min_deg = float(self.get_parameter('gimbal_min_deg').value)
        self.gimbal_max_deg = float(self.get_parameter('gimbal_max_deg').value)
        self.gimbal_rate_hz = float(self.get_parameter('gimbal_rate_hz').value)
        # start with a safe downwards angle (match webots default)
        self.gimbal_angle_deg = 80.0

        self.roll = None
        self.pitch_rad = None
        self.yaw = None

        self.state = "OK"

        # ---- Subskrypcja markera ----
        # self.sub = self.create_subscription(MiddleOfAruco, self.detections_topic, self.on_marker, 10)
        self.sub = self.create_subscription(Detection2DArray, self.detections_topic, self.on_detection, 10)

        self.ex_px = 0.0
        self.ey_px = 0.0

        # Stan filtrowany błędu (normalized)
        self.ex_f = 0.0
        self.ey_f = 0.0
        self.last_seen = time.time()

        # Dodanie flag dotyczących osiągnięcia punktu celowego
        self.x_flag = False
        self.y_flag = False

        self.altitude = None
        self.telemetry_sub = self.create_subscription(
            Telemetry,
            '/knr_hardware/telemetry',
            self.telemetry_callback,
            10)

        self.get_logger().info(
            f"Start follow_detections: img=({self.img_w}x{self.img_h}) "
            f"kp={self.kp} max_vel={self.max_vel} deadband={self.deadband_px}px"
        )

        self.set_gimbal_cli = self.create_client(SetGimbalAngle, 'set_gimbal_angle')
        while not self.set_gimbal_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Gimbal control service unavailable, waiting...')

        # # GPS (get altitude) client — block until available (matches your example)
        # self.gps_cli = self.create_client(GetLocationRelative, 'knr_hardware/get_location_relative')
        # while not self.gps_cli.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('GPS service not available, waiting again...')

        # Attitude client — block until available (simple pattern from example)
        # self.atti_cli = self.create_client(GetAttitude, 'knr_hardware/get_attitude')
        # while not self.atti_cli.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('attitude service not available, waiting again...')
        # self._attitude_fail_count = 0

        # centering flag and gimbal timer (timer runs but only acts when centering=True)
        self.centering = False
        # ensure safe non-zero period
        period = max(0.01, 1.0 / float(self.gimbal_rate_hz))
        self.gimbal_timer = self.create_timer(period, self.gimbal_control_loop)
        # lightweight gimbal debug counter (log every N steps)
        self._gimbal_dbg_cnt = 0

        # set initial gimbal angle asynchronously (do not block)
        try:
            req0 = SetGimbalAngle.Request()
            req0.angle_degrees = float(self.gimbal_angle_deg)
            self.set_gimbal_cli.call_async(req0)
        except Exception:
            pass

    def telemetry_callback(self, msg):
        """Callback to update altitude from telemetry topic."""
        self.altitude = msg.alt
        self.roll = msg.roll
        self.pitch_rad = msg.pitch
        self.yaw = msg.yaw

        #Definiowanie misji

        # Arm & takeoff — jak w repo
        # try:
        #     self.arm()
        #     self.takeoff(self.target_alt)
        #     self.get_logger().info(f"Arm & takeoff to {self.target_alt} m OK")
        # except Exception as e:
        #     self.get_logger().warn(f"arm/takeoff failed: {e}")

        # Spróbuj włączyć tryb sterowania wektorami (jeśli serwis istnieje)

    def set_gimbal_angle(self, angle_degrees):
        """Sends a request to set gimbal angle and waits for an answer."""
        self.get_logger().info(f"Sending request to set the gimbal to {angle_degrees} degrees...")
        
        self.gimbal_angle_deg = float(angle_degrees)

        req = SetGimbalAngle.Request()
        req.angle_degrees = float(angle_degrees)
        
        future = self.set_gimbal_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Gimbal servo control setting success.")
            else:
                self.get_logger().error(f"Gimbal servo control unsuccessful: {response.message}")
        except Exception as e:
            self.get_logger().error(f'Service calling unsuccessful: {e}')

    # ──────────────────────────────────────────────────────────
    def gimbal_control_loop(self):
        """Periodically adjust gimbal pitch to keep the marker centered vertically.
        Uses the filtered vertical error ey_f (normalized)."""
        # only act when centering requested
        if not getattr(self, "centering", False):
            return
        # only track when we have a recent detection
        if (time.time() - self.last_seen) > self.lost_timeout:
            return

        delta_deg = self.gimbal_kp_deg * float(self.ey_f)

        # integrate with a small step to avoid jumps
        max_step = max(1.0, abs(self.gimbal_kp_deg) * 0.5)  # cap step to avoid jerks
        delta_deg = max(-max_step, min(max_step, delta_deg))
        self.gimbal_angle_deg = clamp(self.gimbal_angle_deg + delta_deg, self.gimbal_min_deg, self.gimbal_max_deg)

        # send non-blocking request
        try:
            req = SetGimbalAngle.Request()
            req.angle_degrees = float(self.gimbal_angle_deg)
            self.set_gimbal_cli.call_async(req)
        except Exception:
            pass

        # debug log occasionally
        self._gimbal_dbg_cnt += 1
        if self._gimbal_dbg_cnt >= max(1, int(self.gimbal_rate_hz)):
            self._gimbal_dbg_cnt = 0
            self.get_logger().info(f"gimbal: ey_f={self.ey_f:.3f} angle={self.gimbal_angle_deg:.2f} last_seen={time.time()-self.last_seen:.2f}s")

    # ──────────────────────────────────────────────────────────
    # def on_marker(self, msg: MiddleOfAruco):
    #     # błąd w pikselach względem środka obrazu
    #     self.ex_px = float(msg.x) - self.cx
    #     self.ey_px = float(msg.y) - self.cy

    #     # martwa strefa
    #     if abs(self.ex_px) < self.deadband_px:
    #         self.ex_px = 0.0
    #         self.x_flag = True
    #     if abs(self.ey_px) < self.deadband_px:
    #         self.ey_px = 0.0
    #         self.y_flag = True

    #     # normalizacja do [-1,1]
    #     ex = self.ex_px / (self.img_w / 2.0)
    #     ey = self.ey_px / (self.img_h / 2.0)

    #     # low-pass
    #     a = clamp(self.lowpass, 0.0, 1.0)
    #     self.ex_f = (1 - a) * self.ex_f + a * ex
    #     self.ey_f = (1 - a) * self.ey_f + a * ey
    #     self.last_seen = time.time()

    def on_detection(self, msg: Detection2DArray):
        self.get_logger().debug(f"Received {len(msg.detections)} detections")
        
        if len(msg.detections) == 0:
            return  # no detections

        # Select the best detection based on score
        best = max(msg.detections, key=lambda d: d.results[0].hypothesis.score)
        
        # Access the class name from the hypothesis
        obj_name = best.results[0].hypothesis.class_id
        score = best.results[0].hypothesis.score
    
        cx = best.bbox.center.position.x
        cy = best.bbox.center.position.y

        # cx = bbox.center.position.x
        # cy = bbox.center.position.y

        # błąd w pikselach względem środka obrazu
        self.ex_px = float(cx) - self.cx
        self.ey_px = float(cy) - self.cy

        self.get_logger().info(f"Tracking: {obj_name} with score: {score:.2f}, detection error: ex_px={self.ex_px}, ey_px={self.ey_px}")

        # martwa strefa
        if abs(self.ex_px) < self.deadband_px:
            self.ex_px = 0.0
            self.x_flag = True
        if abs(self.ey_px) < self.deadband_px:
            self.ey_px = 0.0
            self.y_flag = True

        # normalizacja do [-1,1]
        ex = self.ex_px / (self.img_w / 2.0)
        ey = self.ey_px / (self.img_h / 2.0)

        # low-pass
        a = clamp(self.lowpass, 0.0, 1.0)
        self.ex_f = (1 - a) * self.ex_f + a * ex
        self.ey_f = (1 - a) * self.ey_f + a * ey
        self.last_seen = time.time()

    # ──────────────────────────────────────────────────────────

    def get_altitude(self) -> float | None:
        """Return the most recent altitude from the telemetry topic."""
        return self.altitude

    # ──────────────────────────────────────────────────────────

    def degrees_to_radians(self, degrees: float) -> float:
        return degrees * (math.pi / 180.0)
    
    def radians_to_degrees(self, radians: float) -> float:
        return radians * (180.0 / math.pi)

    def drone_control_loop(self):

        if (time.time() - self.last_seen) > self.lost_timeout:
            # brak markera niedawno -> wyhamuj
            self.get_logger().warn(f"Detection lost for more than {self.lost_timeout}s, stopping!")
            self.send_vectors(0.0, 0.0, 0.0)
            self.state = "OK"
            self.centering = False
            try:
                self.timer.cancel()
            except Exception:
                pass
            return

        h_m = self.get_altitude()
        if h_m is None:
            self.get_logger().error("Altitude not available, skipping control step")
            return
            
        drone_pitch_rad = self.pitch_rad

        if drone_pitch_rad is None:
            # attitude not available this tick, skip control to avoid bad math
            return

        gimbal_angle_rad = self.degrees_to_radians(self.gimbal_angle_deg)

        overall_pitch_rad = gimbal_angle_rad - drone_pitch_rad # drone's pitch is in radians and inverted (negative pitch = nose down)

        if (self.gimbal_angle_deg >= 89.0 and self.radians_to_degrees(overall_pitch_rad) >= 90.0): # to enable going back
            half_vertical_pixel_count = 240.0 # for 480p

            # different approach:
            # pitch / half VFOV = ey_px / (vertical_pixel_count / 2.0)
            # vfov = 75 deg -> 37.5 deg to rad -> 0.6545 radians
            pitch = 0.6545 * self.ey_px / half_vertical_pixel_count

            overall_pitch_rad += pitch
 
        # else:

        denom = math.tan(overall_pitch_rad)
        if abs(denom) < 0.01:
            d_ground_m = 20.0 # max distance to cover
        else:
            d_ground_m = h_m / denom
            
        # drone's gimbal is 0.035m off center in the x direction
        d_ground_m -= 0.05 # TODO correct landing so it lands on target not 'behind' the target

        # Front speed (vx) to reduce ground distance to marker
        vx = self.kp * d_ground_m 
        vx = clamp(vx, -self.max_vel, self.max_vel)

        YAW_KP = 1.25  # Tuning parameter: Rad/s per normalized error
        yaw_rate = YAW_KP * self.ex_f
        
        # Clamp yaw rate to reasonable speed (e.g., 45 deg/s = ~0.8 rad/s)
        yaw_rate = clamp(yaw_rate, -0.8, 0.8)

        # Dodatkowe logowanie do debugowania
        self.get_logger().info(f"Control: h={h_m:.2f}, d={d_ground_m:.2f} -> vx={vx:.2f}, yaw_rate={yaw_rate:.2f}")

        vz = 0.0

        # if (abs(d_ground_m) < 3.0):
        # vz = self.kp * (h_m - self.target_alt)
        # vz = clamp(vz, -self.max_vel, self.max_vel)
        
        if abs(d_ground_m) < 0.2 and abs(self.ex_px) < 20:
            # Osiągnięto cel -> zatrzymaj ruch
            self.get_logger().info("Target reached, stopping approach.")
            self.send_vectors(0.0, 0.0, 0.0)
            self.state = "OK"
            self.centering = False
            try:
                self.timer.cancel()
            except Exception:
                pass
        else:
            # Send vectors. 
            # ARGUMENTS: (Forward_Speed, Yaw_Rate, Vertical_Speed)
            # We are passing yaw_rate in the second argument because we modified drone_handler.py
            self.send_vectors(vx, yaw_rate, vz)
            

    # ──────────────────────────────────────────────────────────
    def fly_to_detection(self):
        # Start gimbal centering + drone approach (control loop)
        self.state = "BUSY"
        self.centering = True
        self.get_logger().info("wlaczam nadlatywanie do detekcji (gimbal centering + approach)")
        # start approach control loop at 50 Hz (or adjust)
        self.timer = self.create_timer(0.02, self.drone_control_loop)
        # self.wait_busy()
 
 
    def center_detection(self):
        self.state = "BUSY"
        self.centering = True
        self.get_logger().info("wlaczam centrowanie detekcji (gimbal only, non-blocking)")
        # gimbal_control_loop is already running on its own timer
 
    def stop_centering(self):
        """Stop gimbal centering (non-blocking)."""
        self.centering = False
        self.state = "OK"
        self.get_logger().info("zatrzymano centrowanie detekcji")
        # if approach control is running, stop it too
        try:
            if hasattr(self, 'timer'):
                self.timer.cancel()
        except Exception:
            pass

    def wait_busy(self):
        self.get_logger().info("busy")
        while self.state == "BUSY":
            rclpy.spin_once(self, timeout_sec=0.05)

    def destroy_node(self):
        # Zatrzymaj na wyjściu
        try:
            self.send_vectors(0.0, 0.0, 0.0)
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    mission = FollowDetections()
    
    try:
        mission.arm()
        mission.set_gimbal_angle(35.0)
        mission.takeoff(5.0)

        # mission.send_set_yaw(290.0, True)
        # time.sleep(3)

        # mission.get_logger().info("Pointing gimbal downwards")

        # Włącz sterowanie wektorami prędkości
        mission.toggle_control()

        mission.fly_to_detection()
        # mission.center_detection()

        time.sleep(2.0)  

        # mission.stop_centering() # Zatrzymuje zarówno gimbal, jak i pętlę sterowania
        # mission.set_gimbal_angle(0.0)
        
        # Pętla główna do obsługi logiki misji
        run_seconds = 60.0
        mission.get_logger().info(f"Running fly_to_detection mission for {run_seconds}s...")
        t0 = time.time()

        while rclpy.ok(): # and (time.time() - t0) < run_seconds:
            rclpy.spin_once(mission, timeout_sec=0.1)
            # Sprawdź, czy misja się zakończyła (np. dron dotarł do celu)
            if mission.state != "BUSY":
                mission.get_logger().info("Mission finished, proceeding to land.")
                break
        
    except KeyboardInterrupt:
        mission.get_logger().info("Keyboard interrupt, stopping mission.")
    finally:
        # Zawsze spróbuj bezpiecznie wylądować
        mission.get_logger().info("Stopping any active loops and landing.")
        mission.land()
        mission.stop_centering() # Zatrzymuje zarówno gimbal, jak i pętlę sterowania
        mission.toggle_control()
        mission.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
