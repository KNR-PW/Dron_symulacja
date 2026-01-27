#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import rclpy
from rclpy.node import Node
import math
import numpy as np
from drone_autonomy.kalman_filter import KalmanFilter

from vision_msgs.msg import Detection2DArray
from drone_interfaces.msg import VelocityVectors, Telemetry
from drone_interfaces.srv import ToggleVelocityControl, SetGimbalAngle
from drone_autonomy.drone_comunication.drone_controller import DroneController
from std_srvs.srv import SetBool
from std_msgs.msg import Float32, Float32MultiArray
from geometry_msgs.msg import Point, Twist, PointStamped

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
        self.declare_parameter('max_vel', 2.0)
        self.declare_parameter('lowpass', 0.3)
        self.declare_parameter('lost_timeout', 0.8)  # s

        # Gimbal tracking params
        self.declare_parameter('gimbal_min_deg', -15.0)
        self.declare_parameter('gimbal_max_deg', 90.0)
        self.declare_parameter('gimbal_rate_hz', 50.0)    # command rate

        self.img_w = int(self.get_parameter('image_width').value)
        self.img_h = int(self.get_parameter('image_height').value)
        self.cx = self.img_w / 2.0
        self.cy = self.img_h / 2.0
        self.kp = float(self.get_parameter('kp').value)
        self.max_vel = float(self.get_parameter('max_vel').value)
        self.lowpass = float(self.get_parameter('lowpass').value)
        self.lost_timeout = float(self.get_parameter('lost_timeout').value)
        self.detections_topic = str(self.get_parameter('detections_topic').value)
        self.target_alt = float(self.get_parameter('target_alt').value)

        # gimbal state
        self.gimbal_min_deg = float(self.get_parameter('gimbal_min_deg').value)
        self.gimbal_max_deg = float(self.get_parameter('gimbal_max_deg').value)
        self.gimbal_rate_hz = float(self.get_parameter('gimbal_rate_hz').value)
        # start with a safe downwards angle (match webots default)
        self.gimbal_angle_deg = 80.0
        self.gimbal_setpoint = 80.0  # Internal integrator state for smooth control

        # --- PID Constants (Unified) ---
        self.pid_yaw_kp = 1.5
        self.pid_yaw_ki = 0.2
        self.pid_yaw_kd = 0.1

        self.roll = None
        self.pitch_rad = None
        self.yaw = None
        self.current_yaw_rate = 0.0

        # --- Performance Monitoring ---
        self.recording = False
        self.error_history = []
        self.time_history = []
        self.start_time = 0.0

        # --- Kalman Filter ---
        self.kf = KalmanFilter()
        self.last_kf_time = time.time()

        self.state = "OK"
        
        # --- Tuning Override ---
        self.override_yaw_error = None # If set (float), control loop uses this instead of vision
        self.override_setpoint = None 

        # ---- Subskrypcja markera ----
        # self.sub = self.create_subscription(MiddleOfAruco, self.detections_topic, self.on_marker, 10)
        self.sub = self.create_subscription(Detection2DArray, self.detections_topic, self.on_detection, 10)

        self.ex_px = 0.0
        self.ey_px = 0.0

        # Stan filtrowany błędu (normalized)
        self.ex_f = 0.0
        self.ey_f = 0.0
        self.last_seen = time.time()

        self.delta_deg_f = 0.0
        
        # --- Ground Truth & Validation ---
        self.car_true_pos = None
        self.drone_true_pos = None
        self.create_subscription(PointStamped, '/sim/ground_truth/car', self.on_car_ground_truth, 10)
        self.create_subscription(PointStamped, '/sim/ground_truth/drone', self.on_drone_ground_truth, 10)
        # ---------------------------------

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
            f"kp={self.kp} max_vel={self.max_vel}"
        )

        self.set_gimbal_cli = self.create_client(SetGimbalAngle, 'set_gimbal_angle')
        while not self.set_gimbal_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Gimbal control service unavailable, waiting...')

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

        # Client to enable the tracker
        self.tracker_client = self.create_client(SetBool, 'enable_tracker')

        # Debug publishers for PID tuning (x=target, y=error, z=output)
        self.pub_dbg_yaw = self.create_publisher(Point, '/debug/yaw', 10)
        self.pub_dbg_vel = self.create_publisher(Point, '/debug/vel', 10)
        self.pub_dbg_gimbal = self.create_publisher(Point, '/debug/gimbal', 10)
        self.pub_dbg_target_pos = self.create_publisher(Point, '/debug/target_pos', 10)
        self.pub_dbg_vx = self.create_publisher(Point, '/debug/vx', 10)
        
        self.pub_car_delta = self.create_publisher(Point, '/debug/car_delta', 10)
        # Benchmark publisher: [true_x, true_y, pred_x, pred_y, dist_error]
        self.pub_benchmark = self.create_publisher(Float32MultiArray, '/debug/tracker_benchmark', 10)

        # Car control publisher
        self.car_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # self.gimbal_pub = self.create_publisher(Float32, '/gimbal/cmd_pitch', 10)
        self.create_subscription(Float32, '/gimbal/current_angle', self.gimbal_status_callback, 10)

    def gimbal_status_callback(self, msg):
        """Updates the known physical state of the gimbal."""
        self.gimbal_angle_deg = msg.data

    def telemetry_callback(self, msg):
        """Callback to update altitude from telemetry topic."""
        self.altitude = msg.alt
        self.roll = msg.roll
        self.pitch_rad = msg.pitch
        self.yaw = msg.yaw
        try:
            self.current_yaw_rate = msg.yaw_speed
        except AttributeError:
            self.current_yaw_rate = 0.0

    def start_recording(self):
        """Resets and starts recording error data."""
        self.error_history = []
        self.time_history = []
        self.start_time = time.time()
        self.recording = True
        self.get_logger().info(">>> Started recording control performance data.")

    def stop_and_report(self, label="Maneuver"):
        """Stops recording and prints metrics."""
        self.recording = False
        if not self.error_history:
            self.get_logger().warn("No data recorded to report.")
            return

        errors = np.array(self.error_history)
        times = np.array(self.time_history)
        
        # Metrics
        mae = np.mean(np.abs(errors))          # Mean Absolute Error
        rmse = np.sqrt(np.mean(errors**2))     # Root Mean Square Error
        max_err = np.max(np.abs(errors))       # Max Overshoot/Error
        
        # Steady State Error (last 10% of samples)
        n_tail = max(1, int(len(errors) * 0.1))
        steady_state_err = np.mean(np.abs(errors[-n_tail:]))

        self.get_logger().info(f"=== Performance Report: {label} ===")
        self.get_logger().info(f"  Duration:       {times[-1]:.2f} s")
        self.get_logger().info(f"  Samples:        {len(errors)}")
        self.get_logger().info(f"  MAE (Avg Err):  {mae:.4f} rad")
        self.get_logger().info(f"  RMSE:           {rmse:.4f} rad")
        self.get_logger().info(f"  Max Error:      {max_err:.4f} rad")
        self.get_logger().info(f"  Steady State:   {steady_state_err:.4f} rad")
        self.get_logger().info(f"=====================================")

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
            # Sync internal state to reality while idle so we are ready to start
            self.gimbal_setpoint = self.gimbal_angle_deg 
            return
        # only track when we have a recent detection
        if (time.time() - self.last_seen) > self.lost_timeout:
            # Sync internal state to reality while lost so we don't jump when we find it
            self.gimbal_setpoint = self.gimbal_angle_deg
            return

        kalman_pred = (self.kf.x[0], self.kf.x[1]) if self.kf.initialized else None
        
        if (time.time() - self.last_seen) > 1.0 and kalman_pred is not None:
            sin_ny = math.sin(-self.yaw)
            cos_ny = math.cos(-self.yaw)
            
            x_target_body = kalman_pred[0] * cos_ny - kalman_pred[1] * sin_ny
            # y_target_body = kalman_pred[0] * sin_ny + kalman_pred[1] * cos_ny

            height = self.altitude
            
            # Fix: Calculate delta as Target - Current (not Current - Target)
            target_angle_abs = self.radians_to_degrees(math.atan2(height, x_target_body))
            delta_deg = target_angle_abs - self.gimbal_angle_deg
        else:
            fov_y_rad = self.degrees_to_radians(75.0) # because 640 x 480, and the cameras fov is 100deg 
            delta_rad = (self.ey_px / (self.img_h / 2.0)) * (fov_y_rad / 2.0)

            delta_deg = self.radians_to_degrees(delta_rad) # webots pid will handle the regulation

        # delta_deg = self.gimbal_kp_deg * float(self.ey_f) # TODO: try and follow the kalman state?
        # delta_deg = self.gimbal_kp_deg * pitch_rel_deg

        # low-pass
        a = clamp(self.lowpass, 0.0, 1.0)
        self.delta_deg_f = (1 - a) * self.delta_deg_f + a * delta_deg

        K_gimbal = 0.2 # damping factor for smoother gimbal movement

        # self.gimbal_setpoint += delta_deg * K_gimbal
        self.gimbal_setpoint = self.gimbal_angle_deg + self.delta_deg_f * K_gimbal
        
        self.gimbal_setpoint = clamp(self.gimbal_setpoint, self.gimbal_min_deg, self.gimbal_max_deg)
        target_angle = self.gimbal_setpoint

        # Publish command via Topic
        # msg = Float32()
        # msg.data = float(target_angle)
        # self.gimbal_pub.publish(msg)

        # send non-blocking request (Service fallback)
        try:
            req = SetGimbalAngle.Request()
            # req.angle_degrees = float(self.gimbal_angle_deg)
            req.angle_degrees = float(target_angle)
            self.set_gimbal_cli.call_async(req)
        except Exception:
            pass

        # Publish debug info
        dbg_msg = Point()
        dbg_msg.x = 0.0 
        dbg_msg.y = float(target_angle) 
        dbg_msg.z = float(self.gimbal_angle_deg)
        self.pub_dbg_gimbal.publish(dbg_msg)

    # ──────────────────────────────────────────────────────────
    # def on_marker(self, msg: MiddleOfAruco):
    #     # błąd w pikselach względem środka obrazu
    #     self.ex_px = float(msg.x) - self.cx
    #     self.ey_px = float(msg.y) - self.cy

    #     # normalizacja do [-1,1]
    #     ex = self.ex_px / (self.img_w / 2.0)
    #     ey = self.ey_px / (self.img_h / 2.0)

    #     # low-pass
    #     a = clamp(self.lowpass, 0.0, 1.0)
    #     self.ex_f = (1 - a) * self.ex_f + a * ex
    #     self.ey_f = (1 - a) * self.ey_f + a * ey
    #     self.last_seen = time.time()

    def on_car_ground_truth(self, msg):
        self.car_true_pos = np.array([msg.point.x, msg.point.y, msg.point.z])
        # self.get_logger().info(f"Car GT: {self.car_true_pos}", throttle_duration_sec=2.0)

    def on_drone_ground_truth(self, msg):
        self.drone_true_pos = np.array([msg.point.x, msg.point.y, msg.point.z])

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

        # błąd w pikselach względem środka obrazu
        self.ex_px = float(cx) - self.cx
        self.ey_px = float(cy) - self.cy

        # self.get_logger().info(f"Tracking: {obj_name} with score: {score:.2f}, detection error: ex_px={self.ex_px}, ey_px={self.ey_px}")
        
        # --- KALMAN FILTER UPDATE (GPS Denied) ---
        if self.altitude is not None and self.pitch_rad is not None and self.yaw is not None:
            # 1. Calculate Distance on Ground (Body Frame)
            gimbal_angle_rad = self.degrees_to_radians(self.gimbal_angle_deg)
            overall_pitch_rad = gimbal_angle_rad - self.pitch_rad

            # if (self.gimbal_angle_deg >= 89.0 and self.radians_to_degrees(overall_pitch_rad) >= 90.0):
            half_vertical_pixel_count = 240.0
            pitch_offset = 0.6545 * self.ey_px / half_vertical_pixel_count
            overall_pitch_rad += pitch_offset

            # --- FIX: Clamp pitch to avoid singularity/inversion ---
            # Assume we never look "up" relative to horizon for ground targets
            overall_pitch_rad = clamp(overall_pitch_rad, 0.01, 1.5) # 5 deg to ~86 deg
            # -----------------------------------------------------

            denom = math.tan(overall_pitch_rad)
            if abs(denom) < 0.1: 
                d_ground = 30.0
            else:
                d_ground = self.altitude / denom
            
            # 2. Calculate Azimuth (Angle offset in Yaw)
            fov_x_rad = self.degrees_to_radians(100.0) 
            azimuth_rel = (self.ex_px / (self.img_w / 2.0)) * (fov_x_rad / 2.0)
            
            # 3. Calculate Position in Body Frame (X=Forward, Y=Right)
            x_body = d_ground 
            y_body = d_ground * math.tan(azimuth_rel)
            
            # 4. Rotate to Yaw-Stabilized Frame
            sin_y = math.sin(self.yaw)
            cos_y = math.cos(self.yaw)
            
            x_stab = x_body * cos_y - y_body * sin_y
            y_stab = x_body * sin_y + y_body * cos_y
            
            self.last_raw_x_stab = x_stab
            self.last_raw_y_stab = y_stab

            self.kf.update(np.array([x_stab, y_stab]))

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

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def drone_control_loop(self):

        # Calculate dt for KF
        now = time.time()
        dt = now - self.last_kf_time
        self.last_kf_time = now

        # --- FIX: Wait for KF initialization ---
        if not self.kf.initialized:
            if (now - self.last_seen) > self.lost_timeout:
                 self.get_logger().warn("Timeout waiting for first detection.")
                 self.send_vectors(0.0, 0.0, 0.0, 0.0)
                 self.state = "OK"
                 self.centering = False
                 try: self.timer.cancel()
                 except: pass
            return
        # ---------------------------------------

        # --- BYPASS KALMAN FILTER (Raw Data) ---
        if hasattr(self, 'last_raw_x_stab'):
             x_pred_stab = self.last_raw_x_stab
             y_pred_stab = self.last_raw_y_stab
        else:
            pred_stab = self.kf.predict(dt)
            x_pred_stab = pred_stab[0]
            y_pred_stab = pred_stab[1]

        # --- VALIDATION LOGIC ---
        if self.car_true_pos is not None and self.drone_true_pos is not None:
             est_car_pos = self.drone_true_pos.copy()
             est_car_pos[0] += x_pred_stab
             est_car_pos[1] -= y_pred_stab

             delta = est_car_pos - self.car_true_pos
             dist_error = np.linalg.norm(delta[:2])  # 2D error

             dbg_car_delta = Point()
             dbg_car_delta.x = delta[0]
             dbg_car_delta.y = delta[1]
             dbg_car_delta.z = dist_error
             self.pub_car_delta.publish(dbg_car_delta)

             # Publish benchmark data
             try:
                 benchmark_msg = Float32MultiArray()
                 benchmark_msg.data = [self.car_true_pos[0], self.car_true_pos[1], est_car_pos[0], est_car_pos[1], dist_error]
                 self.pub_benchmark.publish(benchmark_msg)
             except Exception as e:
                 self.get_logger().warn(f"Failed to publish benchmark data: {e}")
        else:
             self.get_logger().info(
                 f"WAITING FOR GT: Car={self.car_true_pos is not None} Drone={self.drone_true_pos is not None}",
                 throttle_duration_sec=2.0
             )
        # ------------------------

        if (time.time() - self.last_seen) > self.lost_timeout:
            self.get_logger().warn(f"Detection lost for more than {self.lost_timeout}s, stopping!")
            self.send_vectors(0.0, 0.0, 0.0, 0.0)
            self.state = "OK"
            self.centering = False
            try:
                self.timer.cancel()
            except Exception:
                pass
            return

        h_m = self.get_altitude()
        if h_m is None or self.yaw is None:
            self.get_logger().error("Altitude/Yaw not available, skipping control step")
            return
        # --- RECONSTRUCT TARGET IN BODY FRAME ---
        # We have predicted position in Stabilized Frame (x_pred_stab, y_pred_stab)
        # We need to rotate it back to Body Frame using current Yaw
        # x_body = x_stab * cos(-yaw) - y_stab * sin(-yaw)
        # y_body = x_stab * sin(-yaw) + y_stab * cos(-yaw)

        sin_ny = math.sin(-self.yaw)
        cos_ny = math.cos(-self.yaw)

        x_target_body = x_pred_stab * cos_ny - y_pred_stab * sin_ny
        y_target_body = x_pred_stab * sin_ny + y_pred_stab * cos_ny

        dbg_target = Point()
        dbg_target.x = float(x_target_body)
        dbg_target.y = float(y_target_body)
        dbg_target.z = 0.0
        self.pub_dbg_target_pos.publish(dbg_target)

        # Forward distance to target (body X)
        d_ground_m = x_target_body
        # Offset for camera position
        d_ground_m += 0.05 

        # --- STOP 0.5m BEFORE TARGET (standoff control), even if target is behind us ---
        standoff_m = 0.5

        if abs(d_ground_m) < standoff_m:
            d_ground_m = 0.0


        # # If the target is behind (d_ground_m < 0), we want to stop at -standoff_m.
        # # Keep direction stable near zero to avoid sign flipping.
        # if not hasattr(self, "_target_dir"):
        #     self._target_dir = 1.0
        # if abs(d_ground_m) > 0.10:
        #     self._target_dir = 1.0 if d_ground_m >= 0.0 else -1.0

        # desired_distance_m = self._target_dir * standoff_m
        # forward_error_m = d_ground_m - desired_distance_m
        # ---------------------------------------------------------------------------

        # Front speed (vx): drive the forward_error to 0 (supports forward and backward motion)
        vx = self.kp * d_ground_m
        vx = clamp(vx, -self.max_vel, self.max_vel)

        dbg_vx = Point()
        dbg_vx.x = 0.0 # float(forward_error_m)  # control error (m)
        dbg_vx.y = float(vx)               # commanded vx
        dbg_vx.z = float(d_ground_m)       # measured distance to target
        self.pub_dbg_vx.publish(dbg_vx)

        # Yaw Control - calculate angle to target in body frame
        angle_to_target = math.atan2(y_target_body, x_target_body)
        yaw_error = angle_to_target

        # --- RECORDING ---
        if self.recording:
            self.error_history.append(yaw_error)
            self.time_history.append(time.time() - self.start_time)

        # Proportional term
        error = yaw_error
        p_term = self.pid_yaw_kp * error

        # Integral term with anti-windup
        if not hasattr(self, 'integral_error'):
            self.integral_error = 0.0
        self.integral_error += error * dt
        self.integral_error = clamp(self.integral_error, -1.0, 1.0)
        i_term = self.pid_yaw_ki * self.integral_error

        # Derivative term
        if not hasattr(self, 'prev_error'):
            self.prev_error = error
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error
        d_term = self.pid_yaw_kd * derivative

        yaw_rate = p_term  # + i_term + d_term (enable if needed)

        # vy = clamp(self.normalize_angle(yaw_error) / math.pi, -1.0, 1.0) * 2.0
        vy = self.kp * y_target_body * 0.5

        # Clamp yaw rate to reasonable speed (e.g., 45 deg/s = ~0.8 rad/s)
        # yaw_rate = clamp(yaw_rate, -0.8, 0.8)

        vz = 0.0

        # Stop the approach loop once we are at the standoff distance (and roughly aligned)
        # if abs(forward_error_m) < 0.05 and abs(yaw_error) < 0.1:
        #     self.get_logger().info("Standoff reached (~0.5m). Stopping approach.")
        #     self.send_vectors(0.0, 0.0, 0.0, 0.0)
        #     self.state = "OK"
        #     # keep gimbal centering if you want; comment next line to keep tracking
        #     self.centering = False
        #     try:
        #         self.timer.cancel()
        #     except Exception:
        #         pass
        #     return

        # Publish debug info
        dbg_yaw = Point()
        dbg_yaw.x = 0.0
        dbg_yaw.y = float(math.degrees(self.normalize_angle(yaw_error + self.yaw)))
        dbg_yaw.z = float(math.degrees(self.yaw))
        self.pub_dbg_yaw.publish(dbg_yaw)

        dbg_vel = Point()
        dbg_vel.x = 0.0
        dbg_vel.y = float(math.degrees(yaw_rate))
        dbg_vel.z = float(math.degrees(self.current_yaw_rate))
        self.pub_dbg_vel.publish(dbg_vel)

        # Send vectors.
        # ARGUMENTS: (Forward_Speed, Right_Speed, Vertical_Speed, Yaw_Rate)
        self.send_vectors(vx, 0.0, vz, yaw_rate)
        # self.send_vectors(0.0, 0.0, vz, yaw_rate)

    # ──────────────────────────────────────────────────────────
    def fly_to_detection(self):        
        # Start gimbal centering + drone approach (control loop)
        self.state = "BUSY"
        self.centering = True
        self.get_logger().info("Gimbal centering + approach started")
        # start approach control loop at 100 Hz (or adjust)
        self.timer = self.create_timer(0.01, self.drone_control_loop) # TODO: faster?
        # self.wait_busy()
 
 
    def center_detection(self):
        self.state = "BUSY"
        self.centering = True
        self.get_logger().info("Gimbal centering started (gimbal only)")
 
    def stop_centering(self):
        """Stop gimbal centering (non-blocking)."""
        self.centering = False
        self.state = "OK"
        self.get_logger().info("Detection centering stopped")
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
            self.send_vectors(0.0, 0.0, 0.0, 0.0)
        except Exception:
            pass
        super().destroy_node()

    def enable_tracker_node(self, enable=True):
        if not self.tracker_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Tracker service not available. Is hybrid_tracker_node running?")
            return

        req = SetBool.Request()
        req.data = enable
        future = self.tracker_client.call_async(req)
        # We can wait for it or just fire and forget. Waiting is safer.
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f"Tracker enabled: {enable}")
        else:
            self.get_logger().error("Failed to call enable_tracker service")

    def send_car_command(self, linear_x, angular_z=0.0):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.car_pub.publish(msg)
        self.get_logger().info(f"Sent car command: v={linear_x} m/s, w={angular_z} rad/s")


def main():
    rclpy.init()
    mission = FollowDetections()
    
    try:
        mission.arm()
        target_height = 5.0
        
        mission.set_gimbal_angle(45.0)
        mission.takeoff(target_height)
        
        # Ensure velocity control is enabled (True)
        response = mission.toggle_control()
        if not response.result:
            mission.get_logger().info("Velocity control was OFF, toggling again to enable...")
            mission.toggle_control()

        mission.get_logger().info("Enabling Hybrid Tracker Node...")
        mission.enable_tracker_node(True)

        # Wait for drone to reach target altitude
        mission.get_logger().info(f"Waiting to reach target altitude ({target_height}m)...")
        while rclpy.ok():
            rclpy.spin_once(mission, timeout_sec=0.1)
            current_alt = mission.get_altitude()
            
            if current_alt is not None:
                # Check if we are close enough (e.g. within 0.5m)
                if abs(current_alt - target_height) < 0.2:
                    mission.get_logger().info(f"Target altitude reached: {current_alt:.2f}m")
                    break
            
            # Optional: Log status every now and then
            # mission.get_logger().info(f"Climbing... {current_alt}")

        mission.last_seen = time.time()
                    
        mission.send_car_command(4.0, 0.25)

        # mission.center_detection()
        # mission.send_goto_relative(0.0, -6.5, 0.0)
        mission.fly_to_detection()

        mission.get_logger().info("Starting simple car loop: Forward -> Stop -> Turn -> Forward")

        while rclpy.ok():
            rclpy.spin_once(mission, timeout_sec=0.1)

        # t_end = time.time() + 5.0
        # while rclpy.ok() and time.time() < t_end:
        #     rclpy.spin_once(mission, timeout_sec=0.05)

        translation = 12.0 # meters
        back_correction = 1.17

        # time = speed / translation

        # while rclpy.ok():
        #     speed = 3.0
        #     # for speed in [2.0, 4.0, 6.0]:
        #     while True:
        #         # Move Forward
        #         mission.send_car_command(speed, -0.2)
        #         t_end = time.time() + 5.0
        #         while rclpy.ok() and time.time() < t_end:
        #             rclpy.spin_once(mission, timeout_sec=0.05)

        #         # Stop
        #         mission.send_car_command(0.0, 0.0)
        #         t_end = time.time() + 5.0
        #         while rclpy.ok() and time.time() < t_end:
        #             rclpy.spin_once(mission, timeout_sec=0.05)

        #         # # Move Backward
        #         # mission.send_car_command(-speed, 0.0)
        #         # # Apply correction to duration
        #         # t_end = time.time() + 5.0 * (back_correction)
        #         # while rclpy.ok() and time.time() < t_end:
        #         #     rclpy.spin_once(mission, timeout_sec=0.05)

        #         # # Stop
        #         # mission.send_car_command(0.0, 0.0)
        #         # t_end = time.time() + 5.0
        #         # while rclpy.ok() and time.time() < t_end:
        #         #     rclpy.spin_once(mission, timeout_sec=0.05)

        #         speed += 2.0
        

        
    except KeyboardInterrupt:
        mission.get_logger().info("Keyboard interrupt, stopping mission.")
        mission.enable_tracker_node(False) #TODO: Fix
        mission.send_car_command(0.0, 0.0)
    finally:
        mission.get_logger().info("Stopping any active loops and landing.")
        mission.land()
        mission.stop_centering() # Zatrzymuje zarówno gimbal, jak i pętlę sterowania
        
        # Ensure velocity control is disabled
        response = mission.toggle_control()
        if response.result:
             mission.get_logger().info("Velocity control was ON, toggling again to disable...")
             mission.toggle_control()

        mission.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
