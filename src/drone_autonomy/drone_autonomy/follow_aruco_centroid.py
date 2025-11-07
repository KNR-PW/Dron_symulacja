#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import rclpy
from rclpy.node import Node
import math

from drone_interfaces.msg import MiddleOfAruco, VelocityVectors
from drone_interfaces.srv import ToggleVelocityControl, SetGimbalAngle, GetAttitude
from drone_autonomy.drone_comunication.drone_controller import DroneController


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class FollowArucoCentroid(DroneController):
    def __init__(self):
        super().__init__()

        # ---- Parametry ----
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('aruco_topic', '/aruco_markers')
        self.declare_parameter('target_alt', 2.0)
        self.declare_parameter('kp', 0.005)
        self.declare_parameter('deadband_px', 20)
        self.declare_parameter('max_vel', 1.0)
        self.declare_parameter('lowpass', 0.3)
        self.declare_parameter('lost_timeout', 0.8)  # s

        # Gimbal tracking params
        self.declare_parameter('gimbal_kp_deg', 1.5)     # degrees per normalized image unit
        self.declare_parameter('gimbal_min_deg', -45.0)
        self.declare_parameter('gimbal_max_deg', 89.9)
        self.declare_parameter('gimbal_rate_hz', 10.0)    # command rate

        self.img_w = int(self.get_parameter('image_width').value)
        self.img_h = int(self.get_parameter('image_height').value)
        self.cx = self.img_w / 2.0
        self.cy = self.img_h / 2.0
        self.kp = float(self.get_parameter('kp').value)
        self.deadband_px = int(self.get_parameter('deadband_px').value)
        self.max_vel = float(self.get_parameter('max_vel').value)
        self.lowpass = float(self.get_parameter('lowpass').value)
        self.lost_timeout = float(self.get_parameter('lost_timeout').value)
        self.aruco_topic = str(self.get_parameter('aruco_topic').value)
        self.target_alt = float(self.get_parameter('target_alt').value)

        # gimbal state
        self.gimbal_kp_deg = float(self.get_parameter('gimbal_kp_deg').value)
        self.gimbal_min_deg = float(self.get_parameter('gimbal_min_deg').value)
        self.gimbal_max_deg = float(self.get_parameter('gimbal_max_deg').value)
        self.gimbal_rate_hz = float(self.get_parameter('gimbal_rate_hz').value)
        # start with a safe downwards angle (match webots default)
        self.gimbal_angle = 89.9

        self.state = "OK"

        # ---- Subskrypcja markera ----
        self.sub = self.create_subscription(MiddleOfAruco, self.aruco_topic, self.on_marker, 10)

        # Stan filtrowany błędu (normalized)
        self.ex_f = 0.0
        self.ey_f = 0.0
        self.last_seen = time.time()

        self.get_logger().info(
            f"Start follow_aruco_centroid: img=({self.img_w}x{self.img_h}) "
            f"kp={self.kp} max_vel={self.max_vel} deadband={self.deadband_px}px"
        )

        self.set_gimbal_cli = self.create_client(SetGimbalAngle, 'set_gimbal_angle')
        while not self.set_gimbal_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Gimbal control service unavailable, waiting...')

        # persistent attitude client (used by control_loop)
        self.atti_cli = self.create_client(GetAttitude, 'get_attitude')
        while not self.atti_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Attitude service unavailable, waiting...')
        # small guard so we don't spam logs if attitude temporarily fails
        self._attitude_fail_count = 0

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
            req0.angle_degrees = float(self.gimbal_angle)
            self.set_gimbal_cli.call_async(req0)
        except Exception:
            pass

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
        self.gimbal_angle = clamp(self.gimbal_angle + delta_deg, self.gimbal_min_deg, self.gimbal_max_deg)

        # send non-blocking request
        try:
            req = SetGimbalAngle.Request()
            req.angle_degrees = float(self.gimbal_angle)
            self.set_gimbal_cli.call_async(req)
        except Exception:
            pass
        # debug log occasionally
        self._gimbal_dbg_cnt += 1
        if self._gimbal_dbg_cnt % max(1, int(self.gimbal_rate_hz)) == 0:
            self.get_logger().info(f"gimbal: ey_f={self.ey_f:.3f} angle={self.gimbal_angle:.2f} last_seen={time.time()-self.last_seen:.2f}s")

    # ──────────────────────────────────────────────────────────
    def on_marker(self, msg: MiddleOfAruco):
        # błąd w pikselach względem środka obrazu
        ex_px = float(msg.x) - self.cx
        ey_px = float(msg.y) - self.cy

        # martwa strefa
        if abs(ex_px) < self.deadband_px:
            ex_px = 0.0
        if abs(ey_px) < self.deadband_px:
            ey_px = 0.0

        # normalizacja do [-1,1]
        ex = ex_px / (self.img_w / 2.0)
        ey = ey_px / (self.img_h / 2.0)

        # low-pass
        a = clamp(self.lowpass, 0.0, 1.0)
        self.ex_f = (1 - a) * self.ex_f + a * ex
        self.ey_f = (1 - a) * self.ey_f + a * ey

        self.last_seen = time.time()

    # ──────────────────────────────────────────────────────────
    def request_attitude(self, timeout_sec: float = 1.0):
        """Request roll,pitch,yaw from get_attitude service. Returns tuple or (None,None,None) on failure."""
        try:
            req = GetAttitude.Request()
            future = self.atti_cli.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
            resp = future.result()
            if resp is None:
                raise RuntimeError("GetAttitude returned None")
            # assume response has roll,pitch,yaw floats
            return float(resp.roll), float(resp.pitch), float(resp.yaw)
        except Exception as e:
            self._attitude_fail_count += 1
            if self._attitude_fail_count <= 3:
                self.get_logger().error(f'Attitude service failed: {e}')
            return None, None, None

    # ──────────────────────────────────────────────────────────

    def degrees_to_radians(self, degrees: float) -> float:
        return degrees * (math.pi / 180.0)

    def control_loop(self):
        # brak markera niedawno → wyhamuj
        if (time.time() - self.last_seen) > self.lost_timeout:
            # lost marker: stop motion and stop approach
            self.send_vectors(0.0, 0.0, 0.0)
            self.state = "OK"
            self.centering = False
            try:
                self.timer.cancel()
            except Exception:
                pass
            return
        
        h = getattr(self, 'alt', None)
        if h is None:
            self.get_logger().error('Altitude not available, skipping control step')
            return
        _, drone_pitch, _ = self.request_attitude()
        if drone_pitch is None:
            # attitude not available this tick — skip control to avoid bad math
            return

        denom = math.tan(self.degrees_to_radians(self.gimbal_angle + drone_pitch))
        if abs(denom) < 0.001:
            d_ground = 100.0 # max distance to cover
        else:
            d_ground = h / denom
 
        # Proste P: ex -> vy, ey -> vx (kamera do przodu)
        vx = -self.kp * self.ey_f
        vx = clamp(vx, -self.max_vel, self.max_vel)
        vy = +self.kp * d_ground
        vy = clamp(vy, -self.max_vel, self.max_vel)
        if abs(self.ey_f) < 0.05 and abs(self.ex_f) < 0.05:
            # reached centering goal: stop motion and stop approach, keep gimbal centered
            self.send_vectors(0.0, 0.0, 0.0)
            self.state = "OK"
            self.centering = False
            try:
                self.timer.cancel()
            except Exception:
                pass
 
        self.send_vectors(vx, vy, 0.0)

    # ──────────────────────────────────────────────────────────
    def fly_to_aruco(self):
        # Start gimbal centering + drone approach (control loop)
        self.state = "BUSY"
        self.centering = True
        self.get_logger().info("wlaczam nadlatywanie do aruco (gimbal centering + approach)")
        # start approach control loop at 10 Hz (or adjust)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.wait_busy()
 
 
    def center_aruco(self):
        self.state = "BUSY"
        self.centering = True
        self.get_logger().info("wlaczam centrowanie aruco (gimbal only, non-blocking)")
        # gimbal_control_loop is already running on its own timer
 
    def stop_centering(self):
        """Stop gimbal centering (non-blocking)."""
        self.centering = False
        self.state = "OK"
        self.get_logger().info("zatrzymano centrowanie aruco")
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
    mission = FollowArucoCentroid()
    mission.arm()
    mission.takeoff(5.0)

    mission.get_logger().info("Pointing gimbal downwards")
    mission.set_gimbal_angle(89.9)
    # enable velocity/vector control if needed (single call)
    mission.toggle_control()

    # start centering (non-blocking)
    mission.fly_to_aruco()
    # mission.center_aruco()

    mission.toggle_control()
    # let centering run for N seconds (adjust as needed)
    try:
        run_seconds = 60.0
        mission.get_logger().info(f"Centering for {run_seconds}s...")
        t0 = time.time()
        while time.time() - t0 < run_seconds:
            rclpy.spin_once(mission, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass

    # stop centering, then land
    mission.stop_centering()
    mission.get_logger().info("Stopping centering and landing")
    mission.land()


if __name__ == '__main__':
    main()
