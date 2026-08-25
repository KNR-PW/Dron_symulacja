#!/usr/bin/env python3
"""
Na tym etapie node tylko:
  - subskrybuje obraz(y) glebi z jednej lub wielu kamer stereo,
  - wymija przeszkode, ale nie wraca na swoj pierwotny tor lotu
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from drone_interfaces.msg import VelocityVectors
from drone_interfaces.srv import ToggleVelocityControl, SetMode
from cv_bridge import CvBridge


class ObstacleAvoidanceOak(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_oak')

        self.declare_parameter('depth_topics', ['oak/stereo/image_raw'])
        depth_topics = self.get_parameter('depth_topics').value
        self.declare_parameter('slow_distance', 2.0)
        self.declare_parameter('stop_distance', 1.0)
        self.declare_parameter('avoid_speed', 0.5)
        self.declare_parameter('cruise_speed', 1.0)
        self.declare_parameter('max_range', 15.0)
        self.declare_parameter('min_valid_mm', 300)
        self.declare_parameter('roi_top_frac', 0.1)
        self.declare_parameter('roi_bottom_frac', 0.85)
        self.declare_parameter('steer_deadband', 0.3)
        self.declare_parameter('clear_margin', 0.5)
        self.declare_parameter('careful_speed_frac', 0.3)
        self.declare_parameter('guard_enabled', True)
        self.declare_parameter('max_avoid_duration', 10.0)

        self.slow_distance = float(self.get_parameter('slow_distance').value)
        self.stop_distance = float(self.get_parameter('stop_distance').value)
        self.avoid_speed = float(self.get_parameter('avoid_speed').value)
        self.cruise_speed = float(self.get_parameter('cruise_speed').value)
        self.max_range_mm = float(self.get_parameter('max_range').value) * 1000.0
        self.min_valid_mm = float(self.get_parameter('min_valid_mm').value)
        self.steer_deadband = float(self.get_parameter('steer_deadband').value)
        self.roi_top_frac = float(self.get_parameter('roi_top_frac').value)
        self.roi_bottom_frac = float(self.get_parameter('roi_bottom_frac').value)
        self.clear_margin = float(self.get_parameter('clear_margin').value)
        self.careful_speed_frac = float(self.get_parameter('careful_speed_frac').value)
        self.max_avoid_duration = float(self.get_parameter('max_avoid_duration').value)

        self.bridge = CvBridge()
        self.latest_depth = {}
        self.avoiding = False
        self.returning = False
        
        self.committed_steer_sign = 0.0

        self.lateral_time = 0.0
        self.timed_out = False
        self._last_diag_log_time = None
        self.return_remaining = 0.0
        self._last_step_time = None
        self.velocity_control_confirmed = None
        self.toggle_pending = False

        self.depth_subscribers = []
        for topic in depth_topics:
            sub = self.create_subscription(
                Image, topic,
                lambda msg, t=topic: self.depth_callback(msg, t),
                10)
            self.depth_subscribers.append(sub)

        self.velocity_vectors_publisher = self.create_publisher(
            VelocityVectors, 'knr_hardware/velocity_vectors', 10)
        self.avoiding_publisher = self.create_publisher(
            Bool, 'obstacle_avoidance_oak/avoiding', 10)
        self.toggle_velocity_control_client = self.create_client(
            ToggleVelocityControl, 'knr_hardware/toggle_v_control')
        self.guard_srv = self.create_service(
            SetMode, 'set_brake_on_obstacle', self.set_guard_callback)

        self.guard_active = bool(self.get_parameter('guard_enabled').value)

        self.get_logger().info(
            f"ObstacleAvoidanceOak gotowy. Depth topics: {list(depth_topics)}, "
            f"guard_active: {self.guard_active}")

    def depth_callback(self, msg: Image, topic: str):
        self.latest_depth[topic] = self.bridge.imgmsg_to_cv2(msg, '16UC1')
        self.evaluate_obstacles()

    def evaluate_obstacles(self):
        now = self.get_clock().now()
        dt = 0.0
        if self._last_step_time is not None:
            dt = max(0.0, (now - self._last_step_time).nanoseconds / 1e9)
        self._last_step_time = now

        if not self.guard_active:
            if self.avoiding or self.returning:
                self._sync_velocity_control(False)
            self._set_avoiding(False)
            self.returning = False
            self.committed_steer_sign = 0.0
            self.lateral_time = 0.0
            self.timed_out = False
            return

        nearest_m = float('inf')
        nearest_pixel = None
        steer = 0.0
        have_frame = False

        for topic, depth in self.latest_depth.items():
            height = depth.shape[0]
            row_start = int(height * self.roi_top_frac)
            row_end = max(row_start + 1, int(height * self.roi_bottom_frac))
            roi = depth[row_start:row_end, :]

            valid_mask = (roi > self.min_valid_mm) & (roi < self.max_range_mm)
            if not np.any(valid_mask):
                continue
            have_frame = True
            roi_valid = roi[valid_mask]
            frame_min_mm = float(roi_valid.min())
            if frame_min_mm / 1000.0 < nearest_m:
                nearest_m = frame_min_mm / 1000.0
                ys, xs = np.where((roi == frame_min_mm) & valid_mask)
                nearest_pixel = (topic, int(ys[0]) + row_start, int(xs[0]))

            width = roi.shape[1]
            third = max(1, width // 3)
            left = roi[:, :third]
            right = roi[:, width - third:]

            left_mask = (left > self.min_valid_mm) & (left < self.max_range_mm)
            right_mask = (right > self.min_valid_mm) & (right < self.max_range_mm)

            left_dist = float(left[left_mask].min()) / 1000.0 if np.any(left_mask) else self.max_range_mm / 1000.0
            right_dist = float(right[right_mask].min()) / 1000.0 if np.any(right_mask) else self.max_range_mm / 1000.0

            steer += right_dist - left_dist

        should_avoid = have_frame and nearest_m <= self.slow_distance

        still_close = have_frame and nearest_m <= (self.slow_distance + self.clear_margin)

        if should_avoid and not self.avoiding and nearest_pixel is not None:
            topic, row, col = nearest_pixel
            self.get_logger().info(
                f'Najblizszy piksel: {nearest_m:.2f} m na ({row},{col}) w {topic}')

        if (should_avoid or still_close) and nearest_pixel is not None:
            log_now = self._last_diag_log_time is None or \
                (now - self._last_diag_log_time).nanoseconds / 1e9 >= 2.0
            if log_now:
                self._last_diag_log_time = now
                topic, row, col = nearest_pixel
                self.get_logger().info(
                    f'[diag] wciaz omijam {self.lateral_time:.1f}s | '
                    f'nearest={nearest_m:.2f}m na ({row},{col}) w {topic}')

        if should_avoid:

            self.returning = False
            self._sync_velocity_control(True)
            self._set_avoiding(True)

            if self.lateral_time > self.max_avoid_duration:
                if not self.timed_out:
                    self.timed_out = True
                    self.get_logger().error(
                        f'Omijanie trwa juz {self.lateral_time:.1f}s bez powodzenia - '
                        f'zatrzymuje sie calkowicie zamiast leciec dalej w jedna strone. '
                        f'Sprawdz odczyt glebi.')
                self._publish_velocity(0.0, 0.0, 0.0, 0.0)
                return

            if nearest_m <= self.stop_distance:
                vx = 0.0
            else:
                ratio = (nearest_m - self.stop_distance) / (self.slow_distance - self.stop_distance)
                vx = self.cruise_speed * max(0.0, min(1.0, ratio))

            if abs(steer) >= self.steer_deadband:
                self.committed_steer_sign = 1.0 if steer > 0.0 else -1.0
            elif self.committed_steer_sign == 0.0 and nearest_m <= self.stop_distance:
                self.committed_steer_sign = 1.0 if steer >= 0.0 else -1.0

            vy = self.avoid_speed * self.committed_steer_sign
            self.lateral_time += dt
            self._publish_velocity(vx, vy, 0.0, 0.0)
            return

        if self.avoiding and still_close:
            vy = self.avoid_speed * self.committed_steer_sign
            self.lateral_time += dt
            self._publish_velocity(self.cruise_speed * self.careful_speed_frac, vy, 0.0, 0.0)
            return

        self._set_avoiding(False)

        if self.returning or self.lateral_time > 0.05:
            if not self.returning:
                self.returning = True
                self.return_remaining = self.lateral_time
                self.lateral_time = 0.0
                self.timed_out = False
                self.get_logger().warn(
                    f'Wracam na pierwotna linie lotu: {self.return_remaining:.2f}s '
                    f'korekty w kierunku {"lewo" if self.committed_steer_sign > 0 else "prawo"}')
                self.avoiding_publisher.publish(Bool(data=True))

            if self.return_remaining > 0.0:
                vy = -self.avoid_speed * self.committed_steer_sign
                self._publish_velocity(self.cruise_speed, vy, 0.0, 0.0)
                self.return_remaining -= dt
                return

            self.get_logger().info('Powrot na linie lotu zakonczony')
            self.returning = False
            self.committed_steer_sign = 0.0
            self.avoiding_publisher.publish(Bool(data=False))
            return

    def _set_avoiding(self, avoiding: bool):
        if avoiding != self.avoiding:
            self.get_logger().warn('Przeszkoda wykryta - wlaczam omijanie') if avoiding \
                else self.get_logger().info('Droga wolna - konczę omijanie')
        self.avoiding = avoiding
        self.avoiding_publisher.publish(Bool(data=avoiding))

    def _sync_velocity_control(self, enable: bool):
        if self.velocity_control_confirmed == enable or self.toggle_pending:
            return
        if not self.toggle_velocity_control_client.service_is_ready():
            return

        self.toggle_pending = True
        future = self.toggle_velocity_control_client.call_async(
            ToggleVelocityControl.Request())
        future.add_done_callback(lambda f: self._on_toggle_response(f, enable))

    def _on_toggle_response(self, future, enable: bool):
        self.toggle_pending = False
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().error(f'Toggle velocity control nie powiodl sie: {e}')
            return

        self.velocity_control_confirmed = resp.result
        self.get_logger().info(f'Tryb predkosciowy potwierdzony: {resp.result}')

        if resp.result != enable:
            self._sync_velocity_control(enable)

    def _publish_velocity(self, vx: float, vy: float, vz: float, yaw: float):
        msg = VelocityVectors()
        msg.vx = float(vx)
        msg.vy = float(vy)
        msg.vz = float(vz)
        msg.yaw = float(yaw)
        self.velocity_vectors_publisher.publish(msg)

    def set_guard_callback(self, request, response):
        self.guard_active = request.mode.upper() in ['ON', 'TRUE', '1', 'ENABLE']
        self.get_logger().info(f"Guard active: {self.guard_active}")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceOak()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
