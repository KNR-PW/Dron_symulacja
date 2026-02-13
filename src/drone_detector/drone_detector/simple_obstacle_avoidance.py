#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Simple Obstacle Avoidance - Prosty filtr bezpieczeństwa dla drona.

Funkcje:
1. Filtruje topic velocity_vectors_user -> velocity_vectors
   - Ogranicza prędkość liniowo gdy przeszkoda jest blisko
   - Nie ingeruje w inne tryby lotu (np. goto_relative, missions)
2. Safety Guard (Strażnik Bezpieczeństwa):
   - Nasłuchuje rzeczywistej prędkości (current_velocity)
   - Jeśli wykryje, że dron leci na przeszkodę (w trybie AUTO/GOTO),
     WŁĄCZA tryb velocity i wysyła STOP.
   - Działa na zasadzie fizyki hamowania (d = v^2/2a).

Autor: KNR PW
Data: 2026-01
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from drone_interfaces.msg import VelocityVectors
from drone_interfaces.srv import ToggleVelocityControl, SetMode # Reusing SetMode for simple bool toggle if needed, or create custom service.
# Using standard SetBool would be better but keeping deps minimal. Let's use a simple boolean parameter or custom srv?
# User requested "latwo wlaczyc i wylaczyc". Let's use SetMode with "ON"/"OFF" or just a bool param accessible via ros2 param set.
# Or better: create a simple service using SetMode (mode="ON"/"OFF") as requested in plan.
import numpy as np
import math


class SimpleObstacleAvoidance(Node):
    """
    Prosty node do unikania przeszkód.
    """
    
    def __init__(self):
        super().__init__('simple_obstacle_avoidance')
        
        # =====================================================================
        # PARAMETRY
        # =====================================================================
        
        # Geometria tunelu bezpieczeństwa
        self.declare_parameter('tunnel_radius', 1.0)       # [m] Promień tunelu
        self.declare_parameter('vertical_limit', 0.75)     # [m] Pionowy wycinek
        
        # Parametry hamowania
        self.declare_parameter('stop_margin', 1.5)         # [m] Minimalna odległość zatrzymania (bufor)
        self.declare_parameter('braking_decel', 0.8)       # [m/s²] Opóźnienie hamowania
        self.declare_parameter('max_range', 15.0)          # [m] Zasięg lidaru
        
        self.declare_parameter('guard_enabled', False)      # Czy strażnik jest domyślnie aktywny
        
        # Wczytaj parametry
        self.tunnel_radius = float(self.get_parameter('tunnel_radius').value)
        self.vertical_limit = float(self.get_parameter('vertical_limit').value)
        self.stop_margin = float(self.get_parameter('stop_margin').value)
        self.braking_decel = float(self.get_parameter('braking_decel').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.guard_active = self.get_parameter('guard_enabled').value
        
        # =====================================================================
        # KOMUNIKACJA
        # =====================================================================
        
        # Lidar
        self.lidar_sub = self.create_subscription(PointCloud2, '/lidar', self.lidar_callback, 10)
        
        # Input: User Velocity (teleoperacja / filtr)
        self.cmd_sub = self.create_subscription(VelocityVectors, 'knr_hardware/velocity_vectors_user', self.cmd_callback, 10)
        
        # Input: Real Drone Velocity (od Strażnika)
        self.vel_sub = self.create_subscription(VelocityVectors, 'knr_hardware/current_velocity', self.velocity_guard_callback, 10)
        
        # Output: Filtered/Safe Velocity
        self.cmd_pub = self.create_publisher(VelocityVectors, 'knr_hardware/velocity_vectors', 10)
        
        # Service Client: Przełączanie trybu (wymuszanie velocity control)
        self.toggle_ctrl_client = self.create_client(ToggleVelocityControl, 'knr_hardware/toggle_v_control')
        
        # Service Server: Włączanie/Wyłączanie Strażnika
        self.guard_srv = self.create_service(SetMode, 'set_brake_on_obstacle', self.set_guard_callback)
        
        # =====================================================================
        # STAN
        # =====================================================================
        self.latest_points_2d = None
        
        self.get_logger().info('='*60)
        self.get_logger().info('SimpleObstacleAvoidance READY')
        self.get_logger().info(f'  Guard Active: {self.guard_active}')
        self.get_logger().info(f'  Decel: {self.braking_decel} m/s^2')
        self.get_logger().info('='*60)

    # =========================================================================
    # SERWIS KONFIGURACJI
    # =========================================================================
    def set_guard_callback(self, request, response):
        """Ustawia flagę aktywności strażnika. Mode: 'ON' lub 'OFF'."""
        mode = request.mode.upper()
        if mode in ['ON', 'TRUE', '1', 'ENABLE']:
            self.guard_active = True
            self.get_logger().info("SAFETY GUARD: ENABLED")
        elif mode in ['OFF', 'FALSE', '0', 'DISABLE']:
            self.guard_active = False
            self.get_logger().info("SAFETY GUARD: DISABLED")
        else:
            self.get_logger().warn(f"Unknown mode: {mode}. Use ON/OFF.")
        return response

    # =========================================================================
    # LIDAR PROCESSING
    # =========================================================================
    def lidar_callback(self, msg: PointCloud2):
        self.latest_points_2d = self.process_point_cloud(msg)
    
    def process_point_cloud(self, msg: PointCloud2) -> np.ndarray:
        """Konwersja PointCloud2 -> 2D numpy array (x, -y)."""
        # (Ten kod pozostaje bez zmian logicznych, skopiowany z poprzedniej wersji)
        offset_x, offset_y, offset_z = -1, -1, -1
        for field in msg.fields:
            if field.name == 'x': offset_x = field.offset
            elif field.name == 'y': offset_y = field.offset
            elif field.name == 'z': offset_z = field.offset
        
        if offset_x < 0 or offset_y < 0 or offset_z < 0: return None
        points_num = msg.width * msg.height
        if points_num == 0: return None
        
        data = np.frombuffer(msg.data, dtype=np.uint8)
        if len(data) != points_num * msg.point_step: return None
        
        try:
            points_data = data.reshape(points_num, msg.point_step)
            x = points_data[:, offset_x:offset_x+4].copy().view(np.float32).flatten()
            y = points_data[:, offset_y:offset_y+4].copy().view(np.float32).flatten()
            z = points_data[:, offset_z:offset_z+4].copy().view(np.float32).flatten()
            
            mask = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
            mask &= (np.abs(z) <= self.vertical_limit)
            mask &= (x*x + y*y <= self.max_range**2)
            
            # Zwróć X, -Y (inwersja klatki)
            return np.column_stack((x[mask], -y[mask]))
        except Exception:
            return None

    # =========================================================================
    # LOGIKA UNIKANIA (WSPÓLNA)
    # =========================================================================
    def calculate_safe_speed(self, vx, vy, points) -> float:
        """
        Zwraca maksymalną bezpieczną prędkość w danym kierunku.
        Używa fizyki: d_stop = margin + v^2 / 2a
        Odwracając: v_max = sqrt( 2a * (d_free - margin) )
        """
        speed = math.hypot(vx, vy)
        if speed < 0.05 or points is None or len(points) == 0:
            return 999.0 # No limit
            
        ux, uy = vx/speed, vy/speed
        free_dist = self.find_free_distance(points, ux, uy)
        
        if free_dist == float('inf'):
            return 999.0
            
        braking_distance = free_dist - self.stop_margin
        if braking_distance <= 0:
            return 0.0
            
        # v = sqrt(2 * a * d)
        return math.sqrt(2.0 * self.braking_decel * braking_distance)

    def find_free_distance(self, points: np.ndarray, ux: float, uy: float) -> float:
        """Znajdź wolną przestrzeń w tunelu."""
        px, py = points[:, 0], points[:, 1]
        along = px * ux + py * uy
        cross = np.abs(px * uy - py * ux)
        
        mask = (along > 0) & (cross < self.tunnel_radius)
        if not np.any(mask):
            return float('inf')
            
        # Geometria kolizji z okręgiem
        delta = np.sqrt(np.maximum(self.tunnel_radius**2 - cross[mask]**2, 0))
        dist = along[mask] - delta
        return float(np.min(dist))

    # =========================================================================
    # 1. FILTR KOMEND UŻYTKOWNIKA (MANUAL)
    # =========================================================================
    def cmd_callback(self, msg: VelocityVectors):
        safe_limit = self.calculate_safe_speed(msg.vx, msg.vy, self.latest_points_2d)
        
        current_speed = math.hypot(msg.vx, msg.vy)
        safe_cmd = VelocityVectors()
        safe_cmd.vz = float(msg.vz)
        safe_cmd.yaw = float(msg.yaw)
        
        if current_speed <= safe_limit:
            safe_cmd.vx = float(msg.vx)
            safe_cmd.vy = float(msg.vy)
        else:
            if safe_limit <= 0:
                safe_cmd.vx = 0.0
                safe_cmd.vy = 0.0
            else:
                scale = safe_limit / current_speed
                safe_cmd.vx = float(msg.vx * scale)
                safe_cmd.vy = float(msg.vy * scale)
                
        self.cmd_pub.publish(safe_cmd)

    # =========================================================================
    # 2. SAFETY GUARD (AUTO WSPOMAGANIE)
    # =========================================================================
    def velocity_guard_callback(self, msg: VelocityVectors):
        """Sprawdza czy RZECZYWISTA prędkość nie prowadzi do kolizji."""
        if not self.guard_active:
            return
            
        vx, vy = msg.vx, msg.vy
        speed = math.hypot(vx, vy)
        
        # Ignoruj małe prędkości (szum)
        if speed < 0.2:
            return
            
        limit = self.calculate_safe_speed(vx, vy, self.latest_points_2d)
        
        # WARUNEK INTERWENCJI:
        # Jeśli aktualna prędkość jest wyraźnie większa niż bezpieczna
        # (dodajemy małą histerezę 0.1 m/s żeby nie panikować na granicy)
        if speed > (limit + 0.1):
            self.get_logger().warn(f"OBSTACLE DETECTED! Speed: {speed:.2f} > Limit: {limit:.2f}. BRAKING!")
            self.trigger_emergency_brake()

    def trigger_emergency_brake(self):
        """Sekwencja awaryjnego zatrzymania."""
        # Włącz Velocity Control (przerywa GoTo).
        # Używamy "call_async", a w callbacku sprawdzamy czy udało się włączyć.
        # Jeśli toggle wyłączył (bo był włączony), włączamy ponownie.
        if self.toggle_ctrl_client.service_is_ready():
            future = self.toggle_ctrl_client.call_async(ToggleVelocityControl.Request())
            future.add_done_callback(self._ensure_velocity_mode_enabled)

    def _ensure_velocity_mode_enabled(self, future):
        try:
            resp = future.result()
            is_enabled = resp.result
            
            if not is_enabled:
                # Jeśli toggle wyłączył sterowanie (bo było ON), włączamy je ponownie.
                self.get_logger().info("Toggle turned OFF. Re-toggling to ON...")
                self.toggle_ctrl_client.call_async(ToggleVelocityControl.Request())
            else:
                self.get_logger().info("Velocity Control ACTIVATED.")
                
            # Teraz wyślij STOP (w pętli przez parę cykli albo raz? Raz może zgubić UDP)
            # Najlepiej wysłać serię zer.
            stop_msg = VelocityVectors() # 0,0,0
            # Wyślij kilka razy dla pewności
            for _ in range(5):
                self.cmd_pub.publish(stop_msg)
                
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SimpleObstacleAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
