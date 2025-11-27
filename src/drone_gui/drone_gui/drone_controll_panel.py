#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FOLLOW ARUCO SIMULATOR - GUI do śledzenia markera ArUco

GUI pozwala na:
- Zmianę parametrów regulatora (kp, max_vel, deadband)
- Zadawanie wysokości lotu
- Start/Stop śledzenia
- Arm/Disarm, Takeoff/Land
- Włączanie/wyłączanie sterowania wektorami
"""

import sys
import time
import rclpy
from rclpy.node import Node

from drone_interfaces.msg import MiddleOfAruco, VelocityVectors, Telemetry
from drone_interfaces.srv import ToggleVelocityControl
# Import DroneController z drone_autonomy (jest to zależność)
from drone_autonomy.drone_comunication.drone_controller import DroneController

from PyQt5.QtWidgets import (QApplication, QMainWindow, QPushButton, QLabel, 
                             QVBoxLayout, QHBoxLayout, QWidget, QSlider,
                             QDoubleSpinBox, QGroupBox, QGridLayout, QFormLayout)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QFont, QColor, QPalette


def clamp(v, lo, hi):
    """Ograniczenie wartości do przedziału [lo, hi]"""
    return max(lo, min(hi, v))


class FollowArucoSimulator(DroneController):
    def __init__(self):
        super().__init__()
        
        # Inicjalizacja zmiennych telemetrii (aby uniknąć AttributeError przed pierwszym callbackiem)
        self.flight_mode = "UNKNOWN"
        self.battery_percentage = 0
        self.alt = 0.0
        self.speed = 0.0
        
        # Callback do GUI dla logów
        self.gui_log_callback = None

        # ---- Parametry ----
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('aruco_topic', '/aruco_markers')

        self.img_w = int(self.get_parameter('image_width').value)
        self.img_h = int(self.get_parameter('image_height').value)
        self.cx = self.img_w / 2.0
        self.cy = self.img_h / 2.0
        self.aruco_topic = str(self.get_parameter('aruco_topic').value)

        # Parametry dynamiczne (kontrolowane z GUI) - UPROSZCZONE
        self.kp = 0.25  # Niższe wzmocnienie = stabilniejszy lot
        self.deadband_px = 50  # Większa strefa martwa
        self.max_vel = 0.6  # Mniejsza maksymalna prędkość
        self.vel_smooth = 0.5  # Wygładzanie (0=szybka reakcja, 0.9=bardzo gładkie)
        self.lowpass = 0.3     # Filtr dolnoprzepustowy dla pozycji markera
        self.lost_timeout = 1.0
        self.target_alt = 5.0
        
        # UWAGA: Jeśli dron krąży przy większych prędkościach, zwiększ vel_smooth do 0.7-0.8
        # To spowoduje wolniejszą reakcję ale stabilniejszy lot

        # ---- Subskrypcja markera ----
        self.sub = self.create_subscription(MiddleOfAruco, self.aruco_topic, self.on_marker, 10)
        
        # ---- Subskrypcja telemetrii ----
        # USUNIĘTO: self.sub_telemetry = self.create_subscription(Telemetry, '/drone/telemetry', self.on_telemetry, 10)
        # Korzystamy z telemetrii z klasy bazowej DroneController (self.alt, self.speed, etc.)

        # Stan filtrowany błędu (normalized)
        self.ex_f = 0.0
        self.ey_f = 0.0
        self.last_seen = 0.0
        self.marker_visible = False

        # Flaga śledzenia
        self.tracking_active = False
        
        # Timer sterowania - zawsze aktywny
        self.timer = self.create_timer(0.1, self.control_loop)

        # Stan drona (do wygładzania)
        self.vx_smooth = 0.0
        self.vy_smooth = 0.0
        self.vx_current = 0.0
        self.vy_current = 0.0
        self.vz_current = 0.0

        # Ręczne sterowanie
        self.manual_control_active = False
        self.manual_vx = 0.0
        self.manual_vy = 0.0
        self.manual_vz = 0.0
        
        # Flaga trwającego obrotu (aby wstrzymać wysyłanie wektorów)
        self.yaw_in_progress = False

        self.get_logger().info(f"Follow Aruco Simulator uruchomiony")
    
    def log_to_gui(self, msg, level='info'):
        """Wyślij log do GUI jeśli callback jest ustawiony"""
        # Logowanie tylko do konsoli, usunięto log_label z GUI
        if level == 'info':
            self.get_logger().info(msg)
        elif level == 'warn':
            self.get_logger().warn(msg)
        elif level == 'error':
            self.get_logger().error(msg)

    # ──────────────────────────────────────────────────────────
    # USUNIĘTO: def on_telemetry(self, msg: Telemetry):
    # Korzystamy z _telemetry_cb w DroneController

    def on_marker(self, msg: MiddleOfAruco):
        """Callback odbierający pozycję markera ArUco"""
        # Błąd w pikselach względem środka obrazu
        ex_px = float(msg.x) - self.cx
        ey_px = float(msg.y) - self.cy

        # Martwa strefa
        if abs(ex_px) < self.deadband_px:
            ex_px = 0.0
        if abs(ey_px) < self.deadband_px:
            ey_px = 0.0

        # Normalizacja do [-1, 1]
        ex = ex_px / (self.img_w / 2.0)
        ey = ey_px / (self.img_h / 2.0)

        # Filtr dolnoprzepustowy
        a = clamp(self.lowpass, 0.0, 1.0)
        self.ex_f = (1 - a) * self.ex_f + a * ex
        self.ey_f = (1 - a) * self.ey_f + a * ey

        self.last_seen = time.time()
        self.marker_visible = True

    # ──────────────────────────────────────────────────────────
    def control_loop(self):
        """Główna pętla sterowania"""
        # Jeśli trwa obrót (Yaw), nie wysyłaj wektorów prędkości (bo nadpisują yaw_rate=0)
        if self.yaw_in_progress:
            return

        # Jeśli ręczne sterowanie jest aktywne, użyj go
        if self.manual_control_active:
            self.send_vectors(self.manual_vx, self.manual_vy, self.manual_vz)
            self.vx_current = self.manual_vx
            self.vy_current = self.manual_vy
            self.vz_current = self.manual_vz
            return
        
        # Jeśli śledzenie nie jest aktywne, wyślij zero
        if not self.tracking_active:
            self.send_vectors(0.0, 0.0, 0.0)
            return

        # Sprawdź czy marker jest widoczny
        if self.last_seen == 0.0 or (time.time() - self.last_seen) > self.lost_timeout:
            self.marker_visible = False
            self.send_vectors(0.0, 0.0, 0.0)
            self.vx_current = 0.0
            self.vy_current = 0.0
            self.vz_current = 0.0
            return

        # PROSTY regulator P (bez D, bez komplikacji)
        vx_target = -self.kp * self.ey_f  # Błąd Y -> prędkość X (forward/back)
        vy_target = +self.kp * self.ex_f  # Błąd X -> prędkość Y (left/right)

        # Ogranicz do max_vel
        vx_target = clamp(vx_target, -self.max_vel, self.max_vel)
        vy_target = clamp(vy_target, -self.max_vel, self.max_vel)

        # Wygładzanie (exponential moving average) - zapobiega skokowym zmianom
        alpha = clamp(self.vel_smooth, 0.0, 0.9)
        self.vx_smooth = (1 - alpha) * vx_target + alpha * self.vx_smooth
        self.vy_smooth = (1 - alpha) * vy_target + alpha * self.vy_smooth

        # Zapisz dla GUI
        self.vx_current = self.vx_smooth
        self.vy_current = self.vy_smooth
        self.vz_current = 0.0

        # Wyślij wygładzone prędkości
        self.send_vectors(self.vx_smooth, self.vy_smooth, 0.0)

    # ──────────────────────────────────────────────────────────
    def start_tracking(self):
        """Rozpocznij śledzenie"""
        if not self.tracking_active:
            self.tracking_active = True
            self.get_logger().info("Śledzenie włączone")

    def stop_tracking(self):
        """Zatrzymaj śledzenie"""
        if self.tracking_active:
            self.tracking_active = False
            self.send_vectors(0.0, 0.0, 0.0)
            self.vx_current = 0.0
            self.vy_current = 0.0
            self.vz_current = 0.0
            self.get_logger().info("Śledzenie wyłączone")

    # ──────────────────────────────────────────────────────────
    # Ręczne sterowanie
    # ──────────────────────────────────────────────────────────
    
    def set_manual_control(self, vx, vy, vz):
        """Ustaw ręczne sterowanie"""
        self.manual_control_active = True
        self.manual_vx = vx
        self.manual_vy = vy
        self.manual_vz = vz
    
    def stop_manual_control(self):
        """Zatrzymaj ręczne sterowanie"""
        self.manual_control_active = False
        self.manual_vx = 0.0
        self.manual_vy = 0.0
        self.manual_vz = 0.0

    # ──────────────────────────────────────────────────────────
    # Non-blocking wersje dla GUI
    # ──────────────────────────────────────────────────────────
    
    def arm_async(self):
        """Non-blocking arm - wysyła komendę i zwraca natychmiast"""
        self.get_logger().info('Setting mode to GUIDED...')
        # Najpierw ustaw tryb GUIDED
        from drone_interfaces.srv import SetMode
        req = SetMode.Request()
        req.mode = 'GUIDED'
        future = self._mode_client.call_async(req)
        # Po ustawieniu trybu, wyślij akcję ARM
        future.add_done_callback(lambda f: self._send_arm_goal())
        return True
    
    def _send_arm_goal(self):
        """Wyślij akcję ARM po ustawieniu trybu GUIDED"""
        from drone_interfaces.action import Arm
        self.get_logger().info('Arming drone...')
        goal = Arm.Goal()
        send_future = self._arm_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_arm_goal_response)
    
    def _on_arm_goal_response(self, future):
        """Callback po wysłaniu arm goal"""
        try:
            goal_handle = future.result()
            if goal_handle.accepted:
                self.get_logger().info('ARM goal accepted')
                # Monitoruj wynik
                result_future = goal_handle.get_result_async()
                result_future.add_done_callback(self._on_arm_result)
            else:
                self.get_logger().warn('ARM goal rejected')
        except Exception as e:
            self.get_logger().error(f'ARM goal error: {e}')
    
    def _on_arm_result(self, future):
        """Callback po zakończeniu ARM"""
        try:
            result = future.result()
            self.log_to_gui(f'✓ ARM completed with status: {result.status}', 'info')
            # Po ARM włącz velocity control
            self.toggle_velocity_control_async()
        except Exception as e:
            self.log_to_gui(f'✗ ARM result error: {e}', 'error')
    
    def toggle_velocity_control_async(self):
        """Non-blocking toggle velocity control"""
        from drone_interfaces.srv import ToggleVelocityControl
        self.get_logger().info('Enabling velocity control...')
        req = ToggleVelocityControl.Request()
        future = self.toggle_velocity_control_cli.call_async(req)
        future.add_done_callback(self._on_velocity_control_response)
    
    def _on_velocity_control_response(self, future):
        """Callback po włączeniu velocity control"""
        try:
            result = future.result()
            if result.result:
                self.log_to_gui('✓ Velocity control ENABLED', 'info')
            else:
                self.log_to_gui('✗ Velocity control DISABLED', 'warn')
        except Exception as e:
            self.log_to_gui(f'✗ Velocity control error: {e}', 'error')
    
    def takeoff_async(self, altitude: float):
        """Non-blocking takeoff - wysyła goal i zwraca natychmiast"""
        self.get_logger().info(f'Takeoff async to {altitude} m')
        from drone_interfaces.action import Takeoff
        goal = Takeoff.Goal()
        goal.altitude = altitude
        # Wyślij goal bez czekania
        send_future = self._takeoff_client.send_goal_async(goal)
        # Możemy dodać callback jeśli chcemy
        send_future.add_done_callback(self._on_takeoff_goal_response)
        return True
    
    def land_async(self):
        """Non-blocking land - wysyła komendę i zwraca natychmiast"""
        self.get_logger().info('Land async...')
        # land() wywołuje _set_mode('LAND')
        from drone_interfaces.srv import SetMode
        req = SetMode.Request()
        req.mode = 'LAND'
        future = self._mode_client.call_async(req)
        future.add_done_callback(lambda f: self.get_logger().info('LAND command sent'))
        return True
    
    def _on_takeoff_goal_response(self, future):
        """Callback po wysłaniu takeoff goal"""
        try:
            goal_handle = future.result()
            if goal_handle.accepted:
                self.get_logger().info('Takeoff goal accepted')
                # Możemy monitorować wynik jeśli chcemy
                result_future = goal_handle.get_result_async()
                result_future.add_done_callback(self._on_takeoff_result)
            else:
                self.get_logger().warn('Takeoff goal rejected')
        except Exception as e:
            self.get_logger().error(f'Takeoff goal error: {e}')
    
    def _on_takeoff_result(self, future):
        """Callback po zakończeniu takeoff"""
        try:
            result = future.result()
            self.get_logger().info(f'Takeoff completed with status: {result.status}')
        except Exception as e:
            self.get_logger().error(f'Takeoff result error: {e}')

    def set_yaw_async(self, yaw: float, relative: bool = True):
        """Non-blocking set yaw"""
        self.get_logger().info(f'Setting yaw async to {yaw} rad, relative={relative}')
        self.yaw_in_progress = True  # Zablokuj wysyłanie wektorów
        from drone_interfaces.action import SetYawAction
        goal = SetYawAction.Goal(yaw=yaw, relative=relative)
        send_future = self._yaw_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_yaw_goal_response)
        return True

    def _on_yaw_goal_response(self, future):
        try:
            goal_handle = future.result()
            if goal_handle.accepted:
                self.get_logger().info('Yaw goal accepted')
                result_future = goal_handle.get_result_async()
                result_future.add_done_callback(self._on_yaw_result)
            else:
                self.get_logger().warn('Yaw goal rejected')
                self.yaw_in_progress = False  # Odblokuj w przypadku odrzucenia
        except Exception as e:
            self.get_logger().error(f'Yaw goal error: {e}')
            self.yaw_in_progress = False

    def _on_yaw_result(self, future):
        try:
            result = future.result()
            self.get_logger().info(f'Yaw completed with status: {result.status}')
        except Exception as e:
            self.get_logger().error(f'Yaw result error: {e}')
        finally:
            self.yaw_in_progress = False  # Odblokuj po zakończeniu (sukces lub błąd)

    def destroy_node(self):
        """Cleanup"""
        if self.timer is not None:
            self.timer.cancel()
        try:
            self.send_vectors(0.0, 0.0, 0.0)
        except Exception:
            pass
        super().destroy_node()


# ═════════════════════════════════════════════════════════════
# GUI
# ═════════════════════════════════════════════════════════════

class FollowArucoGUI(QMainWindow):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        
        # Ustaw callback do logów
        self.ros_node.gui_log_callback = self.on_ros_log
        
        self.init_ui()
        
        # Timer do spinowania ROS2
        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(self.spin_ros)
        self.ros_timer.start(10)  # 100 Hz
        
        # Timer do aktualizacji GUI
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_display)
        self.update_timer.start(100)  # 10 Hz
        
        # Akcje w toku (dla non-blocking actions)
        self.pending_action = None  # 'arm', 'takeoff', 'land', None

    def init_ui(self):
        """Inicjalizacja interfejsu"""
        self.setWindowTitle('Follow ArUco Simulator')
        self.setGeometry(100, 100, 600, 950)
        
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout()
        central_widget.setLayout(main_layout)

        # ═══ STATUS (NOWY UKŁAD) ═══
        status_group = QGroupBox("Status Drona")
        status_layout = QGridLayout()
        
        # Wiersz 1: Tryb i Bateria
        self.mode_label = QLabel("Mode: UNKNOWN")
        self.mode_label.setStyleSheet("font-weight: bold; font-size: 12pt;")
        status_layout.addWidget(self.mode_label, 0, 0)
        
        self.battery_label = QLabel("Bat: 0%")
        self.battery_label.setAlignment(Qt.AlignRight)
        status_layout.addWidget(self.battery_label, 0, 1)
        
        # Wiersz 2: Wysokość i Prędkość (z telemetrii)
        self.telemetry_alt_label = QLabel("Alt: 0.0 m")
        status_layout.addWidget(self.telemetry_alt_label, 1, 0)
        
        self.telemetry_speed_label = QLabel("Speed: 0.0 m/s")
        self.telemetry_speed_label.setAlignment(Qt.AlignRight)
        status_layout.addWidget(self.telemetry_speed_label, 1, 1)
        
        # Wiersz 3: Status Śledzenia (Duży)
        self.tracking_status_label = QLabel("ŚLEDZENIE: WYŁĄCZONE")
        self.tracking_status_label.setAlignment(Qt.AlignCenter)
        self.tracking_status_label.setStyleSheet("background-color: #ddd; color: #333; font-weight: bold; padding: 5px; border-radius: 4px;")
        status_layout.addWidget(self.tracking_status_label, 2, 0, 1, 2)
        
        # Wiersz 4: Marker
        self.marker_label = QLabel("Marker: NIEWIDOCZNY")
        self.marker_label.setAlignment(Qt.AlignCenter)
        self.marker_label.setStyleSheet("color: red; font-weight: bold;")
        status_layout.addWidget(self.marker_label, 3, 0, 1, 2)

        # Wiersz 5: Wektory prędkości (wysyłane)
        self.velocity_label = QLabel("Cmd Vel: [0.0, 0.0, 0.0]")
        self.velocity_label.setAlignment(Qt.AlignCenter)
        self.velocity_label.setStyleSheet("color: blue; font-family: monospace;")
        status_layout.addWidget(self.velocity_label, 4, 0, 1, 2)

        status_group.setLayout(status_layout)
        main_layout.addWidget(status_group)

        # ═══ STEROWANIE DRONEM ═══
        drone_group = QGroupBox("Sterowanie dronem")
        drone_layout = QGridLayout()
        
        # Wysokość zadana
        drone_layout.addWidget(QLabel("Zadana wys:"), 0, 0)
        self.alt_spin = QDoubleSpinBox()
        self.alt_spin.setRange(1.0, 20.0)
        self.alt_spin.setValue(5.0)
        self.alt_spin.setSuffix(" m")
        self.alt_spin.setDecimals(1)
        self.alt_spin.valueChanged.connect(self.on_alt_changed)
        drone_layout.addWidget(self.alt_spin, 0, 1)
        
        # Przyciski ARM/DISARM
        self.btn_arm = QPushButton("ARM")
        self.btn_arm.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold; padding: 10px;")
        self.btn_arm.clicked.connect(self.on_arm)
        drone_layout.addWidget(self.btn_arm, 1, 0)
        
        self.btn_disarm = QPushButton("DISARM")
        self.btn_disarm.setStyleSheet("background-color: #f44336; color: white; font-weight: bold; padding: 10px;")
        self.btn_disarm.clicked.connect(self.on_disarm)
        drone_layout.addWidget(self.btn_disarm, 1, 1)
        
        # Przyciski TAKEOFF/LAND
        self.btn_takeoff = QPushButton("TAKEOFF")
        self.btn_takeoff.setStyleSheet("background-color: #2196F3; color: white; font-weight: bold; padding: 10px;")
        self.btn_takeoff.clicked.connect(self.on_takeoff)
        drone_layout.addWidget(self.btn_takeoff, 2, 0)
        
        self.btn_land = QPushButton("LAND")
        self.btn_land.setStyleSheet("background-color: #FF9800; color: white; font-weight: bold; padding: 10px;")
        self.btn_land.clicked.connect(self.on_land)
        drone_layout.addWidget(self.btn_land, 2, 1)
        
        # Przycisk VELOCITY CONTROL
        self.btn_velocity = QPushButton("Sterowanie wektorami prędkości")
        self.btn_velocity.setStyleSheet("background-color: #9C27B0; color: white; font-weight: bold; padding: 10px;")
        self.btn_velocity.clicked.connect(self.on_toggle_velocity)
        drone_layout.addWidget(self.btn_velocity, 3, 0, 1, 2)  # Szeroki przycisk na 2 kolumny
        
        drone_group.setLayout(drone_layout)
        main_layout.addWidget(drone_group)

        # ═══ PARAMETRY REGULATORA ═══
        params_group = QGroupBox("Parametry śledzenia")
        params_layout = QGridLayout()
        
        # Kp
        params_layout.addWidget(QLabel("Kp (proporcjonalne):"), 0, 0)
        self.kp_spin = QDoubleSpinBox()
        self.kp_spin.setRange(0.1, 2.0)
        self.kp_spin.setValue(0.25)  # Niższe dla stabilności
        self.kp_spin.setSingleStep(0.05)
        self.kp_spin.setDecimals(2)
        self.kp_spin.valueChanged.connect(self.on_kp_changed)
        params_layout.addWidget(self.kp_spin, 0, 1)
        
        # Wygładzanie prędkości
        params_layout.addWidget(QLabel("Wygładzanie:"), 1, 0)
        self.smooth_spin = QDoubleSpinBox()
        self.smooth_spin.setRange(0.0, 0.9)
        self.smooth_spin.setValue(0.5)
        self.smooth_spin.setSingleStep(0.1)
        self.smooth_spin.setDecimals(1)
        self.smooth_spin.valueChanged.connect(self.on_smooth_changed)
        params_layout.addWidget(self.smooth_spin, 1, 1)
        
        # Max velocity
        params_layout.addWidget(QLabel("Max prędkość:"), 2, 0)
        self.maxvel_spin = QDoubleSpinBox()
        self.maxvel_spin.setRange(0.1, 5.0)
        self.maxvel_spin.setValue(0.6)  # Niższe dla stabilności
        self.maxvel_spin.setSingleStep(0.1)
        self.maxvel_spin.setSuffix(" m/s")
        self.maxvel_spin.setDecimals(1)
        self.maxvel_spin.valueChanged.connect(self.on_maxvel_changed)
        params_layout.addWidget(self.maxvel_spin, 2, 1)
        
        # Deadband
        params_layout.addWidget(QLabel("Deadband:"), 3, 0)
        self.deadband_spin = QDoubleSpinBox()
        self.deadband_spin.setRange(0, 100)
        self.deadband_spin.setValue(50)  # Większa strefa martwa
        self.deadband_spin.setSuffix(" px")
        self.deadband_spin.setDecimals(0)
        self.deadband_spin.valueChanged.connect(self.on_deadband_changed)
        params_layout.addWidget(self.deadband_spin, 3, 1)
        
        # Lowpass
        params_layout.addWidget(QLabel("Filtr (lowpass):"), 4, 0)
        self.lowpass_spin = QDoubleSpinBox()
        self.lowpass_spin.setRange(0.0, 1.0)
        self.lowpass_spin.setValue(0.3)
        self.lowpass_spin.setSingleStep(0.1)
        self.lowpass_spin.setDecimals(1)
        self.lowpass_spin.valueChanged.connect(self.on_lowpass_changed)
        params_layout.addWidget(self.lowpass_spin, 4, 1)
        
        params_group.setLayout(params_layout)
        main_layout.addWidget(params_group)

        # ═══ RĘCZNE STEROWANIE (UKŁAD KLAWIATURY) ═══
        manual_group = QGroupBox("Ręczne sterowanie (trzymaj przycisk)")
        manual_layout = QGridLayout()
        
        # Wiersz 1: Q, W, E, R
        # Q - Yaw Left
        self.btn_yaw_left = QPushButton("Q\nObrót L")
        self.btn_yaw_left.setMinimumSize(60, 50)
        self.btn_yaw_left.pressed.connect(lambda: self.on_manual_press('yaw_left'))
        self.btn_yaw_left.released.connect(self.on_manual_release)
        manual_layout.addWidget(self.btn_yaw_left, 0, 0)
        
        # W - Forward
        self.btn_forward = QPushButton("W\nPrzód")
        self.btn_forward.setMinimumSize(60, 50)
        self.btn_forward.pressed.connect(lambda: self.on_manual_press('forward'))
        self.btn_forward.released.connect(self.on_manual_release)
        manual_layout.addWidget(self.btn_forward, 0, 1)
        
        # E - Yaw Right
        self.btn_yaw_right = QPushButton("E\nObrót P")
        self.btn_yaw_right.setMinimumSize(60, 50)
        self.btn_yaw_right.pressed.connect(lambda: self.on_manual_press('yaw_right'))
        self.btn_yaw_right.released.connect(self.on_manual_release)
        manual_layout.addWidget(self.btn_yaw_right, 0, 2)
        
        # R - Up
        self.btn_up_z = QPushButton("R\nGóra")
        self.btn_up_z.setMinimumSize(60, 50)
        self.btn_up_z.pressed.connect(lambda: self.on_manual_press('up'))
        self.btn_up_z.released.connect(self.on_manual_release)
        manual_layout.addWidget(self.btn_up_z, 0, 3)
        
        # Wiersz 2: A, S, D, F
        # A - Left
        self.btn_left_manual = QPushButton("A\nLewo")
        self.btn_left_manual.setMinimumSize(60, 50)
        self.btn_left_manual.pressed.connect(lambda: self.on_manual_press('left'))
        self.btn_left_manual.released.connect(self.on_manual_release)
        manual_layout.addWidget(self.btn_left_manual, 1, 0)
        
        # S - Backward
        self.btn_backward = QPushButton("S\nTył")
        self.btn_backward.setMinimumSize(60, 50)
        self.btn_backward.pressed.connect(lambda: self.on_manual_press('backward'))
        self.btn_backward.released.connect(self.on_manual_release)
        manual_layout.addWidget(self.btn_backward, 1, 1)
        
        # D - Right
        self.btn_right_manual = QPushButton("D\nPrawo")
        self.btn_right_manual.setMinimumSize(60, 50)
        self.btn_right_manual.pressed.connect(lambda: self.on_manual_press('right'))
        self.btn_right_manual.released.connect(self.on_manual_release)
        manual_layout.addWidget(self.btn_right_manual, 1, 2)
        
        # F - Down
        self.btn_down_z = QPushButton("F\nDół")
        self.btn_down_z.setMinimumSize(60, 50)
        self.btn_down_z.pressed.connect(lambda: self.on_manual_press('down'))
        self.btn_down_z.released.connect(self.on_manual_release)
        manual_layout.addWidget(self.btn_down_z, 1, 3)
        
        # Prędkość ręczna i Toggle Control
        manual_layout.addWidget(QLabel("Prędkość ręczna:"), 2, 0, 1, 2)
        self.manual_speed_spin = QDoubleSpinBox()
        self.manual_speed_spin.setRange(0.1, 3.0)
        self.manual_speed_spin.setValue(1.0)
        self.manual_speed_spin.setSuffix(" m/s")
        self.manual_speed_spin.setDecimals(1)
        manual_layout.addWidget(self.manual_speed_spin, 2, 2, 1, 2)
        
        # Zapamiętaj które klawisze są wciśnięte
        self.pressed_manual_keys = set()
        
        manual_group.setLayout(manual_layout)
        main_layout.addWidget(manual_group)

        # ═══ ŚLEDZENIE ═══
        tracking_group = QGroupBox("Śledzenie markera")
        tracking_layout = QVBoxLayout()
        
        self.btn_start_tracking = QPushButton("START ŚLEDZENIE")
        self.btn_start_tracking.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
                color: white;
                font-size: 14pt;
                font-weight: bold;
                padding: 15px;
                border-radius: 5px;
            }
            QPushButton:pressed {
                background-color: #45a049;
            }
        """)
        self.btn_start_tracking.clicked.connect(self.on_start_tracking)
        tracking_layout.addWidget(self.btn_start_tracking)
        
        self.btn_stop_tracking = QPushButton("STOP ŚLEDZENIE")
        self.btn_stop_tracking.setStyleSheet("""
            QPushButton {
                background-color: #f44336;
                color: white;
                font-size: 14pt;
                font-weight: bold;
                padding: 15px;
                border-radius: 5px;
            }
            QPushButton:pressed {
                background-color: #da190b;
            }
        """)
        self.btn_stop_tracking.clicked.connect(self.on_stop_tracking)
        tracking_layout.addWidget(self.btn_stop_tracking)
        
        tracking_group.setLayout(tracking_layout)
        main_layout.addWidget(tracking_group)

    # ═══════════════════════════════════════════════════════
    # Callbacks
    # ═══════════════════════════════════════════════════════
    
    def on_alt_changed(self, value):
        self.ros_node.target_alt = value
    
    def on_kp_changed(self, value):
        self.ros_node.kp = value
    
    def on_smooth_changed(self, value):
        self.ros_node.vel_smooth = value
    
    def on_maxvel_changed(self, value):
        self.ros_node.max_vel = value
    
    def on_deadband_changed(self, value):
        self.ros_node.deadband_px = int(value)
    
    def on_lowpass_changed(self, value):
        self.ros_node.lowpass = value
    
    def on_arm(self):
        try:
            self.ros_node.arm_async()
            self.ros_node.get_logger().info("ARM (async)")
            self.pending_action = 'arm'
        except Exception as e:
            self.ros_node.get_logger().error(f"ARM failed: {e}")
    
    def on_disarm(self):
        try:
            # Zatrzymaj śledzenie i wyślij zero
            self.ros_node.stop_tracking()
            self.ros_node.send_vectors(0.0, 0.0, 0.0)
            # DISARM nie istnieje w DroneController, więc po prostu zatrzymujemy
            self.ros_node.get_logger().info("Stopped (DISARM not available)")
            self.pending_action = None  # Clear pending action
        except Exception as e:
            self.ros_node.get_logger().error(f"Stop failed: {e}")
    
    def on_takeoff(self):
        try:
            alt = self.alt_spin.value()
            self.ros_node.takeoff_async(alt)
            self.ros_node.get_logger().info(f"TAKEOFF do {alt}m (async)")
            self.pending_action = 'takeoff'
        except Exception as e:
            self.ros_node.get_logger().error(f"TAKEOFF failed: {e}")
    
    def on_land(self):
        try:
            self.ros_node.land_async()
            self.ros_node.get_logger().info("LAND (async)")
            self.pending_action = 'land'
        except Exception as e:
            self.ros_node.get_logger().error(f"LAND failed: {e}")
    
    def on_toggle_velocity(self):
        """Włącz/wyłącz sterowanie wektorami prędkości"""
        try:
            self.ros_node.toggle_velocity_control_async()
        except Exception as e:
            self.ros_node.get_logger().error(f"Toggle velocity failed: {e}")
    
    def on_start_tracking(self):
        self.ros_node.start_tracking()
    
    def on_stop_tracking(self):
        self.ros_node.stop_tracking()
    
    # ═══════════════════════════════════════════════════════
    # Ręczne sterowanie
    # ═══════════════════════════════════════════════════════
    
    def on_manual_press(self, direction):
        """Obsługa wciśnięcia przycisku ręcznego sterowania"""
        speed = self.manual_speed_spin.value()
        vx, vy, vz = 0.0, 0.0, 0.0
        
        # Obsługa YAW (obrót)
        if direction == 'yaw_left':
            # Obrót w lewo o 10 stopni
            self.ros_node.set_yaw_async(-0.17, relative=True) # ~10 deg in rad
            return # Nie ustawiamy prędkości liniowej
        elif direction == 'yaw_right':
            # Obrót w prawo o 10 stopni
            self.ros_node.set_yaw_async(0.17, relative=True)
            return

        if direction == 'forward':
            vx = speed
        elif direction == 'backward':
            vx = -speed
        elif direction == 'left':
            vy = speed
        elif direction == 'right':
            vy = -speed
        elif direction == 'up':
            vz = -speed  # -down = up
        elif direction == 'down':
            vz = speed  # +down = down
        # YAW na razie pominięty - dodamy później
        
        self.ros_node.set_manual_control(vx, vy, vz)
    
    def on_manual_release(self):
        """Obsługa puszczenia przycisku ręcznego sterowania"""
        self.ros_node.stop_manual_control()
    
    def keyPressEvent(self, event):
        """Obsługa wciśnięcia klawisza"""
        if event.isAutoRepeat():
            return
        
        key = event.key()
        if key in self.pressed_manual_keys:
            return
        
        self.pressed_manual_keys.add(key)
        
        if key == Qt.Key_W:
            self.on_manual_press('forward')
        elif key == Qt.Key_S:
            self.on_manual_press('backward')
        elif key == Qt.Key_A:
            self.on_manual_press('left')
        elif key == Qt.Key_D:
            self.on_manual_press('right')
        elif key == Qt.Key_Q:
            self.on_manual_press('yaw_left')
        elif key == Qt.Key_E:
            self.on_manual_press('yaw_right')
        elif key == Qt.Key_R:
            self.on_manual_press('up')
        elif key == Qt.Key_F:
            self.on_manual_press('down')
    
    def keyReleaseEvent(self, event):
        """Obsługa puszczenia klawisza"""
        if event.isAutoRepeat():
            return
        
        key = event.key()
        if key in self.pressed_manual_keys:
            self.pressed_manual_keys.remove(key)
            self.on_manual_release()
    
    # ═══════════════════════════════════════════════════════
    # Update GUI
    # ═══════════════════════════════════════════════════════
    
    def update_display(self):
        """Aktualizuj wyświetlane informacje"""
        # 1. Status Drona (Mode, Battery, Alt, Speed)
        # Korzystamy z atrybutów klasy bazowej DroneController
        self.mode_label.setText(f"Mode: {self.ros_node.flight_mode}")
        self.battery_label.setText(f"Bat: {self.ros_node.battery_percentage}%")
        
        # Kolorowanie baterii
        if self.ros_node.battery_percentage < 20:
            self.battery_label.setStyleSheet("color: red; font-weight: bold;")
        elif self.ros_node.battery_percentage < 50:
            self.battery_label.setStyleSheet("color: orange;")
        else:
            self.battery_label.setStyleSheet("color: green;")

        self.telemetry_alt_label.setText(f"Alt: {self.ros_node.alt:.1f} m")
        self.telemetry_speed_label.setText(f"Speed: {self.ros_node.speed:.1f} m/s")

        # 2. Status Śledzenia
        if self.pending_action:
            self.tracking_status_label.setText(f"AKCJA: {self.pending_action.upper()}...")
            self.tracking_status_label.setStyleSheet("color: orange; font-weight: bold; font-size: 14pt;")
        elif self.ros_node.manual_control_active:
            self.tracking_status_label.setText("STEROWANIE RĘCZNE")
            self.tracking_status_label.setStyleSheet("color: blue; font-weight: bold; font-size: 14pt;")
        elif self.ros_node.tracking_active:
            self.tracking_status_label.setText("ŚLEDZENIE AKTYWNE")
            self.tracking_status_label.setStyleSheet("color: green; font-weight: bold; font-size: 14pt;")
        else:
            self.tracking_status_label.setText("OCZEKIWANIE")
            self.tracking_status_label.setStyleSheet("color: gray; font-weight: bold; font-size: 14pt;")

        # 3. Marker
        if self.ros_node.marker_visible:
            err_x = self.ros_node.ex_f
            err_y = self.ros_node.ey_f
            self.marker_label.setText(f"Marker: WIDOCZNY | Błąd: ({err_x:.2f}, {err_y:.2f})")
            self.marker_label.setStyleSheet("color: green; font-weight: bold;")
        else:
            self.marker_label.setText("Marker: NIEWIDOCZNY")
            self.marker_label.setStyleSheet("color: red; font-weight: bold;")
        
        # 4. Prędkość zadana
        vx = self.ros_node.vx_current
        vy = self.ros_node.vy_current
        vz = self.ros_node.vz_current
        self.velocity_label.setText(f"Zadana prędkość: ({vx:.2f}, {vy:.2f}, {vz:.2f}) m/s")
    
    def spin_ros(self):
        """Spinuj ROS2 node"""
        rclpy.spin_once(self.ros_node, timeout_sec=0)
    
    def on_ros_log(self, msg, level='info'):
        """Callback do wyświetlania logów ROS w GUI"""
        # Ustaw kolor w zależności od poziomu
        if level == 'error':
            color = 'red'
        elif level == 'warn':
            color = 'orange'
        else:
            color = 'blue'
        
        self.log_label.setText(f"Log: {msg}")
        self.log_label.setStyleSheet(f"color: {color}; font-size: 9pt;")
    
    def closeEvent(self, event):
        """Cleanup przy zamykaniu"""
        self.ros_timer.stop()
        self.update_timer.stop()
        self.ros_node.stop_tracking()
        event.accept()


# ═════════════════════════════════════════════════════════════
# Main
# ═════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    
    # Utwórz ROS2 node
    simulator = FollowArucoSimulator()
    
    # Utwórz aplikację Qt
    app = QApplication(sys.argv)
    
    # Utwórz GUI
    gui = FollowArucoGUI(simulator)
    gui.show()
    
    # Uruchom aplikację
    try:
        exit_code = app.exec_()
    except KeyboardInterrupt:
        pass
    finally:
        simulator.destroy_node()
        rclpy.shutdown()
        sys.exit(exit_code)


if __name__ == '__main__':
    main()
