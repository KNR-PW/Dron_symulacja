#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FOLLOW ARUCO SIMULATOR - Panel Sterowania Dronem
Wersja uproszczona i zoptymalizowana.

Główne funkcje:
- Podgląd telemetrii (bateria, tryb, wysokość, prędkość)
- Sterowanie parametrami regulatora P (Follow ArUco)
- Ręczne sterowanie dronem (klawiatura / przyciski)
- Logi systemowe w GUI
"""

import sys
import time
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import (QApplication, QMainWindow, QPushButton, QLabel, 
                             QVBoxLayout, QHBoxLayout, QWidget,
                             QDoubleSpinBox, QGroupBox, QGridLayout,
                             QFormLayout)
from PyQt5.QtCore import QTimer, Qt

# Import komunikatów i serwisu
from drone_interfaces.msg import MiddleOfAruco
from drone_autonomy.drone_comunication.drone_controller import DroneController


def clamp(v, lo, hi):
    """Pomocnicza funkcja ograniczająca wartość do zakresu [lo, hi]"""
    return max(lo, min(hi, v))


class FollowArucoSimulator(DroneController):
    """
    Logika sterowania dronem. 
    Dziedziczy po DroneController, co daje dostęp do metod arm, takeoff, land, itp.
    """
    def __init__(self):
        super().__init__()
        
        # 1. Inicjalizacja zmiennych (bezpieczne wartości startowe)
        self.flight_mode = "UNKNOWN"
        self.battery_percentage = 0
        self.alt = 0.0
        self.speed = 0.0
        
        # Callback do logowania w GUI
        self.gui_log_callback = None

        # 2. Parametry (ROS2 parameters)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('aruco_topic', '/aruco_markers')

        self.img_w = int(self.get_parameter('image_width').value)
        self.img_h = int(self.get_parameter('image_height').value)
        self.cx = self.img_w / 2.0
        self.cy = self.img_h / 2.0
        self.aruco_topic = str(self.get_parameter('aruco_topic').value)

        # 3. Konfiguracja sterowania (Domyślne wartości)
        self.kp = 0.25          # Wzmocnienie regulatora P
        self.ki = 0.01          # Wzmocnienie regulatora I
        self.kd = 0.05          # Wzmocnienie regulatora D
        self.deadband_px = 50   # Strefa martwa (w pikselach)
        self.max_vel = 0.6      # Maksymalna prędkość (m/s)
        self.vel_smooth = 0.5   # Współczynnik wygładzania prędkości (0.0 - 0.9)
        self.lowpass = 0.3      # Filtr dolnoprzepustowy pozycji markera
        self.lost_timeout = 1.0 # Czas po którym uznajemy marker za utracony
        self.target_alt = 5.0   # Zadana wysokość przelotu
        
        # 4. Subskrypcje
        self.sub_aruco = self.create_subscription(MiddleOfAruco, self.aruco_topic, self.on_marker, 10)
        # Telemetria jest obsługiwana w klasie bazowej DroneController (_telemetry_cb)

        # 5. Zmienne stanu wewnętrznego
        self.ex_f = 0.0
        self.ey_f = 0.0
        self.last_seen = 0.0
        self.marker_visible = False
        self.tracking_active = False
        
        # Stan PID
        self.pid_err_vx_prev = 0.0
        self.pid_err_vy_prev = 0.0
        self.pid_integ_vx = 0.0
        self.pid_integ_vy = 0.0
        self.pid_last_time = time.time()

        self.vx_smooth = 0.0
        self.vy_smooth = 0.0
        self.vx_current = 0.0
        self.vy_current = 0.0
        self.vz_current = 0.0

        # Flagi sterowania ręcznego i akcji
        self.manual_control_active = False
        self.manual_vx = 0.0
        self.manual_vy = 0.0
        self.manual_vz = 0.0
        self.manual_yaw_rate = 0.0
        
        # Stan sterowania wektorowego (czy ArduPilot przyjmuje wektory)
        self.velocity_control_enabled = False

        # Zmienne do wyświetlania
        self.vx_current = 0.0
        self.vy_current = 0.0
        self.vz_current = 0.0
        self.yaw_rate_current = 0.0

        # Timer głównej pętli sterowania (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Follow Aruco Simulator (Logic) READY")

    # -------------------------------------------------------------------------
    # Logika Śledzenia i Sterowania
    # -------------------------------------------------------------------------
    
    def on_marker(self, msg: MiddleOfAruco):
        """Callback pozycji markera - oblicza błąd i filtruje sygnał"""
        # Oblicz błąd w pikselach (od środka obrazu)
        ex_px = float(msg.x) - self.cx
        ey_px = float(msg.y) - self.cy

        # Zastosuj strefę martwą (deadband)
        if abs(ex_px) < self.deadband_px: ex_px = 0.0
        if abs(ey_px) < self.deadband_px: ey_px = 0.0

        # Normalizacja do zakresu [-1, 1] (w przybliżeniu)
        ex = ex_px / (self.img_w / 2.0)
        ey = ey_px / (self.img_h / 2.0)

        # Filtr dolnoprzepustowy (Low-pass filter)
        a = clamp(self.lowpass, 0.0, 1.0)
        self.ex_f = (1 - a) * self.ex_f + a * ex
        self.ey_f = (1 - a) * self.ey_f + a * ey

        self.last_seen = time.time()
        self.marker_visible = True

    def control_loop(self):
        """Główna pętla obliczająca prędkości sterujące"""
        if self._handle_manual_control():
            return

        if not self.tracking_active:
            self._hold_position()
            return

        if self._marker_timed_out():
            self.marker_visible = False
            self._hold_position()
            return

        self._apply_pid_tracking()

    def _update_current_vel(self, vx, vy, vz, yaw):
        """Aktualizuje podgląd prędkości dla GUI"""
        self.vx_current = vx
        self.vy_current = vy
        self.vz_current = vz
        self.yaw_rate_current = yaw

    def _push_velocity(self, vx, vy, vz, yaw):
        """Wysyła wektor prędkości i aktualizuje GUI"""
        self._update_current_vel(vx, vy, vz, yaw)
        self.send_vectors(vx, vy, vz, yaw)

    def _reset_pid_state(self):
        self.pid_integ_vx = 0.0
        self.pid_integ_vy = 0.0
        self.pid_err_vx_prev = 0.0
        self.pid_err_vy_prev = 0.0
        self.pid_last_time = time.time()
        self.vx_smooth = 0.0
        self.vy_smooth = 0.0

    def _handle_manual_control(self):
        if not self.manual_control_active:
            return False
        self._push_velocity(self.manual_vx, self.manual_vy, self.manual_vz, self.manual_yaw_rate)
        return True

    def _hold_position(self):
        self._push_velocity(0.0, 0.0, 0.0, 0.0)

    def _marker_timed_out(self):
        return self.last_seen == 0.0 or (time.time() - self.last_seen) > self.lost_timeout

    def _apply_pid_tracking(self):
        now = time.time()
        dt = now - self.pid_last_time
        if dt <= 0:
            dt = 0.1
        self.pid_last_time = now

        err_vx = -self.ey_f
        err_vy = self.ex_f

        self.pid_integ_vx = clamp(self.pid_integ_vx + err_vx * dt, -1.0, 1.0)
        self.pid_integ_vy = clamp(self.pid_integ_vy + err_vy * dt, -1.0, 1.0)

        deriv_vx = (err_vx - self.pid_err_vx_prev) / dt
        deriv_vy = (err_vy - self.pid_err_vy_prev) / dt
        self.pid_err_vx_prev = err_vx
        self.pid_err_vy_prev = err_vy

        vx_target = (self.kp * err_vx) + (self.ki * self.pid_integ_vx) + (self.kd * deriv_vx)
        vy_target = (self.kp * err_vy) + (self.ki * self.pid_integ_vy) + (self.kd * deriv_vy)

        vx_target = clamp(vx_target, -self.max_vel, self.max_vel)
        vy_target = clamp(vy_target, -self.max_vel, self.max_vel)

        alpha = clamp(self.vel_smooth, 0.0, 0.9)
        self.vx_smooth = (1 - alpha) * vx_target + alpha * self.vx_smooth
        self.vy_smooth = (1 - alpha) * vy_target + alpha * self.vy_smooth

        self._push_velocity(self.vx_smooth, self.vy_smooth, 0.0, 0.0)

    # -------------------------------------------------------------------------
    # Metody Publiczne (API dla GUI)
    # -------------------------------------------------------------------------

    def start_tracking(self):
        if not self.tracking_active:
            self.tracking_active = True
            self._reset_pid_state()
            self.log_to_gui("Śledzenie włączone (PID Reset)", 'info')

    def stop_tracking(self):
        if self.tracking_active:
            self.tracking_active = False
            self._hold_position()
            self._reset_pid_state()
            self.log_to_gui("Śledzenie wyłączone", 'info')

    def set_manual_control(self, vx, vy, vz, yaw):
        self.manual_control_active = True
        self.manual_vx = vx
        self.manual_vy = vy
        self.manual_vz = vz
        self.manual_yaw_rate = yaw

    def stop_manual_control(self):
        self.manual_control_active = False
        self.manual_vx = 0.0
        self.manual_vy = 0.0
        self.manual_vz = 0.0
        self.manual_yaw_rate = 0.0
        self._hold_position()
        self._reset_pid_state()

    def log_to_gui(self, msg, level='info'):
        """Przekazuje logi do GUI"""
        self.get_logger().info(f"[GUI LOG] {msg}")
        if self.gui_log_callback:
            self.gui_log_callback(msg, level)

    # -------------------------------------------------------------------------
    # Asynchroniczne wywołania ROS (Non-blocking)
    # -------------------------------------------------------------------------

    def arm_async(self):
        self.log_to_gui('Rozpoczynam procedurę ARM...', 'info')
        # 1. Ustaw tryb GUIDED
        from drone_interfaces.srv import SetMode
        req = SetMode.Request()
        req.mode = 'GUIDED'
        future = self._mode_client.call_async(req)
        future.add_done_callback(lambda _: self._send_arm_goal())
        return True

    def _send_arm_goal(self):
        # 2. Wyślij ARM action
        from drone_interfaces.action import Arm
        goal = Arm.Goal()
        self._arm_client.send_goal_async(goal).add_done_callback(self._on_arm_goal_response)

    def _on_arm_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.log_to_gui('ARM odrzucony przez kontroler lotu', 'error')
            return
        goal_handle.get_result_async().add_done_callback(self._on_arm_result)

    def _on_arm_result(self, future):
        result = future.result().result
        self.log_to_gui(f'ARM zakończony sukcesem.', 'success')
        # Automatycznie włącz sterowanie wektorami po uzbrojeniu - USUNIĘTE
        # self.toggle_velocity_control_async() 

    def toggle_velocity_control_async(self):
        from drone_interfaces.srv import ToggleVelocityControl
        req = ToggleVelocityControl.Request()
        self.toggle_velocity_control_cli.call_async(req).add_done_callback(self._on_vel_ctrl_response)

    def _on_vel_ctrl_response(self, future):
        try:
            result = future.result()
            if result.result:
                self.velocity_control_enabled = True
                self.log_to_gui('Sterowanie wektorowe WŁĄCZONE', 'success')
            else:
                self.velocity_control_enabled = False
                self.log_to_gui('Sterowanie wektorowe WYŁĄCZONE', 'warn')
        except Exception as e:
            self.log_to_gui(f'Błąd przełączania sterowania: {e}', 'error')

    def takeoff_async(self, altitude):
        self.log_to_gui(f'Startuję (Takeoff) na {altitude}m...', 'info')
        from drone_interfaces.action import Takeoff
        goal = Takeoff.Goal()
        goal.altitude = float(altitude)
        self._takeoff_client.send_goal_async(goal).add_done_callback(self._on_takeoff_goal_response)

    def _on_takeoff_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.log_to_gui('Takeoff odrzucony', 'error')
            return
        self.log_to_gui('Takeoff zaakceptowany, wznoszenie...', 'info')
        goal_handle.get_result_async().add_done_callback(self._on_takeoff_result)

    def _on_takeoff_result(self, future):
        result = future.result().result
        self.log_to_gui(f'Takeoff zakończony.', 'success')
        # Włącz sterowanie wektorowe po osiągnięciu wysokości
        self.toggle_velocity_control_async()

    def land_async(self):
        self.log_to_gui('Lądowanie (LAND)...', 'info')
        from drone_interfaces.srv import SetMode
        req = SetMode.Request()
        req.mode = 'LAND'
        self._mode_client.call_async(req)

    def destroy_node(self):
        if self.timer: self.timer.cancel()
        self.send_vectors(0.0, 0.0, 0.0, 0.0)
        super().destroy_node()


# =============================================================================
# GUI (PyQt5)
# =============================================================================

class FollowArucoGUI(QMainWindow):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.pressed_manual_keys = set()
        
        # Setup UI
        self.init_ui()
        
        # Timery
        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(self.spin_ros)
        self.ros_timer.start(10)  # 100 Hz dla responsywności ROS

        self.ui_timer = QTimer()
        self.ui_timer.timeout.connect(self.update_display)
        self.ui_timer.start(100)  # 10 Hz odświeżanie UI

    def init_ui(self):
        """Buduje interfejs użytkownika"""
        self.setWindowTitle('Follow ArUco - Panel Sterowania')
        self.setGeometry(100, 100, 500, 850)
        
        # Główny widget i layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        self.main_layout = QVBoxLayout()
        central_widget.setLayout(self.main_layout)

        # Sekcje interfejsu
        self.create_status_section()
        self.create_control_section()
        self.create_params_section()
        self.create_manual_control_section()
        self.create_tracking_section()
        
        # Ustawienie fokusu na główne okno, aby przechwytywać klawisze
        self.setFocusPolicy(Qt.StrongFocus)
        self.setFocus()

    def mousePressEvent(self, event):
        """Kliknięcie w tło przywraca fokus do okna (dla sterowania klawiaturą)"""
        self.setFocus()
        super().mousePressEvent(event)

    def create_status_section(self):
        group = QGroupBox("Status Drona")
        layout = QGridLayout()
        
        # Wiersz 1: Wysokość i Prędkość
        self.alt_label = QLabel("Alt: 0.0 m")
        self.speed_label = QLabel("Speed: 0.0 m/s")
        self.speed_label.setAlignment(Qt.AlignRight)
        layout.addWidget(self.alt_label, 0, 0)
        layout.addWidget(self.speed_label, 0, 1)
        
        # Wiersz 2: Status Markera
        self.marker_label = QLabel("MARKER: SZUKAM...")
        self.marker_label.setAlignment(Qt.AlignCenter)
        self.marker_label.setStyleSheet("background-color: #ddd; padding: 5px; border-radius: 4px;")
        layout.addWidget(self.marker_label, 1, 0, 1, 2)

        # Wiersz 3: Komendy Prędkości
        self.cmd_vel_label = QLabel("Cmd: [0.0, 0.0, 0.0]")
        self.cmd_vel_label.setAlignment(Qt.AlignCenter)
        self.cmd_vel_label.setStyleSheet("color: blue; font-family: monospace;")
        layout.addWidget(self.cmd_vel_label, 2, 0, 1, 2)
        
        group.setLayout(layout)
        self.main_layout.addWidget(group)

    def _bind_spinbox(self, spinbox, attr_name):
        spinbox.valueChanged.connect(lambda value, name=attr_name: setattr(self.ros_node, name, value))

    def create_control_section(self):
        group = QGroupBox("Podstawowe Sterowanie")
        layout = QGridLayout()
        
        # Wybór wysokości
        layout.addWidget(QLabel("Wysokość Startu:"), 0, 0)
        self.alt_spin = QDoubleSpinBox()
        self.alt_spin.setRange(0.5, 50.0)
        self.alt_spin.setValue(5.0)
        self.alt_spin.setSuffix(" m")
        layout.addWidget(self.alt_spin, 0, 1)
        
        # Przyciski
        self.btn_arm = QPushButton("ARM")
        self.btn_arm.setStyleSheet("background-color: #4CAF50; color: white;")
        self.btn_arm.setFocusPolicy(Qt.NoFocus)
        self.btn_arm.clicked.connect(lambda: self.ros_node.arm_async())

        self.btn_takeoff = QPushButton("TAKEOFF")
        self.btn_takeoff.setStyleSheet("background-color: #2196F3; color: white;")
        self.btn_takeoff.setFocusPolicy(Qt.NoFocus)
        self.btn_takeoff.clicked.connect(lambda: self.ros_node.takeoff_async(self.alt_spin.value()))
        
        self.btn_land = QPushButton("LAND")
        self.btn_land.setStyleSheet("background-color: #FF9800; color: white;")
        self.btn_land.setFocusPolicy(Qt.NoFocus)
        self.btn_land.clicked.connect(lambda: self.ros_node.land_async())
        
        layout.addWidget(self.btn_arm, 1, 0)
        layout.addWidget(self.btn_takeoff, 1, 1)
        layout.addWidget(self.btn_land, 2, 0, 1, 2)

        # Przycisk sterowania wektorowego
        self.btn_vel_ctrl = QPushButton("WŁĄCZ Sterowanie Wektorowe")
        self.btn_vel_ctrl.setStyleSheet("background-color: #9C27B0; color: white; font-weight: bold; padding: 5px;")
        self.btn_vel_ctrl.setFocusPolicy(Qt.NoFocus)
        self.btn_vel_ctrl.clicked.connect(lambda: self.ros_node.toggle_velocity_control_async())
        layout.addWidget(self.btn_vel_ctrl, 3, 0, 1, 2)
        
        group.setLayout(layout)
        self.main_layout.addWidget(group)

    def create_params_section(self):
        group = QGroupBox("Parametry Lotu")
        layout = QFormLayout()
        param_specs = [
            {"label": "Kp (Proporcjonalny):", "attr": "kp", "min": 0.0, "max": 5.0, "step": 0.05, "value": self.ros_node.kp},
            {"label": "Ki (Całkujący):", "attr": "ki", "min": 0.0, "max": 2.0, "step": 0.01, "value": self.ros_node.ki},
            {"label": "Kd (Różniczkujący):", "attr": "kd", "min": 0.0, "max": 2.0, "step": 0.01, "value": self.ros_node.kd},
            {"label": "Max Prędkość:", "attr": "max_vel", "min": 0.1, "max": 5.0, "step": 0.1, "value": self.ros_node.max_vel, "suffix": " m/s"},
            {"label": "Wygładzanie (0-0.9):", "attr": "vel_smooth", "min": 0.0, "max": 0.9, "step": 0.1, "value": self.ros_node.vel_smooth},
        ]

        for spec in param_specs:
            spin = QDoubleSpinBox()
            spin.setRange(spec["min"], spec["max"])
            spin.setValue(spec["value"])
            spin.setSingleStep(spec["step"])
            if spec.get("suffix"):
                spin.setSuffix(spec["suffix"])
            self._bind_spinbox(spin, spec["attr"])
            layout.addRow(spec["label"], spin)

        group.setLayout(layout)
        self.main_layout.addWidget(group)

    def create_manual_control_section(self):
        group = QGroupBox("Sterowanie Ręczne (Klawisze WSAD + QE + RF)")
        layout = QGridLayout()
        
        info = QLabel("W/S: Przód/Tył | A/D: Lewo/Prawo | Q/E: Obrót | R/F: Góra/Dół")
        info.setAlignment(Qt.AlignCenter)
        info.setStyleSheet("font-size: 10pt; color: gray;")
        layout.addWidget(info, 0, 0, 1, 4)

        # Prędkość manualna
        layout.addWidget(QLabel("Prędkość manualna:"), 1, 0, 1, 2)
        self.manual_speed_spin = QDoubleSpinBox()
        self.manual_speed_spin.setRange(0.1, 5.0)
        self.manual_speed_spin.setValue(1.0)
        self.manual_speed_spin.setSuffix(" m/s")
        self.manual_speed_spin.editingFinished.connect(self.setFocus)
        layout.addWidget(self.manual_speed_spin, 1, 2, 1, 2)

        # Krok obrotu
        layout.addWidget(QLabel("Prędkość obrotu (deg/s):"), 2, 0, 1, 2)
        self.manual_yaw_spin = QDoubleSpinBox()
        self.manual_yaw_spin.setRange(1.0, 180.0)
        self.manual_yaw_spin.setValue(15.0)
        self.manual_yaw_spin.setSuffix(" deg/s")
        self.manual_yaw_spin.editingFinished.connect(self.setFocus)
        layout.addWidget(self.manual_yaw_spin, 2, 2, 1, 2)

        group.setLayout(layout)
        self.main_layout.addWidget(group)

    def create_tracking_section(self):
        group = QGroupBox("Autonomia")
        layout = QHBoxLayout()
        
        self.btn_track_start = QPushButton("START ŚLEDZENIA")
        self.btn_track_start.setStyleSheet("background-color: #4CAF50; color: white; padding: 10px; font-weight: bold;")
        self.btn_track_start.setFocusPolicy(Qt.NoFocus)
        self.btn_track_start.clicked.connect(lambda: self.ros_node.start_tracking())
        
        self.btn_track_stop = QPushButton("STOP ŚLEDZENIA")
        self.btn_track_stop.setStyleSheet("background-color: #f44336; color: white; padding: 10px; font-weight: bold;")
        self.btn_track_stop.setFocusPolicy(Qt.NoFocus)
        self.btn_track_stop.clicked.connect(lambda: self.ros_node.stop_tracking())
        
        layout.addWidget(self.btn_track_start)
        layout.addWidget(self.btn_track_stop)
        
        group.setLayout(layout)
        self.main_layout.addWidget(group)

    # -------------------------------------------------------------------------
    # Pętla Główna i Aktualizacje
    # -------------------------------------------------------------------------

    def spin_ros(self):
        rclpy.spin_once(self.ros_node, timeout_sec=0.001)

    def update_display(self):
        # 1. Telemetria
        self.alt_label.setText(f"Alt: {self.ros_node.alt:.1f} m")
        self.speed_label.setText(f"Speed: {self.ros_node.speed:.1f} m/s")

        # 2. Status Markera
        if self.ros_node.marker_visible:
            self.marker_label.setText(f"MARKER WIDOCZNY (Err: {self.ros_node.ex_f:.2f}, {self.ros_node.ey_f:.2f})")
            self.marker_label.setStyleSheet("background-color: #c8e6c9; color: #2e7d32; font-weight: bold; padding: 5px;")
        else:
            self.marker_label.setText("MARKER NIEWIDOCZNY")
            self.marker_label.setStyleSheet("background-color: #ffcdd2; color: #c62828; font-weight: bold; padding: 5px;")

        # 3. Status Śledzenia (nadpisuje styl markera jeśli aktywne)
        if self.ros_node.tracking_active:
             self.marker_label.setText(self.marker_label.text() + " [ŚLEDZENIE ON]")

        # 4. Komendy
        self.cmd_vel_label.setText(f"Vel: [{self.ros_node.vx_current:.2f}, {self.ros_node.vy_current:.2f}, {self.ros_node.vz_current:.2f}, {self.ros_node.yaw_rate_current:.2f}]")

        # 5. Aktualizacja przycisku sterowania wektorowego
        if self.ros_node.velocity_control_enabled:
            self.btn_vel_ctrl.setText("Sterowanie Wektorowe: WŁĄCZONE")
            self.btn_vel_ctrl.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold; padding: 5px;")
        else:
            self.btn_vel_ctrl.setText("Sterowanie Wektorowe: WYŁĄCZONE (Kliknij aby włączyć)")
            self.btn_vel_ctrl.setStyleSheet("background-color: #f44336; color: white; font-weight: bold; padding: 5px;")

    # -------------------------------------------------------------------------
    # Obsługa Klawiatury
    # -------------------------------------------------------------------------

    def keyPressEvent(self, event):
        if event.isAutoRepeat(): return
        key = event.key()
        if key in self.pressed_manual_keys: return
        
        self.pressed_manual_keys.add(key)
        self.process_keys()

    def keyReleaseEvent(self, event):
        if event.isAutoRepeat(): return
        key = event.key()
        if key in self.pressed_manual_keys:
            self.pressed_manual_keys.remove(key)
            self.process_keys()

    def process_keys(self):
        """Przetwarza wciśnięte klawisze i ustawia wektor sterowania"""
        if not self.pressed_manual_keys:
            self.ros_node.stop_manual_control()
            return

        vx, vy, vz, yaw_rate = 0.0, 0.0, 0.0, 0.0
        speed = self.manual_speed_spin.value()
        
        # Klawisze kierunkowe
        if Qt.Key_W in self.pressed_manual_keys: vx += speed
        if Qt.Key_S in self.pressed_manual_keys: vx -= speed
        if Qt.Key_A in self.pressed_manual_keys: vy -= speed
        if Qt.Key_D in self.pressed_manual_keys: vy += speed
        if Qt.Key_R in self.pressed_manual_keys: vz -= speed # Up (NED negative)
        if Qt.Key_F in self.pressed_manual_keys: vz += speed # Down (NED positive)
        
        # Obsługa obrotu (ciągła)
        # Pobierz prędkość obrotu w stopniach/s i zamień na radiany/s
        yaw_speed_deg = self.manual_yaw_spin.value()
        yaw_speed_rad = yaw_speed_deg * 3.14159265 / 180.0

        if Qt.Key_Q in self.pressed_manual_keys:
            yaw_rate -= yaw_speed_rad
        if Qt.Key_E in self.pressed_manual_keys:
            yaw_rate += yaw_speed_rad

        # Jeśli wciśnięto jakikolwiek klawisz ruchu lub obrotu, wyślij komendę
        if vx != 0 or vy != 0 or vz != 0 or yaw_rate != 0:
            self.ros_node.set_manual_control(vx, vy, vz, yaw_rate)
        else:
            self.ros_node.stop_manual_control()


def main(args=None):
    rclpy.init(args=args)
    node = FollowArucoSimulator()
    app = QApplication(sys.argv)
    gui = FollowArucoGUI(node)
    gui.show()
    
    try:
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
