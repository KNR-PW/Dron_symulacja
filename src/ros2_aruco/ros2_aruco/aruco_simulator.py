#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SYMULATOR ARUCO Z GUI - do testowania bez kamery

Jak używać:
-----------
ros2 run ros2_aruco aruco_simulator

Otworzy się okno GUI z 4 przyciskami do sterowania pozycją markera:
- Góra (W) - przesuwa marker w górę
- Dół (S) - przesuwa marker w dół  
- Lewo (A) - przesuwa marker w lewo
- Prawo (D) - przesuwa marker w prawo

Co to robi:
-----------
Publikuje pozycje markera ArUco na topic /aruco_markers
Możesz sterować pozycją markera za pomocą przycisków lub klawiszy WASD
"""

import sys
import rclpy
from rclpy.node import Node
from drone_interfaces.msg import MiddleOfAruco
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QLabel, QVBoxLayout, QHBoxLayout, QWidget, QGridLayout
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QFont


class ArucoSimulator(Node):
    def __init__(self):
        super().__init__('aruco_simulator')
        
        # PARAMETRY
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('margin', 50)  # Odstęp od krawędzi ekranu
        
        # Pobierz wartości
        self.img_w = self.get_parameter('image_width').value
        self.img_h = self.get_parameter('image_height').value
        self.margin = self.get_parameter('margin').value
        
        # Stałe pozycje markera przy krawędziach
        self.marker_positions = {
            'up': (self.img_w // 2, self.margin),                    # Góra: środek X, blisko górnej krawędzi
            'down': (self.img_w // 2, self.img_h - self.margin),     # Dół: środek X, blisko dolnej krawędzi
            'left': (self.margin, self.img_h // 2),                  # Lewo: blisko lewej krawędzi, środek Y
            'right': (self.img_w - self.margin, self.img_h // 2)     # Prawo: blisko prawej krawędzi, środek Y
        }
        
        # Flaga aktywnego kierunku (None = brak publikacji)
        self.active_direction = None
        
        # Publisher
        self.pub = self.create_publisher(MiddleOfAruco, 'aruco_markers', 10)
        
        # Timer - 30 Hz (jak prawdziwa kamera)
        self.timer = self.create_timer(1.0/30.0, self.publish_marker)
        
        self.get_logger().info(f"Symulator uruchomiony: {self.img_w}x{self.img_h}, margin={self.margin}px")
    
    def activate_direction(self, direction):
        """Aktywuj marker w danym kierunku"""
        self.active_direction = direction
        x, y = self.marker_positions[direction]
        if direction == 'up':
            self.get_logger().info(f"↑ Marker góra: ({x}, {y})")
        elif direction == 'down':
            self.get_logger().info(f"↓ Marker dół: ({x}, {y})")
        elif direction == 'left':
            self.get_logger().info(f"← Marker lewo: ({x}, {y})")
        elif direction == 'right':
            self.get_logger().info(f"→ Marker prawo: ({x}, {y})")
    
    def deactivate_marker(self):
        """Dezaktywuj marker (przestań publikować)"""
        if self.active_direction is not None:
            self.get_logger().info("Marker ukryty")
            self.active_direction = None
    
    def publish_marker(self):
        """Publikuj marker tylko gdy jest aktywny kierunek"""
        if self.active_direction is not None:
            x, y = self.marker_positions[self.active_direction]
            msg = MiddleOfAruco()
            msg.x = x
            msg.y = y
            self.pub.publish(msg)


class ArucoSimulatorGUI(QMainWindow):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.init_ui()
        
        # Timer do spinowania ROS2
        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(self.spin_ros)
        self.ros_timer.start(10)  # 100 Hz
    
    def init_ui(self):
        """Inicjalizacja interfejsu użytkownika"""
        self.setWindowTitle('ArUco Simulator')
        self.setGeometry(100, 100, 420, 320)
        
        # Widget centralny
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Layout główny
        main_layout = QVBoxLayout()
        central_widget.setLayout(main_layout)
        
        # Label z pozycją
        self.position_label = QLabel('Brak markera (wciśnij WASD)')
        self.position_label.setAlignment(Qt.AlignCenter)
        pos_font = QFont()
        pos_font.setPointSize(14)
        pos_font.setBold(True)
        self.position_label.setFont(pos_font)
        main_layout.addWidget(self.position_label)
        
        # Timer do aktualizacji pozycji
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_position_label)
        self.update_timer.start(100)  # Aktualizuj co 100ms
        
        # Grid z przyciskami sterowania
        button_layout = QGridLayout()
        main_layout.addLayout(button_layout)
        
        # Przycisk GÓRA (wiersz 0, kolumna 1)
        self.btn_up = QPushButton('↑\nW')
        self.btn_up.setMinimumSize(100, 70)
        self.btn_up.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
                color: white;
                font-size: 24pt;
                font-weight: bold;
                border-radius: 10px;
            }
            QPushButton:pressed {
                background-color: #45a049;
            }
        """)
        self.btn_up.pressed.connect(lambda: self.on_press('up'))
        self.btn_up.released.connect(self.on_release)
        button_layout.addWidget(self.btn_up, 0, 1)
        
        # Przycisk LEWO (wiersz 1, kolumna 0)
        self.btn_left = QPushButton('←\nA')
        self.btn_left.setMinimumSize(100, 70)
        self.btn_left.setStyleSheet("""
            QPushButton {
                background-color: #2196F3;
                color: white;
                font-size: 24pt;
                font-weight: bold;
                border-radius: 10px;
            }
            QPushButton:pressed {
                background-color: #0b7dda;
            }
        """)
        self.btn_left.pressed.connect(lambda: self.on_press('left'))
        self.btn_left.released.connect(self.on_release)
        button_layout.addWidget(self.btn_left, 1, 0)
        
        # Przycisk PRAWO (wiersz 1, kolumna 2)
        self.btn_right = QPushButton('→\nD')
        self.btn_right.setMinimumSize(100, 70)
        self.btn_right.setStyleSheet("""
            QPushButton {
                background-color: #2196F3;
                color: white;
                font-size: 24pt;
                font-weight: bold;
                border-radius: 10px;
            }
            QPushButton:pressed {
                background-color: #0b7dda;
            }
        """)
        self.btn_right.pressed.connect(lambda: self.on_press('right'))
        self.btn_right.released.connect(self.on_release)
        button_layout.addWidget(self.btn_right, 1, 2)
        
        # Przycisk DÓŁ (wiersz 2, kolumna 1)
        self.btn_down = QPushButton('↓\nS')
        self.btn_down.setMinimumSize(100, 70)
        self.btn_down.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
                color: white;
                font-size: 24pt;
                font-weight: bold;
                border-radius: 10px;
            }
            QPushButton:pressed {
                background-color: #45a049;
            }
        """)
        self.btn_down.pressed.connect(lambda: self.on_press('down'))
        self.btn_down.released.connect(self.on_release)
        button_layout.addWidget(self.btn_down, 2, 1)
        
        # Zapamiętaj który klawisz jest wciśnięty
        self.pressed_keys = set()
    
    def keyPressEvent(self, event):
        """Obsługa wciśnięcia klawisza"""
        if event.isAutoRepeat():
            return  # Ignoruj auto-repeat
        
        key = event.key()
        if key == Qt.Key_W and key not in self.pressed_keys:
            self.pressed_keys.add(key)
            self.on_press('up')
        elif key == Qt.Key_S and key not in self.pressed_keys:
            self.pressed_keys.add(key)
            self.on_press('down')
        elif key == Qt.Key_A and key not in self.pressed_keys:
            self.pressed_keys.add(key)
            self.on_press('left')
        elif key == Qt.Key_D and key not in self.pressed_keys:
            self.pressed_keys.add(key)
            self.on_press('right')
    
    def keyReleaseEvent(self, event):
        """Obsługa puszczenia klawisza"""
        if event.isAutoRepeat():
            return  # Ignoruj auto-repeat
        
        key = event.key()
        if key in self.pressed_keys:
            self.pressed_keys.remove(key)
            self.on_release()
    
    def on_press(self, direction):
        """Obsługa wciśnięcia przycisku/klawisza"""
        self.ros_node.activate_direction(direction)
        self.update_position_label()
    
    def on_release(self):
        """Obsługa puszczenia przycisku/klawisza"""
        self.ros_node.deactivate_marker()
        self.update_position_label()
    
    def update_position_label(self):
        """Aktualizuj wyświetlaną pozycję"""
        if self.ros_node.active_direction is not None:
            x, y = self.ros_node.marker_positions[self.ros_node.active_direction]
            direction_name = {
                'up': '↑ GÓRA',
                'down': '↓ DÓŁ',
                'left': '← LEWO',
                'right': '→ PRAWO'
            }[self.ros_node.active_direction]
            self.position_label.setText(f'{direction_name}: ({x}, {y})')
        else:
            self.position_label.setText('Brak markera (wciśnij WASD)')
    
    def spin_ros(self):
        """Spinuj ROS2 node"""
        rclpy.spin_once(self.ros_node, timeout_sec=0)
    
    def closeEvent(self, event):
        """Cleanup przy zamykaniu okna"""
        self.ros_timer.stop()
        self.update_timer.stop()
        event.accept()


def main(args=None):
    rclpy.init(args=args)
    
    # Utwórz ROS2 node
    simulator = ArucoSimulator()
    
    # Utwórz aplikację Qt
    app = QApplication(sys.argv)
    
    # Utwórz GUI
    gui = ArucoSimulatorGUI(simulator)
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
