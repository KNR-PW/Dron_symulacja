#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PROSTY SYMULATOR ARUCO - do testowania bez kamery

Jak używać:
-----------
# Marker stoi w miejscu (domyślnie środek ekranu)
ros2 run drone_autonomy aruco_simulator

# Marker porusza się po okręgu
ros2 run drone_autonomy aruco_simulator --ros-args -p mode:=circle

# Marker z szumem (jak prawdziwa kamera)
ros2 run drone_autonomy aruco_simulator --ros-args -p noise:=5.0

# Zmiana w trakcie działania
ros2 param set /aruco_simulator mode circle
ros2 param set /aruco_simulator speed 60.0

Co to robi:
-----------
Publikuje pozycje markera ArUco na topic /aruco_markers
Tak jak prawdziwy aruco_node, ale bez kamery.
Twoje node'y (follow_aruco_centroid, PID_test itp.) nie wiedzą że to symulacja!

Tryby:
------
'static' - marker stoi w miejscu
'circle' - marker porusza się po okręgu
'line'   - marker porusza się w poziomie
'square' - marker porusza się po kwadracie
"""

import rclpy
from rclpy.node import Node
from drone_interfaces.msg import MiddleOfAruco
import math
import random


class ArucoSimulator(Node):
    def __init__(self):
        super().__init__('aruco_simulator')
        
        # PARAMETRY - możesz je zmieniać!
        self.declare_parameter('mode', 'static')      # static, circle, line, square
        self.declare_parameter('speed', 40.0)         # prędkość ruchu (px/s)
        self.declare_parameter('noise', 0.0)          # szum (piksele), np. 2.0 = lekki, 5.0 = duży
        self.declare_parameter('center_x', 320)       # środek X (dla circle)
        self.declare_parameter('center_y', 240)       # środek Y (dla circle)
        self.declare_parameter('radius', 100)         # promień okręgu (piksele)
        
        # Pobierz wartości
        self.mode = self.get_parameter('mode').value
        self.speed = self.get_parameter('speed').value
        self.noise = self.get_parameter('noise').value
        self.center_x = self.get_parameter('center_x').value
        self.center_y = self.get_parameter('center_y').value
        self.radius = self.get_parameter('radius').value
        
        # Publisher
        self.pub = self.create_publisher(MiddleOfAruco, 'aruco_markers', 10)
        
        # Timer - 30 Hz (jak prawdziwa kamera)
        self.timer = self.create_timer(1.0/30.0, self.publish_marker)
        self.time = 0.0
        
        self.get_logger().info(f"Symulator uruchomiony: mode={self.mode}, speed={self.speed}, noise={self.noise}")
    
    def publish_marker(self):
        # Odczytaj aktualne parametry (mogły się zmienić!)
        self.mode = self.get_parameter('mode').value
        self.speed = self.get_parameter('speed').value
        self.noise = self.get_parameter('noise').value
        
        # Oblicz pozycję w zależności od trybu
        if self.mode == 'circle':
            # Ruch po okręgu
            angle = self.time * self.speed / 100.0
            x = self.center_x + self.radius * math.cos(angle)
            y = self.center_y + self.radius * math.sin(angle)
            
        elif self.mode == 'line':
            # Ruch w poziomie (tam i z powrotem)
            x = self.center_x + 150 * math.sin(self.time * self.speed / 100.0)
            y = self.center_y
            
        elif self.mode == 'square':
            # Ruch po kwadracie
            progress = (self.time * self.speed / 100.0) % 4.0
            side = self.radius
            if progress < 1.0:
                x = self.center_x - side + progress * 2 * side
                y = self.center_y - side
            elif progress < 2.0:
                x = self.center_x + side
                y = self.center_y - side + (progress - 1.0) * 2 * side
            elif progress < 3.0:
                x = self.center_x + side - (progress - 2.0) * 2 * side
                y = self.center_y + side
            else:
                x = self.center_x - side
                y = self.center_y + side - (progress - 3.0) * 2 * side
                
        else:  # 'static'
            # Marker stoi w miejscu
            x = self.center_x
            y = self.center_y
        
        # Dodaj szum (jak prawdziwa kamera)
        if self.noise > 0:
            x += random.gauss(0, self.noise)
            y += random.gauss(0, self.noise)
        
        # Ogranicz do ekranu (640x480)
        x = max(0, min(640, int(x)))
        y = max(0, min(480, int(y)))
        
        # Publikuj
        msg = MiddleOfAruco()
        msg.x = x
        msg.y = y
        self.pub.publish(msg)
        
        self.time += 1.0/30.0


def main(args=None):
    rclpy.init(args=args)
    simulator = ArucoSimulator()
    
    try:
        rclpy.spin(simulator)
    except KeyboardInterrupt:
        pass
    finally:
        simulator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
