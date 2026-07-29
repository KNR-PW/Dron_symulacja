import rclpy
from rclpy.node import Node
from drone_interfaces.msg import VelocityVectors
from drone_interfaces.srv import ToggleVelocityControl, SetMode 
import numpy as np
import math


class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')
        self.depth_subscriber = self.create_subscription
        (
            
        )
        self.velocity_vectors_publisher = self.create_publisher(
            VelocityVectors,
            'velocity_vectors',
            10
        )
        self.toggle_velocity_control_client = self.create_client(
            ToggleVelocityControl,
            'toggle_velocity_control'
        )
        self.set_mode_client = self.create_client(
            SetMode,
            'set_mode'
        )