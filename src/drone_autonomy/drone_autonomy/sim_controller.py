#!/usr/bin/env python3
"""
Simple PX4 SITL controller - uses /fmu/... topics directly
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleCommand, OffboardControlMode, TrajectorySetpoint, VehicleStatus


class SimController(Node):
    def __init__(self):
        super().__init__('sim_controller')
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos)
        self.offboard_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        
        self.status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', self._status_cb, qos)
        
        self.armed = False
        self.offboard = False
        self.get_logger().info('SimController ready')

    def _status_cb(self, msg):
        self.armed = (msg.arming_state == 2)
        self.offboard = (msg.nav_state == 14)

    def _send_cmd(self, cmd, p1=0.0, p2=0.0, p3=0.0, p4=0.0, p5=0.0, p6=0.0, p7=0.0):
        msg = VehicleCommand()
        msg.command = cmd
        msg.param1, msg.param2, msg.param3, msg.param4 = p1, p2, p3, p4
        msg.param5, msg.param6, msg.param7 = p5, p6, p7
        msg.target_system, msg.target_component = 1, 1
        msg.source_system, msg.source_component = 1, 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.cmd_pub.publish(msg)

    def _send_offboard_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_pub.publish(msg)

    def _send_setpoint(self, x=0.0, y=0.0, z=-5.0, yaw=0.0):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.setpoint_pub.publish(msg)

    def arm(self):
        self.get_logger().info('Arming...')
        # Send setpoints first
        for _ in range(20):
            self._send_offboard_mode()
            self._send_setpoint(0.0, 0.0, -5.0)
            rclpy.spin_once(self, timeout_sec=0.1)
        # Offboard mode
        self._send_cmd(176, 1.0, 6.0)  # MAV_CMD_DO_SET_MODE
        rclpy.spin_once(self, timeout_sec=0.5)
        # Arm
        self._send_cmd(400, 1.0)  # ARM
        rclpy.spin_once(self, timeout_sec=0.5)
        self.get_logger().info('Armed!')

    def takeoff(self, alt=10.0):
        self.get_logger().info(f'Takeoff to {alt}m')
        for i in range(50):
            self._send_offboard_mode()
            self._send_setpoint(0.0, 0.0, -alt)
            rclpy.spin_once(self, timeout_sec=0.1)

    def goto(self, x, y, z):
        self.get_logger().info(f'Goto ({x}, {y}, {-z}m)')
        for _ in range(50):
            self._send_offboard_mode()
            self._send_setpoint(x, y, -z)
            rclpy.spin_once(self, timeout_sec=0.1)

    def land(self):
        self.get_logger().info('Landing...')
        self._send_cmd(21)  # MAV_CMD_NAV_LAND
