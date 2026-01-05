#!/usr/bin/env python3
"""
Auto-generated mission: mission_20260102_152428
Summary: Launch to 10m height, fly forward 10m, take photo, return to start. (1 photo(s))
Generated: 2026-01-02 15:24:28

This mission uses SimController for PX4 SITL simulation.
Includes photo capture via /make_photo ROS2 service.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleCommand, OffboardControlMode, TrajectorySetpoint, VehicleStatus
import time

from drone_interfaces.srv import MakePhoto


# ============================================================================
# SimController - prosty kontroler dla symulacji PX4 SITL
# ============================================================================

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
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = -5.0  # Default altitude (negative = up in NED)
        
        
        # Photo service client
        self.photo_client = self.create_client(MakePhoto, '/make_photo')
        
        
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

    def _send_setpoint(self, x, y, z, yaw=0.0):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.setpoint_pub.publish(msg)

    def arm(self):
        self.get_logger().info('Arming...')
        for _ in range(20):
            self._send_offboard_mode()
            self._send_setpoint(0.0, 0.0, self.current_z)
            rclpy.spin_once(self, timeout_sec=0.1)
        self._send_cmd(176, 1.0, 6.0)  # OFFBOARD mode
        rclpy.spin_once(self, timeout_sec=0.5)
        self._send_cmd(400, 1.0)  # ARM
        rclpy.spin_once(self, timeout_sec=0.5)
        self.get_logger().info('Armed!')

    def takeoff(self, alt=10.0):
        self.get_logger().info(f'Takeoff to {alt}m')
        self.current_z = -alt  # NED: negative = up
        for _ in range(50):
            self._send_offboard_mode()
            self._send_setpoint(self.current_x, self.current_y, self.current_z)
            rclpy.spin_once(self, timeout_sec=0.1)

    def goto(self, x, y, z):
        self.get_logger().info(f'Goto ({x}, {y}, {-z}m)')
        self.current_x = x
        self.current_y = y
        self.current_z = -abs(z)  # Ensure negative for altitude
        for _ in range(50):
            self._send_offboard_mode()
            self._send_setpoint(self.current_x, self.current_y, self.current_z)
            rclpy.spin_once(self, timeout_sec=0.1)

    def goto_relative(self, north, east, down):
        """Move relative to current position."""
        new_x = self.current_x + north
        new_y = self.current_y + east
        new_z = self.current_z + down  # down+ = descend = more negative
        self.get_logger().info(f'Goto relative N:{north}, E:{east}, D:{down} -> ({new_x}, {new_y}, {-new_z}m)')
        self.current_x = new_x
        self.current_y = new_y
        self.current_z = new_z
        for _ in range(50):
            self._send_offboard_mode()
            self._send_setpoint(self.current_x, self.current_y, self.current_z)
            rclpy.spin_once(self, timeout_sec=0.1)

    def land(self):
        self.get_logger().info('Landing...')
        self._send_cmd(21)  # MAV_CMD_NAV_LAND
    
    
    def take_photo(self, prefix='photo'):
        """Take a photo using /make_photo service."""
        if not self.photo_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Photo service not available!')
            return False
        
        req = MakePhoto.Request()
        req.prefix = prefix
        req.ext = 'jpg'
        
        future = self.photo_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            result = future.result().success
            self.get_logger().info(f'Photo result: {{result}}')
            return 'saved' in result.lower()
        else:
            self.get_logger().error('Photo service call failed')
            return False
    


# ============================================================================
# Global drone instance and helper functions
# ============================================================================

drone = None
current_alt = 10.0  # Track current altitude for RTL


def take_photo(prefix='photo'):
    """Helper function to take photo."""
    global drone
    if drone:
        return drone.take_photo(prefix)
    return False



def main(args=None):
    global drone, current_alt
    
    rclpy.init(args=args)
    drone = SimController()
    
    print("🚁 Starting mission: mission_20260102_152428")
    
    # Arm drone
    drone.arm()
    time.sleep(2)
    
    # === MISSION WAYPOINTS ===
    # Step 1: Takeoff to 10.0m
    drone.takeoff(10.0)
    time.sleep(3)
    # Step 2: Move relative N:10.0, E:0.0, D:0.0
    drone.goto_relative(10.0, 0.0, 0.0)
    time.sleep(3)
    # Step 3: Take photo
    take_photo('mission_photo')
    time.sleep(1)
    # Step 4: Return to launch
    drone.goto(0.0, 0.0, current_alt)
    time.sleep(3)
    
    print("✅ Mission complete!")
    
    drone.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
