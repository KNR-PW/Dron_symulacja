#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from controller import Supervisor

# Check if we can import controller
try:
    from controller import Supervisor
except ImportError:
    sys.stderr.write("Error: 'controller' module not found. This script must be run by Webots.\n")
    sys.exit(1)

class GroundTruthPublisher(Node):
    def __init__(self, supervisor):
        super().__init__('ground_truth_publisher')
        self.supervisor = supervisor
        
        # Publishers
        self.drone_pub = self.create_publisher(PointStamped, '/sim/ground_truth/drone', 10)
        self.car_pub = self.create_publisher(PointStamped, '/sim/ground_truth/car', 10)
        
        # Get nodes by DEF name
        self.drone_node = self.supervisor.getFromDef('Iris')
        self.car_node = self.supervisor.getFromDef('Tesla3')
        
        if self.drone_node is None:
            self.get_logger().error("Could not find DEF Iris in the world.")
        else:
            self.get_logger().info("Found DEF Iris")
            
        if self.car_node is None:
             self.get_logger().error("Could not find DEF Tesla3 in the world.")
        else:
            self.get_logger().info("Found DEF Tesla3")

    def publish_stepped(self):
        # Publish Drone Position
        if self.drone_node:
            pos = self.drone_node.getPosition()
            msg = PointStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            msg.point.x = pos[0]
            msg.point.y = pos[1]
            msg.point.z = pos[2]
            self.drone_pub.publish(msg)

        # Publish Car Position
        if self.car_node:
            pos = self.car_node.getPosition()
            orientation = self.car_node.getOrientation()
            
            # Offset the position by 1.5m in the car's local X direction
            # Orientation matrix columns are global axes corresponding to local X, Y, Z
            # Column 0 is local X vector in global coordinates
            # Indices: 0, 3, 6
            dx = orientation[0] * 1.5
            dy = orientation[3] * 1.5
            dz = orientation[6] * 1.5

            msg = PointStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            msg.point.x = pos[0] + dx
            msg.point.y = pos[1] + dy
            msg.point.z = pos[2] + dz
            self.car_pub.publish(msg)

def main():
    # Initialize Webots global supervisor
    supervisor = Supervisor()
    timestep = int(supervisor.getBasicTimeStep())

    # Initialize ROS 2 context
    rclpy.init(args=None)
    
    # Create the ROS 2 node
    ground_truth_node = GroundTruthPublisher(supervisor)

    ground_truth_node.get_logger().info("Starting Ground Truth Loop...")

    # Main Control Loop
    while supervisor.step(timestep) != -1:
        if not rclpy.ok():
            break
            
        ground_truth_node.publish_stepped()
        
        # Process any incoming ROS messages (not strictly needed for just publishing, but good for logs/time)
        rclpy.spin_once(ground_truth_node, timeout_sec=0)

    # Cleanup
    ground_truth_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
