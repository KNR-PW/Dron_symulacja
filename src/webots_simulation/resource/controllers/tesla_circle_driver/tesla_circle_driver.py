"""tesla_circle_driver controller."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from vehicle import Driver

class CarDriverNode(Node):
    def __init__(self):
        super().__init__('webots_car_driver')
        
        # Subscribe to /cmd_vel
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        
        # commands sent by the mission
        self.target_speed_kmh = 0.0
        self.target_steering_rad = 0.0
        
        self.get_logger().info("ROS 2 Car Driver Node started. Listening on /cmd_vel...")

    def cmd_vel_callback(self, msg):
        # 1. Map Linear X (m/s) to Speed (km/h)
        self.target_speed_kmh = msg.linear.x * 3.6
        
        # 2. Map Angular Z to Steering Angle (radians)
        max_steering = 0.85 
        self.target_steering_rad = max(min(msg.angular.z, max_steering), -max_steering)

def main(args=None):
    # 1. Initialize Webots Driver
    # Note: This requires the node to be run as an <extern> controller or inside Webots
    driver = Driver()
    timestep = int(driver.getBasicTimeStep())
    
    # 2. Initialize ROS 2
    rclpy.init(args=args)
    car_node = CarDriverNode()

    # 3. Main Simulation Loop
    while driver.step() != -1:
        # Process ROS 2 callbacks (check for new cmd_vel messages)
        rclpy.spin_once(car_node, timeout_sec=0)
        
        # Apply the values received from ROS to the Webots vehicle
        driver.setCruisingSpeed(car_node.target_speed_kmh)
        driver.setSteeringAngle(car_node.target_steering_rad)

    # Cleanup
    car_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()