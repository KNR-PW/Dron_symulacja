#!/usr/bin/env python3
import rclpy
import time
from drone_autonomy.sim_controller import SimController

def main():
    rclpy.init()
    drone = SimController()
    
    drone.arm()
    time.sleep(2)
    
    drone.takeoff(10.0)
    time.sleep(3)
    
    drone.goto(10.0, 0.0, 10.0)
    time.sleep(3)
    
    drone.goto(10.0, 10.0, 10.0)
    time.sleep(3)
    
    drone.goto(0.0, 0.0, 10.0)
    time.sleep(3)
    
    drone.land()
    time.sleep(2)
    
    drone.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
