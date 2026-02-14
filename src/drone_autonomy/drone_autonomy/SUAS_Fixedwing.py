import rclpy
import time
from drone_comunication import DroneController

def main(args=None):
    rclpy.init(args=args)
    mission = DroneController()
    mission.arm()
    mission.takeoff(20.0)   # 50 feet = 15.24 meters, so we can use 20 meters for a safe takeoff margin
    time.sleep(5) # Wait for the drone to stabilize after takeoff
    mission._set_mode("FIXED_WING")
    for i in range(10): # 1 mile = 1.6km so this mission does 10 * 250m = 2.5km
        mission.send_goto_global(47.398136, 8.549139, 10.0) # Move to the first waypoint
        time.sleep(1) # Wait for the drone to reach the waypoint
        mission.send_goto_global(47.397321, 8.546019, 10.0) # Move to the second waypoint
        time.sleep(1) # Wait for the drone to reach the waypoint
    mission.land()
    # mission.rtl()
    mission.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()