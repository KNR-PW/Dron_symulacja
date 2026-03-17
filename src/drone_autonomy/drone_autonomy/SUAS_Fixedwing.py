import rclpy
import time
import math
from drone_comunication import DroneController
base_lat = 47.398136
base_lon = 8.549139
alt = 20.0
meters = 1000.0 

times = 1 
dlat = meters / 111320
dlon = meters / (111320 * math.cos(math.radians(base_lat)))

def main(args=None):
    rclpy.init(args=args)
    mission = DroneController()
    mission.arm()
    mission.takeoff(20.0) # takeff in Quadcopter mode, so we can safely takeoff and stabilize before switching to fixed wing mode
    time.sleep(4) # Wait for the drone to stabilize after takeoff
    mission._set_mode("FIXED_WING") 
    for i in range(times):  # 1 mile = 1.6km 4*1000 = 4000 so gives us some leeway for eventual GPS inaccuracies
        mission.send_goto_global(base_lat, base_lon, alt)
        mission.send_goto_global(base_lat, base_lon + dlon, alt)
        mission.send_goto_global(base_lat + dlat, base_lon + dlon, alt)
        mission.send_goto_global(base_lat + dlat, base_lon, alt)
    mission.send_goto_global(base_lat + 0*dlat, base_lon, 50.0)
    time.sleep(1)
    mission.send_goto_global(base_lat + 1*dlat, base_lon, 70.0) 
    time.sleep(1)
    mission.send_goto_global(base_lat + 2*dlat, base_lon, 50.0) 
    time.sleep(1)
    mission.send_goto_global(base_lat, base_lon, 30.0) 
    time.sleep(1)
    mission._set_mode("MULTICOPTER") #change back to multicopter mode to safely land
    time.sleep(5) 
    mission.land()
    mission.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()