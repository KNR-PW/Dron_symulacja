import rclpy
import time
import math
from drone_comunication import DroneController

# Zmienne misji
base_lat = 47.398136
base_lon = 8.549139  # Bazowa szerokość i długość geograficzna (punkt startowy)
alt = 20.0  # Wysokosc na ktorej latamy

meters = 25.0 # Odleglosc kwadratu w metrach i wzniosu, bo akurat pasuje dla 25m

times = 20 # Liczba powtórzeń kwadratu, 1 mile = 1.6km więc 20 * 25m = 500m

# 1° latitude ≈ 111320 m
dlat = meters / 111320

# 1° longitude zależy od szerokości geograficznej
dlon = meters / (111320 * math.cos(math.radians(base_lat)))

def main(args=None):
    rclpy.init(args=args)
    mission = DroneController()
    mission.arm()
    mission.takeoff(20.0)   # 50 feet = 15.24 meters, so we can use 20 meters for a safe takeoff margin
    time.sleep(5) # Wait for the drone to stabilize after takeoff
    mission._set_mode("MULTICOPTER")
    for i in range(times): # 1 mile = 1.6km so 100*20 = 2km gives us some leeway for eventual GPS inaccuracies
        mission.send_goto_global(base_lat, base_lon, alt)
        time.sleep(1) # Wait for the drone to reach the waypoint
        mission.send_goto_global(base_lat, base_lon + dlon, alt)
        time.sleep(1) # Wait for the drone to reach the waypoint
        mission.send_goto_global(base_lat + dlat, base_lon + dlon, alt)
        time.sleep(1) # Wait for the drone to reach the waypoint
        mission.send_goto_global(base_lat + dlat, base_lon, alt)
        time.sleep(1) # Wait for the drone to reach the waypoint
    mission.send_goto_global(base_lat + 0*dlat, base_lon, 20.0)  # W1 Testy wznoszenia i opadania incline arctan(10/25) = 21.8° >= 20° więc powinno być bezpiecznie
    mission.send_goto_global(base_lat + 1*dlat, base_lon, 30.0)  # W2 (+10m)
    mission.send_goto_global(base_lat + 2*dlat, base_lon, 20.0)  # W3 (-10m)
    mission.send_goto_global(base_lat + 3*dlat, base_lon, 30.0)  # W4 (+10m)
    mission.land()
    mission.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()