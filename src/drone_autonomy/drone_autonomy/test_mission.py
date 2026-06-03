import rclpy
import time
import math
from drone_comunication import DroneController

# Zmienne misji
base_lat = 47.398136
base_lon = 8.549139  # Bazowa szerokość i długość geograficzna (punkt startowy)
alt = 20.0  # Wysokosc na ktorej latamy

meters = 25.0 # Odleglosc kwadratu w metrach i wzniosu, bo akurat pasuje dla 25m


# 1° latitude ≈ 111320 m
dlat = meters / 111320

# 1° longitude zależy od szerokości geograficznej
dlon = meters / (111320 * math.cos(math.radians(base_lat)))

def main(args=None):
    rclpy.init(args=args)
    mission = DroneController()
    mission.arm()
    mission.takeoff(5.0)
    time.sleep(5)
    mission.send_goto_relative(1.0, 0.0, 0.0)
    time.sleep(1)
    mission.send_goto_relative(0.0, 1.0, 0.0)
    time.sleep(1)
    mission.send_goto_relative(-1.0, 0.0, 0.0)
    time.sleep(1)
    mission.send_goto_relative(0.0, -1.0, 0.0)
    time.sleep(1)
    mission.land()
    mission.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()