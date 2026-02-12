import rclpy
import time
from drone_comunication.drone_controller import DroneController

def main(args=None):
    rclpy.init(args=args)
    
    print("--- STARTUJĘ MISJĘ TESTOWĄ ---")
    mission = DroneController()

    # 1. Uzbrojenie
    print("1. ARM")
    mission.arm()

    # 2. Start na 5 metrów
    print("2. TAKEOFF 5m")
    mission.takeoff(3.0)
    time.sleep(2) # Czekamy chwilę na stabilizację

    print("--- Enabling Obstacle Avoidance ---")
    mission.set_obstacle_avoidance(True)
    
    # 4. Lot w prawo (5m East)
    print("4. GOTO RELATIVE (5m East)")
    mission.send_goto_relative(30.0,0.0, 0.0)
    time.sleep(2)

    # 5. Powrót (5m South, 5m West)
    print("5. GOTO RELATIVE (-5m North, -5m East)")
    mission.send_goto_relative(-30.0, 0.0, 0.0)
    time.sleep(2)

    # 6. Lądowanie
    print("6. LAND")
    mission.land()

    mission.destroy_node()
    rclpy.shutdown()
    print("--- KONIEC MISJI ---")

if __name__ == "__main__":
    main()