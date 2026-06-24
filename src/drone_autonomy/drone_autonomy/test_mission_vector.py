import rclpy
import time
import math
from drone_comunication import DroneController

# Prosta misja na sterowaniu wektorowym (FRD: front / right / down):
#   takeoff 5 m -> lewo -> przod -> prawo -> obrot yaw 90 deg -> ladowanie
#
# Konwencja wektorow (jak w gui_panel.py):
#   vx  = przod (+) / tyl (-)        [m/s]
#   vy  = prawo (+) / lewo (-)       [m/s]
#   vz  = dol  (+) / gora (-)        [m/s]  (NED: gora jest ujemna)
#   yaw = predkosc obrotu, zegarowo (+) / przeciwnie (-)   [rad/s]

TAKEOFF_ALT = 5.0     # wysokosc startu [m]
SPEED       = 1.0     # predkosc przelotu [m/s]
MOVE_TIME   = 3.0     # czas kazdego ruchu [s]  -> ~3 m przy 1 m/s
YAW_RATE    = math.radians(30)   # predkosc obrotu [rad/s] -> 30 deg/s
YAW_ANGLE   = 90.0    # docelowy obrot [deg]
RATE_HZ     = 10.0    # czestotliwosc publikowania wektorow [Hz]


def fly_vector(mission, vx, vy, vz, yaw, duration):
    """Publikuje wektor predkosci strumieniowo (RATE_HZ) przez 'duration' sekund."""
    period = 1.0 / RATE_HZ
    end = time.time() + duration
    while time.time() < end:
        mission.send_vectors(vx, vy, vz, yaw)
        time.sleep(period)


def hold(mission, duration=1.0):
    """Zatrzymanie w miejscu (zerowy wektor)."""
    fly_vector(mission, 0.0, 0.0, 0.0, 0.0, duration)


def main(args=None):
    rclpy.init(args=args)
    mission = DroneController()

    # --- Start ---
    mission.arm()
    mission.takeoff(TAKEOFF_ALT)
    time.sleep(5)                 # stabilizacja po starcie

    # --- Wlacz sterowanie wektorowe ---
    mission.toggle_control()

    # --- Lewo ---
    fly_vector(mission, 0.0, -SPEED, 0.0, 0.0, MOVE_TIME)
    hold(mission, 1.0)

    # --- Przod ---
    fly_vector(mission, SPEED, 0.0, 0.0, 0.0, MOVE_TIME)
    hold(mission, 1.0)

    # --- Prawo ---
    fly_vector(mission, 0.0, SPEED, 0.0, 0.0, MOVE_TIME)
    hold(mission, 1.0)

    # --- Obrot yaw o 90 deg (open-loop: rate * czas = kat) ---
    yaw_time = math.radians(YAW_ANGLE) / YAW_RATE
    fly_vector(mission, 0.0, 0.0, 0.0, YAW_RATE, yaw_time)
    hold(mission, 1.0)

    # --- Wylacz sterowanie wektorowe i ladowanie ---
    mission.toggle_control()
    mission.land()

    mission.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
