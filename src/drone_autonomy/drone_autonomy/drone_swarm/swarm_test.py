import random
import time
from multiprocessing import Process

import rclpy

from drone_autonomy.drone_comunication import DroneController

ALT_DRONE_1 = 15.0
ALT_DRONE_2 = 12.0

N_STEPS = 4
MAX_STEP = 8.0
SETTLE_TIME = 2.0


def random_route(n_steps=N_STEPS, max_step=MAX_STEP):
    return [
        (round(random.uniform(-max_step, max_step), 1),
         round(random.uniform(-max_step, max_step), 1),
         0.0)
        for _ in range(n_steps)
    ]


def mirror_route(route):
    return [(north, -east, down) for north, east, down in route]


def fly_route(namespace, route, altitude):
    rclpy.init()
    drone = DroneController(f'{namespace}_mission', namespace=namespace)

    drone.arm()
    drone.takeoff(altitude)
    time.sleep(5)

    for i, (north, east, down) in enumerate(route, start=1):
        drone.get_logger().info(
            f'[{namespace}] punkt {i}/{len(route)}: N={north} E={east} D={down}')
        drone.send_goto_relative(north, east, down)
        time.sleep(SETTLE_TIME)

    drone.get_logger().info(f'[{namespace}] koniec trasy, RTL')
    drone.rtl()

    drone.destroy_node()
    rclpy.shutdown()


def main():
    route = random_route()
    mirrored = mirror_route(route)
    print(f'Trasa drona 1 (losowa):    {route}')
    print(f'Trasa drona 2 (lustrzana): {mirrored}')

    drone1 = Process(target=fly_route, args=('drone1', route, ALT_DRONE_1))
    drone2 = Process(target=fly_route, args=('drone2', mirrored, ALT_DRONE_2))

    drone1.start()
    drone2.start()
    drone1.join()
    drone2.join()


if __name__ == '__main__':
    main()
