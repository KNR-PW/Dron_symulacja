import rclpy
import time
from drone_comunication import DroneController
from drone_interfaces.srv import MakePhoto

class Mission(DroneController):
    def __init__(self):

        super().__init__()
        # self.make_photo_cli = self.create_client(MakePhoto, '/make_photo')  # albo '/mission/mission_make_photo' jeśli masz namespace

        # self.make_photo_cli.wait_for_service(timeout_sec=5.0)
        
        self.photo_idx = 1  # licznik do Zdj1, Zdj2...

    def make_photo(self, name: str | None = None, ext: str = 'jpg') -> bool:
        if name is None:
            name = f"Zdj{self.photo_idx}"
            
        req = MakePhoto.Request()
        req.prefix = name
        req.ext = ext

        fut = self.make_photo_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
        if fut.result() is None:
            self.get_logger().error('MakePhoto: brak odpowiedzi z serwisu')
            return False

        resp = fut.result()
        ok = False
        if hasattr(resp, "success"):
            ok = resp.success if isinstance(resp.success, bool) else str(resp.success).lower().startswith("saved")
        elif hasattr(resp, "feedback"):
            ok = str(resp.feedback).lower().startswith("saved")
        self.get_logger().info(f'MakePhoto({name}.{ext}) -> {ok}')

        if ok:
            self.photo_idx += 1   # zwiększ po sukcesie
        return ok

    def make_photo_next(self, ext: str = 'jpg') -> bool:
        """Skrót: zawsze kolejne ZdjN."""
        return self.make_photo(name=None, ext=ext)

def main(args=None):
    rclpy.init(args=args)
    mission = Mission()
    # mission.make_photo()  # Powinno zrobic zdj1
    mission.arm()
    mission.takeoff(5.0)
    mission.send_goto_global(-35.363319396972656, 149.16531372070312, 5.0)
    # mission.make_photo_next()  #Powinno zrobic zdj2
    time.sleep(5)
    # mission.send_goto_relative( 8.0, 0.0, 0.0)
    # mission.send_set_yaw(3.14/2)
    # mission.toggle_control()
    # mission.send_vectors(1.0,0.0,0.0)
    # time.sleep(1)
    # mission.send_vectors(1.0,0.0,0.0)
    # time.sleep(1)
    # mission.send_vectors(1.0,0.0,0.0)
    # time.sleep(1)
    # mission.toggle_control()
    mission.land()
    mission.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()