import rclpy
import time
from drone_comunication import DroneController
from drone_interfaces.srv import SetGimbalAngle, MakePhoto # Upewnij się, że nazwa pakietu jest poprawna

TIME_STEP = 2
class Mission(DroneController):
    def __init__(self):

        super().__init__()
        # self.make_photo_cli = self.create_client(MakePhoto, '/make_photo')  # albo '/mission/mission_make_photo' jeśli masz namespace

        # self.make_photo_cli.wait_for_service(timeout_sec=5.0)
        self.photo_idx = 1  # licznik do Zdj1, Zdj2...

        self.toggle_velocity_control_cli = self.create_client(ToggleVelocityControl,'toggle_v_control')

        self.velocity_publisher = self.create_publisher(VelocityVectors,'velocity_vectors', 10)

        self.set_gimbal_cli = self.create_client(SetGimbalAngle, 'set_gimbal_angle')
        while not self.set_gimbal_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Gimbal control service unavailable, waiting...')

    def set_gimbal_angle(self, angle_degrees):
        """Sends a request to set gimbal angle and waits for an answer."""
        self.get_logger().info(f"Sending request to set the gimbal to {angle_degrees} degrees...")
        req = SetGimbalAngle.Request()
        req.angle_degrees = float(angle_degrees)
        
        future = self.set_gimbal_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Gimbal servo control setting success.")
            else:
                self.get_logger().error(f"Gimbal servo control unsuccessful: {response.message}")
        except Exception as e:
            self.get_logger().error(f'Service calling unsuccessful: {e}')

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

    def send_vectors(self, vx, vy, vz):
        vectors = VelocityVectors()
        vectors.vx = float(vx)
        vectors.vy = float(vy)
        vectors.vz = float(vz)
        self.velocity_publisher.publish(vectors)
        
    def toggle_control(self):
        req = ToggleVelocityControl.Request()
        future = self.toggle_velocity_control_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10)
        if future.result().result:
            self.get_logger().info("true")
        else:
            self.get_logger().info("false")
        return future.result()
    
    def fly_in_square(self):
        self.state = "BUSY"
        self.i = 0
        self.get_logger().info("czasownik")
        self._timer = self.create_timer(TIME_STEP, self.timer_callback)
        self.wait_busy()

    def timer_callback(self):
        if self.i < 2:
            self.get_logger().info("przod")
            self.send_vectors(0.5,0,0)
        elif self.i < 4:
            self.get_logger().info("prawo")
            self.send_vectors(0,0.5,0)
        elif self.i < 6:
            self.get_logger().info("tyl")
            self.send_vectors(-0.5,0,0)
        elif self.i < 8:
            self.get_logger().info("lewo")
            self.send_vectors(0,-0.5,0)
        else:
            self._timer.cancel()
            self.state = "OK"
        self.i += 1

    def wait_busy(self):
        self.get_logger().info("busy")
        while self.state == "BUSY":
            #out = self.pid.compute(220-self.middle_of_aruco[1])
            #self.get_logger().info(f"output of pid: {out}")
            rclpy.spin_once(self, timeout_sec=0.05)
  

def main(args=None):
    rclpy.init(args=args)
    mission = Mission()
    # mission.make_photo()  # Powinno zrobic zdj1
    mission.arm()
    mission.takeoff(5.0)

    # mission.send_goto_global(-35.363319396972656, 149.16531372070312, 5.0)
    # mission.make_photo_next()  #Powinno zrobic zdj2
    # time.sleep(5)

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

    mission.set_gimbal_angle(30.0)
    time.sleep(3) 

    # Ustaw gimbal prosto
    mission.set_gimbal_angle(0.0)
    time.sleep(3) 

    # Ustaw gimbal w górę o 15 stopni
    mission.set_gimbal_angle(-15.0)
    time.sleep(3) 

    mission.set_gimbal_angle(0.0)

    mission.land()
    mission.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()