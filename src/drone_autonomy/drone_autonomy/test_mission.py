from drone_interfaces.msg import VelocityVectors
from drone_interfaces.srv import ToggleVelocityControl
import rclpy
import time
from drone_comunication import DroneController

TIME_STEP = 2

class Mission(DroneController):
    def __init__(self):
        super().__init__()
        self.toggle_velocity_control_cli = self.create_client(ToggleVelocityControl,'toggle_v_control')

        self.velocity_publisher = self.create_publisher(VelocityVectors,'velocity_vectors', 10)

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
            self.get_logger().info("przud")
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
    mission.arm()
    mission.takeoff(5.0)
    # mission.send_goto_relative( 8.0, 0.0, 0.0)
    # mission.send_set_yaw(3.14/2)
    # mission.toggle_control()
    # mission.send_vectors(1.0,0.0,0.0)
    # time.sleep(1)
    # mission.send_vectors(1.0,0.0,0.0)
    # time.sleep(1)
    # mission.send_vectors(1.0,0.0,0.0)
    # time.sleep(1)
    # mission.fly_in_square()
    # mission.toggle_control()
    mission.land()
    mission.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()