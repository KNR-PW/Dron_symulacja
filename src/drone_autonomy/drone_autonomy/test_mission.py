from drone_comunication import Hardware_com
from drone_interfaces.msg import MiddleOfAruco, VelocityVectors
from drone_interfaces.srv import ToggleVelocityControl
import rclpy
import time

TIME_STEP = 2

class PID:
    def __init__(self,KP,KI,KD):
        self.kp = KP
        self.ki = KI
        self.kd = KD 
        self.error = 0
        self.integral_error = 0
        self.error_last = 0
        self.derivative_error = 0
        self.output = 0

    def compute(self, error):
        self.error = error
        self.integral_error += self.error * TIME_STEP
        self.derivative_error = (self.error - self.error_last) / TIME_STEP
        self.error_last = self.error
        self.output = self.kp*self.error + self.ki*self.integral_error + self.kd*self.derivative_error

        return self.output
		
    def get_kpe(self):
        return self.kp*self.error
    def get_kde(self):
        return self.kd*self.derivative_error
    def get_kie(self):
        return self.ki*self.integral_error



class Mission(Hardware_com):
    def __init__(self):
        super().__init__("test")
        self.midl = self.create_subscription(MiddleOfAruco, 'aruco_markers', self.point_of_aruco ,10)

        self.toggle_velocity_control_cli = self.create_client(ToggleVelocityControl,'toggle_v_control')

        self.velocity_publisher = self.create_publisher(VelocityVectors,'velocity_vectors', 10)

    
    def point_of_aruco(self, msg):
        self.get_logger().info(f"Point of middle [{msg.x}, {msg.y}]")
        self.middle_of_aruco = [msg.x, msg.y]

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
        self.pid = PID(2,0.2,2)
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
            #self.pid.compute(self.)
            rclpy.spin_once(self, timeout_sec=0.05)
    
def main(args=None):
    rclpy.init(args=args)
    mission = Mission()
    mission.arm()
    mission.takeoff(5.0)

    #mission.send_goto_relative( 8.0, 0.0, 0.0)
    mission.toggle_control()
    mission.fly_in_square()
    #mission.send_vectors(0.5,0.5,0)
    #time.sleep(1)
    #mission.send_vectors(0,0,0)
    #time.sleep(10)
    mission.toggle_control()
    mission.land()
    mission.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
