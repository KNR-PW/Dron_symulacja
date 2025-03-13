class DroneUtils(Node):
    def __init__(self):
        super().__init__('drone_utils')
        self.vehicle = None

        ## DECLARE PARAMETERS
        #---------------------'  give here ip to flight controler   '
        self.declare_parameter('fc_ip', '/dev/ttyACM0')   

    def __del__(self):
        if self.vehicle:
            self.vehicle.mode=VehicleMode("RTL")

    def shoot_callback(self, goal_handle):
        self.get_logger().info(f"Incoming shoot goal for color: {goal_handle.request.color}")
        stop = 1000
        shoot = 1200
        load =1500

        left = 2000
        mid = 1400
        right = 800

        self.set_servo(10,stop)
        self.set_servo(11,stop)
        time.sleep(1)
        self.set_servo(9,left if goal_handle.request.color == 'yellow' else right)
        self.set_servo(10,load)
        self.set_servo(11,load)
        time.sleep(2)
        self.set_servo(10,shoot)
        self.set_servo(11,shoot)
        self.set_servo(9,mid - 300 if goal_handle.request.color == 'yellow' else mid+300)
        time.sleep(1)
        self.set_servo(10,stop)
        self.set_servo(11,stop)
        self.set_servo(9,mid)

        self.get_logger().info("Shoot action completed:" + goal_handle.request.color)
        goal_handle.succeed()
        result = Shoot.Result()
        return result

def main():
    rclpy.init()
    
    drone = DroneUtils()

    rclpy.spin(drone)

    drone.destroy_node()

    rclpy.shutdown()


if __name__ == 'main':
    main()
