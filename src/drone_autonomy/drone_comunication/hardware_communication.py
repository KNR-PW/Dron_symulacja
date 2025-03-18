import rclpy  # Python Client Library for ROS 2
from rclpy.node import Node
from rclpy.action import ActionClient

from drone_interfaces.msg import Telemetry
from drone_interfaces.srv import GetLocationRelative, GetAttitude, SetYaw, SetMode
from drone_interfaces.action import GotoRelative, GotoGlobal, Shoot, Arm, Takeoff, SetYawAction

class Hardware_com(Node):
    def __init__(self):
        super().__init__("mission")

        # DECLARE service clients
        self.mode_cli = self.create_client(SetMode, 'set_mode')

        self.gps_cli = self.create_client(GetLocationRelative, "get_location_relative")
        while not self.gps_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("GPS service not available, waiting again...")

        self.atti_cli = self.create_client(GetAttitude, "get_attitude")
        while not self.atti_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("attitude service not available, waiting again...")

        # DECLARE actions client
        self.goto_rel_action_client = ActionClient(self, GotoRelative, "goto_relative")
        self.goto_glob_action_client = ActionClient(self, GotoGlobal, "goto_global")
        self.shoot_action_client = ActionClient(self, Shoot, "shoot")
        self.takeoff_action_client = ActionClient(self, Takeoff, 'takeoff')
        self.arm_action_client = ActionClient(self, Arm, 'Arm')
        self.yaw_action_client = ActionClient(self, SetYawAction, 'Set_yaw')

        # DECLARE subscriptions
        self.tlemetry_sub = self.create_subscription(Telemetry, 'telemetry', self.telemetry_callback, 10)

        self.get_logger().info("Mission node created")
        self.__state = "OK"

        self.__voltage_spikes = 0
        self.__battery_failsafe = False

    # DECLARE send mode request functions

    def arm(self):
        request = SetMode.Request()
        request.mode = "GUIDED"
        mode_future = self.mode_cli.call_async(request)
        rclpy.spin_until_future_complete(self, mode_future, timeout_sec=10)
        self.get_logger().info("Mode request sucesfull")
        self.get_logger().info("Sending ARM action goal")

        goal_msg = Arm.Goal()
        self.state = "BUSY"
        while not self.arm_action_client.wait_for_server():
            self.get_logger().info('waiting for ARM server...')
        self.arm_future = self.arm_action_client.send_goal_async(goal_msg)
        self.arm_future.add_done_callback(self.arm_response_callback)
        self._wait_busy()

    def arm_response_callback(self, future):
        goal_handle = future.result()
        self.get_logger().info('waiting for arm response...')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.arm_get_result_callback)

    def arm_get_result_callback(self, future):
        result = future.result().result.result
        self.state = "OK"

    def land(self):
        request = SetMode.Request()
        request.mode = "LAND"
        mode_future = self.mode_cli.call_async(request)
        rclpy.spin_until_future_complete(self, mode_future, timeout_sec=10)
        self.get_logger().info("Mode LAND request sucesfull")

    # DECLARE gps functions

    def get_gps(self):
        self.get_logger().info("Sending GPS request")
        request_gps = GetLocationRelative.Request()
        gps_future = self.gps_cli.call_async(request_gps)
        rclpy.spin_until_future_complete(self, gps_future, timeout_sec=5)
        if gps_future.result() is not None:
            self.north = gps_future.result().north
            self.east = gps_future.result().east
            self.down = gps_future.result().down
            self.drone_amplitude = -self.down
            self.get_logger().info("GPS Recieved")
        else:
            self.get_logger().info("GPS request failed")
            self.drone_amplitude = 0
        return [self.north, self.east, self.down]
    
    # DECLARE yaw functions

    def get_yaw(self):
        self.get_logger().info("Sending yaw request")
        request_attitude = GetAttitude.Request()
        atti_future = self.atti_cli.call_async(request_attitude)
        rclpy.spin_until_future_complete(self, atti_future, timeout_sec=5)
        yaw = atti_future.result().yaw
        return yaw
    
    def set_yaw_action(self, yaw: float, relative: bool = True):
        self.state = "BUSY"
        self.get_logger().info("Sending set yaw action goal")
        goal_msg = SetYawAction.Goal()
        goal_msg.yaw = yaw
        goal_msg.relative = relative

        while not self.yaw_action_client.wait_for_server():
            self.get_logger().info("waiting for yaw server...")
        
        self.send_goal_future = self.yaw_action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.set_yaw_response)
        self.get_logger().info("Yaw action sent")
        self._wait_busy()

    def set_yaw_response(self, future):
        self.get_logger().info("Yaw response callback")
        self._goal_handle = future.result()
        self.get_result_future = self._goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.set_yaw_result_callback)
    
    def set_yaw_result_callback(self, kamil):
        self.get_logger().info("Yaw  action finished")
        self.state = "OK"

    # DECLARE go_to functions

    def takeoff(self, height: float):
        self.get_logger().info("Sending TAKE-OFF action goal")
        goal_msg = Takeoff.Goal()
        goal_msg.altitude = height
        while not self.takeoff_action_client.wait_for_server():
            self.get_logger().info('waiting for TAKE-OFF server...')
        self.state = "BUSY"
        self.takeoff_future = self.takeoff_action_client.send_goal_async(goal_msg)
        self.takeoff_future.add_done_callback(self.takeoff_response_callback)
        self._wait_busy()

    def takeoff_response_callback(self, future):
        self.get_logger().info('waiting for takeoff response...')
        self._goal_handle = future.result()
        self.takeoff_get_result_future = self._goal_handle.get_result_async()
        self.takeoff_get_result_future.add_done_callback(self.takeoff_get_result_callback)

    def takeoff_get_result_callback(self, future):
        result = future.result().result.result
        self.state = "OK"

    def send_goto_relative(self, north: float, east: float, down: float):
        self.state = "BUSY"
        self.get_logger().info("Sending goto relative action goal")
        goal_msg = GotoRelative.Goal()
        goal_msg.north = north
        goal_msg.east = east
        goal_msg.down = down
        
        while not self.goto_rel_action_client.wait_for_server():
            self.get_logger().info("waiting for goto server...")

        self.send_goal_future = self.goto_rel_action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goto_rel_response_callback)
        self.get_logger().info("Goto action sent")

    def goto_rel_response_callback(self, future):
        self.get_logger().info("Goto rel response callback")
        self._goal_handle = future.result()
        self.get_result_future = self._goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.goto_rel_result_callback)

    def goto_rel_result_callback(self, future):
        self.get_logger().info("Goto rel  action finished")
        self.state = "OK"

    def send_goto_global(self, lat: float, lon: float, alt: float):
        self.state = "BUSY"
        self.get_logger().info("Sending goto global action goal")
        goal_msg = GotoGlobal.Goal()
        goal_msg.lat = lat
        goal_msg.lon = lon
        goal_msg.alt = alt
        while not self.goto_rel_action_client.wait_for_server():
            self.get_logger().info("waiting for goto server...")

        self.send_goal_future = self.goto_glob_action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goto_glob_response_callback)
        self.get_logger().info("Goto action sent")

    def goto_glob_response_callback(self, future):
        self.get_logger().info("Goto rel response callback")
        self._goal_handle = future.result()
        self.get_result_future = self._goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.goto_glob_result_callback)

    def goto_glob_result_callback(self, future):
        self.get_logger().info("Goto rel  action finished")
        self.state = "OK"

    # DECLARE shoot action

    def send_shoot_goal(self, color):
        self.get_logger().info(f"Sending shoot goal, color: {color}")
        self.state = "BUSY"
        goal_msg = Shoot.Goal()
        goal_msg.color = color
        self.send_goal_future = self.shoot_action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.shoot_response_callback)
        self.get_logger().info("Shoot action sent")       

    def shoot_response_callback(self, future):
        self.get_logger().info("Shoot response callback")
        goal_handle = future.result()
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.shoot_result_callback)

    def shoot_result_callback(self, future):
        self.get_logger().info("Shoot action finished")
        self.state = "OK"

    # DECLARE telemetry function

    def telemetry_callback(self, msg):
        self.get_logger().info(f"Actual baterry level {msg.battery_percentage},{msg.battery_voltage}")
        voltage_threshold = 12

        if self.__voltage_spikes >= 5 and not self.__battery_failsafe:
            future = self._goal_handle.cancel_goal_async()
            future.add_done_callback(self._emergency_flight)
            self.__battery_failsafe = True

        if msg.battery_voltage < voltage_threshold and not self.__battery_failsafe:
            self.__voltage_spikes += 1

    # DECLARE private function

    def _emergency_flight(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled')
            self.get_logger().info('Start emegrency mission')
            self.land()
            rclpy.shutdown()

        else:
            self.get_logger().info('Goal failed to cancel')
            future = self._goal_handle.cancel_goal_async()
            future.add_done_callback(self._emergency_flight)

    def _wait_busy(self):
        while self.state == "BUSY":
            rclpy.spin_once(self, timeout_sec=0.05)

if __name__ == "__main__":
    print("you shoudn't run this program directly from this file")