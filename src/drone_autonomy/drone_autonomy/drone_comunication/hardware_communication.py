import rclpy  # Python Client Library for ROS 2
from rclpy.node import Node
from rclpy.action import ActionClient

from drone_interfaces.msg import Telemetry
from drone_interfaces.srv import GetLocationRelative, GetAttitude, SetYaw, SetMode
from drone_interfaces.action import GotoRelative, GotoGlobal, Arm, Takeoff, SetYawAction

import time
class Hardware_com(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)

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
        self.takeoff_action_client = ActionClient(self, Takeoff, 'takeoff')
        self.arm_action_client = ActionClient(self, Arm, 'Arm')
        self.yaw_action_client = ActionClient(self, SetYawAction, 'Set_yaw')

        # DECLARE subscriptions
        self.tlemetry_sub = self.create_subscription(Telemetry, 'telemetry', self.telemetry_callback, 10)

        self.get_logger().info("Mission node created")
        
        self.__state = "OK"
        self.__voltage_spikes = 0
        self.__battery_failsafe = False
        self.__alarm = False
        self.__emergency_flight_data = self.get_gps()

    # DECLARE send mode request functions

    def arm(self):
        request = SetMode.Request()
        request.mode = "GUIDED"
        mode_future = self.mode_cli.call_async(request)
        rclpy.spin_until_future_complete(self, mode_future, timeout_sec=10)
        self.get_logger().info("Mode request sucesfull")
        self.get_logger().info("Sending ARM action goal")

        goal_msg = Arm.Goal()
        self.__state = "BUSY"
        while not self.arm_action_client.wait_for_server():
            self.get_logger().info('waiting for ARM server...')
        self.arm_future = self.arm_action_client.send_goal_async(goal_msg)
        self.arm_future.add_done_callback(self._arm_response_callback)
        self._wait_busy()

    def _arm_response_callback(self, future):
        goal_handle = future.result()
        self.get_logger().info('waiting for arm response...')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self._arm_get_result_callback)

    def _arm_get_result_callback(self, future):
        result = future.result().result.result
        self.__state = "OK"

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
        rclpy.spin_until_future_complete(self, gps_future, timeout_sec=10)
        if gps_future.result() is not None:
            self.north = gps_future.result().north
            self.east = gps_future.result().east
            self.down = gps_future.result().down
            self.drone_amplitude = -self.down
            self.get_logger().info("GPS Recieved")
        else:
            self.get_logger().info("GPS request failed")
            self.drone_amplitude = 0
            return None
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
        self.__state = "BUSY"
        self.get_logger().info("Sending set yaw action goal")
        goal_msg = SetYawAction.Goal()
        goal_msg.yaw = yaw
        goal_msg.relative = relative

        while not self.yaw_action_client.wait_for_server():
            self.get_logger().info("waiting for yaw server...")
        
        self.send_goal_future = self.yaw_action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self._set_yaw_response)
        self.get_logger().info("Yaw action sent")
        self._wait_busy()

    def _set_yaw_response(self, future):
        self.get_logger().info("Yaw response callback")
        self._goal_handle = future.result()
        self.get_result_future = self._goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self._set_yaw_result_callback)
    
    def _set_yaw_result_callback(self, kamil):
        self.get_logger().info("Yaw  action finished")
        self.__state = "OK"

    # DECLARE go_to functions

    def takeoff(self, height: float):
        self.get_logger().info("Sending TAKE-OFF action goal")
        goal_msg = Takeoff.Goal()
        goal_msg.altitude = height
        while not self.takeoff_action_client.wait_for_server():
            self.get_logger().info('waiting for TAKE-OFF server...')
        self.__state = "BUSY"
        self.takeoff_future = self.takeoff_action_client.send_goal_async(goal_msg)
        self.takeoff_future.add_done_callback(self._takeoff_response_callback)
        self._wait_busy()

    def _takeoff_response_callback(self, future):
        self.get_logger().info('waiting for takeoff response...')
        self._goal_handle = future.result()
        self.takeoff_get_result_future = self._goal_handle.get_result_async()
        self.takeoff_get_result_future.add_done_callback(self._takeoff_get_result_callback)

    def _takeoff_get_result_callback(self, future):
        result = future.result().result.result
        self.get_logger().info("Set Takeoff action finished")
        self.__state = "OK"

    def send_goto_relative(self, north: float, east: float, down: float):
        self.get_logger().info("Sending goto relative action goal")
        goal_msg = GotoRelative.Goal()
        goal_msg.north = north
        goal_msg.east = east
        goal_msg.down = down
        self.__state = "BUSY"
        
        while not self.goto_rel_action_client.wait_for_server():
            self.get_logger().info("waiting for goto server...")

        self.send_goal_future = self.goto_rel_action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self._goto_rel_response_callback)
        self.get_logger().info("Goto action sent")
        self._wait_busy()

    def _goto_rel_response_callback(self, future):
        self.get_logger().info("Goto rel response callback")
        self._goal_handle = future.result()
        self.get_result_future = self._goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self._goto_rel_result_callback)

    def _goto_rel_result_callback(self, future):
        self.get_logger().info("Goto rel  action finished")
        self.__state = "OK"

    def send_goto_global(self, lat: float, lon: float, alt: float):
        self.__state = "BUSY"
        self.get_logger().info("Sending goto global action goal")
        goal_msg = GotoGlobal.Goal()
        goal_msg.lat = lat
        goal_msg.lon = lon
        goal_msg.alt = alt
        while not self.goto_glob_action_client.wait_for_server():
            self.get_logger().info("waiting for goto server...")

        self.send_goal_future = self.goto_glob_action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self._goto_glob_response_callback)
        self.get_logger().info("Goto action sent")
        self._wait_busy()

    def _goto_glob_response_callback(self, future):
        self.get_logger().info("Goto rel response callback")
        self._goal_handle = future.result()
        self.get_result_future = self._goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self._goto_glob_result_callback)

    def _goto_glob_result_callback(self, future):
        self.get_logger().info("Goto rel  action finished")
        self.__state = "OK"

    # DECLARE telemetry function

    def telemetry_callback(self, msg):
        #self.get_logger().info(f"Actual baterry level {msg.battery_percentage},{msg.battery_voltage}")
        voltage_threshold = 12

        if self.__voltage_spikes >= 5 and not self.__battery_failsafe:
            future = self._goal_handle.cancel_goal_async()
            future.add_done_callback(self._emergency_flight_request)
            self.get_logger().info("anuluje")
            self.__battery_failsafe = True

        if msg.battery_voltage < voltage_threshold and not self.__battery_failsafe:
            self.__voltage_spikes += 1

    # DECLARE private function

    def fly_to_base(self, north: float, east: float, down: float):
        self.get_logger().info("Sending goto relative action goal")
        goal_msg = GotoRelative.Goal()
        goal_msg.north = north
        goal_msg.east = east
        goal_msg.down = down
        self.__state = "BUSY"
        
        while not self.goto_rel_action_client.wait_for_server():
            self.get_logger().info("waiting for goto server...")

        send_goal_future = self.goto_rel_action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._fly_to_base_response_callback)
        self.get_logger().info("Goto action sent")
        self._wait_busy()

    def _fly_to_base_response_callback(self, future):
        self.get_logger().info("Goto rel response callback")
        _goal_handle = future.result()
        get_result_future = _goal_handle.get_result_async()
        get_result_future.add_done_callback(self._fly_to_base_result_callback)

    def _fly_to_base_result_callback(self, future):
        self.get_logger().info("Goto rel  action finished")
        self.__state = "OK"

    def _emergency_flight_request(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled')
            self.get_logger().info('Start emegrency mission')
           # while self.__state == "BUSY":
            #    time.sleep(0.1)
            self.__alarm = True

        else:
            self.get_logger().info('Goal failed to cancel')
            future = self._goal_handle.cancel_goal_async()
            future.add_done_callback(self._emergency_flight_request)

    def _emergency_flight(self):
        self.__alarm = False
        self.fly_to_base(self.__emergency_flight_data[0],
                                  self.__emergency_flight_data[1],
                                  self.__emergency_flight_data[2])
        self.land()
        rclpy.shutdown()

    def _wait_busy(self):
        while self.__state == "BUSY":
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.__alarm:
            self._emergency_flight()

if __name__ == "__main__":
    print("you shoudn't run this program directly from this file")