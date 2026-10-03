"""
drone_handler_px4 - warstwa sprzetowa KNR, poprawki znalezione przez ERC 2026 i pracę inżynierską KT.
"""
import time
import math
import haversine as hv

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup

from drone_interfaces.msg import VelocityVectors, Telemetry
from drone_interfaces.srv import SetMode, ToggleVelocityControl, SetServo, VtolServoCalib
from drone_interfaces.action import Arm, Takeoff, GotoGlobal, GotoRelative, SetYawAction

from px4_msgs.msg import (
    VehicleStatus,
    VehicleCommand,
    OffboardControlMode,
    VehicleGlobalPosition,
    TrajectorySetpoint,
    VehicleLocalPosition,
    VehicleAttitude,
    BatteryStatus,
    ActuatorServos,
)


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def quat_to_euler(q): # Ladnie napisane teraz i jawnie
    w, x, y, z = q
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


class GlobalPosition():
    def __init__(self):
        self.alt = float(0.0)   # AMSL [m], wczesniej nie patrzyl na to wgl
        self.lat = float(0.0)
        self.lon = float(0.0)


class LocalPosition():
    def __init__(self):
        self.x = float(0.0)     # North [m]
        self.y = float(0.0)     # East [m]
        self.z = float(0.0)     # Down [m] (ujemne = nad startem)
        self.vx = float(0.0)
        self.vy = float(0.0)
        self.vz = float(0.0)
        self.heading = float(0.0)


class BatteryInfo():
    def __init__(self):
        self.voltage = float(0.0)
        self.current = float(0.0)
        self.number_of_cells = 0
        self.remaining = -1.0   # 0..1 z PX4, -1 = brak


class DroneHandlerPX4(Node):
    def __init__(self):
        super().__init__('drone_handler_px4')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        NAMESPACE = 'knr_hardware/'

        # parametry, do zabawy, ustawione pod ERC 2026 i moja inzynierke
        self.declare_parameter('vel_timeout', 0.5)        # [s] Velocity setpoint lecial w nieskonczonosc jak padl handler
        self.declare_parameter('arm_timeout', 30.0)       # [s] Arm lecial w nieskonczonosc kolejny bug 
        self.declare_parameter('goto_tolerance', 0.2)     # [m] Wszystko w jendym miejscu
        self.declare_parameter('yaw_tolerance_deg', 3.0)  # [deg] Yaw tolerance znacznie za duze bylo 30 stopni
        self.declare_parameter('takeoff_timeout', 60.0)   # [s] Takeoff sie konczyl jak tylko wzlecial do gory
        self.declare_parameter('dev', False)
        p = lambda n: self.get_parameter(n).value          # noqa: E731
        self.vel_timeout = float(p('vel_timeout'))
        self.arm_timeout = float(p('arm_timeout'))
        self.goto_tolerance = float(p('goto_tolerance'))
        self.yaw_tolerance = math.radians(float(p('yaw_tolerance_deg')))
        self.takeoff_timeout = float(p('takeoff_timeout'))
        self.dev_mode = bool(p('dev'))

        self._servo_controls = [0.0] * 8

        # Wszystkie callbacki w grupie reentrant: akcje czekaja w petlach, a timery
        # (heartbeat offboard, watchdog) i subskrypcje musza dzialac rownolegle.
        cbg = ReentrantCallbackGroup()

        # uslugi
        self.mode = self.create_service(SetMode, NAMESPACE + 'set_mode', self.set_mode_callback,
                                        callback_group=cbg)
        self.toggle_velocity_control_srv = self.create_service(
            ToggleVelocityControl, NAMESPACE + 'toggle_v_control', self.toggle_velocity_control,
            callback_group=cbg)
        self.servo = self.create_service(SetServo, NAMESPACE + 'set_servo', self.set_servo_callback,
                                         callback_group=cbg)
        self.calib_servo_srv = self.create_service(VtolServoCalib, NAMESPACE + 'calib_servo',
                                                   self.calib_servo_callback, callback_group=cbg)
        # akcje
        self.arm = ActionServer(self, Arm, NAMESPACE + 'Arm', self.arm_callback,
                                callback_group=cbg)
        self.takeoff = ActionServer(self, Takeoff, NAMESPACE + 'takeoff', self.takeoff_callback,
                                    cancel_callback=self.cancel_callback, callback_group=cbg)
        self.goto_global = ActionServer(self, GotoGlobal, NAMESPACE + 'goto_global',
                                        self.goto_global_action,
                                        cancel_callback=self.cancel_callback, callback_group=cbg)
        self.goto_rel = ActionServer(self, GotoRelative, NAMESPACE + 'goto_relative',
                                     self.goto_relative_action,
                                     cancel_callback=self.cancel_callback, callback_group=cbg)
        self.yaw = ActionServer(self, SetYawAction, NAMESPACE + 'Set_yaw', self.yaw_callback,
                                cancel_callback=self.cancel_callback, callback_group=cbg)

        # subskrypcje PX4
        self.status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1', self.vehicle_status_callback, qos_profile,
            callback_group=cbg)
        self.global_position_sub = self.create_subscription(
            VehicleGlobalPosition, '/fmu/out/vehicle_global_position',
            self.vehicle_global_position_callback, qos_profile, callback_group=cbg)
        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self.vehicle_local_position_callback, qos_profile, callback_group=cbg)
        self.attitude_sub = self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude', self.attitude_callback, qos_profile,
            callback_group=cbg)
        self.battery_receiver = self.create_subscription(
            BatteryStatus, '/fmu/out/battery_status_v1', self.battery_callback, qos_profile,
            callback_group=cbg)
        self.vector_receiver = self.create_subscription(
            VelocityVectors, NAMESPACE + 'velocity_vectors', self.velocity_control_callback, 10,
            callback_group=cbg)

        # publishery - Wczesniej byly wgl dwa XD
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.actuator_pub = self.create_publisher(ActuatorServos, '/fmu/in/actuator_servos', 10)
        self.telemetry_publisher = self.create_publisher(Telemetry, NAMESPACE + 'telemetry', 10)

        # stan
        self.px4_alive_flag = False
        self.px4_watchdog = 0                 # time.monotonic_ns() ostatniej pozycji, bo na WSL mi sie zjebalo raz
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arm_state = VehicleStatus.ARMING_STATE_ARMED
        self.failsafe = False
        self.flightCheck = False
        self.offboard_setpoint_counter = 0
        self.flight_mode_flag = False         # False = pozycja, True = predkosc
        self._is_fixed_wing = False
        self._goto_global_acceptance_m_fw = 80.0
        self._goto_global_acceptance_m_mc = 2.0
        self.current_setpoint = None          # (x, y, z, yaw) NED, publikowany 10 Hz w trybie pozycji
        self.trueYaw = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self._last_vel_ns = 0                 # FIX ostatnie velocity_vectors (monotonic), zeby nie lecial w niesk.
        self._vel_timeout_active = False
        self._direct_actuator = False         # ale to zjebane jest napisane przepraszam
        self._battery_wait_logged = False

        self.global_position = GlobalPosition()
        self.local_position = LocalPosition()
        self.battery_info = BatteryInfo()
        self.state_decoder = (
            "Manual", "Altitude control", "Position control", "Auto mission mode", "Auto loiter",
            "Return to launch", "Position slow", "Free5", "Altitude cruise", "Free3", "Acro",
            "Free2", "Descend", "Termination", "Offboard", "Stabilize", "Free1", "Takeoff", "Land",
            "Follow target", "Precision land", "Orbit", "Vtol takeoff", "External1", "External2",
            "External3", "External4", "External5", "External6", "External7", "External8", "Max")

        self.timer = self.create_timer(0.1, self.timer_callback, callback_group=cbg)
        self.telemetry_timer = self.create_timer(0.1, self.telemetry_callback, callback_group=cbg)

        if self.dev_mode:
            self.get_logger().warn("DEV MODE is enabled.")
        self.get_logger().info("starting knr drone handler px4")

    def nav_name(self, s):
        return self.state_decoder[s] if 0 <= s < len(self.state_decoder) else f"nav_state {s}"

    def armed(self):
        return self.arm_state == VehicleStatus.ARMING_STATE_ARMED

    def set_position_mode(self, why=""):
        """goto/takeoff/yaw potrzebuja trybu pozycyjnego - inaczej timer nie publikuje setpointu."""
        if self.flight_mode_flag:
            self.get_logger().info(f"tryb velocity -> pozycja ({why})")
            self.flight_mode_flag = False

    def publish_offboard_control_heartbeat_signal(self):
        msg = OffboardControlMode()
        msg.position = not self.flight_mode_flag
        msg.velocity = self.flight_mode_flag
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def timer_callback(self) -> None:
        if not self.px4_alive_flag:
            return
        if not self._direct_actuator:                 # bez sprzecznych trybow, zjebanie napisane v69
            self.publish_offboard_control_heartbeat_signal()

        if self.flight_mode_flag:
            # FIX 30.09 brak komend predkosci -> zawis. Wczesniej PX4 trzymal ostatni
            # setpoint predkosci w nieskonczonosc (dron lecial dalej po padnieciu klienta).
            stale_s = (time.monotonic_ns() - self._last_vel_ns) / 1e9
            if stale_s > self.vel_timeout:
                if not self._vel_timeout_active:
                    self.get_logger().warn(f"brak velocity_vectors od {stale_s:.1f} s -> zawis")
                    self._vel_timeout_active = True
                self.publish_velocity_setpoint(0.0, 0.0, 0.0, 0.0)
        elif self.current_setpoint is not None:
            x, y, z, yaw = self.current_setpoint
            self._publish_setpoint_raw(x, y, z, yaw)

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1
        if self.offboard_setpoint_counter == 10:
            self.get_logger().info("Vehicle is ready to be set into offboard mode")

        # FIX 29.08 zegar monotoniczny - skok zegara systemowego nie wyzwala falszywego alarmu
        if time.monotonic_ns() - self.px4_watchdog > 1e9:
            self.px4_alive_flag = False
            self.get_logger().warn("Vehicle is missing ERROR PX4 not found")
            self.offboard_setpoint_counter = 0

    def vehicle_status_callback(self, msg):
        if msg.nav_state != self.nav_state:
            self.get_logger().info(f"NAV_STATUS: {self.nav_name(msg.nav_state)} {msg.nav_state}")
        if msg.arming_state != self.arm_state:
            self.get_logger().info(f"ARM STATUS: {msg.arming_state}")
        if msg.failsafe != self.failsafe:
            self.get_logger().info(f"FAILSAFE: {msg.failsafe}")
        if msg.pre_flight_checks_pass != self.flightCheck:
            if msg.pre_flight_checks_pass:
                self.get_logger().info("Drone can be armed")
            else:
                self.get_logger().warn("Drone can't be armed")
        self.nav_state = msg.nav_state
        self.arm_state = msg.arming_state
        self.failsafe = msg.failsafe
        self.flightCheck = msg.pre_flight_checks_pass

    def vehicle_global_position_callback(self, msg):
        self.global_position.alt = msg.alt
        self.global_position.lat = msg.lat
        self.global_position.lon = msg.lon

    def vehicle_local_position_callback(self, msg):
        lp = self.local_position
        lp.x, lp.y, lp.z = msg.x, msg.y, msg.z
        lp.vx, lp.vy, lp.vz = msg.vx, msg.vy, msg.vz
        lp.heading = msg.heading
        self.px4_alive_flag = True
        self.px4_watchdog = time.monotonic_ns()

    def velocity_control_callback(self, msg):
        if not self.flight_mode_flag:
            return
        self._last_vel_ns = time.monotonic_ns()
        if self._vel_timeout_active:
            self.get_logger().info("velocity_vectors wznowione")
            self._vel_timeout_active = False
        # FIX 02.10 body FRD (vx przod, vy prawo) -> NED przez yaw drona. Wynik identyczny
        # jak w poprzedniej wersji (tam: trueYaw = yaw - pi i dwa minusy, ktore sie znosily top 10 code)
        c, s = math.cos(self.trueYaw), math.sin(self.trueYaw)
        v_n = msg.vx * c - msg.vy * s
        v_e = msg.vx * s + msg.vy * c
        self.get_logger().debug(f"velocity body ({msg.vx:.2f},{msg.vy:.2f},{msg.vz:.2f}) "
                                f"-> NED ({v_n:.2f},{v_e:.2f}), yaw_rate {msg.yaw:.2f} rad/s")
        self.publish_velocity_setpoint(v_n, v_e, msg.vz, msg.yaw)

    def attitude_callback(self, msg):
        # FIX 02.10 poprawna formula (wczesniej odwrocony mianownik dawal yaw - pi)
        self.roll, self.pitch, self.trueYaw = quat_to_euler(msg.q)

    def battery_callback(self, msg):
        self.battery_info.voltage = msg.voltage_v
        self.battery_info.current = msg.current_a
        self.battery_info.number_of_cells = msg.cell_count
        self.battery_info.remaining = msg.remaining

    def publish_vehicle_command(self, command, **params) -> None:
        msg = VehicleCommand()
        msg.command = command
        for i in range(1, 8):
            setattr(msg, f"param{i}", float(params.get(f"param{i}", 0.0)))
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def _publish_setpoint_raw(self, x: float, y: float, z: float, yaw: float):
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z)]
        msg.velocity = [float('nan')] * 3
        msg.yaw = float(yaw)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float, yaw="ORIGINAL"):
        if yaw == "ORIGINAL":
            yaw = self.local_position.heading
        self.current_setpoint = (x, y, z, yaw)
        self._publish_setpoint_raw(x, y, z, yaw)
        return (x, y, z, yaw)

    def publish_velocity_setpoint(self, x: float = 0.0, y: float = 0.0, z: float = 0.0,
                                  yaw_speed: float = 0.0):
        """Predkosc w NED [m/s], yaw_speed [rad/s]."""
        msg = TrajectorySetpoint()
        msg.velocity = [float(x), float(y), float(z)]
        msg.position = [float('nan')] * 3
        msg.yaw = float('nan')
        msg.yawspeed = float(yaw_speed)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def change_flight_mode_flag(self):
        self.flight_mode_flag = not self.flight_mode_flag
        if self.flight_mode_flag:
            self._last_vel_ns = 0          # do pierwszej komendy: zawis
            self._vel_timeout_active = False

    def telemetry_callback(self):
        if self.battery_info.number_of_cells == 0 and self.battery_info.remaining < 0:
            if not self._battery_wait_logged:            # wczesniej: log co 0.1 s bez konca wkurwialo mega
                self.get_logger().info("waiting for battery status")
                self._battery_wait_logged = True
            return
        msg = Telemetry()
        b = self.battery_info
        if b.remaining >= 0.0:
            msg.battery_percentage = int(round(b.remaining * 100))
        elif b.number_of_cells > 0:
            msg.battery_percentage = int((b.voltage / float(b.number_of_cells * 4.2)) * 100)
        msg.battery_voltage = b.voltage
        msg.battery_current = b.current
        msg.global_lat = self.global_position.lat
        msg.global_lon = self.global_position.lon
        msg.global_alt = self.global_position.alt
        msg.flight_mode = self.nav_name(self.nav_state)
        msg.speed = math.sqrt(self.local_position.vx ** 2 + self.local_position.vy ** 2)
        msg.lat = self.global_position.lat
        msg.lon = self.global_position.lon
        msg.alt = self.global_position.alt
        msg.roll = float(self.roll)                       # wczesniej puste XD
        msg.pitch = float(self.pitch)
        msg.yaw = float(self.trueYaw)
        self.telemetry_publisher.publish(msg)

    def set_mode_callback(self, request, response):
        mode = request.mode.upper()
        self.get_logger().info(f"set_mode {mode}")
        if mode == 'GUIDED':
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        elif mode == 'LAND':
            self.current_setpoint = None
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        elif mode == 'RTL':
            self.current_setpoint = None
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)
        elif mode == 'HOLD':
            # AUTO / LOITER - zawis niezalezny od komputera pokladowego
            self.current_setpoint = None
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                                         param1=1.0, param2=4.0, param3=3.0)
        elif mode == 'FIXED_WING':
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_VTOL_TRANSITION, param1=4.0)
            self._is_fixed_wing = True
        elif mode == 'MULTICOPTER':
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_VTOL_TRANSITION, param1=3.0)
            self._is_fixed_wing = False
        else:
            self.get_logger().warn(f"nieznany tryb '{request.mode}' - zignorowany "
                                   f"(GUIDED, LAND, RTL, HOLD, FIXED_WING, MULTICOPTER)")
        return SetMode.Response()

    def toggle_velocity_control(self, request, response):
        self.change_flight_mode_flag()
        self.get_logger().info(f"tryb: {'velocity' if self.flight_mode_flag else 'pozycja'}")
        response.result = self.flight_mode_flag
        return response

    def arm_callback(self, goal_handle):
        self.get_logger().info('-- Arm action registered --')
        feedback_msg = Arm.Feedback()
        result = Arm.Result()
        t0 = time.monotonic()

        while not self.flightCheck and not self.dev_mode:
            if time.monotonic() - t0 > self.arm_timeout:                   # FIX ARM, stop
                self.get_logger().error("Arm: dron nie stal sie gotowy (preflight) - przerywam")
                goal_handle.abort()
                result.result = 0
                return result
            feedback_msg.feedback = "Waiting for vehicle to become armable..."
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        t_send = 0.0
        while not self.armed():
            if time.monotonic() - t_send > 2.0:                            # FIX ARM, ponawianie
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                                             param1=1.0)
                t_send = time.monotonic()
                self.get_logger().info('Arm command sent')
            if time.monotonic() - t0 > self.arm_timeout:
                self.get_logger().error("Arm: PX4 nie uzbroil drona - przerywam "
                                        "(powod w konsoli PX4: 'Arming denied')")
                goal_handle.abort()
                result.result = 0
                return result
            feedback_msg.feedback = "Waiting for drone to become armed..."
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.2)

        self.get_logger().info("Vehicle is now armed.")
        goal_handle.succeed()
        result.result = 1
        return result

    def takeoff_callback(self, goal_handle):
        feedback_msg = Takeoff.Feedback()
        result = Takeoff.Result()
        alt = float(goal_handle.request.altitude)
        target_z = -alt
        self.set_position_mode("takeoff")
        x0, y0 = self.local_position.x, self.local_position.y
        self.publish_position_setpoint(x0, y0, target_z)
        t0 = time.monotonic()

        # FIX wysokosc WZGLEDNA z lokalnego NED
        # akcja konczyla sie natychmiast(WORKAROUND NA ERC, juz usuniete)
        while -self.local_position.z < 0.95 * alt:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Takeoff canceled')
                return result
            if time.monotonic() - t0 > self.takeoff_timeout:
                self.get_logger().error(f"Takeoff: brak wysokosci {alt} m po "
                                        f"{self.takeoff_timeout:.0f} s")
                goal_handle.abort()
                result.result = 0
                return result
            feedback_msg.altitude = float(-self.local_position.z)
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.2)

        self.get_logger().info(f"Reached target altitude {-self.local_position.z:.1f} m")
        goal_handle.succeed()
        result.result = 1
        return result

    def goto_global_action(self, goal_handle):
        req = goal_handle.request
        self.get_logger().info(f'-- Goto global: lat {req.lat} lon {req.lon} alt(AMSL) {req.alt} --')
        target = GlobalPosition()
        target.lat, target.lon, target.alt = req.lat, req.lon, req.alt

        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_REPOSITION, param1=-1.0, param2=1.0,
                                     param5=target.lat, param6=target.lon, param7=target.alt)
        feedback_msg = GotoGlobal.Feedback()
        acceptance_radius = (self._goto_global_acceptance_m_fw if self._is_fixed_wing
                             else self._goto_global_acceptance_m_mc)
        feedback_msg.distance = self.get_distance_global(self.global_position, target)
        while feedback_msg.distance > acceptance_radius:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return GotoGlobal.Result()
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_REPOSITION, param1=-1.0,
                                         param2=1.0, param5=target.lat, param6=target.lon,
                                         param7=target.alt)
            feedback_msg.distance = self.get_distance_global(self.global_position, target)
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        # FIX powrot do offboard w miejscu docelowym. DO_REPOSITION bierze wysokosc AMSL,
        # a setpoint offboard jest w lokalnym NED - przeliczenie z roznicy wysokosci.
        z_target = self.local_position.z - (target.alt - self.global_position.alt)
        self.set_position_mode("goto_global")
        self.publish_position_setpoint(self.local_position.x, self.local_position.y, z_target)
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        goal_handle.succeed()
        result = GotoGlobal.Result()
        result.result = 1
        return result

    def get_distance_global(self, a: GlobalPosition, b: GlobalPosition):
        return hv.haversine((a.lat, a.lon), (b.lat, b.lon)) * 1000.0

    def goto_relative_action(self, goal_handle):
        req = goal_handle.request
        dest = LocalPosition()
        dest.x = self.local_position.x + req.north
        dest.y = self.local_position.y + req.east
        dest.z = self.local_position.z + req.down
        self.get_logger().info(f'-- Goto relative: N {dest.x:.1f} E {dest.y:.1f} D {dest.z:.1f} --')

        self.set_position_mode("goto_relative")
        self.publish_position_setpoint(dest.x, dest.y, dest.z)

        feedback_msg = GotoRelative.Feedback()
        feedback_msg.distance = self.calculate_remaining_distance_rel(dest)
        timeout = 30.0 + 3.0 * feedback_msg.distance                      
        t0 = time.monotonic()
        while feedback_msg.distance > self.goto_tolerance:                
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal goto rel canceled')
                return GotoRelative.Result()
            if time.monotonic() - t0 > timeout:
                self.get_logger().error(f"goto_relative: timeout, zostalo "
                                        f"{feedback_msg.distance:.1f} m (setpoint utrzymany)")
                goal_handle.abort()
                result = GotoRelative.Result()
                result.result = 0
                return result
            feedback_msg.distance = self.calculate_remaining_distance_rel(dest)
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.1)

        # setpoint zostaje - dron trzyma pozycje miedzy akcjami
        goal_handle.succeed()
        result = GotoRelative.Result()
        result.result = 1
        return result

    def calculate_remaining_distance_rel(self, d: LocalPosition):
        return math.sqrt((d.x - self.local_position.x) ** 2 + (d.y - self.local_position.y) ** 2
                         + (d.z - self.local_position.z) ** 2)


    def yaw_callback(self, goal_handle):
        """[FIX 4/9] Obrot przez setpoint pozycyjny z zadanym yaw (PX4 sam prowadzi
        obrot najkrotsza droga). yaw [rad]; relative=True -> wzgledem obecnego kursu."""
        req = goal_handle.request
        heading = self.local_position.heading
        target = wrap_pi(req.yaw + heading if req.relative else req.yaw)
        self.get_logger().info(f'-- Set yaw: {math.degrees(target):.1f} deg '
                               f'({"wzgl." if req.relative else "abs."}) --')

        prev_velocity_mode = self.flight_mode_flag       # [FIX 4] zawsze zdefiniowane
        self.set_position_mode("Set_yaw")
        lp = self.local_position
        self.publish_position_setpoint(lp.x, lp.y, lp.z, target)

        feedback_msg = SetYawAction.Feedback()
        result = SetYawAction.Result()
        t0 = time.monotonic()
        while True:
            err = wrap_pi(target - self.local_position.heading)            # FIX - razy -
            feedback_msg.angle = float(abs(err))
            goal_handle.publish_feedback(feedback_msg)
            if abs(err) < self.yaw_tolerance:
                break
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                break
            if time.monotonic() - t0 > 30.0:
                self.get_logger().error(f"Set_yaw: timeout, blad {math.degrees(err):.1f} deg")
                goal_handle.abort()
                break
            time.sleep(0.1)

        if prev_velocity_mode:                            # przywroc tryb sprzed akcji
            self.change_flight_mode_flag()
        if goal_handle.is_active:
            goal_handle.succeed()
            result.result = 1
        return result

    def set_servo(self, servo_id: int, pwm: float):
        pwm = max(0.0, min(pwm, 1000.0))                  # PWM 0..1000 -> -1..1
        value = (pwm - 500) / 500.0
        index = servo_id - 1
        if 0 <= index < len(self._servo_controls):
            self._servo_controls[index] = value
        msg = ActuatorServos()
        msg.control = list(self._servo_controls)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.actuator_pub.publish(msg)

    def set_servo_callback(self, request, response):
        self.set_servo(request.servo_id, request.pwm)
        return SetServo.Response()

    def _angle_to_pwm(self, angle_deg: float) -> int:
        """0 deg -> 0 (do gory), 90 deg -> 1000 (do przodu)."""
        return int((max(0.0, min(angle_deg, 90.0)) / 90.0) * 1000.0)

    def _enable_direct_actuator(self):
        msg = OffboardControlMode()
        msg.direct_actuator = True
        msg.position = msg.velocity = msg.acceleration = False
        msg.attitude = msg.body_rate = msg.thrust_and_torque = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def calib_servo(self, angle_deg_4: float, angle_deg_5: float):
        pwm4, pwm5 = self._angle_to_pwm(angle_deg_4), self._angle_to_pwm(angle_deg_5)
        self.get_logger().info(f"calib_servo: S4 {angle_deg_4} deg pwm={pwm4}, "
                               f"S5 {angle_deg_5} deg pwm={pwm5}")
        self._direct_actuator = True          # timer wstrzymuje heartbeat pozycji/predkosci
        try:
            start = time.monotonic()
            while time.monotonic() - start < 10.0:
                self._enable_direct_actuator()
                self.set_servo(4, pwm4)
                self.set_servo(5, pwm5)
                time.sleep(1 / 50)
        finally:
            self._direct_actuator = False

    def calib_servo_callback(self, request, response):
        if self.armed():                      # tylko na ziemi
            response.success = False
            response.message = "Kalibracja serw tylko z rozbrojonym dronem"
            self.get_logger().error(response.message)
            return response
        try:
            self.calib_servo(request.angle_4, request.angle_5)
            response.success = True
            response.message = "Tilts calibrated successfully"
        except Exception as e:                
            self.get_logger().error(f"Calib tilts error: {e}")
            response.success = False
            response.message = f"Error: {e}"
        return response

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT


def main():
    rclpy.init()
    drone = DroneHandlerPX4()
    executor = MultiThreadedExecutor()
    executor.add_node(drone)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        drone.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
